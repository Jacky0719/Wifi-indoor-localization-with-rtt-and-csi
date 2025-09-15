#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Read FTM/CSI from serial (or stdin), pair each FTM with the first following CSI,
and write one-row-per-pair into CSV.

Outputs:
  - dist_est_m
  - rssi_mean_dbm, rssi_min_dbm
  - RTT robust features: min, k2, median, p10, p25, p75, iqr, mad, mean_trim, std, valid_ratio
  - csi_data_json
"""

import re
import csv
import json
import sys
import time
import argparse
import numpy as np

try:
    import serial  # pyserial
except ImportError:
    serial = None

# ------------------ parsing patterns ------------------
RE_FTM_REPORT = re.compile(r'^ftm_report,([^,]+),(\d+),(\d+),([0-9.]+)\s*$', re.I)
RE_FTM_FRAME  = re.compile(r'^ftm_frame,([^,]+),idx=(\d+),dtoken=(\d+),rssi=(-?\d+),rtt_ps=(\d+)\s*$', re.I)
RE_CSI        = re.compile(r'^csi_data,([^,]+),\[(.*)\]\s*$', re.I)
RE_ANCHOR     = re.compile(r'^anchor,([^,]+),([0-9a-f:]+),ch=(\d+)\s*$', re.I)  # 可忽略，不写 CSV

def parse_csi_payload(payload: str):
    """payload like '0,0,11,-10,...' -> list[int]"""
    if not payload:
        return []
    vals = []
    for t in payload.split(','):
        t = t.strip()
        if not t:
            continue
        try:
            vals.append(int(t))
        except ValueError:
            pass
    return vals

# ------------------ robust RTT features ------------------
def robust_feats_from_rtts(rtts):
    # rtts: list[float] (length m, variable)
    x = np.sort(np.array(rtts, dtype=np.float64))
    m = len(x)
    if m == 0:
        # 全 NaN 行，仍给定长度一致的输出（用 NaN/0）
        return {
            "rtt_min": np.nan, "rtt_k2": np.nan, "rtt_median": np.nan,
            "rtt_p10": np.nan, "rtt_p25": np.nan, "rtt_p75": np.nan,
            "rtt_iqr": np.nan, "rtt_mad": np.nan,
            "rtt_mean_trim": np.nan, "rtt_std": np.nan,
            "rtt_valid_ratio": 0.0
        }

    def q(p):  # linear interpolated quantile
        idx = p * (m - 1)
        lo, hi = int(np.floor(idx)), int(np.ceil(idx))
        w = idx - lo
        return (1 - w) * x[lo] + w * x[hi]

    feats = {
        "rtt_min": x[0],
        "rtt_k2": x[1] if m >= 2 else x[0],
        "rtt_median": q(0.5),
        "rtt_p10": q(0.1),
        "rtt_p25": q(0.25),
        "rtt_p75": q(0.75),
        "rtt_iqr": q(0.75) - q(0.25),
        "rtt_mad": np.median(np.abs(x - q(0.5))),
        "rtt_mean_trim": x[int(0.1 * m): int(0.9 * m)].mean() if m >= 10 else x.mean(),
        "rtt_std": x.std(ddof=0),  # population std
        "rtt_valid_ratio": float(m) / 15.0,  # 期望 burst=15 时
    }
    return feats

def open_serial(port, baud):
    if serial is None:
        sys.stderr.write("pyserial not installed. Run: pip install pyserial\n")
        sys.exit(1)
    return serial.Serial(port=port, baudrate=baud, timeout=0.1)

def write_header_if_needed(csv_path, fieldnames):
    try:
        with open(csv_path, 'r', newline='', encoding='utf-8') as f:
            if f.read(1):
                return
    except FileNotFoundError:
        pass
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        csv.DictWriter(f, fieldnames=fieldnames).writeheader()

def main():
    ap = argparse.ArgumentParser(description="FTM+CSI -> CSV (robust RTT feats, RSSI mean/min)")
    ap.add_argument("--port", default="COM4", help="Serial port, e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=921600, help="Baud rate")
    ap.add_argument("--out",  default="ftm_csi_feats.csv", help="Output CSV path")
    ap.add_argument("--echo", action="store_true", help="Echo raw lines")
    ap.add_argument("--stdin", action="store_true", help="Read from stdin (testing)")
    ap.add_argument("--limit-frames", type=int, default=0,
                    help="If >0, only use the first N frames per FTM for features (e.g., 16)")
    args = ap.parse_args()

    fieldnames = [
        "dist_est_m",
        "rssi_mean_dbm", "rssi_min_dbm",
        "rtt_min", "rtt_k2", "rtt_median",
        "rtt_p10", "rtt_p25", "rtt_p75", "rtt_iqr",
        "rtt_mad", "rtt_mean_trim", "rtt_std", "rtt_valid_ratio",
        "csi_data_json",
    ]
    write_header_if_needed(args.out, fieldnames)

    # 当前等待 CSI 配对的 FTM 块
    current = None  # {anchor, dist_est_m, frames_rtt_ps[], frames_rssi[]}

    # IO
    if args.stdin:
        f_in = sys.stdin
        ser = None
    else:
        ser = open_serial(args.port, args.baud)
        f_in = ser

    out_f = open(args.out, 'a', newline='', encoding='utf-8')
    writer = csv.DictWriter(out_f, fieldnames=fieldnames)

    try:
        while True:
            # 读一行
            if ser:
                bs = ser.readline()
                if not bs:
                    time.sleep(0.01)
                    continue
                line = bs.decode('utf-8', errors='ignore').strip()
            else:
                line = f_in.readline()
                if not line:
                    break
                line = line.strip()

            if args.echo:
                print(line)

            # 解析
            if RE_ANCHOR.match(line):
                # 忽略 anchor；不写 CSV
                continue

            m = RE_FTM_REPORT.match(line)
            if m:
                # 新 FTM；如果上一条还没等到 CSI，按你的规则丢弃
                current = {
                    "anchor": m.group(1),  # 只用于匹配，不写 CSV
                    "dist_est_m": float(m.group(4)),
                    "frames_rtt_ps": [],
                    "frames_rssi": [],
                }
                continue

            m = RE_FTM_FRAME.match(line)
            if m and current and m.group(1) == current["anchor"]:
                # 收集逐帧 RTT/RSSI
                rtt_ps = int(m.group(5))
                rssi   = int(m.group(4))
                current["frames_rtt_ps"].append(float(rtt_ps))
                current["frames_rssi"].append(float(rssi))
                continue

            m = RE_CSI.match(line)
            if m and current and m.group(1) == current["anchor"]:
                # 配对成功：生成一行
                if args.limit_frames:
                    current["frames_rtt_ps"] = current["frames_rtt_ps"][:args.limit_frames]
                    current["frames_rssi"]   = current["frames_rssi"][:args.limit_frames]


                # RSSI 聚合（只要 mean/min）
                rssi_vals = current["frames_rssi"]
                rssi_mean = float(np.mean(rssi_vals)) if rssi_vals else np.nan
                rssi_min  = float(np.min(rssi_vals))  if rssi_vals else np.nan

                # RTT 鲁棒特征
                feats = robust_feats_from_rtts(current["frames_rtt_ps"])

                # CSI
                csi_vals = parse_csi_payload(m.group(2))
                row = {
                    "dist_est_m": current["dist_est_m"],
                    "rssi_mean_dbm": rssi_mean,
                    "rssi_min_dbm": rssi_min,
                    **feats,
                    "csi_data_json": json.dumps(csi_vals, ensure_ascii=False),
                }
                writer.writerow(row)
                out_f.flush()

                # 清空，等待下一条 FTM
                current = None
                continue

            # 其他行忽略
    except KeyboardInterrupt:
        pass
    finally:
        out_f.close()
        if not args.stdin and ser:
            ser.close()

if __name__ == "__main__":
    """
    示例：
      # 串口实时采集：
      python ftm_csi_to_csv.py --port COM5 --baud 921600 --out train.csv

      # 用历史日志（stdin）测试：
      type sample.log | python ftm_csi_to_csv.py --stdin --out train.csv
      # Linux/macOS:
      cat sample.log | python3 ftm_csi_to_csv.py --stdin --out train.csv

      # 只用前 16 帧做特征
      python ftm_csi_to_csv.py --port COM5 --limit-frames 16
    """
    main()
