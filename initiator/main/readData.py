#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import re
import csv
import json
import sys
import time
import argparse
import os

try:
    import serial
except ImportError:
    serial = None

# ---------- Regex ----------
RE_ANCHOR      = re.compile(r'^anchor,([^,]+),([0-9a-f:]+),ch=(\d+)\s*$', re.I)
RE_FTM_REPORT  = re.compile(r'^ftm_report,([^,]+),(\d+),(\d+),([0-9.]+)\s*$', re.I)

RE_FTM_FEATS   = re.compile(
    r'^ftm_feats,([^,]+),'
    r'rtt_ps_min=([0-9.]+),k2=([0-9.]+),median=([0-9.]+),p10=([0-9.]+),p25=([0-9.]+),p75=([0-9.]+),'
    r'iqr=([0-9.]+),mad=([0-9.]+),mean_trim=([0-9.]+),std=([0-9.]+),valid_ratio=([0-9.]+),'
    r'rssi_mean=([-0-9.]+),rssi_min=([-0-9.]+)(?:,rssi_std=([-0-9.]+))?(?:,\[(.*)\])?\s*$',
    re.I
)

RE_CSI_FEAT    = re.compile(
    r'^csi_feat,([^,]+),rssi=(-?\d+),mag=\[(.*)\],phi=\[(.*)\]\s*$',
    re.I
)

RE_META_FROM_OUT = re.compile(r'.*_(?P<dist>[0-9]+(?:\.[0-9]+)?)m_(?P<cond>LOS|NLOS)\.csv$', re.I)

# ---------- Helpers ----------
def parse_float_list(s: str):
    s = s.strip()
    if not s:
        return []
    return [float(p) for p in s.split(',') if p.strip()]

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

def parse_meta_from_out_path(out_path):
    base = os.path.basename(out_path)
    m = RE_META_FROM_OUT.match(base)
    if not m:
        return None, None
    return float(m.group('dist')), m.group('cond').upper()

# ---------- Main ----------
def main():
    ap = argparse.ArgumentParser(description="Pair FTM feats + CSI feats into CSV rows")
    ap.add_argument("--port", default="COM4", help="Serial port")
    ap.add_argument("--baud", type=int, default=921600, help="Baud rate")
    ap.add_argument("--out",  default="ftm_csi_feats_0.0m_LOS.csv", help="Output CSV path")
    ap.add_argument("--echo", action="store_true")
    ap.add_argument("--stdin", action="store_true")
    ap.add_argument("--max", type=int, default=None, help="Maximum number of rows to write before exit")
    args = ap.parse_args()

    gt_dist_m, cond = parse_meta_from_out_path(args.out)
    if gt_dist_m is None:
        print(f"[WARN] Output filename does not match '<dist>m_<LOS|NLOS>': {args.out}", file=sys.stderr)

    fieldnames = [
        "dist_est_m",
        "gt_dist_m", "cond",
        "rtt_min","rtt_k2","rtt_median",
        "rtt_p10","rtt_p25","rtt_p75","rtt_iqr",
        "rtt_mad","rtt_mean_trim","rtt_std","rtt_valid_ratio",
        "ftm_rssi_mean_dbm","ftm_rssi_min_dbm","ftm_rssi_std_dbm",
        "csi_rssi_dbm",
        "csi_mag_json","csi_phi_json",
        "ftm_rtt_list_json",
    ]
    write_header_if_needed(args.out, fieldnames)

    rows = {}
    written_count = 0
    dropped_count = 0
    parsed_line_count = 0
    last_progress_print = time.monotonic()

    if args.stdin:
        f_in = sys.stdin
        ser = None
    else:
        ser = open_serial(args.port, args.baud)
        f_in = ser

    out_f = open(args.out, 'a', newline='', encoding='utf-8')
    writer = csv.DictWriter(out_f, fieldnames=fieldnames)

    def new_state():
        return {"dist": None, "ftm": None, "csi": None, "t": time.monotonic()}

    def ensure_row(key):
        if key not in rows:
            rows[key] = new_state()
        return rows[key]

    def try_flush(key):
        nonlocal written_count
        st = rows.get(key)
        if not st or st["dist"] is None or st["ftm"] is None or st["csi"] is None:
            return
        writer.writerow({
            "dist_est_m": st["dist"],
            "gt_dist_m": gt_dist_m,
            "cond": cond,
            "rtt_min": st["ftm"]["rtt_min"],
            "rtt_k2": st["ftm"]["rtt_k2"],
            "rtt_median": st["ftm"]["rtt_median"],
            "rtt_p10": st["ftm"]["rtt_p10"],
            "rtt_p25": st["ftm"]["rtt_p25"],
            "rtt_p75": st["ftm"]["rtt_p75"],
            "rtt_iqr": st["ftm"]["rtt_iqr"],
            "rtt_mad": st["ftm"]["rtt_mad"],
            "rtt_mean_trim": st["ftm"]["rtt_mean_trim"],
            "rtt_std": st["ftm"]["rtt_std"],
            "rtt_valid_ratio": st["ftm"]["rtt_valid_ratio"],
            "ftm_rssi_mean_dbm": st["ftm"]["rssi_mean"],
            "ftm_rssi_min_dbm": st["ftm"]["rssi_min"],
            "ftm_rssi_std_dbm": st["ftm"].get("rssi_std"),
            "csi_rssi_dbm": st["csi"]["rssi"],
            "csi_mag_json": json.dumps(st["csi"]["mag"], ensure_ascii=False),
            "csi_phi_json": json.dumps(st["csi"]["phi"], ensure_ascii=False),
            "ftm_rtt_list_json": json.dumps(st["ftm"].get("rtt_list", []), ensure_ascii=False),
        })
        out_f.flush()
        written_count += 1
        print(written_count)
        rows[key] = new_state()
        if args.max is not None and written_count >= args.max:
            print(f"[INFO] Reached max rows ({args.max}), exiting...", file=sys.stderr)
            raise KeyboardInterrupt

    TIMEOUT_S = 1.0
    def drop_stale():
        nonlocal dropped_count
        now = time.monotonic()
        for k, st in list(rows.items()):
            has_any = (st["dist"] is not None) or (st["ftm"] is not None) or (st["csi"] is not None)
            if has_any and (now - st["t"]) > TIMEOUT_S:
                dropped_count += 1
                rows[k] = new_state()

    try:
        while True:
            if ser:
                bs = ser.readline()
                if not bs:
                    drop_stale()
                    time.sleep(0.01)
                    continue
                line = bs.decode('utf-8', errors='ignore').strip()
            else:
                line = f_in.readline()
                if not line:
                    break
                line = line.strip()

            parsed_line_count += 1
            if args.echo:
                print(line)

            drop_stale()

            if RE_ANCHOR.match(line):
                continue

            m = RE_FTM_REPORT.match(line)
            if m:
                key = m.group(1)
                st = ensure_row(key)
                st["dist"] = float(m.group(4))
                st["t"] = time.monotonic()
                try_flush(key)
                continue

            m = RE_FTM_FEATS.match(line)
            if m:
                key = m.group(1)
                st = ensure_row(key)
                vals = {
                    "rtt_min": float(m.group(2)), "rtt_k2": float(m.group(3)),
                    "rtt_median": float(m.group(4)), "rtt_p10": float(m.group(5)),
                    "rtt_p25": float(m.group(6)), "rtt_p75": float(m.group(7)),
                    "rtt_iqr": float(m.group(8)), "rtt_mad": float(m.group(9)),
                    "rtt_mean_trim": float(m.group(10)), "rtt_std": float(m.group(11)),
                    "rtt_valid_ratio": float(m.group(12)),
                    "rssi_mean": float(m.group(13)), "rssi_min": float(m.group(14)),
                }
                rssi_std = m.group(15)
                if rssi_std:
                    vals["rssi_std"] = float(rssi_std)
                rtt_list_raw = m.group(16)
                if rtt_list_raw:
                    vals["rtt_list"] = [int(x) for x in rtt_list_raw.split(',') if x.strip()]
                st["ftm"] = vals
                st["t"] = time.monotonic()
                try_flush(key)
                continue

            m = RE_CSI_FEAT.match(line)
            if m:
                key = m.group(1)
                st = ensure_row(key)
                st["csi"] = {
                    "rssi": int(m.group(2)),
                    "mag": parse_float_list(m.group(3)),
                    "phi": parse_float_list(m.group(4)),
                }
                st["t"] = time.monotonic()
                try_flush(key)
                continue
    except KeyboardInterrupt:
        pass
    finally:
        print(f"[SUMMARY] parsed_lines={parsed_line_count}, written_rows={written_count}, dropped_incomplete={dropped_count}", file=sys.stderr)
        out_f.close()
        if not args.stdin and ser:
            ser.close()

if __name__ == "__main__":
    main()
