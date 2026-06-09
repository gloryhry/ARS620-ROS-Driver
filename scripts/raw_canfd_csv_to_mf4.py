#!/usr/bin/env python3
"""Convert ARS620 raw CAN-FD CSV logs to an MDF4 container."""

import argparse
import csv
import sys

def load_rows(path):
    columns = {
        "timestamp": [],
        "can_id": [],
        "is_extended": [],
        "is_rtr": [],
        "is_error": [],
        "fd_flags": [],
        "length": [],
    }
    for index in range(64):
        columns[f"data_{index:02d}"] = []

    with open(path, newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            columns["timestamp"].append(int(row["timestamp_us"]) / 1_000_000.0)
            columns["can_id"].append(int(row["can_id"]))
            columns["is_extended"].append(int(row["is_extended"]))
            columns["is_rtr"].append(int(row["is_rtr"]))
            columns["is_error"].append(int(row["is_error"]))
            columns["fd_flags"].append(int(row["fd_flags"]))
            columns["length"].append(int(row["length"]))
            for index in range(64):
                columns[f"data_{index:02d}"].append(int(row[f"data_{index:02d}"]))
    return columns


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw_csv", help="Input ars620_canfd_*.raw.csv file")
    parser.add_argument("mf4", help="Output ars620_canfd_*.mf4 file")
    args = parser.parse_args()

    try:
        from asammdf import MDF, Signal
        import numpy as np
    except ImportError as exc:
        print(
            "asammdf is required to create MF4 logs. Install it with: "
            "python3 -m pip install asammdf",
            file=sys.stderr,
        )
        raise SystemExit(2) from exc

    columns = load_rows(args.raw_csv)
    timestamps = np.asarray(columns["timestamp"], dtype=np.float64)
    mdf = MDF(version="4.10")

    signals = []
    for name, values in columns.items():
        if name == "timestamp":
            samples = np.asarray(values, dtype=np.float64)
            unit = "s"
        elif name == "can_id":
            samples = np.asarray(values, dtype=np.uint32)
            unit = ""
        else:
            samples = np.asarray(values, dtype=np.uint8)
            unit = ""
        signals.append(Signal(samples=samples, timestamps=timestamps, name=name, unit=unit))

    mdf.append(signals, common_timebase=True)
    mdf.save(args.mf4, overwrite=True)


if __name__ == "__main__":
    main()
