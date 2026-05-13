#!/usr/bin/env python3
"""
Ultrasonic Sensor Comparison Visualization Tool

Modes:
  --live   Real-time serial port monitoring and plotting
  --file   Offline analysis from saved log file
  --demo   Demo mode with simulated data (no hardware needed)

Usage:
  python3 visualize.py --live --port /dev/ttyUSB0
  python3 visualize.py --file sensor_log.txt
  python3 visualize.py --demo
"""

import argparse
import json
import sys
import time
import os

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# ─── Phase Names ───
PHASE_NAMES = {
    0: "Static_20cm",
    1: "Static_100cm",
    2: "Static_300cm",
    3: "Angle_0deg",
    4: "Angle_30deg",
    5: "Angle_45deg",
    6: "Stability_100x",
}

PHASE_SHORT = {
    0: "20cm",
    1: "100cm",
    2: "300cm",
    3: "0°",
    4: "30°",
    5: "45°",
    6: "Stable",
}


# ─── Data Parsing ───
def parse_log_line(line):
    """Parse a single JSON line from the ESP32 output."""
    line = line.strip()
    if not line.startswith("{"):
        return None
    try:
        return json.loads(line)
    except json.JSONDecodeError:
        return None


def parse_log_file(filepath):
    """Parse a complete log file into samples and stats."""
    samples = []
    stats = []

    with open(filepath, "r") as f:
        for line in f:
            data = parse_log_line(line)
            if data is None:
                continue

            if "event" in data:
                if data["event"] == "phase_stats":
                    stats.append(data)
                elif data["event"] == "test_complete":
                    break
            elif "t" in data:
                samples.append(data)

    return samples, stats


def samples_to_dataframe(samples):
    """Convert parsed samples to a pandas DataFrame."""
    if not samples:
        return pd.DataFrame()

    records = []
    for s in samples:
        a = s.get("a", {})
        u = s.get("u", {})
        records.append({
            "timestamp": s["t"],
            "phase": s["ph"],
            "index": s["i"],
            "a02_dist_mm": a.get("d"),
            "a02_valid": a.get("ok", False),
            "urm09_dist_cm": u.get("d"),
            "urm09_valid": u.get("ok", False),
            "urm09_temp": u.get("temp"),
        })

    df = pd.DataFrame(records)
    # Convert -1 (error) to NaN
    df.loc[df["a02_dist_mm"] == -1, "a02_dist_mm"] = np.nan
    df.loc[df["urm09_dist_cm"] == -1, "urm09_dist_cm"] = np.nan
    return df


def generate_demo_data():
    """Generate simulated data for demo mode."""
    np.random.seed(42)
    samples = []
    t = 1000
    target_distances = [20, 100, 300, 20, 20, 20, 50]

    for phase in range(7):
        target = target_distances[phase]
        n = 100 if phase == 6 else 30
        for i in range(n):
            # A02YYUW: mm resolution, slight noise
            a02_mm = int(target * 10 + np.random.normal(0, 3))
            # URM09: cm resolution, more noise at distance
            urm09_cm = int(target + np.random.normal(0, 1 + target * 0.005))

            # Angle degradation simulation
            if phase == 4:  # 30 deg
                if np.random.random() < 0.1:
                    a02_mm = 0
            elif phase == 5:  # 45 deg
                if np.random.random() < 0.3:
                    a02_mm = 0
                if np.random.random() < 0.2:
                    urm09_cm = 0

            samples.append({
                "t": t,
                "ph": phase,
                "i": i,
                "a": {"d": a02_mm, "ok": a02_mm > 0},
                "u": {"d": max(0, urm09_cm), "ok": urm09_cm > 0, "temp": 24.5 + np.random.normal(0, 0.2)},
            })
            t += 100

    return samples


# ─── Visualization ───
def plot_report(df, stats_data=None, output_path=None):
    """Generate the 6-panel comparison report."""
    fig = plt.figure(figsize=(16, 12))
    fig.suptitle("Ultrasonic Sensor Comparison Report\nA02YYUW (UART) vs URM09 (I2C)", fontsize=14, fontweight="bold")

    gs = GridSpec(3, 3, figure=fig, hspace=0.35, wspace=0.3)

    # ─── [1] Mean Distance Bar Chart ───
    ax1 = fig.add_subplot(gs[0, 0:2])
    phases_present = sorted(df["phase"].unique())
    x = np.arange(len(phases_present))
    width = 0.35

    a02_means = []
    u_means = []
    a02_stds = []
    u_stds = []

    for ph in phases_present:
        ph_data = df[df["phase"] == ph]
        a02_vals = ph_data["a02_dist_mm"].dropna() / 10.0  # mm -> cm
        u_vals = ph_data["urm09_dist_cm"].dropna()
        a02_means.append(a02_vals.mean() if len(a02_vals) > 0 else 0)
        u_means.append(u_vals.mean() if len(u_vals) > 0 else 0)
        a02_stds.append(a02_vals.std() if len(a02_vals) > 1 else 0)
        u_stds.append(u_vals.std() if len(u_vals) > 1 else 0)

    bars1 = ax1.bar(x - width / 2, a02_means, width, label="A02YYUW",
                    yerr=a02_stds, capsize=3, color="#2196F3", alpha=0.8)
    bars2 = ax1.bar(x + width / 2, u_means, width, label="URM09",
                    yerr=u_stds, capsize=3, color="#FF9800", alpha=0.8)

    ax1.set_xlabel("Test Phase")
    ax1.set_ylabel("Distance (cm)")
    ax1.set_title("[1] Mean Distance by Phase (with ±σ error bars)")
    ax1.set_xticks(x)
    ax1.set_xticklabels([PHASE_SHORT.get(ph, f"P{ph}") for ph in phases_present], rotation=45)
    ax1.legend()
    ax1.grid(axis="y", alpha=0.3)

    # ─── [2] Standard Deviation Bar Chart ───
    ax2 = fig.add_subplot(gs[0, 2])
    y_pos = np.arange(len(phases_present))

    ax2.barh(y_pos - 0.15, a02_stds, 0.3, label="A02YYUW", color="#2196F3", alpha=0.8)
    ax2.barh(y_pos + 0.15, u_stds, 0.3, label="URM09", color="#FF9800", alpha=0.8)
    ax2.set_yticks(y_pos)
    ax2.set_yticklabels([PHASE_SHORT.get(ph, f"P{ph}") for ph in phases_present])
    ax2.set_xlabel("Std Dev (cm)")
    ax2.set_title("[2] Measurement Std Dev")
    ax2.legend(fontsize=8)
    ax2.grid(axis="x", alpha=0.3)

    # ─── [3] Time Series Plot ───
    ax3 = fig.add_subplot(gs[1, :])

    # Convert timestamps to seconds from start
    t_start = df["timestamp"].min()
    df["time_s"] = (df["timestamp"] - t_start) / 1000.0

    a02_cm = df["a02_dist_mm"] / 10.0

    ax3.plot(df["time_s"], a02_cm, "o", markersize=2, color="#2196F3",
             alpha=0.6, label="A02YYUW (cm)")
    ax3.plot(df["time_s"], df["urm09_dist_cm"], "s", markersize=2, color="#FF9800",
             alpha=0.6, label="URM09 (cm)")

    # Add phase background colors
    colors = plt.cm.Set3(np.linspace(0, 1, len(phases_present)))
    for idx, ph in enumerate(phases_present):
        ph_data = df[df["phase"] == ph]
        if len(ph_data) > 0:
            t_min = ph_data["time_s"].min()
            t_max = ph_data["time_s"].max()
            ax3.axvspan(t_min, t_max, alpha=0.15, color=colors[idx],
                       label=f"P{ph}: {PHASE_SHORT.get(ph, '')}")

    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Distance (cm)")
    ax3.set_title("[3] Full Timeline (colored by phase)")
    ax3.legend(fontsize=7, ncol=4, loc="upper right")
    ax3.grid(alpha=0.3)

    # ─── [4] Failure Rate ───
    ax4 = fig.add_subplot(gs[2, 0])

    a02_fail = []
    u_fail = []
    for ph in phases_present:
        ph_data = df[df["phase"] == ph]
        total = len(ph_data)
        a02_fail.append((1 - ph_data["a02_valid"].sum() / total) * 100 if total > 0 else 0)
        u_fail.append((1 - ph_data["urm09_valid"].sum() / total) * 100 if total > 0 else 0)

    ax4.bar(x - width / 2, a02_fail, width, label="A02YYUW", color="#2196F3", alpha=0.8)
    ax4.bar(x + width / 2, u_fail, width, label="URM09", color="#FF9800", alpha=0.8)
    ax4.set_xticks(x)
    ax4.set_xticklabels([PHASE_SHORT.get(ph, f"P{ph}") for ph in phases_present], rotation=45)
    ax4.set_ylabel("Failure Rate (%)")
    ax4.set_title("[4] Read Failure Rate")
    ax4.legend(fontsize=8)
    ax4.grid(axis="y", alpha=0.3)

    # ─── [5] Stability Distribution (Phase 6 only) ───
    ax5 = fig.add_subplot(gs[2, 1])

    if 6 in phases_present:
        stable_data = df[df["phase"] == 6]
        a02_vals = stable_data["a02_dist_mm"].dropna() / 10.0
        u_vals = stable_data["urm09_dist_cm"].dropna()

        if len(a02_vals) > 0:
            ax5.hist(a02_vals, bins=20, alpha=0.6, color="#2196F3",
                    label=f"A02 σ={a02_vals.std():.2f}cm", density=True)
        if len(u_vals) > 0:
            ax5.hist(u_vals, bins=10, alpha=0.6, color="#FF9800",
                    label=f"URM09 σ={u_vals.std():.2f}cm", density=True)

    ax5.set_xlabel("Distance (cm)")
    ax5.set_ylabel("Density")
    ax5.set_title("[5] Stability Distribution (50cm, 100 samples)")
    ax5.legend(fontsize=8)
    ax5.grid(alpha=0.3)

    # ─── [6] Scatter Correlation ───
    ax6 = fig.add_subplot(gs[2, 2])

    # Match A02 and URM09 readings within same phase
    matched_a = []
    matched_u = []
    for ph in phases_present:
        ph_data = df[df["phase"] == ph].dropna(subset=["a02_dist_mm", "urm09_dist_cm"])
        matched_a.extend(ph_data["a02_dist_mm"] / 10.0)
        matched_u.extend(ph_data["urm09_dist_cm"])

    if len(matched_a) > 1:
        ax6.scatter(matched_u, matched_a, s=10, alpha=0.5, color="#4CAF50")
        # Perfect correlation line
        lim = max(max(matched_a), max(matched_u)) * 1.1
        ax6.plot([0, lim], [0, lim], "k--", alpha=0.3, label="y=x")

        # Pearson correlation
        corr = np.corrcoef(matched_u, matched_a)[0, 1]
        ax6.set_title(f"[6] Sensor Correlation (r={corr:.4f})")
    else:
        ax6.set_title("[6] Sensor Correlation (no data)")

    ax6.set_xlabel("URM09 (cm)")
    ax6.set_ylabel("A02YYUW (cm)")
    ax6.legend(fontsize=8)
    ax6.grid(alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.95])

    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches="tight")
        print(f"Report saved to: {output_path}")
    else:
        plt.savefig("ultrasonic_report.png", dpi=150, bbox_inches="tight")
        print("Report saved to: ultrasonic_report.png")

    plt.show()


# ─── Live Mode ───
def live_mode(port, baudrate=115200):
    """Real-time serial monitoring and plotting."""
    try:
        import serial
    except ImportError:
        print("Error: pyserial not installed. Run: pip install pyserial")
        sys.exit(1)

    print(f"Connecting to {port} at {baudrate} baud...")
    ser = serial.Serial(port, baudrate, timeout=1)
    print("Connected. Waiting for data...")

    samples = []

    try:
        while True:
            line = ser.readline().decode("utf-8", errors="ignore")
            if not line:
                continue

            data = parse_log_line(line)
            if data is None:
                continue

            if "event" in data:
                if data["event"] == "phase_start":
                    print(f"\n--- Phase {data['phase']}: {data['name']} ({data['samples']} samples) ---")
                elif data["event"] == "phase_stats":
                    print(f"  Stats: A02 mean={data['a']['mean']:.1f}mm σ={data['a']['std']:.1f} | "
                          f"URM09 mean={data['u']['mean']:.1f}cm σ={data['u']['std']:.1f}")
                elif data["event"] == "test_complete":
                    print("\n=== Test complete ===")
                    break
            elif "t" in data:
                samples.append(data)
                a_ok = "OK" if data["a"].get("ok") else "ERR"
                u_ok = "OK" if data["u"].get("ok") else "ERR"
                print(f"  [{data['ph']}:{data['i']:3d}] A02={data['a']['d']:5d}mm ({a_ok}) | "
                      f"URM09={data['u']['d']:3d}cm ({u_ok}) T={data['u'].get('temp', 0):.1f}°C")

    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        ser.close()

    if samples:
        df = samples_to_dataframe(samples)
        plot_report(df)


# ─── File Mode ───
def file_mode(filepath):
    """Offline analysis from saved log file."""
    print(f"Loading log file: {filepath}")
    samples, stats = parse_log_file(filepath)
    print(f"Loaded {len(samples)} samples, {len(stats)} phase stats")

    if not samples:
        print("No sample data found in file.")
        return

    df = samples_to_dataframe(samples)
    plot_report(df, stats)


# ─── Demo Mode ───
def demo_mode():
    """Demo mode with simulated data."""
    print("Generating simulated data for demo...")
    samples = generate_demo_data()
    df = samples_to_dataframe(samples)
    plot_report(df)


# ─── Main ───
def main():
    parser = argparse.ArgumentParser(description="Ultrasonic Sensor Comparison Visualization")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--live", action="store_true", help="Real-time serial monitoring")
    group.add_argument("--file", type=str, help="Offline log file analysis")
    group.add_argument("--demo", action="store_true", help="Demo mode with simulated data")

    parser.add_argument("--port", type=str, default="/dev/ttyUSB0",
                        help="Serial port (default: /dev/ttyUSB0)")
    parser.add_argument("--baudrate", type=int, default=115200,
                        help="Baudrate (default: 115200)")
    parser.add_argument("--output", type=str, default=None,
                        help="Output image path (default: ultrasonic_report.png)")

    args = parser.parse_args()

    if args.live:
        live_mode(args.port, args.baudrate)
    elif args.file:
        file_mode(args.file)
    elif args.demo:
        demo_mode()


if __name__ == "__main__":
    main()
