"""
analyze_calib.py — Motor calibration log analyzer
===================================================
Usage:
    python analyze_calib.py <serial_log.txt> [--out-dir <dir>]

Parses [cal] lines from an XDS-UART serial log, aggregates per-step
statistics, fits PWM→RPM and error→voltage models, and saves 4 PNG charts.

Expected log lines (from app_motor_demo firmware):
    [cal] start dir=+1 steps=19 pm_start=100 pm_end=1000 step=50 ...
    [cal] step  dir=+1 idx=0/19 pm=100
    [cal] dir=+1 idx=0/19 pm=100 t=1234 vbat=11820 rpmL=98 rpmR=104 ctL=123 ctR=456
    [cal] done  dir=+1 next=reverse
"""

import argparse
import os
import re
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ── Regular expressions ────────────────────────────────────────────────────────

DROP_INITIAL_FRAMES = 3
MOTOR_SPECS = (
    ("Left", "rpmL_mean", "rpmL", "steelblue"),
    ("Right", "rpmR_mean", "rpmR", "tomato"),
)

# Data sample line
RE_SAMPLE = re.compile(
    r"\[cal\] "
    r"dir=([+-]\d+)\s+"
    r"idx=(\d+)/(\d+)\s+"
    r"pm=([+-]?\d+)\s+"
    r"t=(\d+)\s+"
    r"vbat=(\d+)\s+"
    r"rpmL=([+-]?\d+(?:\.\d+)?)\s+"
    r"rpmR=([+-]?\d+(?:\.\d+)?)"
)

# Header line — carry settle_ms so we can filter post-settle only
RE_HEADER = re.compile(
    r"\[cal\] start\s+dir=([+-]\d+)\s+steps=(\d+)\s+"
    r"pm_start=(\d+)\s+pm_end=(\d+)\s+step=(\d+)\s+"
    r"dwell_ms=(\d+)\s+settle_ms=(\d+)"
)

# Step boundary line — to know when a new step began
RE_STEP = re.compile(
    r"\[cal\] step\s+dir=([+-]\d+)\s+idx=(\d+)/(\d+)\s+pm=([+-]?\d+)"
)


def parse_log(path: str, drop_initial_frames: int = DROP_INITIAL_FRAMES):
    """Return list of sample dicts after settle filtering and frame trimming."""
    samples = []
    settle_ms = 500        # default; updated from header
    step_start_t = {}      # key: (dir, idx) → ms timestamp of that step start
    stable_counts = {}     # key: (dir, idx) → number of kept post-settle frames seen

    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.rstrip()

            m = RE_HEADER.search(line)
            if m:
                settle_ms = int(m.group(7))
                continue

            m = RE_STEP.search(line)
            if m:
                key = (int(m.group(1)), int(m.group(2)))
                # We don't have a timestamp on step lines; mark as None —
                # use the first sample's t for that step instead.
                step_start_t[key] = None
                stable_counts[key] = 0
                continue

            m = RE_SAMPLE.search(line)
            if m:
                d = {
                    "dir":   int(m.group(1)),
                    "idx":   int(m.group(2)),
                    "total": int(m.group(3)),
                    "pm":    int(m.group(4)),
                    "t":     int(m.group(5)),
                    "vbat":  int(m.group(6)),
                    "rpmL":  float(m.group(7)),
                    "rpmR":  float(m.group(8)),
                }
                key = (d["dir"], d["idx"])
                # Register first sample time as step start if not yet set
                if key not in step_start_t or step_start_t[key] is None:
                    step_start_t[key] = d["t"]
                # Only keep samples after settle window
                elapsed = d["t"] - step_start_t[key]
                if elapsed >= settle_ms:
                    stable_counts[key] = stable_counts.get(key, 0) + 1
                    if stable_counts[key] <= drop_initial_frames:
                        continue
                    samples.append(d)

    return samples


def aggregate(samples):
    """Group samples by (dir, idx, pm) and compute mean/std."""
    from collections import defaultdict

    groups = defaultdict(list)
    for s in samples:
        groups[(s["dir"], s["idx"], s["pm"])].append(s)

    rows = []
    for (d, idx, pm), group in sorted(groups.items()):
        rpmL  = np.array([g["rpmL"]  for g in group])
        rpmR  = np.array([g["rpmR"]  for g in group])
        vbat  = np.array([g["vbat"]  for g in group])
        rows.append({
            "dir":       d,
            "idx":       idx,
            "pm":        pm,
            "n":         len(group),
            "rpmL_mean": float(np.mean(rpmL)),
            "rpmL_std":  float(np.std(rpmL)),
            "rpmR_mean": float(np.mean(rpmR)),
            "rpmR_std":  float(np.std(rpmR)),
            "vbat_mean": float(np.mean(vbat)),
            "vbat_std":  float(np.std(vbat)),
            "err_mean":  float(np.mean(rpmR - rpmL)),
            "err_std":   float(np.std(rpmR - rpmL)),
        })
    return rows


def format_signed(value, digits=4):
    return f"{value:+.{digits}f}"


def safe_r2(y_true, y_pred):
    ss_res = np.sum((y_true - y_pred) ** 2)
    ss_tot = np.sum((y_true - np.mean(y_true)) ** 2)
    return 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")


def format_linear_expr(slope, intercept, x_name="pm"):
    return f"y = {format_signed(slope)}*{x_name} {format_signed(intercept)}"


def fit_linear(x, y, label=""):
    """Fit y = a*x + b, return (coeffs, R^2)."""
    coeffs = np.polyfit(x, y, 1)
    y_pred = np.polyval(coeffs, x)
    r2 = safe_r2(y, y_pred)
    if label:
        print(f"  [{label}] {format_linear_expr(coeffs[0], coeffs[1], 'abs(pm)')}  R^2={r2:.4f}")
    return coeffs, r2


def detect_deadzones(agg_rows, rpm_threshold=0.0):
    """Detect per-direction, per-motor deadzone based on first non-zero RPM."""
    deadzones = {}
    dirs = sorted({r["dir"] for r in agg_rows}, reverse=True)

    for dir_value in dirs:
        dir_rows = sorted(
            [r for r in agg_rows if r["dir"] == dir_value],
            key=lambda r: abs(r["pm"]),
        )
        for motor_name, rpm_key, _, _ in MOTOR_SPECS:
            last_zero_row = None
            first_active_row = None
            for row in dir_rows:
                if abs(row[rpm_key]) <= rpm_threshold:
                    last_zero_row = row
                    continue
                first_active_row = row
                break

            deadzones[(dir_value, motor_name)] = {
                "dir": dir_value,
                "motor": motor_name,
                "last_zero_pm": None if last_zero_row is None else last_zero_row["pm"],
                "deadzone_pm": None if first_active_row is None else first_active_row["pm"],
                "first_active_rpm": None if first_active_row is None else first_active_row[rpm_key],
            }

    return deadzones


def print_deadzone_summary(deadzones):
    print("\n── Deadzone analysis (0 RPM -> non-zero RPM) ───────────────────────────")
    for dir_value in sorted({info["dir"] for info in deadzones.values()}, reverse=True):
        dir_name = "forward" if dir_value > 0 else "reverse"
        for motor_name in ("Left", "Right"):
            info = deadzones[(dir_value, motor_name)]
            if info["deadzone_pm"] is None:
                print(f"  [{dir_name:7s} {motor_name:5s}] no deadzone transition detected")
                continue

            last_zero = info["last_zero_pm"]
            deadzone_pm = info["deadzone_pm"]
            rpm_val = info["first_active_rpm"]
            if last_zero is None:
                transition = f"starts active at pm={deadzone_pm:+d}"
            else:
                transition = f"pm {last_zero:+d} -> {deadzone_pm:+d}"
            print(
                f"  [{dir_name:7s} {motor_name:5s}] {transition}, "
                f"deadzone={abs(deadzone_pm)} permille, first RPM={rpm_val:.2f}"
            )


def build_motor_fits(agg_rows):
    """Fit per-direction, per-motor linear models on active region only."""
    fits = {}
    print("\n── PWM -> RPM linear fits (active region) ───────────────────────────────")
    for dir_value in sorted({r["dir"] for r in agg_rows}, reverse=True):
        dir_rows = sorted(
            [r for r in agg_rows if r["dir"] == dir_value],
            key=lambda r: abs(r["pm"]),
        )
        dir_name = "forward" if dir_value > 0 else "reverse"
        for motor_name, rpm_key, _, _ in MOTOR_SPECS:
            active_rows = [r for r in dir_rows if abs(r[rpm_key]) > 0.0]
            if len(active_rows) < 2:
                continue
            x = np.array([abs(r["pm"]) for r in active_rows], dtype=float)
            y = np.array([r[rpm_key] for r in active_rows], dtype=float)
            coeffs, r2 = fit_linear(x, y, f"{dir_name:7s} {motor_name:5s}")
            fits[(dir_value, motor_name)] = {
                "coeffs": coeffs,
                "r2": r2,
                "x_min": float(np.min(x)),
                "x_max": float(np.max(x)),
            }
    return fits


def deadzone_lines_for_plot(deadzones, dir_value=None):
    lines = []
    items = deadzones.values()
    if dir_value is not None:
        items = [info for info in items if info["dir"] == dir_value]

    for info in sorted(items, key=lambda item: (-item["dir"], item["motor"])):
        dir_name = "FWD" if info["dir"] > 0 else "REV"
        if info["deadzone_pm"] is None:
            lines.append(f"{dir_name} {info['motor']}: no deadzone")
        else:
            lines.append(f"{dir_name} {info['motor']} DZ = {abs(info['deadzone_pm'])} permille")
    return lines


def fit_poly(x, y, deg, label=""):
    """Fit polynomial, return (coeffs, R²)."""
    coeffs = np.polyfit(x, y, deg)
    y_pred = np.polyval(coeffs, x)
    r2 = safe_r2(y, y_pred)
    if label:
        deg_str = ["linear", "quadratic", "cubic"][min(deg - 1, 2)]
        terms = " + ".join(
            f"{c:+.4f}*pm^{deg - i}" if i < deg else f"{c:+.4f}"
            for i, c in enumerate(coeffs)
        )
        print(f"  [{label}] {deg_str}: {terms}  R²={r2:.4f}")
    return coeffs, r2


def fit_error_models(pm_arr, vbat_arr, err_arr, v_nominal=11100.0):
    """
    M1: err = a*pm + b
    M2: err = a*pm + c*(V_nominal - vbat) + b

    Returns a dict with fitted params, residuals and summary text.
    """
    valid = np.isfinite(pm_arr) & np.isfinite(vbat_arr) & np.isfinite(err_arr)
    pm_arr = pm_arr[valid]
    vbat_arr = vbat_arr[valid]
    err_arr = err_arr[valid]
    n = len(pm_arr)

    # M1
    A1 = np.column_stack([pm_arr, np.ones(n)])
    m1, _, _, _ = np.linalg.lstsq(A1, err_arr, rcond=None)
    err1_pred = A1 @ m1
    m1_rmse = float(np.sqrt(np.mean((err_arr - err1_pred) ** 2)))

    # M2
    dv = v_nominal - vbat_arr
    A2 = np.column_stack([pm_arr, dv, np.ones(n)])
    m2, _, _, _ = np.linalg.lstsq(A2, err_arr, rcond=None)
    err2_pred = A2 @ m2
    m2_rmse = float(np.sqrt(np.mean((err_arr - err2_pred) ** 2)))

    print("\n── Error model fitting ─────────────────────────────────────────")
    print(f"  M1 (winding only):  err = {m1[0]:+.4f}*pm {m1[1]:+.4f}  RMSE={m1_rmse:.3f} rpm")
    print(f"  M2 (winding+vbat):  err = {m2[0]:+.4f}*pm {m2[1]:+.4f}*(V_nom-vbat) {m2[2]:+.4f}  RMSE={m2_rmse:.3f} rpm")

    improvement = (m1_rmse - m2_rmse) / m1_rmse * 100 if m1_rmse > 0 else 0.0
    if improvement > 10.0:
        conclusion = f"Voltage drop is a significant factor (M2 improves RMSE by {improvement:.1f}%)"
    else:
        conclusion = f"Winding asymmetry dominates; voltage effect is minor (M2 improvement {improvement:.1f}%)"
    print(f"  Conclusion: {conclusion}")

    return {
        "pm": pm_arr,
        "vbat": vbat_arr,
        "err": err_arr,
        "m1": {"params": m1, "rmse": m1_rmse, "pred": err1_pred, "residuals": err_arr - err1_pred},
        "m2": {"params": m2, "rmse": m2_rmse, "pred": err2_pred, "residuals": err_arr - err2_pred},
        "conclusion": conclusion,
        "v_nominal": v_nominal,
    }


def make_plots(samples, agg_rows, deadzones, motor_fits, error_models, out_dir):
    """Generate and save the 4 diagnostic charts."""
    os.makedirs(out_dir, exist_ok=True)

    all_pm    = np.array([s["pm"]   for s in samples], dtype=float)
    all_rpmL  = np.array([s["rpmL"] for s in samples], dtype=float)
    all_rpmR  = np.array([s["rpmR"] for s in samples], dtype=float)
    all_vbat  = np.array([s["vbat"] for s in samples], dtype=float)
    all_err   = all_rpmR - all_rpmL

    # ── Fig 1: PWM → RPM by direction, with fit and deadzone ─────────────────
    fig, axes = plt.subplots(1, 2, figsize=(13, 5), sharey=True)
    for ax, dir_value in zip(axes, [1, -1]):
        dir_mask = np.array([s["dir"] == dir_value for s in samples], dtype=bool)
        pm_mag = np.abs(all_pm[dir_mask])
        rpmL_dir = all_rpmL[dir_mask]
        rpmR_dir = all_rpmR[dir_mask]
        ax.scatter(pm_mag, rpmL_dir, s=8, alpha=0.25, color="steelblue", label="Left samples")
        ax.scatter(pm_mag, rpmR_dir, s=8, alpha=0.25, color="tomato", label="Right samples")

        dir_rows = sorted(
            [r for r in agg_rows if r["dir"] == dir_value],
            key=lambda r: abs(r["pm"]),
        )
        if dir_rows:
            pm_row = np.array([abs(r["pm"]) for r in dir_rows], dtype=float)
            ax.plot(pm_row, [r["rpmL_mean"] for r in dir_rows], color="steelblue", marker="o",
                    linewidth=1.0, markersize=3, alpha=0.8, label="Left mean")
            ax.plot(pm_row, [r["rpmR_mean"] for r in dir_rows], color="tomato", marker="o",
                    linewidth=1.0, markersize=3, alpha=0.8, label="Right mean")

        note_lines = []
        for motor_name, _, _, color in MOTOR_SPECS:
            fit = motor_fits.get((dir_value, motor_name))
            if fit is not None:
                x_fit = np.linspace(fit["x_min"], fit["x_max"], 120)
                y_fit = np.polyval(fit["coeffs"], x_fit)
                ax.plot(x_fit, y_fit, color=color, linestyle="--", linewidth=1.6,
                        label=f"{motor_name} fit")
                note_lines.append(
                    f"{motor_name}: {format_linear_expr(fit['coeffs'][0], fit['coeffs'][1], 'abs(PWM)')}"
                )
                note_lines.append(f"{motor_name}: R^2={fit['r2']:.4f}")

            deadzone = deadzones.get((dir_value, motor_name))
            if deadzone is not None and deadzone["deadzone_pm"] is not None:
                dz = abs(deadzone["deadzone_pm"])
                ax.axvline(dz, color=color, linewidth=1.1, linestyle=":", alpha=0.8)
                note_lines.append(f"{motor_name}: DZ={dz} permille")

        ax.set_xlabel("abs(PWM) (permille)")
        ax.set_title("Forward abs(PWM) -> RPM" if dir_value > 0 else "Reverse abs(PWM) -> RPM")
        ax.grid(True, alpha=0.3)
        if note_lines:
            ax.text(
                0.02, 0.98, "\n".join(note_lines),
                transform=ax.transAxes, va="top", ha="left", fontsize=7,
                bbox=dict(boxstyle="round", facecolor="white", alpha=0.85, edgecolor="0.7"),
            )

    axes[0].set_ylabel("RPM")
    handles, labels = axes[0].get_legend_handles_labels()
    if handles:
        fig.legend(handles, labels, loc="lower center", ncol=3, fontsize=8)
    fig.suptitle("PWM -> RPM with linear fits and deadzones")
    fig.tight_layout(rect=(0, 0.08, 1, 0.95))
    p = os.path.join(out_dir, "fig1_pwm_rpm.png")
    fig.savefig(p, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved {p}")

    # ── Fig 2: RPM error vs PWM, colour = vbat ───────────────────────────────
    fig, ax = plt.subplots(figsize=(8, 5))
    sc = ax.scatter(all_pm, all_err, c=all_vbat, cmap="plasma",
                    s=8, alpha=0.5)
    plt.colorbar(sc, ax=ax, label="vbat (mV)")
    ax.axhline(0, color="gray", linewidth=0.8, linestyle=":")
    m1 = error_models["m1"]["params"]
    pm_sorted = np.linspace(float(np.min(all_pm)), float(np.max(all_pm)), 200)
    ax.plot(pm_sorted, m1[0] * pm_sorted + m1[1], color="black", linestyle="--",
            linewidth=1.4, label="M1 fit")
    ax.set_xlabel("PWM (‰)")
    ax.set_ylabel("RPM error  (rpmR − rpmL)")
    ax.set_title("RPM error vs PWM  (colour = Vbat)")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    note_lines = [
        f"M1: err = {format_signed(m1[0])}*pm {format_signed(m1[1])}",
        f"M1 RMSE = {error_models['m1']['rmse']:.3f} rpm",
    ] + deadzone_lines_for_plot(deadzones)
    ax.text(
        0.02, 0.98, "\n".join(note_lines),
        transform=ax.transAxes, va="top", ha="left", fontsize=7,
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.85, edgecolor="0.7"),
    )
    p = os.path.join(out_dir, "fig2_err_vs_pwm.png")
    fig.savefig(p, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved {p}")

    # ── Fig 3: RPM error vs vbat, colour = PWM ───────────────────────────────
    fig, ax = plt.subplots(figsize=(8, 5))
    sc = ax.scatter(all_vbat, all_err, c=all_pm, cmap="viridis",
                    s=8, alpha=0.5)
    plt.colorbar(sc, ax=ax, label="PWM (‰)")
    ax.axhline(0, color="gray", linewidth=0.8, linestyle=":")
    m2 = error_models["m2"]["params"]
    ax.set_xlabel("Vbat (mV)")
    ax.set_ylabel("RPM error  (rpmR − rpmL)")
    ax.set_title("RPM error vs Vbat  (colour = PWM)")
    ax.grid(True, alpha=0.3)
    ax.text(
        0.02, 0.98,
        "\n".join([
            f"M2: err = {format_signed(m2[0])}*pm {format_signed(m2[1])}*(Vnom-vbat) {format_signed(m2[2])}",
            f"M2 RMSE = {error_models['m2']['rmse']:.3f} rpm",
            error_models["conclusion"],
        ]),
        transform=ax.transAxes, va="top", ha="left", fontsize=7,
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.85, edgecolor="0.7"),
    )
    p = os.path.join(out_dir, "fig3_err_vs_vbat.png")
    fig.savefig(p, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved {p}")

    # ── Fig 4: residual histogram (M1 vs M2) ─────────────────────────────────
    res1 = error_models["m1"]["residuals"]
    res2 = error_models["m2"]["residuals"]

    fig, ax = plt.subplots(figsize=(8, 5))
    bins = np.linspace(
        min(res1.min(), res2.min()), max(res1.max(), res2.max()), 40
    )
    ax.hist(res1, bins=bins, alpha=0.5, label="M1 residuals (winding only)", color="steelblue")
    ax.hist(res2, bins=bins, alpha=0.5, label="M2 residuals (winding+vbat)", color="tomato")
    ax.axvline(0, color="gray", linewidth=0.8, linestyle=":")
    ax.set_xlabel("Residual (rpm)")
    ax.set_ylabel("Count")
    ax.set_title("Model residual distribution")
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.text(
        0.98, 0.98,
        "\n".join([
            f"M1 RMSE = {error_models['m1']['rmse']:.3f} rpm",
            f"M2 RMSE = {error_models['m2']['rmse']:.3f} rpm",
            error_models["conclusion"],
        ]),
        transform=ax.transAxes, va="top", ha="right", fontsize=8,
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.85, edgecolor="0.7"),
    )
    p = os.path.join(out_dir, "fig4_residuals.png")
    fig.savefig(p, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved {p}")


def print_aggregated_table(agg_rows):
    header = (
        f"{'dir':>4} {'idx':>4} {'pm':>6} {'n':>4} "
        f"{'rpmL_mean':>10} {'rpmL_std':>9} "
        f"{'rpmR_mean':>10} {'rpmR_std':>9} "
        f"{'vbat_mean':>10} "
        f"{'err_mean':>9} {'err_std':>8}"
    )
    print(header)
    print("-" * len(header))
    for r in agg_rows:
        print(
            f"{r['dir']:>4} {r['idx']:>4} {r['pm']:>6} {r['n']:>4} "
            f"{r['rpmL_mean']:>10.2f} {r['rpmL_std']:>9.2f} "
            f"{r['rpmR_mean']:>10.2f} {r['rpmR_std']:>9.2f} "
            f"{r['vbat_mean']:>10.0f} "
            f"{r['err_mean']:>9.2f} {r['err_std']:>8.2f}"
        )


def main():
    parser = argparse.ArgumentParser(
        description="Analyze motor calibration sweep log from MSPM0G3507"
    )
    parser.add_argument("log_file", help="Path to serial log .txt file")
    parser.add_argument(
        "--out-dir", default=None,
        help="Output directory for PNG charts (default: same dir as log file)"
    )
    parser.add_argument(
        "--v-nominal", type=float, default=11100.0,
        help="Nominal battery voltage in mV used in M2 fit (default: 11100)"
    )
    args = parser.parse_args()

    if not os.path.isfile(args.log_file):
        print(f"Error: file not found: {args.log_file}", file=sys.stderr)
        sys.exit(1)

    out_dir = args.out_dir or os.path.dirname(os.path.abspath(args.log_file))

    print(f"Parsing {args.log_file} ...")
    samples = parse_log(args.log_file)
    print(
        f"  {len(samples)} stable samples found after settle filtering "
        f"and dropping the first {DROP_INITIAL_FRAMES} frames of each step"
    )

    if len(samples) == 0:
        print("No valid [cal] data samples found. Check that the log contains "
              "lines matching the pattern: [cal] dir=... idx=... pm=... t=... vbat=... rpmL=... rpmR=...")
        sys.exit(1)

    vbat_vals = [s["vbat"] for s in samples]
    if max(vbat_vals) < 1000:
        print("WARNING: vbat values are all below 1000 mV. "
              "Battery may not have been connected during calibration. "
              "Voltage-related analysis will not be meaningful.")

    agg_rows = aggregate(samples)
    print(f"\n── Aggregated per-step statistics ({len(agg_rows)} steps) ──────────────────────────")
    print_aggregated_table(agg_rows)

    deadzones = detect_deadzones(agg_rows)
    print_deadzone_summary(deadzones)
    motor_fits = build_motor_fits(agg_rows)

    # Error models
    pm_all   = np.array([s["pm"]   for s in samples], dtype=float)
    vbat_all = np.array([s["vbat"] for s in samples], dtype=float)
    err_all  = np.array([s["rpmR"] - s["rpmL"] for s in samples], dtype=float)
    error_models = fit_error_models(pm_all, vbat_all, err_all, v_nominal=args.v_nominal)

    print("\n── Generating charts ───────────────────────────────────────────────────")
    make_plots(samples, agg_rows, deadzones, motor_fits, error_models, out_dir)
    print("\nDone.")


if __name__ == "__main__":
    main()
