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


def parse_log(path: str):
    """Return list of sample dicts, filtered to post-settle only."""
    samples = []
    settle_ms = 500        # default; updated from header
    step_start_t = {}      # key: (dir, idx) → ms timestamp of that step start
    last_step_key = None

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
                last_step_key = key
                # We don't have a timestamp on step lines; mark as None —
                # use the first sample's t for that step instead.
                step_start_t[key] = None
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


def fit_poly(x, y, deg, label=""):
    """Fit polynomial, return (coeffs, R²)."""
    coeffs = np.polyfit(x, y, deg)
    y_pred = np.polyval(coeffs, x)
    ss_res = np.sum((y - y_pred) ** 2)
    ss_tot = np.sum((y - np.mean(y)) ** 2)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
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

    Returns (m1_params, m1_rmse, m2_params, m2_rmse, conclusion)
    """
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

    return m1, m1_rmse, m2, m2_rmse, conclusion


def make_plots(samples, agg_rows, out_dir, v_nominal=11100.0):
    """Generate and save the 4 diagnostic charts."""
    os.makedirs(out_dir, exist_ok=True)

    all_pm    = np.array([s["pm"]   for s in samples], dtype=float)
    all_rpmL  = np.array([s["rpmL"] for s in samples], dtype=float)
    all_rpmR  = np.array([s["rpmR"] for s in samples], dtype=float)
    all_vbat  = np.array([s["vbat"] for s in samples], dtype=float)
    all_err   = all_rpmR - all_rpmL

    fwd_mask = all_pm > 0

    # ── Fig 1: PWM → RPM (forward only) ─────────────────────────────────────
    fig, ax = plt.subplots(figsize=(8, 5))
    fpm  = all_pm[fwd_mask]
    frpmL = all_rpmL[fwd_mask]
    frpmR = all_rpmR[fwd_mask]

    ax.scatter(fpm, frpmL, s=8, alpha=0.4, color="steelblue", label="rpmL samples")
    ax.scatter(fpm, frpmR, s=8, alpha=0.4, color="tomato",    label="rpmR samples")

    for y_arr, col, label in [(frpmL, "steelblue", "Left"), (frpmR, "tomato", "Right")]:
        if len(fpm) > 2:
            pm_sorted = np.sort(np.unique(fpm))
            for deg, ls in [(1, "--"), (2, "-")]:
                c, r2 = np.polyfit(fpm, y_arr, deg), None
                c = np.polyfit(fpm, y_arr, deg)
                fit_y = np.polyval(c, pm_sorted)
                ax.plot(pm_sorted, fit_y, color=col, linestyle=ls,
                        label=f"{label} deg{deg}")

    ax.set_xlabel("PWM (‰)")
    ax.set_ylabel("RPM")
    ax.set_title("Forward PWM → RPM")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)
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
    ax.set_xlabel("PWM (‰)")
    ax.set_ylabel("RPM error  (rpmR − rpmL)")
    ax.set_title("RPM error vs PWM  (colour = Vbat)")
    ax.grid(True, alpha=0.3)
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
    ax.set_xlabel("Vbat (mV)")
    ax.set_ylabel("RPM error  (rpmR − rpmL)")
    ax.set_title("RPM error vs Vbat  (colour = PWM)")
    ax.grid(True, alpha=0.3)
    p = os.path.join(out_dir, "fig3_err_vs_vbat.png")
    fig.savefig(p, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved {p}")

    # ── Fig 4: residual histogram (M1 vs M2) ─────────────────────────────────
    valid = np.isfinite(all_pm) & np.isfinite(all_vbat) & np.isfinite(all_err)
    pm_v  = all_pm[valid]
    vbat_v = all_vbat[valid]
    err_v  = all_err[valid]

    n = len(pm_v)
    A1 = np.column_stack([pm_v, np.ones(n)])
    m1, _, _, _ = np.linalg.lstsq(A1, err_v, rcond=None)
    res1 = err_v - A1 @ m1

    dv = v_nominal - vbat_v
    A2 = np.column_stack([pm_v, dv, np.ones(n)])
    m2, _, _, _ = np.linalg.lstsq(A2, err_v, rcond=None)
    res2 = err_v - A2 @ m2

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
    print(f"  {len(samples)} stable samples found after settle filtering")

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

    # Forward-only fitting for PWM→RPM
    fwd_rows = [r for r in agg_rows if r["dir"] > 0]
    if len(fwd_rows) >= 3:
        pm_f  = np.array([r["pm"]        for r in fwd_rows])
        rpmL_f = np.array([r["rpmL_mean"] for r in fwd_rows])
        rpmR_f = np.array([r["rpmR_mean"] for r in fwd_rows])
        print("\n── Forward PWM → RPM polynomial fits ──────────────────────────────────")
        fit_poly(pm_f, rpmL_f, 1, "Left  linear ")
        fit_poly(pm_f, rpmL_f, 2, "Left  quad   ")
        fit_poly(pm_f, rpmR_f, 1, "Right linear ")
        fit_poly(pm_f, rpmR_f, 2, "Right quad   ")

    # Error models
    pm_all   = np.array([s["pm"]   for s in samples], dtype=float)
    vbat_all = np.array([s["vbat"] for s in samples], dtype=float)
    err_all  = np.array([s["rpmR"] - s["rpmL"] for s in samples], dtype=float)
    fit_error_models(pm_all, vbat_all, err_all, v_nominal=args.v_nominal)

    print("\n── Generating charts ───────────────────────────────────────────────────")
    make_plots(samples, agg_rows, out_dir, v_nominal=args.v_nominal)
    print("\nDone.")


if __name__ == "__main__":
    main()
