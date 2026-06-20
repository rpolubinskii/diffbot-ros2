#!/usr/bin/env python3
"""Analyze a diffbot SLAM test bag and print a closure/health report.

Usage:
    .venv/bin/python analyze_slam.py <bag_dir> [--deep]

<bag_dir> must contain the rtabmap launch .log and the rosbag2 .db3 + metadata.
--deep also deserializes /rtabmap/icp_odom + /imu/data_body to reconstruct the
icp-vs-gyro heading and the reweighter behavior (slower).

The report is organized as GATES so a glance tells you what passed/failed:
  DEPLOY   - did the intended params actually load on the robot?
  CLOSURE  - did loop closures land? inliers? what's blocking them?
  HEALTH   - icp tracking, camera starvation, optimizer warnings, CPU.
"""
import re
import sys
from pathlib import Path

GREEN, RED, YEL, DIM, RST = "\033[32m", "\033[31m", "\033[33m", "\033[2m", "\033[0m"


def find_log(bag: Path) -> Path | None:
    logs = sorted(bag.glob("*.log"))
    return logs[0] if logs else None


def grep_count(text: str, pat: str) -> int:
    return len(re.findall(pat, text))


def first(text: str, pat: str):
    m = re.search(pat, text)
    return m.group(1) if m else None


def analyze_log(text: str) -> dict:
    r = {}
    # DEPLOY: which params actually loaded (rtabmap echoes "Setting ... = X")
    def param(name):
        return first(text, rf'Setting RTAB-Map parameter "{re.escape(name)}"="([^"]+)"')
    r["Kp/DetectorStrategy"] = param("Kp/DetectorStrategy")
    r["Vis/FeatureType"] = param("Vis/FeatureType")
    r["Optimizer/Robust"] = param("Optimizer/Robust")
    r["Optimizer/Strategy"] = param("Optimizer/Strategy")
    r["RGBD/OptimizeMaxError"] = param("RGBD/OptimizeMaxError")
    r["Rtabmap/DetectionRate"] = param("Rtabmap/DetectionRate")

    # CLOSURE
    r["added"] = grep_count(text, r"Added loop closure")
    r["zero_inlier"] = grep_count(text, r"Not enough inliers 0/")
    r["low_inlier"] = grep_count(text, r"Not enough inliers (?:[1-9]|1[01])/12")
    r["optimizer_rej"] = grep_count(text, r"wrong loop closure has been detected")
    r["proximity"] = grep_count(text, r"[Pp]roximity")
    # best inliers achieved (X/Y across all "inliers X/Y" lines)
    inliers = [int(m) for m in re.findall(r"inliers (\d+)/12", text)]
    r["best_inliers"] = max(inliers) if inliers else None
    r["n_candidates"] = len(inliers)
    # optimizer culprit edges + ratios
    r["opt_ratios"] = sorted(float(x) for x in re.findall(r"graph error ratio of ([0-9.]+)", text))
    edges = re.findall(r"edge (\d+->\d+), type=(\d)", text)
    r["culprit_edges"] = {}
    for e, t in edges:
        r["culprit_edges"][f"{e} (type={t})"] = r["culprit_edges"].get(f"{e} (type={t})", 0) + 1

    # HEALTH
    r["null_guess"] = grep_count(text, r"null guess")
    r["no_data"] = grep_count(text, r"Did not receive data")
    r["limit_oob"] = grep_count(text, r"limit out of bounds")
    r["neg_hessian"] = grep_count(text, r"negative hessian")
    delays = [float(x) for x in re.findall(r"delay=([0-9.]+)", text)]
    r["delay_max"] = max(delays) if delays else None
    r["delay_last"] = delays[-1] if delays else None
    # steady-state: median ignores the one-off startup spike that maxes out
    r["delay_med"] = sorted(delays)[len(delays) // 2] if delays else None
    node_ids = [int(x) for x in re.findall(r"between \d+ and (\d+)", text)]
    r["max_node"] = max(node_ids) if node_ids else None
    return r


def fmt_gate(ok: bool | None, label: str, detail: str = "") -> str:
    if ok is None:
        mark = f"{YEL}?{RST}"
    else:
        mark = f"{GREEN}PASS{RST}" if ok else f"{RED}FAIL{RST}"
    return f"  [{mark}] {label}{(' — ' + detail) if detail else ''}"


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    bag = Path(sys.argv[1])
    deep = "--deep" in sys.argv[2:]
    if not bag.is_dir():
        print(f"{RED}not a directory: {bag}{RST}")
        sys.exit(1)
    log = find_log(bag)
    if not log:
        print(f"{RED}no .log file in {bag}{RST}")
        sys.exit(1)

    text = log.read_text(errors="replace")
    r = analyze_log(text)

    print(f"\n=== {bag.name} ===  ({log.name})")

    # DEPLOY gate
    print(f"\n{DIM}DEPLOY (params actually loaded on the robot){RST}")
    ft = r["Vis/FeatureType"]
    ftname = {"1": "SIFT", "2": "ORB", "6": "GFTT/BRIEF", "8": "GFTT/ORB"}.get(ft, ft)
    print(f"  feature type Vis/FeatureType={ft} ({ftname}), Kp/DetectorStrategy={r['Kp/DetectorStrategy']}")
    print(f"  Optimizer/Robust={r['Optimizer/Robust']}  Strategy={r['Optimizer/Strategy']}  "
          f"OptimizeMaxError={r['RGBD/OptimizeMaxError']}  DetectionRate={r['Rtabmap/DetectionRate']}")

    # CLOSURE gate
    print(f"\n{DIM}CLOSURE{RST}")
    print(fmt_gate(r["added"] > 0, f"Added loop closure: {r['added']}",
                   "" if r["added"] else "ZERO closures landed"))
    bi = r["best_inliers"]
    print(fmt_gate(bi is not None and bi >= 12,
                   f"best inliers this run: {bi}/12 over {r['n_candidates']} candidates",
                   f"zero-inlier={r['zero_inlier']} low(1-11)={r['low_inlier']}"))
    if r["opt_ratios"]:
        print(f"  optimizer rejections: {r['optimizer_rej']}  ratios {r['opt_ratios'][:3]}..{r['opt_ratios'][-1]:.2f}")
        if r["culprit_edges"]:
            top = sorted(r["culprit_edges"].items(), key=lambda kv: -kv[1])[:3]
            print(f"  culprit edges: " + ", ".join(f"{e}×{n}" for e, n in top)
                  + f"  {DIM}(type=0 = bad odometry/neighbor edge){RST}")
    print(f"  proximity mentions: {r['proximity']}")

    # HEALTH gate
    print(f"\n{DIM}HEALTH{RST}")
    print(fmt_gate(r["null_guess"] == 0, f"icp null-guess: {r['null_guess']}"))
    print(fmt_gate(r["no_data"] == 0, f"camera starvation (Did not receive data): {r['no_data']}"))
    print(fmt_gate(r["limit_oob"] == 0, f"icp limit-out-of-bounds: {r['limit_oob']}"))
    if r["neg_hessian"]:
        print(f"  {YEL}g2o negative-hessian warnings: {r['neg_hessian']} (cosmetic: pose-cov output only){RST}")
    print(fmt_gate(r["delay_med"] is None or r["delay_med"] < 1.0,
                   f"rtabmap delay median={r['delay_med']}s (max={r['delay_max']}s, last={r['delay_last']}s)",
                   "gate on median; max is usually a startup spike"))
    print(f"  map nodes (max id seen): {r['max_node']}  "
          f"{DIM}(>400 = strong revisit trajectory; <300 = weak, closures untestable){RST}")

    if deep:
        deep_analyze(bag)
    print()


def deep_analyze(bag: Path):
    """icp covariance + icp-vs-gyro-vs-EKF heading (needs rosbags + the venv)."""
    import math, bisect
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)

    def yaw(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    icp, gyro, ekf = [], [], []
    with Reader(bag) as rd:
        for con, t, raw in rd.messages():
            if con.topic == "/rtabmap/icp_odom":
                m = ts.deserialize_cdr(raw, con.msgtype)
                icp.append((t * 1e-9, yaw(m.pose.pose.orientation),
                            m.twist.twist.angular.z, m.pose.covariance[35], m.pose.covariance[0]))
            elif con.topic == "/imu/data_body":
                m = ts.deserialize_cdr(raw, con.msgtype)
                gyro.append((t * 1e-9, m.angular_velocity.z))
            elif con.topic == "/odom":
                m = ts.deserialize_cdr(raw, con.msgtype)
                ekf.append((t * 1e-9, yaw(m.pose.pose.orientation)))
    print(f"\n{DIM}DEEP (icp covariance + heading){RST}")
    if not icp:
        print("  no /rtabmap/icp_odom in bag")
        return
    ratios = [c35 / c0 for _, _, _, c35, c0 in icp if c0 > 0]
    print(f"  icp yaw:x cov ratio: {min(ratios):.3f}..{max(ratios):.3f} "
          f"({'FIXED scalar' if max(ratios)-min(ratios) < 1e-6 else 'varies'})")
    if ekf and gyro:
        def angd(a, b): return math.atan2(math.sin(a - b), math.cos(a - b))
        net_icp = math.degrees(angd(icp[-1][1], icp[0][1]))
        net_ekf = math.degrees(angd(ekf[-1][1], ekf[0][1]))
        print(f"  net heading: icp={net_icp:+.1f}°  EKF={net_ekf:+.1f}°  "
              f"(EKF==icp means the abs-yaw pin still dominates)")


if __name__ == "__main__":
    main()
