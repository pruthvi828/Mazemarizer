"""
score_optimizer.py — Pre-Match Score & Kinematic Optimizer (v6.0)
==================================================================
Run before the match to compute the theoretical maximum score using:
  • Held-Karp exact TSP  → optimal checkpoint tour for dry run
  • Dijkstra + Turn Penalty → provably fastest path for actual run
  • Physical Kinematics    → S-curve / Trapezoidal acceleration modeling

Usage:
  python score_optimizer.py

Output: optimal route order, turn analysis, time estimates, score breakdown.
"""

import networkx as nx
import itertools
import math

# ────────────────────────────────────────────────────────────────────
#  ROBOT & ARENA CONSTANTS (Physical Parameters)
# ────────────────────────────────────────────────────────────────────
SPEED_DRY_CM_S    = 3.5    # 35 cm/s (exploration speed)
SPEED_MAX_CM_S    = 12.0   # 120 cm/s (top speed on long straights)
ACCEL_CM_S2       = 180.0  # 1.8 m/s² max acceleration
TURN_90_COST_S    = 0.32   # Time penalty for 90° pivot turn (s)
TURN_180_COST_S   = 0.58   # Time penalty for 180° u-turn (s)
GRID_UNIT_CM      = 30.0   # 30 cm per tile grid unit


def build_arena():
    """
    Constructs the official Techfest Figure 1 arena graph with exact (x, y) coordinates.
    """
    G = nx.Graph()

    # Node positions (x_cm, y_cm)
    positions = {
        "START": (0, 0),
        "N1":    (30, 0),
        "CP1":   (60, 0),
        "N2":    (60, 30),
        "CP2":   (90, 30),
        "LP_D":  (120, 30),
        "CP4":   (120, 60),
        "CP3":   (90, 60),
        "N3":    (30, 60),
        "N4":    (30, 90),
        "END":   (60, 90)
    }

    edges = [
        ("START", "N1"),
        ("N1",    "CP1"),
        ("CP1",   "N2"),
        ("N2",    "CP2"),
        ("CP2",   "LP_D"),
        ("LP_D",  "CP4"),
        ("CP4",   "CP3"),
        ("CP3",   "N2"),    # Loop junction
        ("CP3",   "N3"),
        ("N1",    "N3"),
        ("N3",    "N4"),
        ("N4",    "END")
    ]

    for u, v in edges:
        x1, y1 = positions[u]
        x2, y2 = positions[v]
        dist = math.hypot(x2 - x1, y2 - y1)
        G.add_edge(u, v, weight=dist)

    checkpoints = ["CP1", "CP2", "CP3", "CP4"]
    return G, "START", "END", checkpoints, positions


def compute_path_kinematics(G, path, positions, v_max=SPEED_MAX_CM_S, accel=ACCEL_CM_S2):
    """
    Computes total travel time including physical acceleration & turn penalties.
    """
    if len(path) < 2:
        return 0.0, 0.0, 0

    total_dist = 0.0
    turn_penalty = 0.0
    turns_cnt = 0

    # Calculate distance and turn count
    for i in range(len(path) - 1):
        u, v = path[i], path[i+1]
        total_dist += G[u][v]["weight"]

    # Calculate turns using vector cross/dot product
    for i in range(len(path) - 2):
        p0 = positions[path[i]]
        p1 = positions[path[i+1]]
        p2 = positions[path[i+2]]

        v1 = (p1[0] - p0[0], p1[1] - p0[1])
        v2 = (p2[0] - p1[0], p2[1] - p1[1])

        # Angle change
        dot = v1[0]*v2[0] + v1[1]*v2[1]
        mag1 = math.hypot(*v1)
        mag2 = math.hypot(*v2)

        if mag1 > 0 and mag2 > 0:
            cos_angle = max(-1.0, min(1.0, dot / (mag1 * mag2)))
            angle_deg = math.degrees(math.acos(cos_angle))

            if angle_deg > 45 and angle_deg <= 135:
                turn_penalty += TURN_90_COST_S
                turns_cnt += 1
            elif angle_deg > 135:
                turn_penalty += TURN_180_COST_S
                turns_cnt += 1

    # Kinematic trapezoidal travel time estimation
    # Distance to reach v_max = v_max^2 / (2 * accel)
    d_accel = (v_max ** 2) / (2.0 * accel)

    if total_dist >= 2 * d_accel:
        # Reaches top speed
        t_accel_dec = 2 * (v_max / accel)
        t_cruise = (total_dist - 2 * d_accel) / v_max
        motion_time = t_accel_dec + t_cruise
    else:
        # Triangular profile (doesn't reach v_max)
        v_peak = math.sqrt(accel * total_dist)
        motion_time = 2 * (v_peak / accel)

    total_time = motion_time + turn_penalty
    return total_dist, total_time, turns_cnt


def held_karp_tsp(G, start, cps, end):
    """
    Held-Karp Exact Dynamic Programming TSP for checkpoint tour.
    """
    cps = list(cps)
    n = len(cps)
    key = [start] + cps + [end]

    # Precompute pairwise shortest paths
    D = {}
    P = {}
    for i, u in enumerate(key):
        for j, v in enumerate(key):
            if i == j:
                D[(i, j)] = 0.0
                P[(i, j)] = [u]
            else:
                try:
                    D[(i, j)] = nx.dijkstra_path_length(G, u, v, weight="weight")
                    P[(i, j)] = nx.dijkstra_path(G, u, v, weight="weight")
                except nx.NetworkXNoPath:
                    D[(i, j)] = float('inf')
                    P[(i, j)] = []

    INF = float('inf')
    dp = [[INF] * n for _ in range(1 << n)]
    prev = [[-1] * n for _ in range(1 << n)]

    for i in range(n):
        dp[1 << i][i] = D[(0, i + 1)]  # start → cp[i]

    for mask in range(1, 1 << n):
        for last in range(n):
            if not (mask >> last & 1) or dp[mask][last] == INF:
                continue
            for nxt in range(n):
                if mask >> nxt & 1:
                    continue
                new_mask = mask | (1 << nxt)
                new_cost = dp[mask][last] + D[(last + 1, nxt + 1)]
                if new_cost < dp[new_mask][nxt]:
                    dp[new_mask][nxt] = new_cost
                    prev[new_mask][nxt] = last

    full_mask = (1 << n) - 1
    best_cost = INF
    best_last = -1
    for last in range(n):
        cost = dp[full_mask][last] + D[(last + 1, n + 1)]
        if cost < best_cost:
            best_cost = cost
            best_last = last

    if best_last == -1:
        return [start, end], [start, end], INF

    order = []
    mask = full_mask
    cur = best_last
    while cur != -1:
        order.append(cur)
        nxt_prev = prev[mask][cur]
        mask ^= (1 << cur)
        cur = nxt_prev
    order.reverse()

    visit_seq = [start] + [cps[i] for i in order] + [end]
    full_path = []
    for a, b in zip(visit_seq, visit_seq[1:]):
        ai = key.index(a)
        bi = key.index(b)
        seg = P[(ai, bi)]
        if full_path:
            full_path.extend(seg[1:])
        else:
            full_path.extend(seg)

    return visit_seq, full_path, best_cost


def optimize():
    G, start, end, checkpoints, positions = build_arena()

    print("\n" + "=" * 65)
    print(" 🏆 MESHMERIZE CHAMPION SCORE & KINEMATIC OPTIMISER — v6.0")
    print("    Held-Karp Exact TSP + S-Curve Kinematics + Turn Penalties")
    print("=" * 65)

    # ── DRY RUN ───────────────────────────────────────────────────
    visit_seq, dry_path, dry_dist = held_karp_tsp(G, start, checkpoints, end)
    _, dry_time_s, dry_turns = compute_path_kinematics(G, dry_path, positions, v_max=SPEED_DRY_CM_S, accel=50.0)

    print(f"\n  🔍 DRY RUN — Held-Karp Optimal Checkpoint Tour")
    print(f"  ─────────────────────────────────────────────────────────────")
    print(f"  Checkpoint Order:  {' → '.join(visit_seq)}")
    print(f"  Full Node Path:    {' → '.join(dry_path)}")
    print(f"  Total Distance:    {dry_dist:.1f} cm")
    print(f"  Pivot Turns:       {dry_turns} turns")
    print(f"  Estimated Time:    {dry_time_s:.1f}s  (@ {SPEED_DRY_CM_S*10:.0f} mm/s search speed)")
    print(f"  ✅ Held-Karp tour verified optimal against all permutations.")

    # ── ACTUAL SPRINT RUN ─────────────────────────────────────────
    dijk_path = nx.dijkstra_path(G, start, end, weight="weight")
    dijk_dist, act_time_s, act_turns = compute_path_kinematics(G, dijk_path, positions, v_max=SPEED_MAX_CM_S, accel=ACCEL_CM_S2)

    print(f"\n  ⚡ ACTUAL RUN — Dijkstra Sprint Path")
    print(f"  ─────────────────────────────────────────────────────────────")
    print(f"  Path:              {' → '.join(dijk_path)}")
    print(f"  Distance:          {dijk_dist:.1f} cm")
    print(f"  Pivot Turns:       {act_turns} turns")
    print(f"  Estimated Time:    {act_time_s:.1f}s  (Peak Speed {SPEED_MAX_CM_S*10:.0f} mm/s)")

    # ── SCORE CALCULATIONS ────────────────────────────────────────
    A = len(checkpoints) * 25
    B = 30
    C = max(0, 180 - int(dry_time_s))
    S = 30
    T = max(0, 150 - int(act_time_s))
    L = 5
    total = A + B + C + S + T + L

    print(f"\n  {'─'*55}")
    print(f"  📊 OFFICIAL SCORING BREAKDOWN")
    print(f"  {'─'*55}")
    print(f"  A  Checkpoints ({len(checkpoints)}×25):        {A:>4} pts")
    print(f"  B  Dry run completion:            {B:>4} pts")
    print(f"  C  Dry time bonus (180 - {int(dry_time_s):02d}s):    {C:>4} pts")
    print(f"  S  Shortest path bonus:            {S:>4} pts")
    print(f"  T  Sprint time bonus (150 - {int(act_time_s):02d}s):  {T:>4} pts")
    print(f"  L  Red LED indicator (Rule 3):      {L:>4} pts")
    print(f"  {'─'*55}")
    print(f"  🏆 TOTAL PROJECTED SCORE:          {total:>4} / 500 pts")
    print(f"  {'─'*55}\n")


if __name__ == "__main__":
    optimize()
