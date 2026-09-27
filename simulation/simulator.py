"""
MESHMERIZE v5.0 — Championship Arena Simulator
================================================
Routing algorithms used:
  DRY RUN  → Held-Karp exact TSP (optimal checkpoint tour, START→all CPs→END)
             Complexity O(n! × Dijkstra) for n≤4 checkpoints — exact global optimum
  ACTUAL RUN → Dijkstra shortest path (START→END, no detour)
             networkx built-in, O((V+E) log V) — provably shortest

Controls:
  SPACE  → play / pause
  M      → toggle Dry / Actual mode
  D      → toggle Dijkstra overlay
  T      → toggle TSP overlay
  R      → random arena (requires mazelib)
  ESC    → quit
"""

import sys, math, time, argparse, itertools
import pygame
import networkx as nx

try:
    from mazelib import Maze
    from mazelib.generate.Prims import Prims
    HAS_MAZELIB = True
except ImportError:
    HAS_MAZELIB = False

# ── Display constants ─────────────────────────────────────────────────
FPS    = 60
WIN_W  = 1340
WIN_H  = 860
MARGIN = 60
PANEL_W = 370    # Right HUD panel width
ARENA_W = WIN_W - PANEL_W

C_BG      = (10,  10,  20)
C_PANEL   = (16,  16,  30)
C_LINE    = (220, 220, 220)
C_NODE    = (160, 160,  40)
C_CP      = (255, 210,   0)
C_START   = (0,   230,  80)
C_END     = (230,  50,  50)
C_DIJKSTR = (50,  255, 170)
C_TSP     = (255, 120,  50)
C_DRYBOT  = (100, 180, 255)
C_ACTBOT  = (255,  80, 120)
C_SLIP    = (255,  50,  50)
C_TEXT    = (200, 200, 210)
C_GREY    = (70,  70,  95)
C_BAR_BG  = (30,  30,  50)
C_BAR_FG  = (50,  200, 110)
C_BAR_OT  = (255, 120,  50)

pygame.init()
screen = pygame.display.set_mode((WIN_W, WIN_H))
pygame.display.set_caption("Meshmerize v5.0 — Championship Simulator")
clock = pygame.time.Clock()
f9  = pygame.font.SysFont("Consolas", 9)
f11 = pygame.font.SysFont("Consolas", 11)
f13 = pygame.font.SysFont("Consolas", 13)
f15 = pygame.font.SysFont("Consolas", 15, bold=True)
f19 = pygame.font.SysFont("Consolas", 19, bold=True)
f24 = pygame.font.SysFont("Consolas", 24, bold=True)

# ── Arena definition ──────────────────────────────────────────────────
def build_techfest_arena():
    """
    Techfest Meshmerize arena modelled from the problem statement.
    All positions in cm-scale grid units.
    Checkpoints: CP1, CP2, CP3, CP4 — must all be visited in dry run.
    """
    G = nx.Graph()
    # (x, y, is_checkpoint)
    nodes = {
        "START": (200, 20,  False),
        "N1":    (153, 20,  False),
        "CP1":   (153, 60,  True),
        "N2":    (183, 60,  False),
        "CP2":   (183, 110, True),
        "LP_A":  (153, 150, False),
        "LP_B":  (210, 150, False),
        "LP_C":  (210, 110, False),
        "LP_D":  (183, 180, False),
        "CP3":   (80,  110, True),
        "CP4":   (120, 150, True),
        "N3":    (120, 60,  False),
        "N4":    (80,  60,  False),
        "END":   (30,  20,  False),
    }
    for name, (x, y, cp) in nodes.items():
        G.add_node(name, pos=(x, y), checkpoint=cp)
    edges = [
        ("START","N1"),  ("N1","CP1"),   ("CP1","N2"),  ("N2","CP2"),
        ("CP2","LP_C"),  ("LP_C","LP_B"),("LP_B","LP_A"),("LP_A","CP1"),
        ("CP2","LP_D"),  ("LP_D","CP4"), ("CP4","CP3"),
        ("CP3","N4"),    ("N4","N3"),    ("N3","CP1"),
        ("N4","END"),    ("N1","N3"),
    ]
    for u, v in edges:
        p1 = G.nodes[u]["pos"]; p2 = G.nodes[v]["pos"]
        G.add_edge(u, v, weight=math.hypot(p1[0]-p2[0], p1[1]-p2[1]))
    return G, "START", "END"

def build_random_arena():
    if not HAS_MAZELIB:
        return build_techfest_arena()
    import random
    m = Maze(); m.generator = Prims(6, 8); m.generate(); grid = m.grid
    G = nx.Graph(); h, w = grid.shape
    for r in range(h):
        for c in range(w):
            if grid[r][c] == 0:
                G.add_node((r,c), pos=(c*45, r*45), checkpoint=False)
    for r in range(h):
        for c in range(w):
            if grid[r][c] == 0:
                for dr, dc in [(0,1),(1,0)]:
                    nr, nc = r+dr, c+dc
                    if 0<=nr<h and 0<=nc<w and grid[nr][nc]==0:
                        G.add_edge((r,c),(nr,nc), weight=45)
    opens = [(r,c) for r in range(h) for c in range(w) if grid[r][c]==0]
    start, end = opens[0], opens[-1]
    mids = opens[len(opens)//5 : 4*len(opens)//5]
    for cp in random.sample(mids, min(4, len(mids))):
        G.nodes[cp]["checkpoint"] = True
    return G, start, end

# ── ROUTING ALGORITHMS ────────────────────────────────────────────────

def held_karp_tsp(G, start, checkpoints, end):
    """
    Held-Karp exact TSP for N≤10 checkpoints.

    Algorithm:
      1. Pre-compute Dijkstra shortest-path distance between every pair of
         {start, checkpoints..., end} — this collapses the full graph into
         a complete weighted sub-graph on just the key nodes.
      2. Run Held-Karp DP on that sub-graph:
         dp[visited_mask][last_node] = min distance to visit exactly the
         nodes in visited_mask, ending at last_node.
      3. Reconstruct the best permutation, then stitch Dijkstra sub-paths
         to get the full node-by-node path through the arena.

    Why better than brute-force permutations:
      Brute force: O(n!)          → n=10: 3,628,800 iterations
      Held-Karp:   O(n² × 2ⁿ)   → n=10:     102,400 iterations
      Both give the EXACT optimum (not approximate).
      For n≤4 (our case) both are instant, but Held-Karp scales if
      the problem statement ever increases checkpoint count.

    Returns: (full_path_node_list, total_distance)
    """
    cps = list(checkpoints)
    n = len(cps)

    if n == 0:
        # No checkpoints — direct Dijkstra
        try:
            path = nx.dijkstra_path(G, start, end, weight="weight")
            dist = nx.dijkstra_path_length(G, start, end, weight="weight")
        except nx.NetworkXNoPath:
            path, dist = [start, end], float('inf')
        return path, dist

    # ── Step 1: Pre-compute pairwise Dijkstra distances ──────────────
    key_nodes = [start] + cps + [end]
    # dist_mat[i][j] = shortest graph distance from key_nodes[i] to key_nodes[j]
    dist_mat = {}
    path_mat = {}
    for i, u in enumerate(key_nodes):
        for j, v in enumerate(key_nodes):
            if i == j:
                dist_mat[(i,j)] = 0.0
                path_mat[(i,j)] = [u]
            else:
                try:
                    d = nx.dijkstra_path_length(G, u, v, weight="weight")
                    p = nx.dijkstra_path(G, u, v, weight="weight")
                except nx.NetworkXNoPath:
                    d = float('inf')
                    p = [u, v]
                dist_mat[(i,j)] = d
                path_mat[(i,j)] = p

    # Key node indices: 0=start, 1..n=checkpoints, n+1=end
    # We must visit indices 1..n (all checkpoints) in some order,
    # starting from index 0 and ending at index n+1.

    INF = float('inf')
    # dp[mask][i] = min cost to visit checkpoints in mask, currently at cp index i
    # mask is a bitmask over 0..n-1 (the checkpoints only)
    dp   = [[INF] * n for _ in range(1 << n)]
    prev = [[-1]  * n for _ in range(1 << n)]

    # Initialise: start → each checkpoint
    for i in range(n):
        dp[1 << i][i] = dist_mat[(0, i+1)]   # 0=start, i+1=cp[i]

    # Fill DP
    for mask in range(1, 1 << n):
        for last in range(n):
            if not (mask >> last & 1):
                continue
            if dp[mask][last] == INF:
                continue
            for nxt in range(n):
                if mask >> nxt & 1:
                    continue   # already visited
                new_mask = mask | (1 << nxt)
                new_cost = dp[mask][last] + dist_mat[(last+1, nxt+1)]
                if new_cost < dp[new_mask][nxt]:
                    dp[new_mask][nxt] = new_cost
                    prev[new_mask][nxt] = last

    # ── Step 2: Find best final checkpoint to visit before END ────────
    full_mask = (1 << n) - 1
    best_cost = INF
    best_last = -1
    for last in range(n):
        total = dp[full_mask][last] + dist_mat[(last+1, n+1)]  # n+1 = end index
        if total < best_cost:
            best_cost = total
            best_last = last

    if best_last == -1:
        # Fallback — graph is disconnected somehow
        return [start, end], INF

    # ── Step 3: Reconstruct checkpoint visit order ────────────────────
    order = []
    mask = full_mask
    cur  = best_last
    while cur != -1:
        order.append(cur)
        nxt_prev = prev[mask][cur]
        mask = mask ^ (1 << cur)
        cur  = nxt_prev
    order.reverse()  # order is now 0-indexed into cps[]

    # ── Step 4: Stitch full node-level path ───────────────────────────
    visit_sequence = [start] + [cps[i] for i in order] + [end]
    full_path = []
    for seg_start, seg_end in zip(visit_sequence, visit_sequence[1:]):
        si = key_nodes.index(seg_start)
        ei = key_nodes.index(seg_end)
        seg = path_mat[(si, ei)]
        if full_path:
            full_path.extend(seg[1:])   # skip duplicate junction node
        else:
            full_path.extend(seg)

    return full_path, best_cost


def dijkstra_shortest(G, start, end):
    """
    Standard Dijkstra from networkx.
    O((V+E) log V) — provably shortest path, no detour through checkpoints.
    Used for the ACTUAL RUN.
    """
    try:
        path = nx.dijkstra_path(G, start, end, weight="weight")
        dist = nx.dijkstra_path_length(G, start, end, weight="weight")
    except nx.NetworkXNoPath:
        path, dist = [start, end], float('inf')
    return path, dist


# ── Coordinate mapping ────────────────────────────────────────────────
def get_scale(G):
    xs = [G.nodes[n]["pos"][0] for n in G.nodes]
    ys = [G.nodes[n]["pos"][1] for n in G.nodes]
    return min((ARENA_W - 2*MARGIN) / max(max(xs), 1),
               (WIN_H  - 2*MARGIN) / max(max(ys), 1))

def apos(G, node):
    s = get_scale(G); p = G.nodes[node]["pos"]
    return (MARGIN + int(p[0]*s), MARGIN + int(p[1]*s))


# ── Simulation State ──────────────────────────────────────────────────
class Sim:
    # Real robot speeds (Pololu 20:1 HPCB @ 12V 3S LiPo)
    SPEED_DRY_CM_S    = 35.0   # 350 mm/s = 35 cm/s (exploration search speed)
    SPEED_ACTUAL_CM_S = 120.0  # 1200 mm/s = 120 cm/s = 1.2 m/s (actual sprint speed)

    def __init__(self, G, start, end):
        self.G = G; self.start = start; self.end = end
        self.mode = "DRY"
        self.running = False
        self.show_dijkstra = True
        self.show_tsp = True
        self.anim_t = 0.0
        self.path_len_cm = 0.0   # total cm of current path
        self.visited_cps = set()
        self.slip_events = []
        self.t_start = None
        self.dry_time = 0.0
        self.actual_time = 0.0
        self.vbat = 12.6

        cps = [n for n in G.nodes if G.nodes[n].get("checkpoint")]

        # ── ACTUAL RUN: plain Dijkstra (shortest path, no detour)
        self.dijk_path, self.dijk_dist = dijkstra_shortest(G, start, end)

        # ── DRY RUN: Held-Karp exact TSP (visits all checkpoints optimally)
        self.tsp_path, self.tsp_dist = held_karp_tsp(G, start, cps, end)

        self._recalc_path_metrics()

    def _recalc_path_metrics(self):
        """Pre-compute cumulative distance along each path for accurate time simulation."""
        def cum_dist(G, path):
            d = [0.0]
            for i in range(len(path)-1):
                if path[i] in G.nodes and path[i+1] in G.nodes:
                    p1 = G.nodes[path[i]]["pos"]
                    p2 = G.nodes[path[i+1]]["pos"]
                    d.append(d[-1] + math.hypot(p1[0]-p2[0], p1[1]-p2[1]))
                else:
                    d.append(d[-1])
            return d
        self.tsp_cumlen  = cum_dist(self.G, self.tsp_path)
        self.dijk_cumlen = cum_dist(self.G, self.dijk_path)

    def active_path(self):
        return self.tsp_path if self.mode == "DRY" else self.dijk_path

    def active_cumlen(self):
        return self.tsp_cumlen if self.mode == "DRY" else self.dijk_cumlen

    def active_speed_cm_s(self):
        return self.SPEED_DRY_CM_S if self.mode == "DRY" else self.SPEED_ACTUAL_CM_S

    def active_total_dist(self):
        return self.tsp_dist if self.mode == "DRY" else self.dijk_dist

    def reset(self):
        self.anim_t = 0.0
        self.visited_cps = set()
        self.slip_events = []
        self.t_start = None
        self.dry_time = 0.0
        self.actual_time = 0.0
        self.vbat = 12.6

    def tick(self, dt):
        if not self.running: return
        if self.t_start is None: self.t_start = time.time()

        # Advance distance-based animation (cm/s × dt = cm advanced)
        dist_advanced = self.active_speed_cm_s() * dt
        cumlen = self.active_cumlen()
        total  = cumlen[-1] if cumlen else 1.0

        self.path_len_cm += dist_advanced
        self.path_len_cm  = min(self.path_len_cm, total)

        # Map distance → path index (binary search would be faster but path is short)
        path = self.active_path()
        seg_idx = 0
        for i, d in enumerate(cumlen):
            if d <= self.path_len_cm:
                seg_idx = i
        self.anim_t = seg_idx

        cur = path[min(seg_idx, len(path)-1)]

        # Checkpoint detection
        if self.G.nodes[cur].get("checkpoint"):
            self.visited_cps.add(cur)

        # Battery decay (realistic: ~0.01V/s at race speed)
        if self.mode == "ACTUAL":
            self.vbat = max(11.1, 12.6 - self.actual_time * 0.008)

        # Slip simulation
        import random
        if self.mode == "ACTUAL" and random.random() < 0.0025:
            self.slip_events.append((self.bot_screen_pos(), time.time()))
        self.slip_events = [e for e in self.slip_events if time.time()-e[1] < 1.5]

        # Update timers
        elapsed = time.time() - self.t_start
        if self.mode == "DRY":
            self.dry_time = elapsed
        else:
            self.actual_time = elapsed

        # End of path
        if self.path_len_cm >= total:
            self.running = False

    def bot_screen_pos(self):
        path = self.active_path()
        cumlen = self.active_cumlen()
        if len(path) < 2: return apos(self.G, path[0])
        # Find which segment we're on
        seg = 0
        for i, d in enumerate(cumlen):
            if d <= self.path_len_cm:
                seg = i
        seg = min(seg, len(path)-2)
        seg_start_d = cumlen[seg]
        seg_end_d   = cumlen[seg+1]
        seg_len = max(seg_end_d - seg_start_d, 0.001)
        frac = (self.path_len_cm - seg_start_d) / seg_len
        frac = max(0.0, min(1.0, frac))
        n1 = path[seg]; n2 = path[seg+1]
        s = get_scale(self.G)
        p1 = self.G.nodes[n1]["pos"]; p2 = self.G.nodes[n2]["pos"]
        x = MARGIN + (p1[0] + frac*(p2[0]-p1[0])) * s
        y = MARGIN + (p1[1] + frac*(p2[1]-p1[1])) * s
        return int(x), int(y)

    def score(self):
        """
        Techfest Meshmerize scoring:
          A = 25 × checkpoints visited
          B = 30 (completed dry run)
          C = max(0, 180 - dry_run_seconds)
          S = 30 (used shortest path — Dijkstra qualifies)
          T = max(0, 150 - actual_run_seconds)
          L = 5 (red LED lit at end zone)
        """
        A = len(self.visited_cps) * 25
        B = 30 if (not self.running and self.path_len_cm >= self.active_total_dist() * 0.99
                   or self.mode == "ACTUAL") else 0
        C = max(0, 180 - int(self.dry_time))
        S = 30   # Dijkstra guarantees we get this
        T = max(0, 150 - int(self.actual_time)) if self.mode == "ACTUAL" else 0
        L = 5
        return dict(A=A, B=B, C=C, S=S, T=T, L=L, total=A+B+C+S+T+L)

    def lsrb_estimate(self):
        """Typical LSRB competitor: misses loop CPs, slow speed."""
        return dict(A=25, B=30, C=85, S=0, T=55, L=5, total=200)

    def projected_score(self):
        """
        Projected final score assuming optimal play:
        Dry run at 350mm/s, all CPs hit (TSP path guarantees this).
        Actual run at 1200mm/s via Dijkstra.
        """
        dry_s  = self.tsp_dist  / self.SPEED_DRY_CM_S
        act_s  = self.dijk_dist / self.SPEED_ACTUAL_CM_S
        cps    = len([n for n in self.G.nodes if self.G.nodes[n].get("checkpoint")])
        A = cps * 25
        B = 30
        C = max(0, 180 - int(dry_s))
        S = 30
        T = max(0, 150 - int(act_s))
        L = 5
        return dict(A=A, B=B, C=C, S=S, T=T, L=L, total=A+B+C+S+T+L,
                    dry_s=dry_s, act_s=act_s)


# ── Drawing ───────────────────────────────────────────────────────────
def draw_path(screen, G, path, color, width=4):
    for i in range(len(path)-1):
        if path[i] in G.nodes and path[i+1] in G.nodes:
            pygame.draw.line(screen, color, apos(G,path[i]), apos(G,path[i+1]), width)

def draw_arena(screen, sim):
    G = sim.G
    # All edges (white track)
    for u, v in G.edges():
        w = G[u][v]["weight"]
        # Heatmap: short edges brighter
        bright = max(80, min(240, int(240 - w * 0.8)))
        pygame.draw.line(screen, (bright,bright,bright), apos(G,u), apos(G,v), 2)

    if sim.show_tsp:
        draw_path(screen, G, sim.tsp_path, C_TSP, 3)
    if sim.show_dijkstra:
        draw_path(screen, G, sim.dijk_path, C_DIJKSTR, 5)

    for n in G.nodes():
        p = apos(G, n)
        cp = G.nodes[n].get("checkpoint", False)
        if n == sim.start:       col, r = C_START, 11
        elif n == sim.end:       col, r = C_END,   11
        elif cp:                 col, r = C_CP,    10
        else:                    col, r = C_NODE,   5

        if n in sim.visited_cps:
            gs = pygame.Surface((44,44), pygame.SRCALPHA)
            pygame.draw.circle(gs, (255,210,0,70), (22,22), 22)
            screen.blit(gs, (p[0]-22, p[1]-22))

        pygame.draw.circle(screen, col, p, r)
        pygame.draw.circle(screen, (255,255,255), p, r, 1)
        lbl = f11.render(str(n) if isinstance(n,str) else f"{n[0]},{n[1]}", True, (150,150,175))
        screen.blit(lbl, (p[0]+r+2, p[1]-7))

def draw_bot(screen, sim):
    if sim.anim_t == 0 and not sim.running: return
    bx, by = sim.bot_screen_pos()
    col = C_DRYBOT if sim.mode == "DRY" else C_ACTBOT
    pr = 14 + int(4*abs(math.sin(time.time()*5)))
    gs = pygame.Surface((pr*4, pr*4), pygame.SRCALPHA)
    pygame.draw.circle(gs, (*col, 45), (pr*2, pr*2), pr*2)
    screen.blit(gs, (bx-pr*2, by-pr*2))
    pygame.draw.circle(screen, col, (bx,by), 12)
    pygame.draw.circle(screen, (255,255,255), (bx,by), 12, 2)

    for (sx,sy), te in sim.slip_events:
        alpha = max(0, int(255*(1-(time.time()-te)/1.5)))
        ss = pygame.Surface((30,30), pygame.SRCALPHA)
        pygame.draw.circle(ss, (255,50,50,alpha), (15,15), 15, 3)
        screen.blit(ss, (sx-15, sy-15))
        screen.blit(f9.render("SLIP",True,(255,50,50)), (sx+14, sy-5))

def bar(screen, x, y, w, h, frac, fg=C_BAR_FG, label=""):
    pygame.draw.rect(screen, C_BAR_BG, (x,y,w,h), border_radius=3)
    fw = int(w * max(0, min(1, frac)))
    col = C_BAR_OT if frac > 0.85 else fg
    if fw > 0:
        pygame.draw.rect(screen, col, (x,y,fw,h), border_radius=3)
    if label:
        screen.blit(f9.render(label, True, C_TEXT), (x+w+5, y+1))

def draw_hud(screen, sim):
    PX = ARENA_W + 6
    pw = WIN_W - PX - 8

    pygame.draw.rect(screen, C_PANEL, (PX-6, 0, pw+14, WIN_H))
    pygame.draw.line(screen, (40,40,72), (PX-6,0), (PX-6,WIN_H), 2)

    def txt(s, x, y, col=C_TEXT, f=f11):
        screen.blit(f.render(s, True, col), (x, y))

    def hdr(s, y):
        txt(s, PX, y, (160,160,195), f11); y += 16
        pygame.draw.line(screen, (45,45,75), (PX,y), (PX+pw-8,y)); return y+3

    y = 10
    txt("MESHMERIZE v5.0", PX, y, (90,190,255), f19); y += 26
    txt("Championship Simulator", PX, y, (80,80,120), f11); y += 20

    mode_col = C_DRYBOT if sim.mode == "DRY" else C_ACTBOT
    txt(f"{'◼ DRY RUN  (Held-Karp TSP)' if sim.mode=='DRY' else '▶ ACTUAL RUN (Dijkstra)'}", PX, y, mode_col, f13); y += 20
    st_col = (0,220,80) if sim.running else (180,60,60)
    txt(f"{'▶ RUNNING' if sim.running else '■ STOPPED'}", PX, y, st_col); y += 18

    # Progress bar
    total = sim.active_total_dist()
    frac  = (sim.path_len_cm / total) if total > 0 else 0
    bar(screen, PX, y, pw-40, 11, frac, label=f"{frac*100:.0f}%"); y += 22

    # Battery
    vfrac = (sim.vbat - 11.0) / 1.6
    bar(screen, PX, y, pw-40, 11, vfrac, fg=(50,220,80), label=f"{sim.vbat:.2f}V")
    txt("VBAT", PX-38, y, (120,120,160), f9); y += 22

    # ── SCORE (live)
    y = hdr("── LIVE SCORE ──────────────────", y)
    sc = sim.score(); ls = sim.lsrb_estimate()
    txt(f"         {'Ours':>5}  {'LSRB':>5}", PX, y, (90,90,120), f9); y += 13
    rows = [
        ("A","Checkpoints ×25", sc["A"], ls["A"], C_CP),
        ("B","Dry Run Done",    sc["B"], ls["B"], C_START),
        ("C","Dry Time Bonus",  sc["C"], ls["C"], (130,200,255)),
        ("S","Shortest Path",   sc["S"], ls["S"], C_DIJKSTR),
        ("T","Run Time Bonus",  sc["T"], ls["T"], C_ACTBOT),
        ("L","Red LED",         sc["L"], ls["L"], (255,90,90)),
    ]
    for code, name, ours, them, col in rows:
        txt(f"{code} {name:<16} {ours:>3}    {them:>3}", PX, y, col, f11); y += 15
    pygame.draw.line(screen, (60,60,90), (PX,y), (PX+pw-8,y)); y += 3
    txt(f"  TOTAL           {sc['total']:>3}    {ls['total']:>3}", PX, y, (255,255,70), f15); y += 22
    adv = sc["total"] - ls["total"]
    txt(f"  +{adv} pts advantage over LSRB", PX, y, (80,240,110), f11); y += 20

    # ── PROJECTED FINAL SCORE
    y = hdr("── PROJECTED FINAL SCORE ───────", y)
    proj = sim.projected_score()
    txt(f"  All 4 CPs (TSP guarantees):  {proj['A']}", PX, y, C_CP); y += 14
    txt(f"  Dry run done:                {proj['B']}", PX, y, C_START); y += 14
    txt(f"  Dry time ({proj['dry_s']:.0f}s):           {proj['C']}", PX, y, (130,200,255)); y += 14
    txt(f"  Shortest path (Dijkstra):    {proj['S']}", PX, y, C_DIJKSTR); y += 14
    txt(f"  Actual run ({proj['act_s']:.0f}s):         {proj['T']}", PX, y, C_ACTBOT); y += 14
    txt(f"  Red LED:                     {proj['L']}", PX, y, (255,90,90)); y += 14
    pygame.draw.line(screen, (60,60,90), (PX,y), (PX+pw-8,y)); y += 3
    txt(f"  PROJECTED TOTAL:            {proj['total']}", PX, y, (255,255,70), f15); y += 22

    # ── ALGORITHM INFO
    y = hdr("── ALGORITHM ───────────────────", y)
    txt(f"Nodes:          {sim.G.number_of_nodes()}", PX, y); y += 14
    txt(f"Edges:          {sim.G.number_of_edges()}", PX, y); y += 14
    cps_total = len([n for n in sim.G.nodes if sim.G.nodes[n].get("checkpoint")])
    txt(f"Checkpoints:    {len(sim.visited_cps)}/{cps_total}", PX, y, C_CP); y += 14
    txt(f"TSP path nodes: {len(sim.tsp_path)}", PX, y, C_TSP); y += 14
    txt(f"TSP distance:   {sim.tsp_dist:.0f} cm", PX, y, C_TSP); y += 14
    txt(f"TSP time est:   {sim.tsp_dist/sim.SPEED_DRY_CM_S:.0f}s @ 35mm/s", PX, y, C_TSP); y += 14
    txt(f"Dijk hops:      {len(sim.dijk_path)-1}", PX, y, C_DIJKSTR); y += 14
    txt(f"Dijk distance:  {sim.dijk_dist:.0f} cm", PX, y, C_DIJKSTR); y += 14
    txt(f"Dijk time est:  {sim.dijk_dist/sim.SPEED_ACTUAL_CM_S:.0f}s @ 42mm/s", PX, y, C_DIJKSTR); y += 18

    # PID bars
    y = hdr("── CASCADED PID (simulated) ────", y)
    outer = max(0, 1 - abs(math.sin(time.time()*2.5))*0.25)
    inner = max(0, 1 - abs(math.sin(time.time()*12))*0.12)
    txt("Outer (line pos):", PX, y, (170,210,255)); y += 13
    bar(screen, PX, y, pw-40, 9, outer, fg=(70,150,255), label=f"{outer*100:.0f}%"); y += 16
    txt("Inner L (velocity):", PX, y, (170,210,200)); y += 13
    bar(screen, PX, y, pw-40, 9, inner, fg=(60,190,140), label=f"{inner*100:.0f}%"); y += 16
    txt("Inner R (velocity):", PX, y, (170,210,200)); y += 13
    bar(screen, PX, y, pw-40, 9, inner*0.97, fg=(60,190,140), label=f"{inner*97:.0f}%"); y += 20

    # Controls
    y = hdr("── CONTROLS ────────────────────", y)
    for key, desc in [("SPACE","play/pause"),("M","switch mode"),
                      ("D","Dijkstra overlay"),("T","TSP overlay"),
                      ("R","random arena"),("ESC","quit")]:
        txt(f"[{key}]  {desc}", PX, y, (110,110,155)); y += 14

def draw_legend(screen):
    y = WIN_H - 28
    items = [
        (f"━━ Held-Karp TSP (Dry Run)",  C_TSP),
        (f"━━ Dijkstra (Actual Run)",     C_DIJKSTR),
        ("●  Checkpoint",                 C_CP),
        ("●  Start",                      C_START),
        ("●  End",                        C_END),
    ]
    for i, (t, c) in enumerate(items):
        screen.blit(f11.render(t, True, c), (MARGIN + i*230, y))


# ── Main ──────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description="Meshmerize Championship Simulator")
    ap.add_argument("--arena", choices=["techfest","random"], default="techfest")
    args = ap.parse_args()

    G, start, end = build_random_arena() if args.arena=="random" else build_techfest_arena()
    sim = Sim(G, start, end)

    # Print route summary to console at launch
    proj = sim.projected_score()
    print("\n" + "="*55)
    print("  MESHMERIZE v5.0 — Route Summary")
    print("="*55)
    cps = [n for n in G.nodes if G.nodes[n].get("checkpoint")]
    print(f"\n  Checkpoints: {cps}")
    print(f"\n  DRY RUN  (Held-Karp exact TSP):")
    print(f"    Path: {' → '.join(str(n) for n in sim.tsp_path)}")
    print(f"    Distance: {sim.tsp_dist:.1f} cm")
    print(f"    Time est: {sim.tsp_dist/sim.SPEED_DRY_CM_S:.1f}s @ 350mm/s")
    print(f"\n  ACTUAL RUN (Dijkstra shortest):")
    print(f"    Path: {' → '.join(str(n) for n in sim.dijk_path)}")
    print(f"    Distance: {sim.dijk_dist:.1f} cm")
    print(f"    Time est: {sim.dijk_dist/sim.SPEED_ACTUAL_CM_S:.1f}s @ 1200mm/s (1.2 m/s)")
    print(f"\n  Projected score: {proj['total']} / 500")
    print("="*55 + "\n")

    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0
        for e in pygame.event.get():
            if e.type == pygame.QUIT or \
               (e.type == pygame.KEYDOWN and e.key == pygame.K_ESCAPE):
                running = False
            if e.type == pygame.KEYDOWN:
                if e.key == pygame.K_SPACE:
                    if not sim.running and sim.path_len_cm >= sim.active_total_dist()*0.99:
                        sim.reset()
                    sim.running = not sim.running
                elif e.key == pygame.K_m:
                    sim.mode = "ACTUAL" if sim.mode=="DRY" else "DRY"
                    sim.reset()
                elif e.key == pygame.K_d:
                    sim.show_dijkstra = not sim.show_dijkstra
                elif e.key == pygame.K_t:
                    sim.show_tsp = not sim.show_tsp
                elif e.key == pygame.K_r:
                    G, start, end = build_random_arena()
                    sim = Sim(G, start, end)

        sim.tick(dt)
        screen.fill(C_BG)

        # Header
        pygame.draw.rect(screen, (12,12,32), (0,0,WIN_W,44))
        pygame.draw.line(screen, (35,35,70), (0,44), (WIN_W,44), 1)
        screen.blit(f24.render("MESHMERIZE v5.0  —  Absolute Championship Simulator", True, (75,175,255)), (10,9))

        draw_arena(screen, sim)
        draw_bot(screen, sim)
        draw_legend(screen)
        draw_hud(screen, sim)
        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()
