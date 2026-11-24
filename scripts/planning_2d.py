"""2D motion-planning toy problems for docs `docs/background/17_trajectories_exercises.md`.

Point robot in [-2, 2]^2 with one diamond obstacle |x|+|y| < 1 (the l1-ball of the MIT notes' local-minima example).
Covers: RRT, RRT-Connect, shortcutting, RRT* parent choice, the shelf-free trajectory optimization with
different initial guesses, and one IRIS-style inflation (closest-point QP + separating hyperplanes + inscribed ellipse).

    .venv/bin/python scripts/planning_2d.py            # print all results
    .venv/bin/python scripts/planning_2d.py --figs DIR # write SVG figures
"""

from __future__ import annotations

import numpy as np

LO, HI = -2.0, 2.0
START, GOAL = np.array([-1.5, 0.0]), np.array([1.5, 0.0])
STEP = 0.25


WORLD = "diamond"  # "diamond": |x|+|y| < 1 is blocked; "wall": a vertical wall at x=0 with a gap at y in (1.2, 1.8)
WALL = ((-0.1, 0.1, -2.0, 1.2), (-0.1, 0.1, 1.8, 2.0))  # x0, x1, y0, y1 of the two wall pieces


def wall_clearance(q) -> float:
    """Largest-of-axes signed distance to the nearest wall piece (>= 0 means free)."""
    return min(max(max(x0 - q[0], q[0] - x1), max(y0 - q[1], q[1] - y1)) for x0, x1, y0, y1 in WALL)


def free_point(q) -> bool:
    if not (LO <= q[0] <= HI and LO <= q[1] <= HI):
        return False
    if WORLD == "wall":
        return bool(wall_clearance(q) >= 0)
    return bool(abs(q[0]) + abs(q[1]) >= 1.0)


def free_segment(a, b, res=0.02) -> bool:
    n = max(2, int(np.linalg.norm(b - a) / res) + 1)
    return all(free_point(a + (b - a) * t) for t in np.linspace(0, 1, n))


def steer(a, b, step=STEP):
    d = b - a
    n = np.linalg.norm(d)
    return b.copy() if n <= step else a + d / n * step


def path_len(p) -> float:
    return float(sum(np.linalg.norm(p[i + 1] - p[i]) for i in range(len(p) - 1)))


# ---------------- RRT ----------------
def rrt(rng, max_iter=4000, goal_bias=0.05):
    nodes, parent = [START.copy()], [-1]
    for it in range(1, max_iter + 1):
        qs = GOAL if rng.random() < goal_bias else rng.uniform(LO, HI, 2)
        i = int(np.argmin([np.linalg.norm(n - qs) for n in nodes]))
        qn = steer(nodes[i], qs)
        if not free_segment(nodes[i], qn):
            continue
        nodes.append(qn)
        parent.append(i)
        if np.linalg.norm(qn - GOAL) < 1e-9 or (np.linalg.norm(qn - GOAL) <= STEP and free_segment(qn, GOAL)):
            nodes.append(GOAL.copy())
            parent.append(len(nodes) - 2)
            return nodes, parent, it
    return nodes, parent, None


def extract(nodes, parent, k=-1):
    out, i = [], len(nodes) - 1 if k < 0 else k
    while i != -1:
        out.append(nodes[i])
        i = parent[i]
    return out[::-1]


# ---------------- RRT-Connect ----------------
def rrt_connect(rng, max_iter=4000):
    """Two trees (from START and from GOAL). Each iteration: extend one tree toward a random sample,
    then greedily CONNECT the other tree toward the new node; swap roles."""
    trees = [([START.copy()], [-1]), ([GOAL.copy()], [-1])]  # [0]: start tree, [1]: goal tree
    a = 0
    for it in range(1, max_iter + 1):
        qs = rng.uniform(LO, HI, 2)
        X, Y = trees[a], trees[1 - a]
        i = int(np.argmin([np.linalg.norm(n - qs) for n in X[0]]))
        qn = steer(X[0][i], qs)
        if free_segment(X[0][i], qn):
            X[0].append(qn); X[1].append(i)
            k = len(X[0]) - 1
            j = int(np.argmin([np.linalg.norm(n - qn) for n in Y[0]]))
            cur = Y[0][j]
            while True:  # CONNECT: keep stepping toward qn until blocked or reached
                nxt = steer(cur, qn)
                if not free_segment(cur, nxt):
                    break
                Y[0].append(nxt); Y[1].append(j)
                j = len(Y[0]) - 1
                cur = nxt
                if np.linalg.norm(cur - qn) < 1e-9:
                    from_x, from_y = extract(X[0], X[1], k), extract(Y[0], Y[1], j)
                    path = from_x + from_y[::-1][1:] if a == 0 else from_y + from_x[::-1][1:]
                    return path, it
        a = 1 - a
    return None, None


# ---------------- shortcutting ----------------
def shortcut(path, rng, iters=200):
    p = [q.copy() for q in path]
    for _ in range(iters):
        if len(p) < 3:
            break
        i, j = sorted(rng.choice(len(p), 2, replace=False))
        if j - i < 2:
            continue
        if free_segment(p[i], p[j]) and np.linalg.norm(p[j] - p[i]) < path_len(p[i:j + 1]) - 1e-9:
            p = p[:i + 1] + p[j:]
    return p


# ---------------- RRT* parent choice (figure of the exercise, grid units) ----------------
def rrt_star_figure():
    q = {"start": (5, 0), "q1": (4, 1), "q2": (6, 1), "near": (4, 3), "new": (3, 3)}
    par = {"q1": "start", "q2": "start", "near": "q1"}
    cost = {"start": 0.0}
    d = lambda a, b: float(np.hypot(q[a][0] - q[b][0], q[a][1] - q[b][1]))
    for k in ("q1", "q2", "near"):
        cost[k] = cost[par[k]] + d(k, par[k])
    return {c: cost[c] + d(c, "new") for c in ("start", "q1", "q2", "near")}, cost


# ---------------- shortcutting figure of the exercise (pixel coordinates read off the image) ----------------
SC = {"start": (59, 157), "1": (11, 77), "2": (97, 20), "6": (195, 62), "8": (307, 22), "10": (468, 63),
      "11": (363, 126), "14": (483, 157), "15": (540, 102), "goal": (563, 181)}
BOX = (252, 55, 345, 248)  # x0, y0, x1, y1


def sc_free(a, b):
    a, b = np.array(a, float), np.array(b, float)
    for t in np.linspace(0, 1, 400):
        x, y = a + (b - a) * t
        if BOX[0] <= x <= BOX[2] and BOX[1] <= y <= BOX[3]:
            return False
    return True


def shortcut_figure():
    names = list(SC)
    dist = lambda a, b: float(np.hypot(SC[a][0] - SC[b][0], SC[a][1] - SC[b][1]))
    out = []
    for i in range(len(names)):
        for j in range(i + 2, len(names)):
            a, b = names[i], names[j]
            old = sum(dist(names[k], names[k + 1]) for k in range(i, j))
            new = dist(a, b)
            if sc_free(SC[a], SC[b]) and new < old - 1e-9:
                out.append((a, b, old, new))
    return out


# ---------------- trajectory optimization with different initial guesses ----------------
def trajopt(init, start=START, goal=GOAL, N=16):
    from pydrake.solvers import MathematicalProgram, Solve
    prog = MathematicalProgram()
    Q = prog.NewContinuousVariables(N + 1, 2, "q")
    for n in range(N):
        d = Q[n + 1] - Q[n]
        prog.AddCost(d @ d)
    prog.AddBoundingBoxConstraint(start, start, Q[0])
    prog.AddBoundingBoxConstraint(goal, goal, Q[N])
    for n in range(1, N):
        if WORLD == "wall":
            for x0, x1, y0, y1 in WALL:  # outside each box: max(axis distances) >= 0
                prog.AddConstraint(lambda x, b=(x0, x1, y0, y1): [max(max(b[0] - x[0], x[0] - b[1]), max(b[2] - x[1], x[1] - b[3]))], [0.02], [np.inf], Q[n])  # 2 cm margin
        else:
            prog.AddConstraint(lambda x: [abs(x[0]) + abs(x[1])], [1.0], [np.inf], Q[n])
    prog.SetInitialGuess(Q, init)
    r = Solve(prog)
    if not r.is_success():
        return None, r.get_solver_id().name()
    return r.GetSolution(Q), r.get_solver_id().name()


def guess(kind, start=START, goal=GOAL, N=16):
    t = np.linspace(0, 1, N + 1)[:, None]
    line = start + (goal - start) * t
    bump = np.zeros_like(line)
    if kind == "up":
        bump[:, 1] = 1.4 * np.sin(np.pi * t[:, 0])
    elif kind == "down":
        bump[:, 1] = -0.6 * np.sin(np.pi * t[:, 0])
    elif kind == "through":
        pass
    return line + bump


# ---------------- IRIS-style inflation ----------------
OBSTACLES = {  # convex polygons as vertex lists
    "A": np.array([[-1.2, 0.8], [-0.4, 0.8], [-0.4, 1.6], [-1.2, 1.6]]),
    "B": np.array([[0.6, 0.5], [1.6, 0.5], [1.6, 1.4], [0.6, 1.4]]),
    "C": np.array([[-1.5, -1.6], [-0.3, -1.6], [-0.3, -0.7], [-1.5, -0.7]]),
    "D": np.array([[0.7, -1.7], [1.5, -1.7], [1.3, -0.6], [0.5, -0.6]]),
}
BOUND = 2.0


def hull_halfspaces(V):
    """Half-spaces A x <= b of a convex polygon given by counter-clockwise vertices."""
    A, b = [], []
    for k in range(len(V)):
        e = V[(k + 1) % len(V)] - V[k]
        n = np.array([e[1], -e[0]])
        n = n / np.linalg.norm(n)
        A.append(n); b.append(n @ V[k])
    return np.array(A), np.array(b)


def closest_point_qp(C, d, V):
    """QP: the point of the convex polygon (vertices V) closest to the ellipse {C u + d : |u|<=1}, in ellipse metric."""
    from pydrake.solvers import MathematicalProgram, Solve
    A, b = hull_halfspaces(V)
    M = np.linalg.inv(C @ C.T)
    prog = MathematicalProgram()
    x = prog.NewContinuousVariables(2, "x")
    prog.AddLinearConstraint(A, -np.inf * np.ones(len(b)), b, x)
    prog.AddQuadraticCost(2 * M, -2 * M @ d, x)  # (x-d)^T M (x-d) up to a constant
    r = Solve(prog)
    xs = r.GetSolution(x)
    return xs, float((xs - d) @ M @ (xs - d))


def separating_hyperplanes(C, d, obstacles):
    """Greedy IRIS step: closest obstacle first; add its tangent hyperplane; drop obstacles it already separates."""
    todo = dict(obstacles)
    planes, used = [], []
    M = np.linalg.inv(C @ C.T)
    while todo:
        best = min(todo, key=lambda k: closest_point_qp(C, d, todo[k])[1])
        xs, dist2 = closest_point_qp(C, d, todo[best])
        a = M @ (xs - d)
        a = a / np.linalg.norm(a)
        bb = a @ xs
        # a x <= bb keeps the ellipse side; sign: obstacle lies on a x >= bb, so use -a
        planes.append((a, bb))
        used.append(best)
        del todo[best]  # always drop the obstacle this plane was built from (solver tolerance)
        for k in list(todo):
            if np.all(todo[k] @ a >= bb - 1e-6):
                del todo[k]
    return planes, used


def inscribed_ellipse(A, b):
    """Maximum-volume ellipse {C u + d} inside {A x <= b} (C symmetric PSD): log-det maximisation."""
    from pydrake.solvers import MathematicalProgram, Solve
    prog = MathematicalProgram()
    Cv = prog.NewSymmetricContinuousVariables(2, "C")
    d = prog.NewContinuousVariables(2, "d")
    prog.AddPositiveSemidefiniteConstraint(Cv)
    for ai, bi in zip(A, b):
        prog.AddLorentzConeConstraint(np.concatenate([[bi - ai @ d], Cv @ ai]))
    prog.AddMaximizeLogDeterminantCost(Cv)
    r = Solve(prog)
    return r.GetSolution(Cv), r.GetSolution(d)


def iris(seed, iters=6):
    C, d = 0.05 * np.eye(2), np.array(seed, float)
    hist = []
    for k in range(iters):
        planes, used = separating_hyperplanes(C, d, OBSTACLES)
        A = np.array([p[0] for p in planes] + [[1, 0], [-1, 0], [0, 1], [0, -1]], float)
        b = np.array([p[1] for p in planes] + [BOUND] * 4, float)
        C, d = inscribed_ellipse(A, b)
        area = np.pi * np.linalg.det(C)
        hist.append((k + 1, len(planes), area, d.copy()))
    return A, b, C, d, hist, used


def polygon_area(A, b):
    """Area of {A x <= b} inside the bounding square, by clipping the square one half-space at a time."""
    poly = [np.array(p, float) for p in ((-BOUND, -BOUND), (BOUND, -BOUND), (BOUND, BOUND), (-BOUND, BOUND))]
    for a, bb in zip(A, b):
        out = []
        for k in range(len(poly)):
            p, q = poly[k], poly[(k + 1) % len(poly)]
            fp, fq = a @ p - bb, a @ q - bb
            if fp <= 0:
                out.append(p)
            if fp * fq < 0:
                out.append(p + (q - p) * fp / (fp - fq))
        poly = out
        if not poly:
            return 0.0
    x, y = np.array(poly).T
    return float(0.5 * abs(x @ np.roll(y, -1) - y @ np.roll(x, -1)))


def resample(path, n):
    """n+1 points spaced evenly along a polyline."""
    path = np.array(path)
    seg = np.r_[0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    t = np.linspace(0, seg[-1], n + 1)
    return np.column_stack([np.interp(t, seg, path[:, 0]), np.interp(t, seg, path[:, 1])])


def wall_demo():
    """Same start and goal, a wall with a gap: sampling vs optimization vs the two combined."""
    global WORLD
    import time
    WORLD = "wall"
    out = {}
    # 1. optimization from the straight line (passes through the wall)
    t0 = time.time()
    Q, solver = trajopt(guess("through", N=80), N=80)
    out["opt_line"] = (Q, time.time() - t0)
    # 2. RRT-Connect over many seeds
    its, lens, times = [], [], []
    first = None
    for sd in range(100):
        t0 = time.time()
        path, it = rrt_connect(np.random.default_rng(sd))
        times.append(time.time() - t0)
        if path is not None:
            its.append(it); lens.append(path_len(path))
            if first is None:
                first = path
    out["rrt"] = (len(its), its, lens, times, first)
    # 3. RRT-Connect path as the initial guess of the optimizer
    t0 = time.time()
    Q2, _ = trajopt(resample(first, 80), N=80)
    out["rrt_opt"] = (Q2, time.time() - t0)
    # 4. a hand-made guess through the gap
    gap = np.array([[-1.5, 0], [0, 1.5], [1.5, 0]])
    t0 = time.time()
    Q3, _ = trajopt(resample(gap, 80), N=80)
    out["gap_opt"] = (Q3, time.time() - t0)
    ok = lambda Q: Q is not None and all(free_segment(Q[i], Q[i + 1], res=0.005) for i in range(len(Q) - 1))
    out["valid"] = {k: ok(out[k][0]) for k in ("opt_line", "rrt_opt", "gap_opt")}
    WORLD = "diamond"
    return out


def run():
    rng = np.random.default_rng(0)
    print("== RRT vs RRT-Connect (100 runs, step 0.25, diamond obstacle)")
    for name in ("RRT", "RRT-Connect"):
        ok, iters, L0, L1 = 0, [], [], []
        for s in range(100):
            r = np.random.default_rng(s)
            if name == "RRT":
                nodes, parent, it = rrt(r)
                path = extract(nodes, parent) if it else None
            else:
                path, it = rrt_connect(r)
            if path is None:
                continue
            ok += 1
            iters.append(it)
            L0.append(path_len(path))
            L1.append(path_len(shortcut(path, r)))
        print(f"{name:12s} success {ok}/100  iterations median {int(np.median(iters))}  length raw {np.mean(L0):.2f} -> shortcut {np.mean(L1):.2f}")
    print("   reference: shortest collision-free path", f"{path_len(np.array([START, [-0.5, 0.5], [0.5, 0.5], GOAL])):.3f}", "(over the diamond's corner region, by symmetry also below)")
    print()
    print("== Shortcut figure (pixels read from the exercise image)")
    for a, b, old, new in shortcut_figure():
        print(f"  {a:>5s} -> {b:<5s} path {old:6.1f} px  direct {new:6.1f} px  saves {old-new:5.1f}")
    print()
    print("== RRT* parent choice (grid units read from the exercise image)")
    costs, c2 = rrt_star_figure()
    print("  cost-to-come of tree nodes:", {k: round(v, 3) for k, v in c2.items()})
    for k, v in costs.items():
        print(f"  parent {k:5s}: cost of q_new = {v:.3f}")
    print()
    print("== Trajectory optimization, N=16, different initial guesses (cost = sum |dq|^2)")
    ts, tg = np.array([-1.5, 0.4]), np.array([1.5, 0.4])
    for kind in ("up", "down", "through"):
        Q, solver = trajopt(guess(kind, ts, tg), ts, tg)
        if Q is None:
            print(f"  guess {kind:8s}: failed ({solver})")
            continue
        side = "above" if Q[:, 1].max() > 0.4 + abs(Q[:, 1].min() - 0.4) else "below"
        print(f"  guess {kind:8s}: ok ({solver}) cost {float(((Q[1:]-Q[:-1])**2).sum()):.4f} length {path_len(Q):.3f} goes {side}")
    print()
    print("== Wall with a gap: sampling vs optimization")
    w = wall_demo()
    Q, tt = w["opt_line"]
    print("  optimization from a straight-line guess (through the wall):", "no solution" if Q is None else f"solver says success, length {path_len(Q):.3f}, segments collision-free: {w['valid']['opt_line']}", f"({tt:.2f} s)")
    n, its, lens, times, first = w["rrt"]
    print(f"  RRT-Connect: success {n}/100, iterations median {int(np.median(its))}, length mean {np.mean(lens):.2f}, time median {1000*np.median(times):.0f} ms")
    Q2, tt = w["rrt_opt"]
    print("  optimization started from an RRT-Connect path:", "no solution" if Q2 is None else f"length {path_len(first):.3f} -> {path_len(Q2):.3f}, segments collision-free: {w['valid']['rrt_opt']}", f"({tt:.2f} s)")
    Q3, tt = w["gap_opt"]
    print("  optimization started from a path through the gap:", "no solution" if Q3 is None else f"length {path_len(Q3):.3f}, segments collision-free: {w['valid']['gap_opt']}", f"({tt:.2f} s)")
    print()
    print("== IRIS-style inflation")
    xs, dist2 = closest_point_qp(0.2 * np.eye(2), np.array([0.0, 0.0]), OBSTACLES["B"])
    print(f"  closest point of B to a circle r=0.2 at the origin: {np.round(xs,3)}, ellipse-metric distance {np.sqrt(dist2):.3f}")
    A, b, C, d, hist, used = iris([0.0, 0.0])
    for k, n, area, dd in hist:
        print(f"  iteration {k}: {n} hyperplanes, ellipse area {area:.3f}, center {np.round(dd,3)}")
    print(f"  polytope area {polygon_area(A, b):.3f}; obstacles touched by planes (closest first, last iteration): {used}")
    ok = True
    for k, V in OBSTACLES.items():
        # an obstacle is excluded when all its vertices lie on the far side of one of the planes
        ok &= any(np.all(V @ A[i] >= b[i] - 1e-6) for i in range(len(b)))
    print("  every obstacle lies fully outside some plane:", ok)


# ---------------- figures (inline SVG, classes from docs/_static/site.css) ----------------
def _pl(pts, style):
    return f'<polyline points="{" ".join(f"{x:.1f},{y:.1f}" for x, y in pts)}" style="fill:none;{style}"/>'


def svg_shortcut() -> str:
    path = ["start", "1", "2", "6", "8", "10", "11", "14", "15", "goal"]
    tree_extra = [("start", "3"), ("3", "4"), ("3", "5"), ("3", "7"), ("7", "13"), ("6", "9"), ("11", "12")]
    P = dict(SC)
    P.update({"3": (155, 189), "4": (139, 141), "5": (107, 262), "7": (227, 221), "13": (275, 277), "9": (227, 118), "12": (387, 197)})
    ox, oy, k = 10, 10, 1.0
    T = lambda n: (ox + P[n][0] * k, oy + P[n][1] * k)
    out = ['<svg class="hf" viewBox="0 0 620 310" role="img" aria-label="shortcutting 연습 문제 그림을 다시 그린 것">']
    out.append(f'<rect x="{ox+BOX[0]}" y="{oy+BOX[1]}" width="{BOX[2]-BOX[0]}" height="{BOX[3]-BOX[1]}" style="fill:color-mix(in srgb,var(--kit-c-danger) 30%,transparent);stroke:var(--kit-c-danger);stroke-width:1.5"/>')
    for a, b in tree_extra:
        out.append(_pl([T(a), T(b)], "stroke:var(--kit-faint);stroke-width:1.6"))
    out.append(_pl([T(n) for n in path], "stroke:var(--kit-accent);stroke-width:3.2"))
    for a, b, col in (("10", "14", "var(--kit-c-tip)"), ("2", "8", "var(--kit-c-tip)"), ("11", "15", "var(--kit-c-warn)")):
        out.append(_pl([T(a), T(b)], f"stroke:{col};stroke-width:2.6;stroke-dasharray:6 4"))
    for n in P:
        x, y = T(n)
        on = n in path
        fill = "var(--kit-accent)" if n in ("start", "goal") else ("var(--kit-fg)" if on else "var(--kit-muted)")
        out.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{6 if n in ("start","goal") else 4.5}" style="fill:{fill}"/>')
        label = {"start": "q_start", "goal": "q_goal"}.get(n, n)
        out.append(f'<text x="{x+7:.0f}" y="{y-7:.0f}" class="hf-t">{label}</text>')
    out.append('<text x="10" y="302" class="hf-t">갈색: RRT가 찾은 해 경로, 초록·주황 점선: 줄일 수 있는 지름길</text>')
    out.append("</svg>")
    return "".join(out)


def svg_rewire() -> str:
    q = {"start": (5, 0), "q1": (4, 1), "q2": (6, 1), "near": (4, 3), "new": (3, 3)}
    u, ox, oy = 62, 70, 30
    T = lambda n: (ox + q[n][0] * u - 90, oy + q[n][1] * u)
    out = ['<svg class="hf" viewBox="0 0 690 250" role="img" aria-label="RRT* 부모 선택">']
    for a, b in (("start", "q1"), ("start", "q2"), ("q1", "near")):
        out.append(_pl([T(a), T(b)], "stroke:var(--kit-muted);stroke-width:3"))
    out.append(_pl([T("near"), T("new")], "stroke:var(--kit-muted);stroke-width:3;stroke-dasharray:4 3"))
    out.append(_pl([T("start"), T("new")], "stroke:var(--kit-c-tip);stroke-width:3;stroke-dasharray:7 4"))
    out.append(_pl([T("q1"), T("new")], "stroke:var(--kit-c-warn);stroke-width:2.4;stroke-dasharray:3 3"))
    for n, lab in (("start", "q_start"), ("q1", "q₁"), ("q2", "q₂"), ("near", "q_near"), ("new", "q_new")):
        x, y = T(n)
        out.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="6" style="fill:var(--kit-fg)"/>')
        out.append(f'<text x="{x+9:.0f}" y="{y-8:.0f}" class="hf-t">{lab}</text>')
    costs, _ = rrt_star_figure()
    out.append('<text x="440" y="44" class="hf-h">q_new의 부모 후보 (한 칸 = 1)</text>')
    rows = [("hf-t", f"q_near 부모 (RRT) → {costs['near']:.3f}"),
            ("hf-t", f"q₂ 부모 → {costs['q2']:.3f}"),
            ("hf-key a", f"q₁ 부모 → {costs['q1']:.3f}"),
            ("hf-key pq", f"q_start 부모 → {costs['start']:.3f} (최소)")]
    for i, (c, l) in enumerate(rows):
        out.append(f'<text x="440" y="{74+i*24}" class="{c}">{l}</text>')
    out.append('<text x="440" y="190" class="hf-t">재배선: q_new를 q_start에</text>')
    out.append('<text x="440" y="212" class="hf-t">직접 이으면 가장 짧다</text>')
    out.append("</svg>")
    return "".join(out)


def svg_localmin() -> str:
    ts, tg = np.array([-1.5, 0.4]), np.array([1.5, 0.4])
    s_, ox, oy = 78, 150, 170
    T = lambda p: (ox + s_ * p[0], oy - s_ * p[1])
    out = ['<svg class="hf" viewBox="0 0 690 340" role="img" aria-label="초기값에 따라 갈리는 두 국소해">']
    dia = [T(p) for p in ((1, 0), (0, 1), (-1, 0), (0, -1))]
    out.append(f'<polygon points="{" ".join(f"{x:.1f},{y:.1f}" for x, y in dia)}" style="fill:color-mix(in srgb,var(--kit-c-danger) 30%,transparent);stroke:var(--kit-c-danger);stroke-width:1.5"/>')
    for kind, col, lab in (("up", "var(--kit-c-tip)", "위로 돌아간 해"), ("down", "var(--kit-c-warn)", "아래로 돌아간 해")):
        Q, _ = trajopt(guess(kind, ts, tg), ts, tg)
        out.append(_pl([T(p) for p in Q], f"stroke:{col};stroke-width:3"))
        for p in Q:
            x, y = T(p)
            out.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="2.8" style="fill:{col}"/>')
    for p, lab in ((ts, "시작"), (tg, "목표")):
        x, y = T(p)
        out.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="6" class="hf-pt k-ee"/><text x="{x+8:.0f}" y="{y-10:.0f}" class="hf-t">{lab}</text>')
    c = {}
    for kind in ("up", "down"):
        Q, _ = trajopt(guess(kind, ts, tg), ts, tg)
        c[kind] = (float(((Q[1:] - Q[:-1]) ** 2).sum()), path_len(Q))
    out.append('<text x="440" y="60" class="hf-h">같은 문제, 다른 초기값</text>')
    out.append(f'<text x="440" y="90" class="hf-key pq">■ 위쪽 초기값 → 비용 {c["up"][0]:.3f} (더 낮음)</text>')
    out.append(f'<text x="440" y="112" class="hf-key a" style="fill:var(--kit-c-warn)">■ 아래쪽 초기값 → 비용 {c["down"][0]:.3f} (국소 최소)</text>')
    out.append('<text x="440" y="146" class="hf-t">아래로 돌던 경로가 위로 넘어가려면</text>')
    out.append('<text x="440" y="168" class="hf-t">장애물을 통과(제약 위반)해야 해서</text>')
    out.append('<text x="440" y="190" class="hf-t">솔버가 넘어가지 못한다</text>')
    out.append("</svg>")
    return "".join(out)


def svg_iris() -> str:
    A, b, C, d, hist, used = iris([0.0, 0.0])
    s_, ox, oy = 72, 175, 160
    T = lambda p: (ox + s_ * p[0], oy - s_ * p[1])
    out = ['<svg class="hf" viewBox="0 0 690 320" role="img" aria-label="IRIS 한 번의 영역 확장">']
    out.append(f'<rect x="{ox-2*s_}" y="{oy-2*s_}" width="{4*s_}" height="{4*s_}" style="fill:none;stroke:var(--kit-faint);stroke-width:1.2;stroke-dasharray:4 3"/>')
    # polytope by clipping
    poly = [np.array(p, float) for p in ((-BOUND, -BOUND), (BOUND, -BOUND), (BOUND, BOUND), (-BOUND, BOUND))]
    for a_, bb in zip(A, b):
        o = []
        for k in range(len(poly)):
            p, q = poly[k], poly[(k + 1) % len(poly)]
            fp, fq = a_ @ p - bb, a_ @ q - bb
            if fp <= 0:
                o.append(p)
            if fp * fq < 0:
                o.append(p + (q - p) * fp / (fp - fq))
        poly = o
    out.append(f'<polygon points="{" ".join(f"{x:.1f},{y:.1f}" for x, y in [T(p) for p in poly])}" style="fill:color-mix(in srgb,var(--kit-accent) 14%,transparent);stroke:var(--kit-accent);stroke-width:2"/>')
    for k, V in OBSTACLES.items():
        out.append(f'<polygon points="{" ".join(f"{x:.1f},{y:.1f}" for x, y in [T(p) for p in V])}" style="fill:color-mix(in srgb,var(--kit-c-danger) 30%,transparent);stroke:var(--kit-c-danger);stroke-width:1.5"/>')
        cx, cy = T(V.mean(0))
        out.append(f'<text x="{cx:.0f}" y="{cy+4:.0f}" text-anchor="middle" class="hf-t">{k}</text>')
    th = np.linspace(0, 2 * np.pi, 80)
    E = [T(C @ np.array([np.cos(t), np.sin(t)]) + d) for t in th]
    out.append(f'<polygon points="{" ".join(f"{x:.1f},{y:.1f}" for x, y in E)}" style="fill:color-mix(in srgb,var(--kit-c-tip) 25%,transparent);stroke:var(--kit-c-tip);stroke-width:2"/>')
    x0, y0 = T([0.0, 0.0])
    out.append(f'<circle cx="{x0:.1f}" cy="{y0:.1f}" r="4" style="fill:var(--kit-fg)"/><text x="{x0+6:.0f}" y="{y0+16:.0f}" class="hf-t">seed</text>')
    out.append('<text x="410" y="60" class="hf-h">IRIS로 키운 영역 (2차원)</text>')
    rows = [("hf-key pq", "■ 초록 타원: 영역 안의 최대 부피 타원"), ("hf-key a", f"■ 갈색 다각형: 초평면 4개 영역 ({polygon_area(A, b):.2f})"), ("hf-key rl", "■ 빨강: 장애물 A, B, C, D"), ("hf-t", ""),
            ("hf-t", "반복마다: 장애물까지 최근접점 QP → 접평면"), ("hf-t", "→ 그 평면들 안의 최대 타원 → 다시 반복"), ("hf-t", f"타원 면적: " + " → ".join(f"{h[2]:.2f}" for h in hist[:4]) + " …")]
    for i, (c, l) in enumerate(rows):
        out.append(f'<text x="410" y="{88+i*22}" class="{c}">{l}</text>')
    out.append("</svg>")
    return "".join(out)


def svg_wall() -> str:
    w = wall_demo()
    global WORLD
    s_, ox, oy = 78, 190, 170
    T = lambda p: (ox + s_ * p[0], oy - s_ * p[1])
    out = ['<svg class="hf" viewBox="0 0 690 340" role="img" aria-label="틈이 있는 벽에서 샘플링, 최적화, 둘의 결합">']
    out.append(f'<rect x="{ox-2*s_}" y="{oy-2*s_}" width="{4*s_}" height="{4*s_}" style="fill:none;stroke:var(--kit-faint);stroke-width:1.2;stroke-dasharray:4 3"/>')
    for x0, x1, y0, y1 in WALL:
        a, b = T((x0, y1)), T((x1, y0))
        out.append(f'<rect x="{a[0]:.1f}" y="{a[1]:.1f}" width="{b[0]-a[0]:.1f}" height="{b[1]-a[1]:.1f}" style="fill:color-mix(in srgb,var(--kit-c-danger) 35%,transparent);stroke:var(--kit-c-danger);stroke-width:1.5"/>')
    n, its, lens, times, first = w["rrt"]
    out.append(_pl([T(p) for p in first], "stroke:var(--kit-muted);stroke-width:2;stroke-dasharray:3 3"))
    out.append(_pl([T(p) for p in w["opt_line"][0]], "stroke:var(--kit-c-danger);stroke-width:2.6;stroke-dasharray:6 4"))
    out.append(_pl([T(p) for p in w["rrt_opt"][0]], "stroke:var(--kit-c-tip);stroke-width:3.4"))
    for p, lab in ((START, "시작"), (GOAL, "목표")):
        x, y = T(p)
        out.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="6" class="hf-pt k-ee"/><text x="{x+8:.0f}" y="{y+20:.0f}" class="hf-t">{lab}</text>')
    L = lambda k: path_len(w[k][0])
    out.append('<text x="420" y="50" class="hf-h">같은 문제를 세 가지로</text>')
    rows = [("hf-key rl", f"■ 직선에서 시작한 최적화: {L('opt_line'):.2f}, 벽을 뚫음"),
            ("hf-t", "   (벽 양옆 웨이포인트 사이를 건너뜀)"),
            ("hf-t", f"··· RRT-Connect 경로: {path_len(first):.2f}, 울퉁불퉁"),
            ("hf-key pq", f"■ RRT 경로에서 시작한 최적화: {L('rrt_opt'):.2f}"),
            ("hf-t", "   (벽 통과 없음, 틈 모서리를 스치며 지남)")]
    for i, (c, l) in enumerate(rows):
        out.append(f'<text x="420" y="{80+i*22}" class="{c}">{l}</text>')
    out.append('<text x="420" y="214" class="hf-t">샘플링: 해를 찾는 일(전역)</text><text x="420" y="236" class="hf-t">최적화: 해를 다듬는 일(국소)</text>')
    out.append("</svg>")
    return "".join(out)


if __name__ == "__main__":
    import pathlib
    import sys

    if "--figs" in sys.argv:
        outdir = pathlib.Path(sys.argv[sys.argv.index("--figs") + 1])
        outdir.mkdir(parents=True, exist_ok=True)
        for name, fn in [("shortcut", svg_shortcut), ("rewire", svg_rewire), ("localmin", svg_localmin), ("iris", svg_iris), ("wall", svg_wall)]:
            (outdir / f"{name}.svg.html").write_text(fn())
            print("wrote", outdir / f"{name}.svg.html")
    else:
        run()
