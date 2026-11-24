"""B-spline basics and the augmented Lagrangian method: numerical checks and doc figures.

Companion to docs `docs/background/15_bspline_and_augmented_lagrangian.md`. Pure numpy.

    PYTHONPATH=. python scripts/bspline_al.py              # numerical checks (printed in the doc)
    PYTHONPATH=. python scripts/bspline_al.py --figs DIR   # write SVG figures for the docs
"""

from __future__ import annotations

import numpy as np

DEG = 3
CTRL = np.array([[0.0, 0.0], [0.9, 2.3], [2.4, 2.7], [3.3, 0.5], [4.6, 0.9], [5.4, 2.5]])


def clamped_knots(n_ctrl: int, deg: int = DEG) -> np.ndarray:
    """Clamped uniform knots on [0, 1]: the curve starts at the first and ends at the last control point."""
    inner = np.linspace(0, 1, n_ctrl - deg + 1)[1:-1]
    return np.concatenate([np.zeros(deg + 1), inner, np.ones(deg + 1)])


def basis(i: int, k: int, t: np.ndarray, s: float) -> float:
    """Cox-de Boor recursion: B_{i,k}(s) for knot vector t (degree k)."""
    if k == 0:
        return 1.0 if (t[i] <= s < t[i + 1]) or (s == t[-1] and t[i] < t[i + 1] == t[-1]) else 0.0
    a = 0.0 if t[i + k] == t[i] else (s - t[i]) / (t[i + k] - t[i]) * basis(i, k - 1, t, s)
    b = 0.0 if t[i + k + 1] == t[i + 1] else (t[i + k + 1] - s) / (t[i + k + 1] - t[i + 1]) * basis(i + 1, k - 1, t, s)
    return a + b


def basis_all(t: np.ndarray, k: int, s: float) -> np.ndarray:
    return np.array([basis(i, k, t, s) for i in range(len(t) - k - 1)])


def curve(P: np.ndarray, s: float, deg: int = DEG) -> np.ndarray:
    t = clamped_knots(len(P), deg)
    return basis_all(t, deg, s) @ P


def deriv_ctrl(P: np.ndarray, t: np.ndarray, k: int):
    """Control points and knots of the derivative: Q_i = k (P_{i+1}-P_i) / (t_{i+k+1}-t_{i+1}); degree k-1."""
    Q = np.array([k * (P[i + 1] - P[i]) / (t[i + k + 1] - t[i + 1]) for i in range(len(P) - 1)])
    return Q, t[1:-1]


def in_hull(pt, pts) -> bool:
    """Point-in-convex-hull test by LP-free check: solve for convex weights with scipy if present, else by SVD NNLS."""
    from scipy.optimize import nnls
    A = np.vstack([np.asarray(pts).T, np.ones(len(pts))])
    b = np.append(pt, 1.0)
    w, res = nnls(A, b)
    return res < 1e-9


def spline_checks():
    t = clamped_knots(len(CTRL))
    ss = np.linspace(0, 1, 201)
    B = np.array([basis_all(t, DEG, s) for s in ss])
    print("knots:", np.round(t, 3))
    print("partition of unity (max |sum-1|):", float(np.abs(B.sum(1) - 1).max()))
    print("non-negative:", bool((B >= -1e-12).all()), "| max nonzero bases at one s:", int(((B > 1e-12).sum(1)).max()), f"(degree+1 = {DEG + 1})")
    print("endpoints interpolate:", np.allclose(curve(CTRL, 0), CTRL[0]), np.allclose(curve(CTRL, 1), CTRL[-1]))
    # convex hull property of the active control points
    ok = True
    for s in ss[:-1]:
        act = np.where(basis_all(t, DEG, s) > 1e-12)[0]
        ok &= in_hull(curve(CTRL, s), CTRL[act])
    print("curve inside hull of its active control points at all samples:", bool(ok))
    # derivative is a B-spline of degree k-1 with coefficients linear in P
    Q, tq = deriv_ctrl(CTRL, t, DEG)
    h = 1e-6
    err = 0.0
    for s in (0.1, 0.37, 0.5, 0.8):
        fd = (curve(CTRL, s + h) - curve(CTRL, s - h)) / (2 * h)
        dv = basis_all(tq, DEG - 1, s) @ Q
        err = max(err, float(np.abs(fd - dv).max()))
    print("derivative control points vs finite difference (max err):", f"{err:.1e}")
    # velocity bound for all t from control points only: max |Q_i| bounds |r'(s)|
    speed = max(np.linalg.norm(basis_all(tq, DEG - 1, s) @ Q) for s in ss[:-1])
    print(f"true max |r'(s)| = {speed:.3f} <= max |Q_i| = {np.linalg.norm(Q, axis=1).max():.3f} (always a bound; equal here because the maximum sits at the clamped end)")


def al_example(rho=10.0, iters=8):
    """min x1^2 + 2 x2^2  s.t. x1 + x2 = 1.  Exact: x = (2/3, 1/3), lambda = -4/3 (for L = f + lambda*h)."""
    H = np.diag([2.0, 4.0])
    g = np.zeros(2)
    a = np.array([1.0, 1.0])
    xs = np.array([2 / 3, 1 / 3])
    rows = []
    x, lam = np.zeros(2), 0.0
    for k in range(iters):
        # argmin_x f + lam*(a.x-1) + rho/2 (a.x-1)^2  (quadratic: solve the linear system)
        x = np.linalg.solve(H + rho * np.outer(a, a), -lam * a + rho * a)
        h = a @ x - 1
        lam = lam + rho * h
        rows.append((k + 1, x.copy(), h, lam))
    pen = []
    for r in (1.0, 10.0, 100.0, 1000.0):
        xp = np.linalg.solve(H + r * np.outer(a, a), r * a)
        pen.append((r, xp, a @ xp - 1))
    return rows, pen, xs


def al_checks():
    rows, pen, xs = al_example()
    print("exact solution x* = (0.6667, 0.3333), lambda* = -1.3333")
    print("penalty only (no multiplier):")
    for r, x, h in pen:
        print(f"  rho={r:7.0f}  x=({x[0]:.4f}, {x[1]:.4f})  violation h={h:+.2e}")
    print("augmented Lagrangian, rho=10 fixed:")
    for k, x, h, lam in rows:
        print(f"  iter {k}  x=({x[0]:.5f}, {x[1]:.5f})  h={h:+.2e}  lambda={lam:+.5f}")


# ---------------- figures ----------------
def _fmt(p):
    return f"{p[0]:.1f},{p[1]:.1f}"


def svg_bspline() -> str:
    t = clamped_knots(len(CTRL))
    sx, sy, ox, oy = 54, 60, 36, 215
    T = lambda p: (ox + sx * p[0], oy - sy * p[1])
    out = ['<svg class="hf" viewBox="0 0 690 270" role="img" aria-label="B-스플라인 곡선과 제어점, 활성 제어점의 볼록 껍질">']
    s0 = 0.45
    act = np.where(basis_all(t, DEG, s0) > 1e-12)[0]
    hull = [T(CTRL[i]) for i in (act[0], act[1], act[3], act[2])]  # a simple quad order for these points
    out.append(f'<polygon points="{" ".join(_fmt(p) for p in hull)}" style="fill:color-mix(in srgb,var(--kit-accent) 16%,transparent);stroke:var(--kit-accent);stroke-width:1.2;stroke-dasharray:4 3"/>')
    out.append(f'<polyline points="{" ".join(_fmt(T(p)) for p in CTRL)}" style="fill:none;stroke:var(--kit-faint);stroke-width:1.4;stroke-dasharray:5 3"/>')
    pts = [T(curve(CTRL, s)) for s in np.linspace(0, 1, 120)]
    out.append(f'<polyline points="{" ".join(_fmt(p) for p in pts)}" class="hf-link" style="stroke-width:4"/>')
    for i, p in enumerate(CTRL):
        x, y = T(p)
        on = i in act
        out.append(f'<rect x="{x-5:.1f}" y="{y-5:.1f}" width="10" height="10" style="fill:{"var(--kit-c-danger)" if on else "var(--kit-subtle)"};stroke:var(--kit-fg);stroke-width:1.6"/>')
        out.append(f'<text x="{x+9:.0f}" y="{y-8:.0f}" class="hf-t">P{i}</text>')
    x, y = T(curve(CTRL, s0))
    out.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="6" class="hf-pt k-ee"/>')
    out.append(f'<text x="{x-12:.0f}" y="{y+22:.0f}" text-anchor="end" class="hf-h">r(0.45)</text>')
    rows = [("hf-h", "3차 B-스플라인, 제어점 6개"), ("hf-t", "굵은 선: 곡선 r(s), 회색 점선: 제어점 연결"), ("hf-t", ""),
            ("hf-key rl", "■ 빨간 제어점 4개 (s = 0.45에서"), ("hf-t", "   0이 아닌 기저 4개: P1~P4만 영향)"), ("hf-key a", "갈색 점선: 그 4개의 볼록 껍질"), ("hf-t", "   곡선 위 점은 항상 이 안에 있음"), ("hf-t", ""),
            ("hf-t", "첫·끝 제어점(P0, P5)은 곡선이 지남")]
    for i, (c, l) in enumerate(rows):
        out.append(f'<text x="400" y="{40+i*22}" class="{c}">{l}</text>')
    out.append("</svg>")
    return "".join(out)


def svg_basis() -> str:
    t = clamped_knots(len(CTRL))
    n = len(CTRL)
    x0, x1, y0, y1 = 50, 650, 190, 40
    cols = ["var(--kit-accent)", "var(--kit-c-tip)", "var(--kit-c-danger)", "var(--kit-c-warn)", "var(--kit-muted)", "var(--kit-fg)"]
    ss = np.linspace(0, 1, 301)
    B = np.array([basis_all(t, DEG, s) for s in ss])
    out = ['<svg class="hf" viewBox="0 0 690 250" role="img" aria-label="3차 B-스플라인의 기저 함수">']
    out.append(f'<line x1="{x0}" y1="{y0}" x2="{x1}" y2="{y0}" class="hf-axis"/>')
    out.append(f'<line x1="{x0}" y1="{y1}" x2="{x1}" y2="{y1}" class="hf-axis"/>')
    out.append(f'<text x="{x0-8}" y="{y1+4}" text-anchor="end" class="hf-t">1</text><text x="{x0-8}" y="{y0+4}" text-anchor="end" class="hf-t">0</text>')
    for k in sorted(set(np.round(t, 6))):
        out.append(f'<line x1="{x0+(x1-x0)*k:.1f}" y1="{y0}" x2="{x0+(x1-x0)*k:.1f}" y2="{y0+5}" style="stroke:var(--kit-muted);stroke-width:1.3"/>')
        out.append(f'<text x="{x0+(x1-x0)*k:.1f}" y="{y0+20}" text-anchor="middle" class="hf-t">{k:.2f}</text>')
    for i in range(n):
        pts = " ".join(f"{x0+(x1-x0)*s:.1f},{y0-(y0-y1)*b:.1f}" for s, b in zip(ss, B[:, i]))
        out.append(f'<polyline points="{pts}" style="fill:none;stroke:{cols[i]};stroke-width:2.4"/>')
        j = int(np.argmax(B[:, i]))
        out.append(f'<text x="{x0+(x1-x0)*ss[j]:.0f}" y="{y0-(y0-y1)*B[j,i]-6:.0f}" text-anchor="middle" class="hf-t" style="fill:{cols[i]}">B{i}</text>')
    out.append('<text x="350" y="232" text-anchor="middle" class="hf-t">s (눈금은 매듭): 어느 s에서도 0이 아닌 기저는 최대 4개이고 합은 1</text>')
    out.append("</svg>")
    return "".join(out)


if __name__ == "__main__":
    import pathlib
    import sys

    if "--figs" not in sys.argv:
        spline_checks()
        print()
        al_checks()
    else:
        outdir = pathlib.Path(sys.argv[sys.argv.index("--figs") + 1])
        outdir.mkdir(parents=True, exist_ok=True)
        for name, fn in [("bspline", svg_bspline), ("basis", svg_basis)]:
            (outdir / f"{name}.svg.html").write_text(fn())
            print("wrote", outdir / f"{name}.svg.html")
