"""Antipodal points as critical points of E = 1/2 * kappa * ||p(t1) - p(t2)||^2.

Numerical companion to docs `docs/background/08_antipodal_hessian.md`. Pure numpy/scipy (no Drake).
Shape model follows MIT Robotic Manipulation, Analytic Antipodal Grasps exercise
(hypercycloidal gear); the ellipse/circle cases are closed-form sanity checks.

    PYTHONPATH=. python scripts/antipodal_hessian.py              # numerical checks
    PYTHONPATH=. python scripts/antipodal_hessian.py --figs DIR   # write SVG figures for the docs
"""

from __future__ import annotations

import numpy as np
from scipy.optimize import root

KAPPA = 2.0  # same value the exercise suggests


# ---------- shapes: p(t) -> R^2 ----------
def gear(t: float) -> np.ndarray:
    a = np.arctan(np.sin(-9 * t) / ((4 / 3) - np.cos(-9 * t)))
    x = 10 * np.cos(t) - 1.5 * np.cos(t + a) - 0.75 * np.cos(10 * t)
    y = -10 * np.sin(t) + 1.5 * np.sin(t + a) + 0.75 * np.sin(10 * t)
    return np.array([x, y])


def ellipse(a: float, b: float):
    return lambda t: np.array([a * np.cos(t), b * np.sin(t)])


# ---------- derivatives (central differences; enough for figures/tests) ----------
def d1(p, t, h=1e-5):
    return (p(t + h) - p(t - h)) / (2 * h)


def d2(p, t, h=1e-4):
    return (p(t + h) - 2 * p(t) + p(t - h)) / h**2


def energy(p, t):
    d = p(t[0]) - p(t[1])
    return 0.5 * KAPPA * d @ d


def grad(p, t):
    d = p(t[0]) - p(t[1])
    return KAPPA * np.array([d @ d1(p, t[0]), -d @ d1(p, t[1])])


def hessian(p, t):
    """Analytic Hessian of E w.r.t. (t1, t2)."""
    d = p(t[0]) - p(t[1])
    v1, v2 = d1(p, t[0]), d1(p, t[1])
    a1, a2 = d2(p, t[0]), d2(p, t[1])
    return KAPPA * np.array(
        [[v1 @ v1 + d @ a1, -v1 @ v2], [-v1 @ v2, v2 @ v2 - d @ a2]]
    )


def hessian_fd(p, t, h=1e-4):
    """Finite-difference Hessian of E, to cross-check `hessian`."""
    H = np.zeros((2, 2))
    for i in range(2):
        for j in range(2):
            e_i, e_j = np.eye(2)[i] * h, np.eye(2)[j] * h
            H[i, j] = (
                energy(p, t + e_i + e_j)
                - energy(p, t + e_i - e_j)
                - energy(p, t - e_i + e_j)
                + energy(p, t - e_i - e_j)
            ) / (4 * h * h)
    return H


def classify(eig: np.ndarray, tol=1e-6) -> str:
    if (eig > tol).all():
        return "min"
    if (eig < -tol).all():
        return "max"
    if (eig > tol).any() and (eig < -tol).any():
        return "saddle"
    return "degenerate"


def inside(poly: np.ndarray, pt: np.ndarray) -> bool:
    """Ray-casting point-in-polygon."""
    x, y = pt
    c = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            c = not c
    return c


def find_pairs(p, n_starts=400, seed=0, eps=1e-3, min_sep=0.2):
    """Roots of grad E = 0 with t1 > t2, distinct points, chord through the interior."""
    rng = np.random.default_rng(seed)
    poly = np.array([p(t) for t in np.linspace(0, 2 * np.pi, 2000, endpoint=False)])
    found = []
    for _ in range(n_starts):
        t0 = rng.uniform(0, 2 * np.pi - eps, 2)
        sol = root(lambda t: grad(p, t), t0, tol=1e-12)
        if not sol.success:
            continue
        t = sol.x % (2 * np.pi)
        if t[0] < t[1]:
            t = t[::-1]
        p1, p2 = p(t[0]), p(t[1])
        if np.linalg.norm(p1 - p2) < min_sep or np.linalg.norm(grad(p, t)) > 1e-6:
            continue
        if not inside(poly, 0.5 * (p1 + p2)):
            continue
        if any(np.linalg.norm(t - f["t"]) < 1e-4 for f in found):
            continue
        eig = np.linalg.eigvalsh(hessian(p, t))
        found.append({"t": t, "eig": eig, "kind": classify(eig), "len": np.linalg.norm(p1 - p2)})
    return found


def run_checks():
    # 1) closed-form checks on an ellipse a=2, b=1
    p = ellipse(2.0, 1.0)
    for name, t in [("major axis", [0.0, np.pi]), ("minor axis", [np.pi / 2, 3 * np.pi / 2])]:
        t = np.array(t)
        H = hessian(p, t)
        print(f"ellipse {name}: |grad|={np.linalg.norm(grad(p, t)):.2e}  "
              f"eig={np.linalg.eigvalsh(H).round(3)}  fd-eig={np.linalg.eigvalsh(hessian_fd(p, t)).round(2)}"
              f"  -> {classify(np.linalg.eigvalsh(H))}")
    pc = ellipse(1.0, 1.0)
    print("circle:", np.linalg.eigvalsh(hessian(pc, np.array([0.3, 0.3 + np.pi]))).round(3))

    # 2) gear: all antipodal pairs found by random restarts
    sols = find_pairs(gear)
    print(f"gear: {len(sols)} distinct pairs")
    for k in ("min", "max", "saddle", "degenerate"):
        ks = [s for s in sols if s["kind"] == k]
        print(f"  {k}: {len(ks)}")
    for s in sorted(sols, key=lambda s: s["kind"])[:40]:
        t = s["t"]
        Hfd = np.linalg.eigvalsh(hessian_fd(gear, t))
        print(f"  {s['kind']:7s} t={t.round(4)} len={s['len']:.3f} eig={s['eig'].round(2)} fd={Hfd.round(1)}")


# =====================================================================
# Figures: inline SVG snippets (styled by `.hf-*` classes in custom.css)
#   PYTHONPATH=. python scripts/antipodal_hessian.py --figs OUT_DIR
# =====================================================================
def signed_convex_curvature(p, t, orient):
    """Curvature of the boundary at p(t); > 0 means locally convex (as seen from the object)."""
    v, a = d1(p, t), d2(p, t)
    return orient * (v[0] * a[1] - v[1] * a[0]) / np.linalg.norm(v) ** 3


def outline_orientation(p):
    pts = np.array([p(t) for t in np.linspace(0, 2 * np.pi, 800, endpoint=False)])
    area2 = np.sum(pts[:, 0] * np.roll(pts[:, 1], -1) - np.roll(pts[:, 0], -1) * pts[:, 1])
    return 1.0 if area2 > 0 else -1.0  # +1 = counter-clockwise


def _poly(pts):
    return " ".join(f"{x:.1f},{y:.1f}" for x, y in pts)


def svg_landscapes() -> str:
    """Three contour panels: min / max / saddle of a 2-variable function."""
    W, Hh, pw = 690, 240, 230
    out = [f'<svg class="hf" viewBox="0 0 {W} {Hh}" role="img" aria-label="Hessian 고유값 부호별 에너지 지형">']
    out.append('<defs><marker id="hf-ar" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" class="hf-head"/></marker></defs>')
    panels = [
        ("local minimum", "최소", "λ₁ > 0, λ₂ > 0", "k-min", (1.0, 1.0 * 2.2)),
        ("local maximum", "최대", "λ₁ < 0, λ₂ < 0", "k-max", (-1.0, -1.0 * 2.2)),
        ("saddle", "안장", "λ₁ > 0, λ₂ < 0", "k-saddle", (1.0, -1.0 * 2.2)),
    ]
    for i, (en, ko, sign, cls, (l1, l2)) in enumerate(panels):
        cx, cy = i * pw + pw / 2, 100
        R = 78  # half size of the plot box
        out.append(f'<g transform="translate({cx:.0f},{cy})">')
        out.append(f'<clipPath id="hf-c{i}"><rect x="{-R}" y="{-R}" width="{2*R}" height="{2*R}" rx="8"/></clipPath>')
        out.append(f'<rect x="{-R}" y="{-R}" width="{2*R}" height="{2*R}" rx="8" class="hf-box"/>')
        out.append(f'<g clip-path="url(#hf-c{i})">')
        s = 22.0  # px per unit
        if l1 * l2 > 0:  # concentric ellipses
            for lv in (0.5, 1.2, 2.2, 3.6):
                c = lv * np.sign(l1)
                rx, ry = np.sqrt(2 * c / l1), np.sqrt(2 * c / l2)
                out.append(f'<ellipse cx="0" cy="0" rx="{rx*s:.1f}" ry="{ry*s:.1f}" class="hf-contour"/>')
        else:  # hyperbolas  f = 0.5*(l1 x^2 + l2 y^2) = c
            ys = np.linspace(-4.5, 4.5, 90)
            for lv in (0.4, 1.2, 2.4):
                for c in (lv, -lv):
                    if c > 0:  # x = ±sqrt((2c - l2 y^2)/l1)
                        xs = np.sqrt(np.maximum((2 * c - l2 * ys**2) / l1, 0))
                        for sg in (1, -1):
                            out.append(f'<polyline points="{_poly(zip(sg*xs*s, -ys*s))}" class="hf-contour"/>')
                    else:      # y = ±sqrt((2c - l1 x^2)/l2)
                        xs = np.linspace(-4.5, 4.5, 90)
                        yv = np.sqrt(np.maximum((2 * c - l1 * xs**2) / l2, 0))
                        for sg in (1, -1):
                            out.append(f'<polyline points="{_poly(zip(xs*s, -sg*yv*s))}" class="hf-contour"/>')
        out.append("</g>")
        # eigen-directions
        out.append(f'<line x1="{-R+10}" y1="0" x2="{R-10}" y2="0" class="hf-axis" marker-end="url(#hf-ar)"/>')
        out.append(f'<line x1="0" y1="{R-10}" x2="0" y2="{-R+10}" class="hf-axis" marker-end="url(#hf-ar)"/>')
        out.append(f'<text x="{R-8}" y="-6" text-anchor="end" class="hf-t">v₁</text>')
        out.append(f'<text x="8" y="{-R+16}" class="hf-t">v₂</text>')
        out.append(f'<circle cx="0" cy="0" r="5" class="hf-pt {cls}"/>')
        out.append("</g>")
        out.append(f'<text x="{cx:.0f}" y="{cy+R+22}" text-anchor="middle" class="hf-h">{ko} <tspan class="hf-t">({en})</tspan></text>')
        out.append(f'<text x="{cx:.0f}" y="{cy+R+40}" text-anchor="middle" class="hf-t">{sign}</text>')
    out.append("</svg>")
    return "".join(out)


def _ellipse_panel(ox, oy, kind, a=2.0, b=1.0, s=44.0):
    """Ellipse with a chord at the major-axis (max) or minor-axis (saddle) pair."""
    cls = "k-max" if kind == "max" else "k-saddle"
    g = [f'<g transform="translate({ox},{oy})">']
    g.append(f'<ellipse cx="0" cy="0" rx="{a*s}" ry="{b*s}" class="hf-shape"/>')
    if kind == "max":
        p1, p2 = (a * s, 0), (-a * s, 0)
        arrows = [  # (point, vector, label offset, label)
            (p1, (0, -34), "+" ), (p2, (0, 34), "+"),
        ]
    else:
        p1, p2 = (0, -b * s), (0, b * s)
    g.append(f'<line x1="{p1[0]:.1f}" y1="{p1[1]:.1f}" x2="{p2[0]:.1f}" y2="{p2[1]:.1f}" class="hf-chord"/>')
    for p in (p1, p2):
        g.append(f'<circle cx="{p[0]:.1f}" cy="{p[1]:.1f}" r="5.5" class="hf-pt {cls}"/>')
    return g, p1, p2


def svg_ellipse() -> str:
    a, b, s = 2.0, 1.0, 44.0
    W, Hh = 690, 285
    out = [f'<svg class="hf" viewBox="0 0 {W} {Hh}" role="img" aria-label="타원에서 접점 쌍과 Hessian 고유방향">']
    out.append('<defs><marker id="hf-arp" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" class="hf-headp"/></marker><marker id="hf-arn" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" class="hf-headn"/></marker></defs>')
    # --- left: major axis pair (max). tangent dirs at the contacts: p1 at (a,0) moves up(-y svg) for t increasing.
    g, p1, p2 = _ellipse_panel(175, 140, "max")
    # rotate (t1,t2 both +): p1 up, p2 down ; slide (t1 +, t2 -): both up
    g.append(f'<line x1="{p1[0]+26}" y1="{p1[1]}" x2="{p1[0]+26}" y2="{p1[1]-34}" class="hf-move neg" marker-end="url(#hf-arn)"/>')
    g.append(f'<line x1="{p2[0]-26}" y1="{p2[1]}" x2="{p2[0]-26}" y2="{p2[1]+34}" class="hf-move neg" marker-end="url(#hf-arn)"/>')
    g.append(f'<line x1="{p1[0]+48}" y1="{p1[1]}" x2="{p1[0]+48}" y2="{p1[1]-34}" class="hf-move neg2" marker-end="url(#hf-arn)"/>')
    g.append(f'<line x1="{p2[0]-48}" y1="{p2[1]}" x2="{p2[0]-48}" y2="{p2[1]-34}" class="hf-move neg2" marker-end="url(#hf-arn)"/>')
    g.append('</g>')
    out += g
    out.append('<text x="175" y="246" text-anchor="middle" class="hf-h">장축 끝 쌍 → 최대</text>')
    out.append('<text x="175" y="266" text-anchor="middle" class="hf-t">λ = −12, −16 (둘 다 음수)</text>')
    # --- right: minor axis pair (saddle)
    g, p1, p2 = _ellipse_panel(515, 140, "saddle")
    # rotate (+): p1 (top in svg is -y: here p1=(0,-b*s) is top) moves left? t increasing CCW: at top (0,b) moves left.
    g.append(f'<line x1="{p1[0]+4}" y1="{p1[1]-22}" x2="{p1[0]-40}" y2="{p1[1]-22}" class="hf-move pos" marker-end="url(#hf-arp)"/>')
    g.append(f'<line x1="{p2[0]-4}" y1="{p2[1]+22}" x2="{p2[0]+40}" y2="{p2[1]+22}" class="hf-move pos" marker-end="url(#hf-arp)"/>')
    g.append(f'<line x1="{p1[0]+4}" y1="{p1[1]-44}" x2="{p1[0]-40}" y2="{p1[1]-44}" class="hf-move neg2" marker-end="url(#hf-arn)"/>')
    g.append(f'<line x1="{p2[0]+4}" y1="{p2[1]+44}" x2="{p2[0]-40}" y2="{p2[1]+44}" class="hf-move neg2" marker-end="url(#hf-arn)"/>')
    g.append('</g>')
    out += g
    out.append('<text x="515" y="246" text-anchor="middle" class="hf-h">단축 끝 쌍 → 안장</text>')
    out.append('<text x="515" y="266" text-anchor="middle" class="hf-t">λ = +12, −4 (부호가 섞임)</text>')
    # legend
    out.append('<g transform="translate(24,18)"><line x1="0" y1="0" x2="26" y2="0" class="hf-move pos" marker-end="url(#hf-arp)"/><text x="34" y="4" class="hf-t">현이 길어지는 움직임 (λ &gt; 0)</text></g>')
    out.append('<g transform="translate(264,18)"><line x1="0" y1="0" x2="26" y2="0" class="hf-move neg" marker-end="url(#hf-arn)"/><text x="34" y="4" class="hf-t">짧아지는 움직임 (λ &lt; 0)</text></g>')
    out.append("</svg>")
    return "".join(out)


def svg_gear_cases() -> str:
    cases = [
        ("최소", "k-min", [2.07, 0.024], "+, +"),
        ("최대", "k-max", [3.9488, 1.6363], "−, −"),
        ("안장", "k-saddle", [5.5851, 2.4435], "−, +"),
    ]
    pts = np.array([gear(t) for t in np.linspace(0, 2 * np.pi, 700)])
    s, pw = 8.6, 230
    out = [f'<svg class="hf" viewBox="0 0 690 270" role="img" aria-label="톱니바퀴의 접점 쌍 사례">']
    for i, (name, cls, t, sg) in enumerate(cases):
        cx, cy = i * pw + pw / 2, 118
        pp = [(cx + x * s, cy - y * s) for x, y in pts]
        out.append(f'<polygon points="{_poly(pp)}" class="hf-shape"/>')
        a, b = gear(t[0]), gear(t[1])
        A, B = (cx + a[0] * s, cy - a[1] * s), (cx + b[0] * s, cy - b[1] * s)
        out.append(f'<line x1="{A[0]:.1f}" y1="{A[1]:.1f}" x2="{B[0]:.1f}" y2="{B[1]:.1f}" class="hf-chord"/>')
        for P in (A, B):
            out.append(f'<circle cx="{P[0]:.1f}" cy="{P[1]:.1f}" r="5.5" class="hf-pt {cls}"/>')
        eig = np.linalg.eigvalsh(hessian(gear, np.array(t)))
        out.append(f'<text x="{cx:.0f}" y="{cy+122}" text-anchor="middle" class="hf-h">{name}</text>')
        out.append(f'<text x="{cx:.0f}" y="{cy+140}" text-anchor="middle" class="hf-t">고유값 부호 ({sg})</text>')
    out.append("</svg>")
    return "".join(out)


if __name__ == "__main__":
    import sys

    if "--figs" not in sys.argv:
        run_checks()
    else:
        import pathlib

        outdir = pathlib.Path(sys.argv[sys.argv.index("--figs") + 1])
        outdir.mkdir(parents=True, exist_ok=True)
        for name, fn in [("landscapes", svg_landscapes), ("ellipse", svg_ellipse), ("gear", svg_gear_cases)]:
            (outdir / f"{name}.svg.html").write_text(fn())
            print("wrote", outdir / f"{name}.svg.html")

        # curvature check for the doc's claim: min <=> 1 - d*kappa_i > 0 for both (concave contacts)
        orient = outline_orientation(gear)
        for label, t in [("min", [2.07, 0.024]), ("max", [3.9488, 1.6363]), ("saddle", [5.5851, 2.4435])]:
            t = np.array(t)
            d = np.linalg.norm(gear(t[0]) - gear(t[1]))
            k = [signed_convex_curvature(gear, ti, orient) for ti in t]
            u = [1 - d * ki for ki in k]
            M = np.array([[u[0], 1], [1, u[1]]])
            print(label, "kappa(convex+)=", np.round(k, 3), "d=%.2f" % d, "u=", np.round(u, 2),
                  "-> eig sign", np.sign(np.linalg.eigvalsh(M)).astype(int),
                  "| true", np.sign(np.linalg.eigvalsh(hessian(gear, t))).astype(int))
