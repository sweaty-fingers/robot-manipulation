"""Planar 2-link arm (unit links): FK, Jacobian, IK, singularities, and doc figures.

Numerical companion to docs `docs/background/07_fk_and_ik.md`. Pure numpy. The 2-link arm matches the
example in MIT Robotic Manipulation (Basic Pick and Place, "Singularities"):
    p = [cos q0 + cos(q0+q1), sin q0 + sin(q0+q1)]

    PYTHONPATH=. python scripts/kinematics_2link.py              # numerical checks
    PYTHONPATH=. python scripts/kinematics_2link.py --figs DIR   # write SVG figures for the docs
"""

from __future__ import annotations

import numpy as np

L1 = L2 = 1.0


# ---------------- kinematics ----------------
def fk(q):
    q0, q1 = q
    return np.array([L1 * np.cos(q0) + L2 * np.cos(q0 + q1), L1 * np.sin(q0) + L2 * np.sin(q0 + q1)])


def jacobian(q):
    q0, q1 = q
    s0, c0, s01, c01 = np.sin(q0), np.cos(q0), np.sin(q0 + q1), np.cos(q0 + q1)
    return np.array([[-L1 * s0 - L2 * s01, -L2 * s01], [L1 * c0 + L2 * c01, L2 * c01]])


def ik_two_solutions(p):
    """Closed-form IK: returns (elbow-down q1>0, elbow-up q1<0) or None if unreachable."""
    x, y = p
    c1 = (x * x + y * y - L1**2 - L2**2) / (2 * L1 * L2)
    if abs(c1) > 1:
        return None
    sols = []
    for sgn in (+1, -1):
        q1 = sgn * np.arccos(c1)
        q0 = np.arctan2(y, x) - np.arctan2(L2 * np.sin(q1), L1 + L2 * np.cos(q1))
        sols.append(np.array([q0, q1]))
    return sols


def diff_ik(q, V):
    return np.linalg.pinv(jacobian(q)) @ V


def run_checks():
    p = np.array([1.2, 0.8])
    sols = ik_two_solutions(p)
    print("target", p)
    for name, q in zip(("q1>0 (elbow-down)", "q1<0 (elbow-up)"), sols):
        print(f"  {name}: q={np.degrees(q).round(2)} deg  FK error={np.linalg.norm(fk(q) - p):.2e}")

    # det J = sin(q1) for unit links
    for q1 in (1.5, 0.5, 0.1, 0.0):
        J = jacobian([0.3, q1])
        print(f"q1={q1:4.1f}: det J={np.linalg.det(J):+.4f} (sin q1={np.sin(q1):+.4f}), "
              f"singular values={np.linalg.svd(J, compute_uv=False).round(4)}")

    # pseudo-inverse near the singularity at q0=0 (arm extended along +x)
    print("differential IK near full extension (q0=0):")
    for q1 in (1.5708, 0.3, 0.1, 0.03):
        q = np.array([0.0, q1])
        vy = diff_ik(q, np.array([0.0, 0.1]))   # perpendicular to the arm: achievable
        vx = diff_ik(q, np.array([0.1, 0.0]))   # along the arm axis: nearly impossible
        print(f"  q1={q1:6.3f}  |qdot| for Vy=0.1: {np.linalg.norm(vy):6.3f}   for Vx=0.1: {np.linalg.norm(vx):7.3f}")

    # finite-difference check of the Jacobian
    q = np.array([0.7, 0.9]); h = 1e-6
    Jfd = np.column_stack([(fk(q + h * e) - fk(q - h * e)) / (2 * h) for e in np.eye(2)])
    print("Jacobian vs finite difference:", np.abs(Jfd - jacobian(q)).max())


# ---------------- figures (inline SVG, styled by `.hf-*` classes in custom.css) ----------------
def _pt(ox, oy, s, x, y):
    return ox + s * x, oy - s * y


def _arm(ox, oy, s, q, cls="hf-link", joints=True):
    q0, q1 = q
    e = (L1 * np.cos(q0), L1 * np.sin(q0))
    p = fk(q)
    a, b, c = _pt(ox, oy, s, 0, 0), _pt(ox, oy, s, *e), _pt(ox, oy, s, *p)
    g = [f'<polyline points="{a[0]:.1f},{a[1]:.1f} {b[0]:.1f},{b[1]:.1f} {c[0]:.1f},{c[1]:.1f}" class="{cls}"/>']
    if joints:
        g.append(f'<circle cx="{a[0]:.1f}" cy="{a[1]:.1f}" r="6" class="hf-joint"/>')
        g.append(f'<circle cx="{b[0]:.1f}" cy="{b[1]:.1f}" r="5" class="hf-joint"/>')
    return g, a, b, c


def _arc(cx, cy, r, a0, a1):
    """SVG arc from angle a0 to a1 (math angles, radians) centered at (cx,cy) in screen coords."""
    x0, y0 = cx + r * np.cos(a0), cy - r * np.sin(a0)
    x1, y1 = cx + r * np.cos(a1), cy - r * np.sin(a1)
    sweep = 0 if a1 > a0 else 1
    return f'<path d="M{x0:.1f},{y0:.1f} A{r},{r} 0 0 {sweep} {x1:.1f},{y1:.1f}" class="hf-arc"/>'


def svg_fk() -> str:
    s, ox, oy = 105, 120, 215
    q = np.array([np.radians(35), np.radians(70)])
    out = ['<svg class="hf" viewBox="0 0 690 250" role="img" aria-label="2링크 팔의 순기구학">']
    arm, a, b, c = _arm(ox, oy, s, q)
    out += arm
    out.append(f'<line x1="{ox-30}" y1="{oy}" x2="{ox+260}" y2="{oy}" class="hf-axis"/>')
    out.append(_arc(a[0], a[1], 40, 0, q[0]))
    ex, ey = b
    d0 = q[0]
    out.append(_arc(ex, ey, 30, d0, d0 + q[1]))
    out.append(f'<line x1="{ex:.1f}" y1="{ey:.1f}" x2="{ex+np.cos(d0)*70:.1f}" y2="{ey-np.sin(d0)*70:.1f}" class="hf-axis"/>')
    out.append(f'<text x="{a[0]+48}" y="{a[1]-8}" class="hf-t">q₀</text>')
    out.append(f'<text x="{ex+36}" y="{ey-24}" class="hf-t">q₁</text>')
    mid1 = ((a[0] + b[0]) / 2 - 18, (a[1] + b[1]) / 2 - 6)
    mid2 = ((b[0] + c[0]) / 2 + 12, (b[1] + c[1]) / 2 - 2)
    out.append(f'<text x="{mid1[0]:.0f}" y="{mid1[1]:.0f}" class="hf-t">l₁</text>')
    out.append(f'<text x="{mid2[0]:.0f}" y="{mid2[1]:.0f}" class="hf-t">l₂</text>')
    out.append(f'<circle cx="{c[0]:.1f}" cy="{c[1]:.1f}" r="7" class="hf-pt k-ee"/>')
    out.append(f'<text x="{c[0]+12:.0f}" y="{c[1]+4:.0f}" class="hf-h">엔드이펙터 p</text>')
    out.append(f'<text x="{a[0]-6}" y="{a[1]+24}" text-anchor="end" class="hf-t">베이스</text>')
    # formula block
    out.append('<text x="430" y="70" class="hf-h">순기구학 (FK)</text>')
    for i, line in enumerate(["p = f(q)", "x = l₁·cos q₀ + l₂·cos(q₀+q₁)", "y = l₁·sin q₀ + l₂·sin(q₀+q₁)", "", "관절 각도 q (2개)", "   → 엔드이펙터 위치 p (2개)", "항상 해가 하나: 그냥 계산하면 됨"]):
        out.append(f'<text x="430" y="{98+i*21}" class="hf-t">{line}</text>')
    out.append("</svg>")
    return "".join(out)


def svg_ik() -> str:
    s, ox, oy = 100, 90, 215
    p = np.array([1.2, 0.8])
    sols = ik_two_solutions(p)
    out = ['<svg class="hf" viewBox="0 0 690 250" role="img" aria-label="같은 목표에 대한 IK의 두 해">']
    # workspace boundary and axis
    out.append(f'<path d="M{ox-s*2:.0f},{oy} A{2*s},{2*s} 0 0 1 {ox+s*2:.0f},{oy}" class="hf-ws"/>')
    out.append(f'<line x1="{ox-30}" y1="{oy}" x2="{ox+2*s+30}" y2="{oy}" class="hf-axis"/>')
    out.append(f'<text x="{ox+2*s-6}" y="{oy-8}" text-anchor="end" class="hf-t">도달 한계 (l₁+l₂)</text>')
    for q, cls, dash in ((sols[1], "hf-link", ""), (sols[0], "hf-link alt", "")):
        arm, a, b, c = _arm(ox, oy, s, q, cls=cls)
        out += arm
    T = _pt(ox, oy, s, *p)
    out.append(f'<circle cx="{T[0]:.1f}" cy="{T[1]:.1f}" r="8" class="hf-target"/>')
    out.append(f'<text x="{T[0]+14:.0f}" y="{T[1]-6:.0f}" class="hf-h">목표 p</text>')
    out.append(f'<text x="{ox-6}" y="{oy+24}" text-anchor="end" class="hf-t">베이스</text>')
    out.append('<text x="430" y="62" class="hf-h">역기구학 (IK): 해가 둘</text>')
    lines = [
        ("hf-key a", f"■ 팔꿈치 위 해: q = ({np.degrees(sols[1][0]):.0f}°, {np.degrees(sols[1][1]):.0f}°)"),
        ("hf-key b", f"■ 팔꿈치 아래 해: q = ({np.degrees(sols[0][0]):.0f}°, {np.degrees(sols[0][1]):.0f}°)"),
        ("hf-t", ""),
        ("hf-t", "같은 p를 만드는 q가 2개 (자유도가 같아도)"),
        ("hf-t", "범위 밖 p → 해 없음, 경계 p → 해 1개"),
        ("hf-t", "7자유도 팔 → 해가 무한히 많음 (여유 자유도)"),
    ]
    for i, (cls, line) in enumerate(lines):
        out.append(f'<text x="430" y="{90+i*22}" class="{cls}">{line}</text>')
    out.append("</svg>")
    return "".join(out)


def svg_manipulability() -> str:
    cases = [(np.radians(100), "q₁ = 100°", "넓고 둥긂: 어느 방향이든 잘 움직임"),
             (np.radians(45), "q₁ = 45°", "길쭉해짐: 방향마다 속도 차이"),
             (np.radians(8), "q₁ = 8°", "거의 납작: 팔 축 방향 속도가 안 나옴")]
    s, pw, es = 48, 230, 34
    out = ['<svg class="hf" viewBox="0 0 690 285" role="img" aria-label="조작성 타원과 특이점">']
    for i, (q1, name, note) in enumerate(cases):
        q = np.array([np.radians(15), q1])
        ox, oy = i * pw + 50, 175
        arm, a, b, c = _arm(ox, oy, s, q)
        J = jacobian(q)
        U, S, _ = np.linalg.svd(J)
        ang = np.degrees(np.arctan2(U[1, 0], U[0, 0]))
        out.append(f'<ellipse cx="{c[0]:.1f}" cy="{c[1]:.1f}" rx="{S[0]*es:.1f}" ry="{S[1]*es:.1f}" transform="rotate({-ang:.1f} {c[0]:.1f} {c[1]:.1f})" class="hf-ell"/>')
        out += arm
        out.append(f'<circle cx="{c[0]:.1f}" cy="{c[1]:.1f}" r="5.5" class="hf-pt k-ee"/>')
        out.append(f'<text x="{i*pw+pw/2:.0f}" y="228" text-anchor="middle" class="hf-h">{name}</text>')
        out.append(f'<text x="{i*pw+pw/2:.0f}" y="247" text-anchor="middle" class="hf-t">σ = {S[0]:.2f}, {S[1]:.2f}</text>')
        out.append(f'<text x="{i*pw+pw/2:.0f}" y="266" text-anchor="middle" class="hf-t">{note}</text>')
    out.append("</svg>")
    return "".join(out)


# ---------------- rotation representation figures (gimbal lock, unit quaternion) ----------------
def rpy_matrix(roll, pitch, yaw):
    """R = Rz(yaw) @ Ry(pitch) @ Rx(roll) (the common ZYX roll-pitch-yaw convention)."""
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def gimbal_checks():
    a = rpy_matrix(np.radians(30), np.pi / 2, np.radians(10))
    b = rpy_matrix(np.radians(50), np.pi / 2, np.radians(30))   # roll+20, yaw+20: same roll - yaw
    c = rpy_matrix(np.radians(30), np.pi / 2, np.radians(40))   # roll - yaw differs
    d = rpy_matrix(np.radians(30), np.radians(60), np.radians(10))
    e = rpy_matrix(np.radians(50), np.radians(60), np.radians(30))
    print("gimbal lock check (pitch=90): same roll-yaw -> same R:", np.allclose(a, b), "| different roll-yaw -> different R:", not np.allclose(a, c))
    print("pitch=60: same roll-yaw but different R:", not np.allclose(d, e))


def _arrow(x0, y0, x1, y1, cls, head=9):
    ang = np.arctan2(y1 - y0, x1 - x0)
    l = (x1 - head * np.cos(ang - 0.4), y1 - head * np.sin(ang - 0.4))
    r = (x1 - head * np.cos(ang + 0.4), y1 - head * np.sin(ang + 0.4))
    return (f'<line x1="{x0:.1f}" y1="{y0:.1f}" x2="{x1:.1f}" y2="{y1:.1f}" class="{cls}"/>'
            f'<polygon points="{x1:.1f},{y1:.1f} {l[0]:.1f},{l[1]:.1f} {r[0]:.1f},{r[1]:.1f}" class="{cls} hf-tip"/>')


def svg_gimbal() -> str:
    """Side view: the yaw axis (fixed, vertical) and the roll axis (the body x axis after pitch)."""
    cases = [(0, "pitch = 0°", "두 축이 수직: roll과 yaw는 다른 회전"),
             (60, "pitch = 60°", "두 축이 가까워짐"),
             (90, "pitch = 90°", "두 축이 겹침: roll과 yaw가 같은 회전")]
    pw, L = 230, 90
    out = ['<svg class="hf" viewBox="0 0 690 290" role="img" aria-label="pitch에 따라 roll 축과 yaw 축이 겹치는 모습">']
    for i, (pd, name, note) in enumerate(cases):
        cx, cy = i * pw + 100, 150
        p = np.radians(pd)
        out.append(_arrow(cx, cy, cx, cy - L, "hf-ax-yaw"))
        ex, ey = cx + L * np.cos(p), cy - L * np.sin(p)
        out.append(_arrow(cx, cy, ex, ey, "hf-ax-roll"))
        out.append(f'<circle cx="{cx}" cy="{cy}" r="4" class="hf-joint"/>')
        if pd:
            out.append(_arc(cx, cy, 34, np.pi / 2, p))
        lx = cx - 8 if pd == 0 else cx - 10
        out.append(f'<text x="{cx-10}" y="{cy-L-8}" text-anchor="end" class="hf-key yw">yaw 축</text>')
        out.append(f'<text x="{ex+(12 if pd == 90 else 8):.0f}" y="{ey+(4):.0f}" class="hf-key rl">roll 축</text>')
        out.append(f'<text x="{i*pw+pw/2:.0f}" y="236" text-anchor="middle" class="hf-h">{name}</text>')
        out.append(f'<text x="{i*pw+pw/2:.0f}" y="258" text-anchor="middle" class="hf-t">{note}</text>')
    out.append("</svg>")
    return "".join(out)


def svg_quat() -> str:
    """Unit quaternion as a point on a circle (the 2D slice of the 4D unit sphere): q and -q are one rotation."""
    cx, cy, r = 150, 150, 95
    th = np.radians(100)
    h = th / 2
    qx, qy = cx + r * np.cos(h), cy - r * np.sin(h)
    nx, ny = cx - r * np.cos(h), cy + r * np.sin(h)
    out = ['<svg class="hf" viewBox="0 0 690 300" role="img" aria-label="단위 쿼터니언과 이중 덮개">']
    out.append(f'<circle cx="{cx}" cy="{cy}" r="{r}" class="hf-ws"/>')
    out.append(f'<line x1="{cx-r-20}" y1="{cy}" x2="{cx+r+20}" y2="{cy}" class="hf-axis"/>')
    out.append(f'<line x1="{cx}" y1="{cy-r-20}" x2="{cx}" y2="{cy+r+20}" class="hf-axis"/>')
    out.append(f'<text x="{cx+r+22}" y="{cy+4}" class="hf-t">w</text>')
    out.append(f'<text x="{cx+6}" y="{cy-r-24}" class="hf-t">n·sin(θ/2)</text>')
    out.append(f'<line x1="{cx}" y1="{cy}" x2="{qx:.1f}" y2="{qy:.1f}" class="hf-move pos"/>')
    out.append(f'<line x1="{cx}" y1="{cy}" x2="{nx:.1f}" y2="{ny:.1f}" class="hf-move neg2"/>')
    out.append(_arc(cx, cy, 30, 0, h))
    out.append(f'<text x="{cx+34}" y="{cy-6}" class="hf-t">θ/2</text>')
    out.append(f'<circle cx="{qx:.1f}" cy="{qy:.1f}" r="6" class="hf-pt k-min"/>')
    out.append(f'<circle cx="{nx:.1f}" cy="{ny:.1f}" r="6" class="hf-pt k-max"/>')
    out.append(f'<text x="{qx+10:.0f}" y="{qy-6:.0f}" class="hf-h">q</text>')
    out.append(f'<text x="{nx-12:.0f}" y="{ny+18:.0f}" text-anchor="end" class="hf-h">−q</text>')
    out.append('<text x="150" y="288" text-anchor="middle" class="hf-t">단위 쿼터니언 = 원 위의 한 점 (실제는 4차원 구)</text>')
    rows = [("hf-h", "q = (cos θ/2, n·sin θ/2)"), ("hf-t", "n: 회전 축(단위 벡터), θ: 회전 각"), ("hf-t", ""),
            ("hf-key pq", "■ q: θ만큼 회전"), ("hf-key nq", "■ −q: 반대편 점이지만 같은 회전"), ("hf-t", ""),
            ("hf-t", "θ가 0°→360°로 돌면 q는 반 바퀴만 감"), ("hf-t", "θ = 360°는 q = −(1,0,0,0), 같은 자세"), ("hf-t", "θ를 720° 돌아야 q가 제자리로 옴"), ("hf-t", ""),
            ("hf-t", "원 위에서는 어디로 가도 끊기거나 겹치는 점이 없음"), ("hf-t", "→ 짐벌락 같은 특이점이 없음")]
    for i, (cls, line) in enumerate(rows):
        out.append(f'<text x="330" y="{52+i*21}" class="{cls}">{line}</text>')
    out.append("</svg>")
    return "".join(out)


if __name__ == "__main__":
    import pathlib
    import sys

    if "--figs" not in sys.argv:
        run_checks()
        gimbal_checks()
    else:
        outdir = pathlib.Path(sys.argv[sys.argv.index("--figs") + 1])
        outdir.mkdir(parents=True, exist_ok=True)
        for name, fn in [("fk", svg_fk), ("ik", svg_ik), ("manip", svg_manipulability), ("gimbal", svg_gimbal), ("quat", svg_quat)]:
            (outdir / f"{name}.svg.html").write_text(fn())
            print("wrote", outdir / f"{name}.svg.html")
