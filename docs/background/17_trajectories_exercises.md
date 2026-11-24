---
description: MIT Robotic Manipulation 6장(Motion Planning)의 연습 문제 일곱 개를 직접 풀어 본 풀이. 그림으로 답이 정해지는 문제는 답과 계산 과정을, 노트북 문제는 2차원·iiwa 예제로 같은 원리를 재현.
---

# 17. 모션 플래닝 연습 문제 풀이

> **이 문서의 성격**: [`trajectories.html`](https://manipulation.csail.mit.edu/trajectories.html) 맨 아래 *Exercises* 절의 일곱 문제를 풉니다. 먼저 알아 둘 점입니다.
> - 원문의 연습 문제는 대부분 **별도 노트북(과제)** 이고 페이지에는 설명 몇 줄만 있습니다. 노트북은 볼 수 없었으므로 **노트북의 정확한 입력·채점 방식은 모릅니다.** 문제 설명에 적힌 단계를 기준으로, 같은 원리를 **제가 만든 예제**로 풀었습니다.
> - 페이지에 **그림과 질문이 함께 있는 문제는 3번**(경로 개선)뿐입니다. 그림 두 장(`shortcutting.png`, `rrtstar_step.png`)을 내려받아 읽었습니다. 노드 좌표는 그림에서 **눈대중으로 읽은 값**이라 정확하지 않습니다. 답이 바뀌지 않는 범위에서만 결론을 냈습니다.
> - 계산은 `scripts/planning_2d.py`(2차원, Drake 최적화), `scripts/planning_drake.py`(iiwa IK, GCS)로 했습니다. iiwa 모델은 Drake가 내려받는 `iiwa14_no_collision`이라 **IK 결과에 충돌 검사가 없습니다.**
> - **풀이 속 모범 답안**은 이 문서가 만든 것이고 공식 정답이 아닙니다. 정답 비교가 필요하면 과제 노트북을 확인하세요.
>
> 먼저 읽기: [모션 플래닝 핵심 개념](16_motion_planning_concepts.md) · [FK와 IK](07_fk_and_ik.md) (특히 5.3.1) · [B-스플라인과 증강 라그랑지안](15_bspline_and_augmented_lagrangian.md)

## 문제 한눈에 보기

| 번호 | 문제 (원문 제목) | 무엇을 하나 | 이 문서의 풀이 | 검증 |
|---|---|---|---|---|
| 1 | Door Opening | 최적화 IK로 찬장 손잡이를 잡고 문을 열기 | 제약 → 최적화 → Drake 코드 → 결과 | iiwa에서 실행 |
| 2 | RRT Motion Planning | RRT, RRT-Connect 구현과 성질 질문 | 알고리즘과 성질 답변 | 2차원에서 실행 |
| 3 | Improving RRT Path Quality | shortcutting, RRT*의 부모 | **그림으로 답이 정해짐** | 좌표 계산 |
| 4 | Decomposing Obstacle-Free Space | IRIS의 QP와 분리 초평면 | 2차원 구현 | 실행 |
| 5 | Plan and Place | 키프레임, IK, RRT-Connect, shortcutting | 단계별 설계 | 부분(2, 3번 재사용) |
| 6 | IRIS and GCS in Joint Space | 관절 공간 IRIS 영역과 GCS 궤적 | 2차원 GCS | 2차원에서 실행 |
| 7 | Trajectory Optimization Around a Shelf Exterior | 충돌 제약과 초기값이 수렴에 미치는 영향 | 개념 답변 + 2차원 비교 | 2차원에서 실행 |

## 1. Door Opening: 최적화 IK로 손잡이 잡고 문 열기

원문의 단계는 세 가지입니다. ① 찬장 손잡이를 잡는 IK의 **제약을 적고**, ② 그것을 **최적화 문제로 정식화**하고, ③ `MathematicalProgram`으로 **구현**합니다.

### 1단계: 제약을 말로 적기

손잡이는 **수직 막대**(길이 0.2 m)라고 두겠습니다. 문제는 "어떻게 잡아야 하나"입니다. 손잡이를 잡는 데 **꼭 필요한 것**만 적어 봅니다([07의 5.3.1](07_fk_and_ik.md)).

| 필요한 것 | 필요 없는 것 |
|---|---|
| 손가락 사이 중심선이 **막대 축 위**에 있다 | 막대의 어느 높이를 잡는지 |
| 관절이 한계 안이다 | 손이 막대 둘레를 어느 각도로 돌아 있는지 |
| (충돌이 있다면) 팔이 문·찬장에 안 닿는다 | 손이 막대에 접근하는 방향 |

### 2단계: 최적화 문제로 쓰기

$$
\min_{q}\ \|q-q_0\|^2\quad\text{s.t.}\quad
p^{W}_{G,1}(q),\ p^{W}_{G,2}(q)\in\{(x_h,y_h,z)\mid |z-z_h|\le 0.1\},\quad q_{min}\le q\le q_{max}
$$

- `p_{G,1}, p_{G,2}`: 그리퍼 프레임에서 **손가락 패드 중심선 위의 두 점** `[0, 0.10, ∓0.02]`를 세계 좌표로 옮긴 것.
- 막대 축은 세로선 `(x_h, y_h, ·)`의 선분이므로 두 점의 `x, y`는 **같아야** 하고 `z`는 `±0.1 m` 안이면 됩니다.
- 비용은 관절 중심(`q₀`)만. 요구는 모두 제약입니다.
- **자유도 세기**(제 계산): 점 하나가 직선 위에 있으려면 직선에 수직인 두 방향이 묶이고, 점이 둘이라 4개가 묶입니다. 6개 중 **2개**(막대를 따라 미끄러짐, 막대 둘레 회전)가 남습니다.

### 3단계: Drake로 구현

```python
# two points on the gripper's pad centerline; x, y exactly on the bar axis, z may slide +-0.1 m
for dz in (-0.02, 0.02):
    lo, hi = handle + [0, 0, -0.10], handle + [0, 0, 0.10]
    lo[:2], hi[:2] = handle[:2], handle[:2]
    ik.AddPositionConstraint(frameB=G, p_BQ=[0, 0.10, dz], frameA=W, p_AQ_lower=lo, p_AQ_upper=hi)
ik.prog().AddQuadraticErrorCost(np.eye(7), q_nominal, ik.q())   # joint-centering cost only
result = Solve(ik.prog())
```

(`scripts/planning_drake.py`의 `solve_ik`. 그리퍼 프레임 `G`는 `x`=닫히는 방향, `y`=접근 방향, `z`=패드를 따르는 방향으로 두었습니다.)

### 결과

**(a) 최소 제약과 자세 전체 고정의 비교** (handle 위치 60곳, `x∈[0.35,0.75]`, `y∈[-0.45,0.45]`, `z∈[0.25,0.75]`):

| 제약 | 해를 찾은 곳 | 비고 |
|---|---|---|
| 최소(두 점이 막대 위) | **59 / 60** | 접근 방향이 +x에서 평균 24° 벗어남(최대 97°). 솔버가 자유도를 실제로 씀 |
| 자세 전체 고정(접근 +x, 닫힘 −y) | 33 / 60 | 방향을 못 바꾸니 닿지 못하는 곳이 늘어남 |

> 33/60은 "해가 없다"는 증명이 아니라 **솔버가 해를 못 찾은 곳**의 수입니다(비볼록 최적화). 그래도 같은 초기값에서 제약이 적은 쪽이 훨씬 많이 풀린 것은 5.3.1의 "제약은 최소로"를 뒷받침합니다.

**(b) 문 열기: 손잡이가 경첩 둘레의 호를 따라 움직일 때** (경첩 `(0.35, 0.45)`, 반지름 0.4 m, 높이 0.4 m, 문 각도 0°→70°, 매 단계 이전 해에서 시작):

| 문 각도 | 손잡이 위치 | 손 접근 방향(수평 각도) | 이전 해와의 최대 관절 변화 |
|---|---|---|---|
| 0° | (0.35, 0.05) | −73° | (편안한 자세에서 시작: 2.12 rad) |
| 10° | (0.42, 0.06) | −59° | 0.29 rad |
| 30° | (0.55, 0.10) | −24° | 0.46 rad |
| 50° | (0.66, 0.19) | −19° | 0.18 rad |
| 70° | (0.73, 0.31) | −11° | 0.17 rad |

- 8개 각도 모두 해가 있었고, 이후 단계의 관절 변화가 0.5 rad 이하로 **매끄럽게** 이어졌습니다.
- 손의 접근 방향이 −73°에서 −11°로 **저절로 돌아갑니다.** 방향을 제약하지 않았기 때문에 솔버가 팔에 편한 쪽을 고른 결과입니다.
- 한계: 이 예제는 **충돌 없는 모델**이고 문이 실제로 밀리는 힘, 막대를 놓치지 않는 파지 조건은 다루지 않습니다.

## 2. RRT Motion Planning: RRT와 RRT-Connect

원문의 단계: Kuka 팔용 RRT 구현, 성질 질문 답하기, RRT-Connect 구현. Kuka 팔(7차원)은 충돌 검사용 자산이 필요해서 여기서는 **2차원 점 로봇**(`[-2,2]²`, 가운데 마름모 장애물 `|x|+|y|<1`, 시작 (-1.5, 0), 목표 (1.5, 0))으로 같은 알고리즘을 구현했습니다. 알고리즘은 차원과 무관합니다.

### 구현

```python
def rrt(rng, max_iter=4000, goal_bias=0.05):
    nodes, parent = [START], [-1]
    for it in range(1, max_iter + 1):
        q_sample = GOAL if rng.random() < goal_bias else rng.uniform(LO, HI, 2)   # 1. sample
        i = nearest(nodes, q_sample)                                              # 2. q_near
        q_new = steer(nodes[i], q_sample, STEP)                                   # 3. one step toward it
        if not free_segment(nodes[i], q_new):                                     # 4. collision check
            continue
        nodes.append(q_new); parent.append(i)                                     #    parent = q_near
        if close_to_goal(q_new) and free_segment(q_new, GOAL):                    # 5. done
            ...
```

RRT-Connect는 시작 트리와 목표 트리를 두고, 한쪽을 `q_sample` 쪽으로 한 걸음 뻗은 뒤 **다른 쪽이 그 새 노드를 향해 막힐 때까지 계속 뻗고**(CONNECT), 역할을 바꿉니다(`scripts/planning_2d.py`).

### 결과 (시드 100개, 걸음 0.25, 목표 편향 5%)

| 알고리즘 | 성공 | 반복 횟수(중앙값) | 경로 길이(평균) | shortcutting 후 |
|---|---|---|---|---|
| RRT | 100 / 100 | 77 | 4.60 | 3.84 |
| RRT-Connect | 100 / 100 | **48** | 4.66 | 3.74 |

기준: 이 문제의 최단 경로 길이는 3.236입니다. 경로는 최단의 **1.4배**쯤으로 울퉁불퉁하고, shortcutting으로도 1.16~1.19배가 남습니다.

### 성질에 대한 답 (원문이 "성질 질문"이라고만 적었으므로 흔한 질문을 모았습니다)

| 질문 | 답과 이유 |
|---|---|
| RRT는 해를 반드시 찾나? | **확률적으로 완전**합니다. 해가 있으면 반복할수록 찾을 확률이 1로 갑니다. 정해진 횟수 안의 보장은 없습니다 |
| 찾은 경로는 최단인가? | **아닙니다.** 항상 `q_near`를 부모로 삼아 한 번 정한 연결을 고치지 않습니다(원문: 증명 가능하게 비최적). 위 표에서도 최단보다 훨씬 깁니다 |
| 왜 RRT-Connect가 빠른가? | 양쪽에서 자라 만나고, CONNECT가 한 번에 여러 걸음을 가서 두 트리를 빨리 붙입니다. 중앙값 77 → 48 |
| 경로가 왜 울퉁불퉁한가? | 무작위 표본 때문입니다. 후처리(문제 3)로 줄입니다 |
| 목표 편향은 왜 쓰나? | 가끔 목표를 표본으로 뽑아 트리가 목표 쪽으로 자라게 합니다. 너무 크면 장애물에 막혀 탐색이 줄어듭니다 |

## 3. Improving RRT Path Quality: 경로 개선 (그림 문제)

이 문제는 그림 두 장과 질문 세 개가 페이지에 있습니다.

### 3-1. shortcutting으로 더 짧아지는 노드 쌍 하나

<figure class="hf-fig"><svg class="hf" viewBox="0 0 620 310" role="img" aria-label="shortcutting 연습 문제 그림을 다시 그린 것"><rect x="262" y="65" width="93" height="193" style="fill:color-mix(in srgb,var(--kit-c-danger) 30%,transparent);stroke:var(--kit-c-danger);stroke-width:1.5"/><polyline points="69.0,167.0 165.0,199.0" style="fill:none;stroke:var(--kit-faint);stroke-width:1.6"/><polyline points="165.0,199.0 149.0,151.0" style="fill:none;stroke:var(--kit-faint);stroke-width:1.6"/><polyline points="165.0,199.0 117.0,272.0" style="fill:none;stroke:var(--kit-faint);stroke-width:1.6"/><polyline points="165.0,199.0 237.0,231.0" style="fill:none;stroke:var(--kit-faint);stroke-width:1.6"/><polyline points="237.0,231.0 285.0,287.0" style="fill:none;stroke:var(--kit-faint);stroke-width:1.6"/><polyline points="205.0,72.0 237.0,128.0" style="fill:none;stroke:var(--kit-faint);stroke-width:1.6"/><polyline points="373.0,136.0 397.0,207.0" style="fill:none;stroke:var(--kit-faint);stroke-width:1.6"/><polyline points="69.0,167.0 21.0,87.0 107.0,30.0 205.0,72.0 317.0,32.0 478.0,73.0 373.0,136.0 493.0,167.0 550.0,112.0 573.0,191.0" style="fill:none;stroke:var(--kit-accent);stroke-width:3.2"/><polyline points="478.0,73.0 493.0,167.0" style="fill:none;stroke:var(--kit-c-tip);stroke-width:2.6;stroke-dasharray:6 4"/><polyline points="107.0,30.0 317.0,32.0" style="fill:none;stroke:var(--kit-c-tip);stroke-width:2.6;stroke-dasharray:6 4"/><polyline points="373.0,136.0 550.0,112.0" style="fill:none;stroke:var(--kit-c-warn);stroke-width:2.6;stroke-dasharray:6 4"/><circle cx="69.0" cy="167.0" r="6" style="fill:var(--kit-accent)"/><text x="76" y="160" class="hf-t">q_start</text><circle cx="21.0" cy="87.0" r="4.5" style="fill:var(--kit-fg)"/><text x="28" y="80" class="hf-t">1</text><circle cx="107.0" cy="30.0" r="4.5" style="fill:var(--kit-fg)"/><text x="114" y="23" class="hf-t">2</text><circle cx="205.0" cy="72.0" r="4.5" style="fill:var(--kit-fg)"/><text x="212" y="65" class="hf-t">6</text><circle cx="317.0" cy="32.0" r="4.5" style="fill:var(--kit-fg)"/><text x="324" y="25" class="hf-t">8</text><circle cx="478.0" cy="73.0" r="4.5" style="fill:var(--kit-fg)"/><text x="485" y="66" class="hf-t">10</text><circle cx="373.0" cy="136.0" r="4.5" style="fill:var(--kit-fg)"/><text x="380" y="129" class="hf-t">11</text><circle cx="493.0" cy="167.0" r="4.5" style="fill:var(--kit-fg)"/><text x="500" y="160" class="hf-t">14</text><circle cx="550.0" cy="112.0" r="4.5" style="fill:var(--kit-fg)"/><text x="557" y="105" class="hf-t">15</text><circle cx="573.0" cy="191.0" r="6" style="fill:var(--kit-accent)"/><text x="580" y="184" class="hf-t">q_goal</text><circle cx="165.0" cy="199.0" r="4.5" style="fill:var(--kit-muted)"/><text x="172" y="192" class="hf-t">3</text><circle cx="149.0" cy="151.0" r="4.5" style="fill:var(--kit-muted)"/><text x="156" y="144" class="hf-t">4</text><circle cx="117.0" cy="272.0" r="4.5" style="fill:var(--kit-muted)"/><text x="124" y="265" class="hf-t">5</text><circle cx="237.0" cy="231.0" r="4.5" style="fill:var(--kit-muted)"/><text x="244" y="224" class="hf-t">7</text><circle cx="285.0" cy="287.0" r="4.5" style="fill:var(--kit-muted)"/><text x="292" y="280" class="hf-t">13</text><circle cx="237.0" cy="128.0" r="4.5" style="fill:var(--kit-muted)"/><text x="244" y="121" class="hf-t">9</text><circle cx="397.0" cy="207.0" r="4.5" style="fill:var(--kit-muted)"/><text x="404" y="200" class="hf-t">12</text><text x="10" y="302" class="hf-t">갈색: RRT가 찾은 해 경로, 초록·주황 점선: 줄일 수 있는 지름길</text></svg><figcaption>그림 1. 연습 문제 그림을 다시 그린 것(노드 좌표는 원본 그림에서 눈대중으로 읽음). 갈색 선이 q_start에서 q_goal로 가는 해 경로(start→1→2→6→8→10→11→14→15→goal)이고, 점선이 지름길 후보다.</figcaption></figure>

**먼저 해 경로를 가려냅니다.** 트리에는 해와 상관없는 가지(3, 4, 5, 7, 13, 9, 12)도 있습니다. 해는 `q_start → 1 → 2 → 6 → 8 → 10 → 11 → 14 → 15 → q_goal`입니다. 지름길은 **이 경로 위의 두 노드**를 잇는 것이어야 합니다.

**답 (하나만 고르면): 노드 10과 노드 14.** 10 → 11 → 14로 돌아가던 것을 직선 10 → 14로 바꿉니다.

| 후보 | 지금 경로 길이 | 직선 길이 | 줄어드는 양 (단위: 그림의 픽셀) | 충돌 |
|---|---|---|---|---|
| **10 – 14** | 246 | 95 | **151** | 없음(상자에서 멀다) |
| 2 – 8 | 226 | 210 | 16 | 없음(상자 위를 지나감) |
| 11 – 15 | 203 | 179 | 25 | 없음 |
| start – 2 | 197 | 142 | 54 | 없음 |

- 다른 정답도 많습니다(1–6, 14–goal, …). 어느 것이든 **경로 위의 두 노드를 직선으로 이었을 때 빨간 상자와 겹치지 않고 더 짧으면** 정답입니다.
- 반대로 **8과 11**처럼 상자를 가로지르는 쌍은 안 됩니다(직선이 상자를 지남).
- 노드 8과 14 이상을 잇는 직선은 상자 윗변과 4픽셀쯤 떨어져 있어서 **눈대중 좌표로는 판단이 불안정**합니다. 답으로 쓰지 않았습니다.

### 3-2. 표준 RRT에서 q_new의 부모는?

<figure class="hf-fig"><svg class="hf" viewBox="0 0 690 250" role="img" aria-label="RRT* 부모 선택"><polyline points="290.0,30.0 228.0,92.0" style="fill:none;stroke:var(--kit-muted);stroke-width:3"/><polyline points="290.0,30.0 352.0,92.0" style="fill:none;stroke:var(--kit-muted);stroke-width:3"/><polyline points="228.0,92.0 228.0,216.0" style="fill:none;stroke:var(--kit-muted);stroke-width:3"/><polyline points="228.0,216.0 166.0,216.0" style="fill:none;stroke:var(--kit-muted);stroke-width:3;stroke-dasharray:4 3"/><polyline points="290.0,30.0 166.0,216.0" style="fill:none;stroke:var(--kit-c-tip);stroke-width:3;stroke-dasharray:7 4"/><polyline points="228.0,92.0 166.0,216.0" style="fill:none;stroke:var(--kit-c-warn);stroke-width:2.4;stroke-dasharray:3 3"/><circle cx="290.0" cy="30.0" r="6" style="fill:var(--kit-fg)"/><text x="299" y="22" class="hf-t">q_start</text><circle cx="228.0" cy="92.0" r="6" style="fill:var(--kit-fg)"/><text x="237" y="84" class="hf-t">q₁</text><circle cx="352.0" cy="92.0" r="6" style="fill:var(--kit-fg)"/><text x="361" y="84" class="hf-t">q₂</text><circle cx="228.0" cy="216.0" r="6" style="fill:var(--kit-fg)"/><text x="237" y="208" class="hf-t">q_near</text><circle cx="166.0" cy="216.0" r="6" style="fill:var(--kit-fg)"/><text x="175" y="208" class="hf-t">q_new</text><text x="440" y="44" class="hf-h">q_new의 부모 후보 (한 칸 = 1)</text><text x="440" y="74" class="hf-t">q_near 부모 (RRT) → 4.414</text><text x="440" y="98" class="hf-t">q₂ 부모 → 5.020</text><text x="440" y="122" class="hf-key a">q₁ 부모 → 3.650</text><text x="440" y="146" class="hf-key pq">q_start 부모 → 3.606 (최소)</text><text x="440" y="190" class="hf-t">재배선: q_new를 q_start에</text><text x="440" y="212" class="hf-t">직접 이으면 가장 짧다</text></svg><figcaption>그림 2. RRT* 연습 문제 그림을 다시 그린 것. 격자 한 칸을 1로 읽었을 때 q_new의 부모 후보별 시작점부터의 총 길이.</figcaption></figure>

**답: `q_near`.** 표준 RRT는 항상 가장 가까운 노드를 부모로 삼습니다. 그림에서 `q_new`는 `q_near`에서 `q_sample` 쪽으로 한 걸음 간 점이므로 `q_near`가 부모입니다.

### 3-3. 시작점까지 최단 경로를 담으려면 부모는?

**답: `q_start`.** 그림의 격자 좌표를 읽어 각 후보를 부모로 했을 때 `q_start`에서 `q_new`까지의 길이를 계산합니다(`q_start` (5,0), `q₁` (4,1), `q₂` (6,1), `q_near` (4,3), `q_new` (3,3)).

| 부모 후보 | 시작점까지 거리 + 후보에서 q_new까지 | 합 |
|---|---|---|
| `q_near` (표준 RRT) | √2 + 2 + 1 | 4.414 |
| `q₂` | √2 + √13 | 5.020 |
| `q₁` | √2 + √5 | 3.650 |
| **`q_start`** | √13 | **3.606** |

- `q_start`에서 `q_new`로 바로 가는 직선이 가장 짧습니다(3.606). 근소한 차이로 `q₁`(3.650)이 뒤를 잇습니다.
- 격자를 눈으로 읽은 값이라 **3.606과 3.650의 차이(약 0.04)는 읽는 오차에 민감합니다.** 그림은 일부러 정수 격자에 맞춰 그려져 있어 위 읽기가 의도에 맞아 보이지만 확정은 아닙니다. `q_near`와 `q₂`가 정답이 아니라는 것은 확실합니다.
- 이 "부모를 비용이 가장 작은 쪽으로 다시 고르고 이웃을 재배선하는" 아이디어가 **RRT\***이고 샘플 수가 무한이면 최적으로 수렴합니다(원문). 직선 `q_start → q_new`가 충돌이 없다는 전제입니다(그림에 장애물 없음).

## 4. Decomposing Obstacle-Free Space: IRIS의 두 조각

원문의 단계: ① **타원에서 장애물까지 최근접점**을 구하는 QP, ② 타원을 **모든 장애물에서 분리하는 초평면**들을 찾는 부분.

### 단계 ①: 최근접점 QP

타원을 `E = {Cu + d : ‖u‖ ≤ 1}`, 볼록 장애물을 `{x : A x ≤ b}`라고 합니다. 타원의 "거리"를 타원 모양으로 잰 값은 `(x−d)ᵀ(CCᵀ)⁻¹(x−d)`이고, 이것이 1이면 타원 경계입니다.

$$
\min_x\ (x-d)^\top (CC^\top)^{-1}(x-d)\quad\text{s.t.}\quad A x\le b
$$

**목적 함수가 볼록 이차, 제약이 선형**이라 QP입니다. 최소값의 제곱근이 1보다 크면 장애물은 타원 밖에 있습니다.

```python
M = np.linalg.inv(C @ C.T)
prog = MathematicalProgram()
x = prog.NewContinuousVariables(2)
prog.AddLinearConstraint(A, -np.inf * np.ones(len(b)), b, x)
prog.AddQuadraticCost(2 * M, -2 * M @ d, x)        # (x-d)^T M (x-d) up to a constant
x_star = Solve(prog).GetSolution(x)
```

작은 확인: 원점의 반지름 0.2 원(`C = 0.2I, d = 0`)과 장애물 B(`[0.6,1.6]×[0.5,1.4]`)의 최근접점은 `(0.6, 0.5)`, 타원 거리는 `√(0.6²+0.5²)/0.2 = 3.905`로 계산값과 같았습니다.

### 단계 ②: 분리 초평면

최근접점 `x*`에서 **타원에 접하는 평면**을 세웁니다. 법선은 `a = (CCᵀ)⁻¹(x* − d)`(정규화), 평면은 `aᵀx = aᵀx*`입니다. 타원은 `aᵀx ≤ aᵀx*` 쪽에, 장애물은 반대쪽에 있습니다. 순서는 다음과 같습니다.

1. 모든 남은 장애물에 대해 최근접점 QP를 풀어 **가장 가까운 장애물**을 고릅니다.
2. 그 장애물의 접평면을 영역에 추가합니다.
3. **이 평면이 이미 다른 쪽에 놓은 장애물**(모든 꼭짓점이 평면 반대쪽)은 목록에서 뺍니다.
4. 장애물이 남지 않을 때까지 반복합니다.

(구현상 주의: 최근접점이 솔버 오차 때문에 평면 위에서 1e-9쯤 어긋나면 "이미 분리됨" 판정이 실패해 **무한 반복**할 수 있습니다. 평면을 만든 장애물은 무조건 목록에서 빼고, 다른 장애물은 1e-6 허용오차로 판정했습니다.)

### 전체 한 바퀴: 타원 부풀리기까지

원문 문제는 ①, ②까지지만, IRIS가 어떻게 동작하는지 보려고 **③ 영역 안의 최대 부피 타원**(로그 행렬식 최대화)을 붙여 반복했습니다.

<figure class="hf-fig"><svg class="hf" viewBox="0 0 690 320" role="img" aria-label="IRIS 한 번의 영역 확장"><rect x="31" y="16" width="288" height="288" style="fill:none;stroke:var(--kit-faint);stroke-width:1.2;stroke-dasharray:4 3"/><polygon points="307.6,172.7 178.6,102.4 31.0,102.4 31.0,207.5 185.7,211.2" style="fill:color-mix(in srgb,var(--kit-accent) 14%,transparent);stroke:var(--kit-accent);stroke-width:2"/><polygon points="88.6,102.4 146.2,102.4 146.2,44.8 88.6,44.8" style="fill:color-mix(in srgb,var(--kit-c-danger) 30%,transparent);stroke:var(--kit-c-danger);stroke-width:1.5"/><text x="117" y="78" text-anchor="middle" class="hf-t">A</text><polygon points="218.2,124.0 290.2,124.0 290.2,59.2 218.2,59.2" style="fill:color-mix(in srgb,var(--kit-c-danger) 30%,transparent);stroke:var(--kit-c-danger);stroke-width:1.5"/><text x="254" y="96" text-anchor="middle" class="hf-t">B</text><polygon points="67.0,275.2 153.4,275.2 153.4,210.4 67.0,210.4" style="fill:color-mix(in srgb,var(--kit-c-danger) 30%,transparent);stroke:var(--kit-c-danger);stroke-width:1.5"/><text x="110" y="247" text-anchor="middle" class="hf-t">C</text><polygon points="225.4,282.4 283.0,282.4 268.6,203.2 211.0,203.2" style="fill:color-mix(in srgb,var(--kit-c-danger) 30%,transparent);stroke:var(--kit-c-danger);stroke-width:1.5"/><text x="247" y="247" text-anchor="middle" class="hf-t">D</text><polygon points="251.6,162.1 250.8,157.9 249.4,153.6 247.2,149.3 244.3,145.0 240.8,140.9 236.7,136.8 232.0,132.9 226.7,129.1 220.8,125.5 214.5,122.1 207.7,118.9 200.5,115.9 192.9,113.2 185.0,110.7 176.8,108.6 168.3,106.8 159.7,105.2 151.0,104.0 142.3,103.1 133.5,102.6 124.8,102.4 116.2,102.5 107.7,103.0 99.4,103.8 91.5,105.0 83.8,106.5 76.5,108.3 69.6,110.4 63.2,112.8 57.2,115.4 51.8,118.4 47.0,121.5 42.7,124.9 39.1,128.5 36.1,132.3 33.8,136.2 32.2,140.2 31.3,144.4 31.0,148.6 31.5,152.9 32.6,157.2 34.4,161.4 37.0,165.7 40.1,169.9 43.9,174.0 48.4,178.0 53.4,181.9 59.0,185.6 65.1,189.1 71.6,192.4 78.7,195.5 86.1,198.4 93.8,200.9 101.9,203.2 110.2,205.2 118.7,206.9 127.4,208.3 136.1,209.3 144.9,210.0 153.7,210.4 162.3,210.4 170.9,210.1 179.3,209.5 187.4,208.5 195.2,207.2 202.7,205.5 209.8,203.6 216.5,201.3 222.7,198.8 228.3,196.0 233.5,193.0 238.0,189.7 242.0,186.2 245.3,182.5 247.9,178.7 249.9,174.7 251.2,170.6 251.8,166.4 251.6,162.1" style="fill:color-mix(in srgb,var(--kit-c-tip) 25%,transparent);stroke:var(--kit-c-tip);stroke-width:2"/><circle cx="175.0" cy="160.0" r="4" style="fill:var(--kit-fg)"/><text x="181" y="176" class="hf-t">seed</text><text x="410" y="60" class="hf-h">IRIS로 키운 영역 (2차원)</text><text x="410" y="88" class="hf-key pq">■ 초록 타원: 영역 안의 최대 부피 타원</text><text x="410" y="110" class="hf-key a">■ 갈색 다각형: 초평면 4개 영역 (4.42)</text><text x="410" y="132" class="hf-key rl">■ 빨강: 장애물 A, B, C, D</text><text x="410" y="154" class="hf-t"></text><text x="410" y="176" class="hf-t">반복마다: 장애물까지 최근접점 QP → 접평면</text><text x="410" y="198" class="hf-t">→ 그 평면들 안의 최대 타원 → 다시 반복</text><text x="410" y="220" class="hf-t">타원 면적: 2.26 → 3.48 → 3.57 → 3.57 …</text></svg><figcaption>그림 3. 장애물 4개가 있는 2차원 공간에서 원점 근처 씨앗(반지름 0.05의 원)을 IRIS로 부풀린 결과. 초록 타원은 갈색 영역 안의 최대 타원이다.</figcaption></figure>

| 반복 | 초평면 수 | 타원 면적 | 타원 중심 |
|---|---|---|---|
| 1 | 4 | 2.26 | (-0.157, 0.020) |
| 2 | 4 | 3.48 | (-0.454, 0.046) |
| 3 | 4 | 3.57 | (-0.468, 0.050) |
| 4~6 | 4 | 3.57 | (-0.467, 0.050) |

- 면적이 3번 만에 거의 수렴했습니다. 영역(갈색 다각형) 면적은 4.42이고 장애물 4개 모두 어느 평면의 바깥에 있음을 확인했습니다.
- 영역은 **볼록**이라 안의 어느 두 점을 직선으로 이어도 충돌이 없습니다. 이것이 GCS의 입력이 됩니다(6번).
- 한계: 여기서는 장애물이 **볼록 다각형**이라 정확한 QP로 풉니다. 로봇 설정 공간(비볼록)에서는 IRIS-NP 같은 변형이 필요합니다([16](16_motion_planning_concepts.md) 5절).

## 5. Plan and Place: 키프레임, IK, RRT-Connect, shortcutting

이 문제는 **이전 장의 글자 조작 과제 자산**(물체 모델, 환경)을 이어 쓰므로 노트북 없이는 실행할 수 없습니다. 네 단계의 설계와 점검 사항을 정리합니다. 2번과 3번의 구현이 그대로 재사용됩니다.

| 단계 | 할 일 | 점검할 것 |
|---|---|---|
| ① 키프레임 정의 | 집기 전 접근, 집기, 들어 올리기, 놓기 전 접근, 놓기 같은 **손의 자세 목록** | 각 자세가 물체 프레임 기준으로 일관된가 |
| ② 키프레임마다 IK | 4장 문제 1처럼 최적화 IK로 **관절 자세**를 구함 | 충돌 제약을 넣었나, 실패한 키프레임은 없나 |
| ③ 키프레임 사이를 RRT-Connect | 관절 공간에서 충돌 없는 경로를 찾음 | 충돌 검사 해상도, 시도 횟수 |
| ④ shortcutting | 경로를 다듬음 | 반복 횟수, 줄어든 길이 |

- IK를 풀 때 **초기값을 이전 키프레임 해**로 주면 관절이 크게 튀지 않습니다(1번 문 열기 결과의 "이전 해와의 최대 관절 변화"가 이 효과를 보여 줍니다).
- 점 로봇에서의 길이 개선은 위 표(RRT 4.60 → 3.84, RRT-Connect 4.66 → 3.74)가 참고가 됩니다. 7차원 팔에서의 개선 폭은 **직접 확인하지 않았습니다.**

## 6. IRIS and GCS in Joint Space: 영역으로 덮고 GCS로 궤적 만들기

원문의 단계: ① 시작·끝 근처에서 **관절 공간 IRIS 영역**을 만들고, ② 그 영역들을 지나는 **GCS 궤적 계획** 문제를 세워 푸는 것. iiwa 관절 공간의 IRIS 영역은 선반 모델이 필요해 실행하지 않았고, **같은 구조를 2차원에서 풀었습니다.**

### 영역

자유 공간 `|x|+|y| ≥ 1`(`[-2,2]²` 안)은 **네 개의 반평면**의 합집합입니다: `x+y ≥ 1`, `-x+y ≥ 1`, `x-y ≥ 1`, `-x-y ≥ 1`. 각각이 볼록 영역이고, 이웃한 두 영역은 겹칩니다(위쪽 두 영역은 `y ≥ 1` 부근에서 만남).

### 구성과 결과

```python
gcs = GcsTrajectoryOptimization(2)
regions = gcs.AddRegions(four_half_planes, order=1)    # order 1: straight segments in each region
source = gcs.AddRegions([Point(start)], order=0)
target = gcs.AddRegions([Point(goal)], order=0)
gcs.AddEdges(source, regions); gcs.AddEdges(regions, target); gcs.AddEdges(regions, regions)
gcs.AddPathLengthCost()
traj, result = gcs.SolvePath(source, target, GraphOfConvexSetsOptions())
```

| 항목 | 결과 |
|---|---|
| 풀이 | 성공 (Clarabel, 볼록 완화 + 반올림) |
| 경로 길이 | **3.2297** (이론 최단 3.2311, 샘플링 오차) |
| 400개 샘플 충돌 없음 | **예** |
| 장애물 꼭짓점 (0, 1) 근처 통과 | (−0.007, 0.997) |

- [16](16_motion_planning_concepts.md) 3절 그림의 궤적 최적화는 웨이포인트에만 제약을 걸어 **3.160**(최단보다 짧음, 즉 모서리를 파고듦)이 나왔는데, GCS는 영역 안에 **연속 경로 전체**가 있음을 보장해 3.2297이 나왔습니다.
- 시작·목표를 어느 영역에서 어느 영역으로 건너는지(이산 선택)도 GCS가 같이 골랐습니다(위쪽 영역을 지남).
- iiwa에서는 영역이 7차원이고 `AddRegions`의 `order`를 3~5(Bézier 곡선)로 올려 미분 연속성과 속도 제약을 겁니다([16](16_motion_planning_concepts.md) 6절). 이 부분은 **실행하지 않았습니다.**

## 7. Trajectory Optimization Around a Shelf Exterior: 환경과 초기값

원문의 단계: ① 충돌 제약 **없이** iiwa 전체 관절 공간에서 기구학 궤적 최적화를 풀기("karate chop" 궤적), ② 충돌 제약을 더하고 ①의 궤적을 **초기값**으로 써서 **왜 기본 솔버 설정 안에서 수렴하지 않는지 진단**, ③ **더 좋은 초기값**이 수렴을 빠르게 하는 이유 조사. 선반과 iiwa 자산이 없어 **실행하지 않았고**, 원리를 2차원 예제로 보이고 질문에는 개념으로 답합니다.

### 진단: 왜 ②가 수렴하지 않나 (개념, iiwa에서 확인한 것은 아님)

| 원인 | 설명 |
|---|---|
| 초기값이 충돌 영역을 통과 | "karate chop"은 충돌 제약이 없을 때 최단이므로 **선반을 뚫고** 지나갑니다. 시작부터 제약을 크게 어긴 상태입니다 |
| 충돌 제약은 비볼록 | 설정 공간에서 선반이 복잡한 덩어리라 어느 쪽으로 돌아갈지 결정이 필요한데, 국소 솔버는 초기값 근처만 봅니다 |
| 어긴 상태에서 벗어나는 길이 길다 | 제약을 어긴 점에서 만족하는 점까지 가는 동안 비용이 나빠졌다 좋아졌다 하는 구간이 있고 반복이 많이 듭니다 |
| 작업 공간 경계 | 팔이 닿는 거리의 끝(경계) 근처에서는 해의 여유가 없습니다 |
| 충돌 제약은 점 위주 | 웨이포인트 사이를 놓치면 한 번 푼 뒤에도 충돌이 남고 재시도가 필요합니다([15](15_bspline_and_augmented_lagrangian.md) 2.7절) |

### 더 좋은 초기값이란 (개념)

- 선반 바깥을 **돌아가는** 웨이포인트를 몇 개 주고 그 사이를 보간합니다(키프레임).
- RRT-Connect나 GCS의 경로를 초기값으로 씁니다(전역 방법이 어느 쪽으로 돌지 정하고, 최적화가 다듬는다).
- 충돌 없는 상태 근처에서 시작하면 제약을 거의 어기지 않아 **반복 횟수가 줄고 같은 골짜기에 머뭅니다.**

### 2차원에서 확인한 것

[16](16_motion_planning_concepts.md) 3절 그림(시작 (-1.5, 0.4), 목표 (1.5, 0.4), 웨이포인트 17개)과 같은 설정입니다.

| 초기값 | 결과 | 비용 |
|---|---|---|
| 위쪽으로 휘어진 곡선 | 위쪽 해 | 0.629 |
| 아래쪽으로 휘어진 곡선 | 아래쪽 해(국소 최소) | 0.994 |
| 장애물을 통과하는 직선 | 위쪽 해 | 0.629 |

- 초기값이 **어느 해로 가는지**를 정합니다(위/아래).
- 반대로 "장애물을 통과하는 직선"에서는 이 2차원 예제의 SNOPT가 **잘 수렴했습니다.** 문제가 작고 장애물이 하나여서입니다. 원문 ②처럼 수렴이 **실패**하는 것은 7차원 팔과 선반 같은 복잡한 충돌 구조에서의 이야기로, 이 예제가 재현한 것은 아닙니다. 이 한계를 꼭 기억하세요.

## 이 문서에서 확인한 것과 확인하지 못한 것

| 구분 | 내용 |
|---|---|
| 직접 실행해서 확인 | iiwa IK 59/60 대 33/60, 문 열기 8개 각도, RRT/RRT-Connect 100회, 최단 경로와 지름길 후보, RRT\* 부모 비용, IRIS 한 바퀴(2차원), GCS(2차원), 궤적 최적화 초기값 비교 |
| 그림에서 눈대중으로 읽음 | 3번의 노드와 격자 좌표 (결론이 민감한 곳은 표시함) |
| 확인하지 못함 | 과제 노트북의 정확한 입력·정답, Kuka 팔에서의 RRT·IRIS·GCS, 선반 환경의 궤적 최적화, 5번의 전체 파이프라인 |

## 참고
- MIT Robotic Manipulation, 6장 [Motion Planning (trajectories.html)](https://manipulation.csail.mit.edu/trajectories.html)의 *Exercises* 절과 그림 `data/shortcutting.png`, `data/rrtstar_step.png` · [교재·강의 페이지](../reference/r3_courses.md#manipulation)
- 코드: `scripts/planning_2d.py`(`.venv/bin/python scripts/planning_2d.py`로 전체 결과 출력, `--figs DIR`로 그림), `scripts/planning_drake.py`(`ik`, `gcs`)
- 개념 정리: [16. 모션 플래닝 핵심 개념](16_motion_planning_concepts.md)
