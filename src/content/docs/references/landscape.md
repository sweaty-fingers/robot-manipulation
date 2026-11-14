---
title: 연구 지형 — 무교란 파지
description: 비동기 접촉에서 물체를 밀지 않고 집는 문제를 단계로 분해하고, 접근 방식별 선행연구와 빈 곳을 정리.
---

> 조사 범위: arXiv 키워드 검색(초록 수준) + 일부 웹 검색, 2026-11-14.
> 이 문서의 "빈 곳"은 **내가 쓴 검색어로 찾지 못했다**는 뜻이지 새롭다는 증명이 아닙니다. 아래 [한계](#조사의-한계)를 먼저 보세요.

## 1. 문제를 단계로 쪼개기

"무교란"은 접촉이 일어나는 구간 전체의 성질이라, 먼저 어느 구간 이야기인지 나눕니다.

| 단계 | 일어나는 일 | 물체가 밀리는 이유 |
|---|---|---|
| ① 접근·프리셰이핑 | 손이 물체 형상에 맞춰 모양을 잡으며 다가감 | 아직 접촉 없음. 여기서 손가락 도착 시점이 정해짐 |
| ② 첫 접촉 | 가장 먼저 닿은 손가락이 힘을 가함 | 다른 손가락이 아직 없으므로 마찰만으로 버텨야 함 |
| ③ 비동기 구간 | 나머지 손가락이 차례로 도착 | 먼저 닿은 손가락이 계속 밀면 물체가 이동 |
| ④ 폐쇄(closure) | 모든 손가락이 닿고 힘이 균형을 이룸 | 힘이 비대칭이면 마지막까지 미끄러짐 |
| ⑤ 안정 파지 | 외란에도 유지 | — |

이 레포의 Phase B 환경([`grasp_env.py`](https://github.com/sweaty-fingers/robot-manipulation/blob/main/robot_manipulation/envs/grasp_env.py))이
재현한 것이 ②–③입니다. 손가락 시작 시차를 0 → 1.0초로 늘리면 물체 변위가
0.01mm → 16.4mm(물체 반지름 30mm의 절반 수준)로 커졌습니다
(`scripts/reproduce_async_push.py`, 단순 위치제어 베이스라인, 1회 시드 기준 — 통계는 아직 없음).

## 2. 접근 방식별 선행연구

### (a) 접촉 전에 해결한다 — 닫는 동작 자체를 설계

- [Grasping by Interconnection](../papers/#grasping-by-interconnection): 피드백 없이도 닫는 동작이 객체 크기 오차 약 1cm, 자세 오차 수 cm를 견디게 설계. **순응적이고 미끄러지는 접촉**을 전제로 함. 무교란 문제와 가장 가까운 설계 철학.
- [Learning Grasps for Underactuated Hands](../papers/#underactuated-hand-grasp-learning): 언더액추에이티드 손은 접촉을 피할 수 없고 오히려 **접촉을 이용해 평형에 도달**함. 접촉 상호작용은 직접 제어되지 않는다는 점을 명시.

### (b) 순응성으로 흡수한다 — impedance / compliant

- 위 두 논문이 이 축에도 걸침. 이 프로젝트의 베이스라인 ②(impedance 그립)에 대응.
- [Transferring Contact, Not Just Motion](../papers/#transferring-contact-not-just-motion): 힘-위치 하이브리드 컨트롤러와 손 간 접촉 하중 표현의 호환.

### (c) 센싱해서 반응한다

- **촉각 기반**: [Grasping in the Dark](../papers/#grasping-in-the-dark)(슬립 감지), [Shear-based Grasp Control](../papers/#shear-based-grasp-control), [T-Rex](../papers/#t-rex). 접촉 **후** 반응이 주 대상.
- **촉각 없이(proprioception만)**: [FEEL](../papers/#feel-finger-force-estimation)(관절 신호에서 손가락별 외력 추정, 0–20N 범위 RMSE 0.251N), [Real-Time Force Regulation](../papers/#whole-hand-force-regulation)(추적된 물체 모델 + proprioception으로 접촉 추정). 우리가 촉각 센서를 쓰지 않기로 한 결정과 같은 방향.

### (d) 시뮬레이션에서 학습한다

- [Dactyl](../papers/#dactyl) → [Visual Dexterity](../papers/#visual-dexterity) → [Sim-to-Real Humanoid RL](../papers/#sim-to-real-humanoid-rl). 모두 **재배치/파지 후 조작**이 주제이고, 접촉 순간의 교란 최소화를 목표로 삼은 것은 아님(초록 기준).
- [Closing the Reality Gap](../papers/#closing-the-reality-gap): 촉각+토크 센싱을 시뮬레이션에서 학습해 전이. 접촉력 추종 가능.

### (e) 대규모 모델(VLA)은 접촉에서 약하다

- [Demystifying VLA Failures](../papers/#demystifying-vla-failures)는 실패를 **정밀도 실패**(접촉 중 작은 보정 동작을 못 냄)와 **힘 실패**(접촉력에 반응 못 함)로 분류. 이 프로젝트의 "VLA는 접촉 구간에 약하다"는 전제의 근거 자료.
- 해결 시도: [ForceVLA](../papers/#forcevla), [FD-VLA](../papers/#fd-vla)(힘 센서 없이 힘 토큰을 증류), [T-Rex](../papers/#t-rex).

## 3. 이 프로젝트에 주는 시사점

1. **베이스라인 매핑**: ① 단순 위치제어 = 현재 Phase B 결과 / ② impedance = (a)(b) 계열 / ③ 접촉력 최적화 = [Real-Time Force Regulation](../papers/#whole-hand-force-regulation) 계열.
2. **지표 설계**: 선행연구들의 지표는 대개 파지 *성공률*, 유지 시간, 재배치 오차입니다. 접촉 시퀀스 동안의 **물체 변위**와 **시차에 대한 강건성**을 1급 지표로 두는 것은 Phase A에서 우리가 정의해야 할 몫.
3. **촉각 없는 접촉 추정**은 연구 자체가 있습니다(FEEL 등). 센서 없이 가겠다는 결정이 고립된 선택이 아님.
4. **하드웨어·센서 가정**: LEAP Hand(촉각 없음)로 시작하되 촉각은 이후 확장 대상. 단계 계획은 [촉각 없이 접촉을 다루는 법](../../concepts/touchless-grasping/#6-촉각을-나중에-더하는-경로-제안), 손과 센서는 [하드웨어 문서](../hardware/).

## 조사의 한계

- 검색은 arXiv 초록과 키워드에 의존했습니다. "비동기 접촉 중 물체 변위"를 직접 지표로 삼는 논문을
  `object displacement`, `contact-induced motion`, `finger closing synchronization`, `gentle grasping` 등으로 찾았지만
  **발견하지 못했습니다.** 다른 용어(예: grasp acquisition, contact transient, impact, pre-grasp disturbance)나
  비-arXiv 자료(ICRA/IROS 고전, 인간공학·신경과학 문헌)는 아직 조사하지 않았습니다.
- 따라서 새롭다고 주장하기 전에 용어를 넓힌 추가 검색과 Google Scholar 확인이 필요합니다.
- 인간 생리 근거(예측적 그립력 제어, 접촉 후 반사 지연 등)는 이전 브리프에 정리돼 있으나 이번에 원문을 확인하지 못했습니다. [논문 목록](../papers/#인간-감각운동-제어)의 해당 항목 참고.
- 어떤 논문도 본문·실험 세부까지 읽지 않았습니다. 수치 인용 전 원문 확인.
