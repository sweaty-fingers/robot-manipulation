---
title: 용어집
description: 이 프로젝트 문서에 나오는 로봇 조작·센싱·학습 용어를 한곳에 정리. 처음 보는 용어는 여기서 확인.
---

가나다·알파벳 순이 아니라 **주제별**로 묶었습니다. 정의는 이 프로젝트 맥락에 맞춘 설명이며, 엄밀한 교과서 정의와 다를 수 있습니다.

## 센싱 — 무엇을 아는가

### Proprioception (고유수용감각, "자기 몸 상태 감각")
**로봇 자신의 몸이 지금 어떤 상태인지를 재는 내부 센서 신호.** 관절 각도(엔코더), 관절 속도, 그리고 흔히 모터 전류·토크(노력, effort)가 여기에 속합니다.
사람이 눈을 감고도 자기 팔이 어디 있는지 아는 감각과 같은 개념입니다.
- **알려주는 것**: 손가락이 어디에 있고 얼마나 빠르게 움직이는가.
- **알려주지 않는 것**: 물체가 어디 있는지, 손가락이 물체에 닿았는지(직접적으로는).
- **이 프로젝트에서**: `env.step()`이 돌려주는 `obs["proprio"]`는 손가락 3개의 위치 3 + 속도 3 = 6차원입니다. 물체 정보는 들어 있지 않습니다.
- 이름이 낯설지만 "관절 센서 값"이라고 보면 됩니다. 접촉 정보를 간접적으로 알아낼 수 있는 이유는 [촉각 없이 접촉을 다루는 법](../touchless-grasping/) 참고.

### Exteroception (외수용감각)
**바깥 세계를 재는 센서.** 카메라(RGB), 깊이 카메라, 라이다 등. 물체 위치를 알려줍니다. proprioception과 짝이 되는 말.

### Tactile sensing (촉각)
**손가락이 물체와 닿은 곳의 접촉을 재는 센서.** 닿았는지, 얼마나 세게 누르는지(법선력), 옆으로 미끄러지는지(전단력/슬립), 접촉 면의 모양까지 재는 종류도 있습니다.
종류: 압력·전극 방식(예: BioTac), 고무 표면을 카메라로 찍는 **광학식**(GelSight·DIGIT·TacTip 계열). 이 프로젝트는 처음엔 쓰지 않고 이후 확장을 고려합니다.

### Joint encoder / Effort(노력)·Torque
- **Joint encoder**: 관절이 얼마나 돌았는지 재는 센서. proprioception의 기본.
- **Effort / Torque**: 관절을 돌리는 힘. 토크 센서가 없는 손도 모터 전류를 토크로 환산(보정)해 쓰기도 합니다.

### Depth image / Point cloud (깊이 이미지 / 포인트클라우드)
깊이 이미지는 픽셀마다 거리 값을 가진 이미지, 포인트클라우드는 그것을 3차원 점들의 집합으로 바꾼 것. RGB보다 시뮬레이션과 실제의 차이가 작다고 알려져 Visual Dexterity가 이걸 씁니다.

## 로봇과 제어

### DoF (Degree of Freedom, 자유도)
독립적으로 움직일 수 있는 방향의 수. 16-DoF 손은 관절 16개를 따로 움직일 수 있다는 뜻.

### Underactuated (언더액추에이티드)
**움직일 수 있는 관절 수보다 모터 수가 적은 구조.** 손가락 하나의 여러 관절을 모터 하나(와이어/연동)로 같이 움직이는 식. 가볍고 싸지만 접촉에서 손가락이 물체 모양을 따라 알아서 휘고, 그 접촉은 직접 제어하기 어렵습니다.
(이 프로젝트의 Phase B 환경은 손가락당 1자유도라 "언더액추에이티드"를 단순화한 모형입니다.)

### Forward kinematics (순기구학)
관절 각도로부터 손가락 끝이 3차원 어디에 있는지 계산하는 것. proprioception에서 손끝 위치를 얻는 방법.

### Position control / PD control
**위치 제어**: "관절을 이 각도로 가라"고 명령. **PD 제어**는 목표와의 오차(P)와 속도(D)로 힘을 내는 가장 기본 방식. Gain(kp, kd)이 클수록 목표에 강하게 끌려가므로 **접촉했을 때 세게 민다**.

### Impedance / Compliance (임피던스 / 순응성)
**위치 오차에 비례해 힘을 내도록 일부러 "스프링처럼" 만든 제어(낮은 강성).** 접촉 타이밍이 어긋나도 과한 힘이 걸리지 않습니다. 하드웨어 자체가 말랑한 경우(소프트 손끝)도 순응성이라고 부릅니다.

### Time-to-contact (접촉까지 시간)
손가락 끝과 물체 표면 사이 거리 ÷ 접근 속도. 이게 손가락마다 다르면 닿는 시점이 어긋난다(= 비동기 접촉).

## 접촉과 파지

### Prehensile / Nonprehensile
**Prehensile**: 쥐기(손가락으로 물체를 잡음). **Nonprehensile**: 쥐지 않고 밀기·굴리기 등. 이 프로젝트가 피하려는 "밀림"은 의도치 않은 비파지 동작입니다.

### Pre-shaping (프리셰이핑)
물체에 닿기 전에 손이 물체 형상에 맞춰 미리 모양을 잡는 것. 사람 손의 특징으로 알려져 있음.

### Asynchronous contact (비동기 접촉)
손가락들이 서로 다른 시각에 물체에 닿는 상황. 먼저 닿은 손가락이 계속 밀면 물체가 움직이기 쉽습니다. 이 프로젝트의 핵심 문제.

### Force closure
접촉점들이 낼 수 있는 힘으로 물체에 어떤 방향의 외력이 걸려도 버틸 수 있는 파지 조건. 파지 안정성의 고전적 기준.

### Coulomb friction (쿨롱 마찰)
마찰력이 법선력에 비례(마찰계수 μ)하고 그 이하일 때만 미끄러지지 않는다는 모델. 시뮬레이션과 실제의 마찰 차이가 sim-to-real 문제의 주요 원인.

### Contact-rich
접촉이 많고 중요한 과제(삽입, 재배치, 파지 등). 위치만 맞추면 되는 pick-and-place와 대비.

### Point contact vs Hydroelastic (Drake의 접촉 모델)
**Point contact**: 접촉을 점 하나(와 힘)로 근사하는 기본 모델, 빠름. **Hydroelastic**: 물체 안에 압력장을 정의해 접촉면 전체의 분포 힘을 계산, 더 정확하고 느림. 우리 환경은 현재 point contact만 사용([시뮬레이션 도구](../../references/simulation-tools/)).

### In-hand reorientation
이미 쥔 물체를 손 안에서 다른 방향으로 돌리는 과제. Dactyl, Visual Dexterity가 대표적.

### COM displacement (무게중심 변위)
접촉 시퀀스 동안 물체 무게중심이 처음 위치에서 이동한 거리. 이 프로젝트가 "무교란"을 측정하려는 후보 지표(Phase A).

## 학습

### RL (Reinforcement Learning, 강화학습)
보상을 최대화하도록 시행착오로 정책을 학습. 시뮬레이션에서 대량으로 돌린다. PPO는 대표 알고리즘.

### Imitation learning / BC (모방학습 / Behavior Cloning)
시연(상태→행동 쌍)을 지도학습으로 따라 하게 학습. Phase 0 파이프라인이 BC입니다.

### Teacher-student / Privileged information / Distillation
**Teacher**는 시뮬레이션에서만 알 수 있는 정보(**privileged**: 물체 속도, 접촉력 등)를 입력으로 받아 쉽게 학습. **Student**는 실제로 얻을 수 있는 관측(비전, proprioception)만 받아 teacher의 행동을 모방(**distillation**, DAgger 등). 실제에서 못 재는 정보를 학습 과정에서만 빌려 쓰는 방법.

### Asymmetric actor-critic
actor(행동을 내는 쪽)는 배포 가능한 관측만, critic(가치를 평가하는 쪽)은 특권 정보까지 보는 구조. teacher-student와 같은 취지의 다른 방식.

### Sim-to-real gap / Domain randomization / System identification
- **Sim-to-real gap**: 시뮬레이션에서 잘 되던 정책이 실제에서 떨어지는 현상.
- **Domain randomization**: 시뮬레이션의 마찰·질량 등을 무작위로 바꿔 학습해 강건하게.
- **System identification**: 실제 로봇 움직임과 맞도록 시뮬레이션 파라미터를 조정.
- **Zero-shot transfer**: 실기에서 추가 학습 없이 바로 적용.

### VLA (Vision-Language-Action model)
이미지와 언어 지시를 받아 로봇 행동을 직접 출력하는 대형 모델(OpenVLA, π0 등). 의미 이해는 강하지만 빠른 접촉 반응에는 약하다는 점이 이 프로젝트의 출발점([연구 지형](../../references/landscape/)).

### Flow matching / Diffusion policy
행동을 한 번에 하나씩이 아니라 노이즈에서 점진적으로 만들어내는 생성 모델 기반 정책 구조. π0가 flow matching을 씁니다.

### Eigengrasp
사람 손 관절의 주요 공통 움직임 몇 개(주성분)로 손 모양을 표현하는 저차원 행동 공간.
