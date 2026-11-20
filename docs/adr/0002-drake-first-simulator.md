---
status: accepted
date: 2026-11-20
deciders: [yoon]
area: sim
summary: 첫 시뮬레이션 백엔드로 Drake를 쓰고, MuJoCo는 RobotEnv 뒤에 나중에 더한다
c4: [envs, drake]
related: [1]
---

# ADR-0002. 첫 시뮬레이션 백엔드로 Drake를 쓴다

## 배경

- `robot_manipulation`은 시뮬레이터를 `RobotEnv` 인터페이스 뒤에 두고 `factory.py`의 등록 한 줄로 바꾸도록 만들었습니다(`robot_manipulation/factory.py`).
- 지금 구현된 백엔드는 Drake 둘입니다: `DrakeEnv`(point-mass reach, Phase 0), `ThreeFingerGraspEnv`(3손가락 파지, Phase B). MuJoCo는 `pyproject.toml`의 `mujoco` extras에 의존성만 있고 `MuJoCoEnv`는 없습니다([배경 09](../background/09_drake_mujoco.md)).
- 연구 과제(무교란 파지)는 손가락이 비동기로 닿을 때 물체가 밀리는지를 접촉 결과로 재야 합니다([EXP-0001](../experiments/0001-async-contact-push.md)).
- 이 결정은 이미 코드에 반영되어 있던 선택을 2026-11-20에 기록으로 남긴 것입니다.

## 결정 요인

1. 접촉과 물리를 정밀하게 볼 수 있는가 (물체 변위, 접촉 결과)
2. 개인 학습 목적에 맞는가 (최적화 기반 planning·제어를 함께 공부)
3. 나중에 다른 시뮬레이터로 옮기기 쉬운가

## 고려한 선택지

### 1. Drake 먼저 (채택)

- **장점**: 정밀한 다물체 동역학과 접촉 모델(hydroelastic 등), 최적화 기반 planning·제어 도구가 함께 있음([배경 09](../background/09_drake_mujoco.md))
- **단점**: action-expert·VLA 연구 생태계(로봇·그리퍼 모델, 벤치마크, 데이터셋)는 MuJoCo 쪽이 훨씬 큼. 대량 병렬 롤아웃은 MuJoCo(+MJX)가 유리
- **채택 이유**: 결정 요인 1·2. 3은 `RobotEnv` 추상화로 지킴

### 2. MuJoCo 먼저

- **장점**: 빠른 시뮬레이션, RL 생태계(`dm_control`, `gymnasium`), MJX 병렬화
- **단점**: 접촉은 soft contact 근사
- **기각 사유**: 지금 단계의 질문은 대량 학습보다 접촉 현상의 확인(결정 요인 1)이고, 학습 목적(결정 요인 2)과도 Drake가 맞음

### 3. 둘을 처음부터 함께

- **장점**: MuJoCo로 학습, Drake로 정밀 검증하는 흔한 역할 분리
- **단점**: 초기 작업량이 두 배, 추상화가 맞는지 확인하기 전에 두 구현을 유지
- **기각 사유**: 추상화(`RobotEnv`)가 성립하는지 두 번째 백엔드를 붙일 때 확인하면 됨

## 결정

**Drake를 첫 시뮬레이션 백엔드로 쓴다.** MuJoCo는 `RobotEnv` 인터페이스를 같게 구현하는 `MuJoCoEnv`로 나중에 더하고, 그때 추상화가 실제로 성립하는지 확인한다.

## 선택지 비교

| 결정 요인 | Drake 먼저 | MuJoCo 먼저 | 함께 |
| --- | --- | --- | --- |
| 1. 접촉·물리 정밀도 | ○ 정밀 접촉 모델 | △ soft contact 근사 | ○ |
| 2. 학습 목적 | ○ planning·제어 도구 | △ | △ 작업량 큼 |
| 3. 옮기기 쉬움 | ○ `RobotEnv` 뒤 | ○ | △ 둘 다 유지 |

## 결과

**좋아지는 점**

- Phase B 환경에서 비동기 접촉에 의한 밀림을 접촉 결과와 물체 변위로 바로 잴 수 있음

**감수할 점**

- 지금 환경은 도형에 hydroelastic 속성을 주지 않아 point contact만 쓰임([R5](../reference/r5_simulation_tools.md))
- MuJoCo 생태계의 로봇 모델·벤치마크를 바로 쓰지 못함

**후속 작업**

- hydroelastic 접촉과 point contact 결과 비교
- `MuJoCoEnv` 추가로 `RobotEnv` 추상화 검증

## 재검토 조건

- 대량 롤아웃 학습(RL/IL)이 중심 작업이 될 때
- LEAP Hand 모델을 시뮬레이터에 올리는 단계에서 모델 파일(URDF/MJCF) 지원이 한쪽에만 있을 때

## 참고

- [배경 09. Drake / MuJoCo 개요](../background/09_drake_mujoco.md)
- `robot_manipulation/factory.py`, `robot_manipulation/envs/`
