---
title: Drake / MuJoCo 개요
description: 시뮬레이터 선택 기준과 Drake, MuJoCo의 차이.
---

Manipulation 모델을 실기(real robot)에 바로 적용하기 전, 시뮬레이터에서 학습·검증하는 단계가 데이터 수집 비용과 안전성 측면에서 필수적입니다.

## Drake vs MuJoCo

| | **Drake** | **MuJoCo** |
| --- | --- | --- |
| 개발 | Toyota Research Institute (TRI) | Google DeepMind |
| 강점 | 정밀한 다물체 동역학, 최적화 기반 planning/control (trajectory optimization), contact modeling의 수학적 엄밀성 | 빠른 시뮬레이션 속도, RL 학습에 최적화, 방대한 생태계(`dm_control`, `gymnasium`) |
| 접촉(contact) 모델 | hydroelastic contact 등 정밀 모델 제공 | soft contact, 속도 대비 근사 |
| 언어 바인딩 | Python, C++ | Python (`mujoco`), MJX(JAX 기반 병렬 시뮬레이션) |
| 주 사용처 | manipulation planning, 제어기 설계/검증 | 대규모 RL 학습, 병렬 롤아웃 |

**선택 기준 메모**: 정밀한 접촉/물리 검증이나 최적화 기반 planning이 중심이면 Drake, 대량 롤아웃으로 policy를 학습(RL/IL)하는 게 중심이면 MuJoCo(+MJX)가 유리. 두 시뮬레이터를 역할 분리해서 같이 쓰는 것도 흔한 패턴(예: MuJoCo로 policy 학습 → Drake로 정밀 검증).

## 이 프로젝트에서의 역할

1. **학습(training)**: 시뮬레이터에서 대량의 rollout을 생성해 policy를 학습하거나, 실기 데이터 부족분을 보강(domain randomization).
2. **검증(validation)**: 학습된 policy를 실기에 배포하기 전, 시뮬레이터에서 먼저 반복 검증 — 성공률, 실패 모드 분석.
3. **sim-to-real gap 분석**: 시뮬레이션과 실기 간 동작 차이를 rerun.io 로그로 비교 ([데이터 파이프라인](../../data-pipeline/rerun-overview/) 참고).

:::note[TODO]
실제 사용 중인 시뮬레이터/버전, 로봇 모델(URDF/MJCF) 경로, domain randomization 파라미터를 여기에 기록.
:::

다음: [시뮬레이션 환경 구축](../sim-environment-setup/)
