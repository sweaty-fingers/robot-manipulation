---
description: Robot manipulation, 모델 학습/검증, rerun.io 기반 재학습 데이터 파이프라인을 정리하는 개인 학습 노트.
---

# Robot Manipulation Lab

Robot manipulation 개념부터 실제 제어 모델 학습·검증, rerun.io 기반 재학습 데이터 파이프라인까지 — 코드와 실습(Colab) 예제를 포함한 학습 노트입니다. 연구 목표는 **무교란 파지(non-perturbing grasp)**: 손가락이 비동기로 닿아도 물체를 밀지 않고 빠르게 쥐는 것입니다(시뮬레이션 Drake, 목표 하드웨어 LEAP Hand).

[개념부터 시작하기](background/01_manipulation_basics.md) · [실습 가이드 보기](guides/g2_colab.md)

## 1. 시작하기

=== "로컬"
    ```bash
    git clone --recurse-submodules https://github.com/sweaty-fingers/robot-manipulation.git
    cd robot-manipulation
    uv venv .venv && uv pip install --python .venv/bin/python -e ".[dev]"
    python scripts/collect_demos.py --config configs/experiments/00_skeleton.yaml --n-episodes 50
    python scripts/train.py --config configs/experiments/00_skeleton.yaml
    ```

=== "Colab (설치 불필요)"
    [Point-Mass Reach 예제](guides/g4_example_reach.md)의 Colab 배지를 눌러 브라우저에서 바로 실행하세요 — 로컬 환경 설정이 전혀 필요 없습니다.

## 2. 어디서 무엇을 보나

| 영역 | 내용 | 보는 곳 |
| --- | --- | --- |
| 개념 | Robot manipulation의 핵심 개념 — 좌표계, 역기구학, 파지(grasping), 지각-제어 루프 | 배경 [01](background/01_manipulation_basics.md)~[08](background/08_antipodal_hessian.md) |
| 시뮬레이션 | Drake / MuJoCo 기반 시뮬레이터, sim-to-real 검증 전략 | 배경 [09](background/09_drake_mujoco.md), [10](background/10_sim_to_real_validation.md), 가이드 [G3](guides/g3_sim_environment.md) |
| 모델 학습 & 검증 | 실제 제어를 위한 모델 학습 파이프라인과 검증 지표 | 배경 [11](background/11_model_training.md), [12](background/12_model_validation.md) |
| 데이터 파이프라인 | rerun.io 기반 로그 수집 → 재학습 루프 설계 | 배경 [13](background/13_rerun.md), [14](background/14_retraining_loop.md) |
| 자료 | 무교란 파지 관련 논문·하드웨어·도구 모음과 연구 지형 | 참고 [사용법](reference/README.md), [R1 연구 지형](reference/r1_landscape.md), [R2 논문 목록](reference/r2_papers.md) |
| 가이드 & 예제 | 로컬/Colab 환경 설정과 Point-Mass Reach 실습 예제 | 실행 가이드 [G1](guides/g1_setup.md)~[G4](guides/g4_example_reach.md) |
| 용어 | 문서에 나오는 용어의 한 줄 뜻 | [용어집](glossary/README.md) |

이 사이트는 [docs-kit](https://github.com/sweaty-fingers/docs-kit)(공통 문서 규격)으로 만듭니다. 문서를 어디에 어떻게 쓰는지는 [문서 규격](docs-standard.md), 사이트 쓰는 법은 [문서 사이트 사용법](docs-kit.md)에 있습니다.
