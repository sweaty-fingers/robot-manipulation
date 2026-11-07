---
title: 모델 학습
description: 실제 제어를 위한 manipulation 모델 학습 파이프라인.
---

## 학습 파이프라인 개요

```
데이터 수집 (teleop demo / sim rollout / 실기 로그)
      ↓  rerun.io 로그 → 학습용 데이터셋 변환
전처리 (정규화, augmentation, action chunking 등)
      ↓
모델 학습 (behavior cloning / diffusion policy / RL fine-tuning 등)
      ↓
체크포인트 저장 + 학습 로그(loss, metric) 기록
      ↓
검증 ([모델 검증](../validation/)) → 시뮬레이션 재검증 → 실기 배포
```

## 실제 구현: `robot_manipulation` 패키지

레포 루트의 `robot_manipulation/` 패키지가 이 파이프라인을 코드로 구현합니다
(모듈화/확장성을 위해 encoder / action-expert / env / data를 분리 — 자세한 설계는
[레포의 `AGENTS.md`](https://github.com/sweaty-fingers/robot-manipulation/blob/main/AGENTS.md) 참고).

```bash
python scripts/collect_demos.py --config configs/experiments/00_skeleton.yaml --n-episodes 50
python scripts/train.py --config configs/experiments/00_skeleton.yaml
python scripts/evaluate.py --config configs/experiments/00_skeleton.yaml
```

`scripts/train.py`는 config로 지정된 encoder/action-expert/env를
`robot_manipulation/factory.py`의 registry를 통해 조립할 뿐, 구체 타입에는 전혀
분기하지 않습니다 — 아키텍처를 바꾸는 건 YAML 수정 한 줄입니다.

```yaml
# configs/experiments/00_skeleton.yaml
action_expert:
  type: mlp          # 나중에 diffusion 등으로 교체 가능, train.py는 안 바뀜
  hidden_dims: [256, 256]
env:
  backend: drake      # MuJoCoEnv 추가되면 여기만 바꾸면 됨
```

현재(Phase 0)는 proprioception만 쓰는 `MLPActionExpert` + `DrakeEnv` point-mass
reach task로 전체 파이프라인(수집→학습→평가)이 동작함을 검증한 상태입니다.
사전학습 vision/audio/tactile encoder, diffusion 기반 action-expert, MuJoCo 백엔드는
다음 단계에서 같은 구조에 모듈 추가로 들어갑니다.

## 재현성 체크리스트

- [x] random seed 고정 (`TrainConfig.seed`)
- [x] config(yaml) 경로를 체크포인트 디렉터리(`checkpoint_dir`)와 함께 보관
- [ ] 데이터셋 버전 고정 (어떤 `.npz`/`.rrd` 묶음으로 학습했는지 기록) — Phase 0에서는
      `data_dir` 하나로 고정, 데이터가 늘어나면 버전 태깅 방식 추가 필요

Colab에서 바로 실행하는 예제는 [예제: Point-Mass Reach](../../examples/example-01-reach/)에서 확인.
