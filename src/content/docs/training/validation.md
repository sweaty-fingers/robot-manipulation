---
title: 모델 검증
description: 학습된 manipulation 모델의 검증 지표와 절차.
---

## 검증 레벨

1. **오프라인 검증**: held-out 데이터셋에 대한 action 예측 오차 (MSE, success-weighted metric).
2. **시뮬레이션 검증**: [Drake/MuJoCo](../../simulation/sim-to-real-validation/)에서 반복 롤아웃으로 task success rate 측정.
3. **실기 검증**: 제한된 조건에서 소규모 실기 테스트.

## 공통 검증 지표

| 지표 | 레벨 | 설명 |
| --- | --- | --- |
| Action MSE / L1 | 오프라인 | 예측 action과 demo action의 차이 |
| Success rate | 시뮬레이션/실기 | task 완료 성공 비율 |
| Time-to-completion | 시뮬레이션/실기 | task 완료까지 걸린 시간 |
| Failure mode distribution | 시뮬레이션/실기 | 실패 원인 분류 (grasp miss, collision, timeout 등) |

## 검증 실행 템플릿

```bash
python evaluate.py \
  --checkpoint runs/exp001/ckpt_100.pt \
  --env sim:pick_and_place \
  --episodes 50 \
  --log-rerun true
```

`--log-rerun true`로 실행하면 각 episode의 관측/액션/성공여부가 rerun.io에 기록되어, [데이터 파이프라인](../../data-pipeline/retraining-loop/)의 실패 사례 재수집 단계로 바로 연결됩니다.

:::note[TODO]
실제 평가 스크립트 경로와 최근 실험 결과 요약을 여기에 기록.
:::
