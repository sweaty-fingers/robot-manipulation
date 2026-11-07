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
검증 ([모델 검증](/training/validation/)) → 시뮬레이션 재검증 → 실기 배포
```

## 학습 실행 템플릿

```bash
python train.py \
  --config configs/pick_and_place.yaml \
  --data-dir data/demos/ \
  --output-dir runs/exp001 \
  --epochs 100
```

```python
# train.py 핵심 루프 (의사코드)
for epoch in range(num_epochs):
    for batch in dataloader:
        obs, action = batch["obs"], batch["action"]
        pred = policy(obs)
        loss = criterion(pred, action)
        loss.backward()
        optimizer.step()
    log_metrics(epoch, loss)
    if epoch % save_every == 0:
        save_checkpoint(policy, f"runs/exp001/ckpt_{epoch}.pt")
```

## 재현성 체크리스트

- [ ] random seed 고정
- [ ] config(yaml/json)와 코드 버전(git commit hash)을 체크포인트와 함께 저장
- [ ] 데이터셋 버전 고정 (어떤 rerun 로그/날짜 범위로 학습했는지 기록)

:::note[TODO]
실제 사용 모델 아키텍처, 하이퍼파라미터, config 파일 경로를 채워 넣기.
:::

Colab에서 바로 실행하는 예제는 [예제: Pick-and-Place 학습](/examples/example-01-pick-and-place/)에서 확인.
