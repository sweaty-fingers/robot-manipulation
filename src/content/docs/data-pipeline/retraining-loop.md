---
title: 재학습 데이터 파이프라인
description: rerun.io 로그를 기반으로 실패 사례를 재수집하고 재학습하는 루프.
---

## 전체 루프

```
배포된 policy 실행 (시뮬레이션 또는 실기)
      ↓  모든 episode를 rerun.io로 로깅 (.rrd)
실패/저성능 episode 선별
      ↓
.rrd → 학습용 데이터셋 포맷으로 변환 (obs/action pair 추출)
      ↓
기존 데이터셋에 병합 (또는 curriculum 방식으로 별도 관리)
      ↓
재학습 → [검증](../../training/validation/) → 재배포
```

## 실패 사례 선별 기준

- `success == False`인 episode
- success는 True지만 지표(시간, 경로 효율성)가 임계값을 벗어난 episode
- 사람이 리뷰 중 rerun 뷰어에서 "이상 행동"으로 태깅한 episode

## `.rrd` → 학습 데이터셋 변환 템플릿

```python
import rerun as rr
from rerun import dataframe as rrd

recording = rrd.load_recording("logs/episode_001.rrd")
view = recording.view(index="step", contents="/**")
table = view.select().read_all()

# table을 obs/action 텐서로 변환해 기존 데이터셋 포맷에 맞춰 저장
```

## 데이터 버전 관리

- 재학습에 사용한 `.rrd` 파일 목록(또는 날짜 범위)을 학습 config와 함께 기록
- 데이터셋 버전과 모델 체크포인트를 1:1로 매핑해서 추적 ([모델 학습](../../training/model-training/) 체크리스트 참고)

:::caution[TODO]
실제 변환 스크립트 경로, 재학습 주기(예: N회 배포마다 1회)를 여기에 기록.
:::
