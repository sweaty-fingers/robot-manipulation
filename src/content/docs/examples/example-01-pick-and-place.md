---
title: "예제 1: Pick-and-Place 학습 데모"
description: MuJoCo 환경에서 간단한 pick-and-place policy를 학습하는 end-to-end 예제.
---

[![Open in Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/sweaty-fingers/vtla/blob/main/notebooks/example-01-pick-and-place.ipynb)

> 레포/계정 이름이 바뀌면 위 배지의 `sweaty-fingers/vtla` 부분을 실제 GitHub `<user>/<repo>`로 맞춰주세요.

이 예제는 다음 흐름을 최소 코드로 보여줍니다.

1. MuJoCo로 간단한 pick-and-place 환경 생성
2. 랜덤/스크립트 policy로 demo rollout 수집
3. rerun.io로 rollout 로깅
4. 수집한 demo로 간단한 behavior cloning 모델 학습

## 핵심 코드 스니펫

```python
import mujoco
import numpy as np
import rerun as rr

rr.init("example01_pick_and_place")

model = mujoco.MjModel.from_xml_path("assets/simple_arm.xml")
data = mujoco.MjData(model)

for step in range(200):
    mujoco.mj_step(model, data)
    rr.set_time_sequence("step", step)
    rr.log("robot/qpos", rr.Scalar(data.qpos[0]))
```

전체 실행 가능한 노트북은 위 Colab 배지로 열어서 셀 순서대로 실행하세요.

## 다음 단계

- 수집한 로그를 [재학습 파이프라인](../../data-pipeline/retraining-loop/)에 연결해 모델을 반복 개선
- 학습된 policy를 [sim-to-real 검증](../../simulation/sim-to-real-validation/) 절차로 평가

:::note[TODO]
실제 로봇/그리퍼 모델로 교체, 성공률 등 결과 기록.
:::
