---
title: 시뮬레이션 환경 구축
description: Drake / MuJoCo 환경 설치 및 로봇 모델 로딩 가이드.
---

## MuJoCo 설치

```bash
pip install mujoco
```

```python
import mujoco

model = mujoco.MjModel.from_xml_path("robot.xml")  # MJCF 파일
data = mujoco.MjData(model)

mujoco.mj_step(model, data)
print(data.qpos)  # 조인트 위치
```

## Drake 설치

```bash
pip install drake
```

```python
from pydrake.all import DiagramBuilder, AddMultibodyPlantSceneGraph, Parser

builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
parser = Parser(plant)
parser.AddModels("robot.urdf")
plant.Finalize()
```

## 체크리스트

- [ ] 로봇 모델 파일(URDF/MJCF) 버전 관리 — 실기 모델과 동일한 치수/질량 파라미터인지 확인
- [ ] 시뮬레이션 타임스텝과 제어 주기 일치 여부
- [ ] 충돌 geometry 단순화(convex hull 등)로 인한 접촉 오차 확인
- [ ] domain randomization 범위 (질량, 마찰 계수, 센서 노이즈) 정의

:::tip
Colab에서 바로 실행해보려면 [Colab 실습 가이드](../../guides/colab-quickstart/)를 참고하세요.
:::

:::note[TODO]
실제 사용 중인 로봇 모델 파일 경로와 설치 스크립트를 여기에 연결.
:::
