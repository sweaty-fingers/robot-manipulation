---
title: rerun.io 개요
description: rerun.io를 로봇 manipulation 로그/시각화에 활용하는 방법.
---

[rerun.io](https://rerun.io)는 로봇 센서 데이터, 3D 트랜스폼, 이미지, 시계열 등을 시간축에 맞춰 기록하고 인터랙티브하게 재생/시각화하는 오픈소스 SDK + 뷰어입니다.

## 왜 rerun인가

- 멀티모달 로그(이미지, 포인트클라우드, 조인트 상태, 액션)를 하나의 타임라인에서 동기화해서 재생 가능
- `.rrd` 파일로 저장 → 나중에 재생/공유 가능, 웹 뷰어로 임베드도 가능
- 시뮬레이션 롤아웃과 실기 로그를 같은 뷰어에서 나란히 비교 가능 (sim-to-real gap 분석에 유용)

## 기본 로깅 예제

실제 사용 중인 `rerun-sdk` 버전은 **0.42**. 로깅 API는 `RerunLogger`
(`robot_manipulation/data/rerun_logger.py`)에 고정된 entity-path 스키마로 감싸뒀습니다.

| entity path | 내용 |
| --- | --- |
| `robot/proprio` | 조인트/위치·속도 등 proprioception 벡터 |
| `action` | 해당 스텝에서 실행한 액션 벡터 |
| `episode/success` | episode 종료 시 성공 여부 (static) |

```python
import rerun as rr

rr.init("manipulation_demo", spawn=True)  # 로컬 뷰어 실행

for t, (obs, action) in enumerate(episode):
    rr.set_time("step", sequence=t)
    rr.log("robot/proprio", rr.Tensor(obs["proprio"]))
    rr.log("action", rr.Tensor(action))
```

:::caution
`rr.set_time_sequence(...)`와 `rr.Scalar(...)`는 구버전 API입니다. 0.42 기준으로는
`rr.set_time("timeline", sequence=t)`와 `rr.Tensor(...)`/`rr.Scalars(...)`를 사용하세요.
:::

## 파일로 저장하고 다시 열기

```python
rr.save("logs/episode_001.rrd")
```

```bash
rerun logs/episode_001.rrd
```

## 웹 임베드 (선택)

`.rrd` 파일을 어딘가에 호스팅해두면 문서 페이지에 뷰어를 iframe으로 임베드할 수 있습니다.

```html
<iframe
  src="https://app.rerun.io/version/latest/index.html?url=https://your-host.example.com/logs/episode_001.rrd"
  width="100%"
  height="480"
></iframe>
```

## 중요: rerun은 시각화 전용, 학습 데이터는 별도 저장

`rerun-sdk` 0.42에는 `.rrd`를 다시 배열로 읽어오는 공개 read-back API가 없습니다
(write 경로만 존재). 그래서 이 프로젝트는 rerun을 **시각화/디버깅 전용**으로 쓰고,
실제 학습 데이터는 수집 시점에 obs/action 배열을 바로 `.npz`로 저장합니다
(`robot_manipulation/data/dataset_builder.py`의 `save_episode`). `.rrd`와 `.npz`는
같은 `episode_id`를 공유해서, 특정 episode를 rerun 뷰어로 열어 보면서 동시에 그
episode의 학습 데이터를 대조할 수 있습니다.

다음: [재학습 데이터 파이프라인](../retraining-loop/)
