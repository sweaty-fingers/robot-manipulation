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

```python
import rerun as rr

rr.init("manipulation_demo", spawn=True)  # 로컬 뷰어 실행

for t, (obs, action) in enumerate(episode):
    rr.set_time_sequence("step", t)
    rr.log("camera/rgb", rr.Image(obs["rgb"]))
    rr.log("robot/joint_pos", rr.Scalar(obs["joint_pos"]))
    rr.log("robot/ee_pose", rr.Transform3D(translation=obs["ee_pos"], rotation=obs["ee_quat"]))
    rr.log("action", rr.Scalar(action))
```

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

:::note[TODO]
실제 사용 중인 rerun 버전, 로깅 스키마(entity path 네이밍 규칙)를 여기에 정리.
:::

다음: [재학습 데이터 파이프라인](/data-pipeline/retraining-loop/)
