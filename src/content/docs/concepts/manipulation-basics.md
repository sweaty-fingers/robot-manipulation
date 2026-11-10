---
title: Robot Manipulation 기초
description: 좌표계, 기구학, 파지(grasping), 지각-제어 루프 등 manipulation의 핵심 개념 정리.
---

## 좌표계와 변환

- **World frame / Base frame / End-effector frame**: 로봇의 자세와 목표 pose는 항상 어떤 좌표계를 기준으로 하는지 명확히 할 것.
- 변환은 homogeneous transform (4x4 matrix) 또는 position + quaternion으로 표현.
- 라이브러리별 쿼터니언 순서 (`wxyz` vs `xyzw`) 차이가 버그의 흔한 원인 — 사용하는 프레임워크(Drake, MuJoCo, ROS)마다 명시적으로 확인.

## 기구학 (Kinematics)

- **Forward Kinematics (FK)**: 조인트 각도 → end-effector pose.
- **Inverse Kinematics (IK)**: 목표 pose → 조인트 각도. 해가 여러 개이거나 없을 수 있음 (redundancy, joint limit, singularity).
- Manipulation 모델 학습 시 action space를 joint space로 할지 task space(end-effector delta pose)로 할지는 데이터 수집 난이도와 일반화 성능에 큰 영향을 줌.

## 파지 (Grasping)

- Grasp pose 표현: contact point, approach vector, gripper width.
- Grasp 품질 평가: force closure, antipodal grasp 등 고전적 기준 vs. 학습 기반 grasp success 예측.

## 지각-제어 루프

```
센서 입력 (RGB-D, force/torque, proprioception)
      ↓
상태 추정 / 표현 학습 (pose estimation, latent encoder)
      ↓
정책(policy) — 모델 추론
      ↓
액션 (joint command / end-effector delta)
      ↓
로봇 / 시뮬레이터 실행 → 다시 센서 입력
```

이 루프의 각 단계가 [시뮬레이션](../../simulation/drake-mujoco-overview/)에서는 어떻게 구성되고, [데이터 파이프라인](../../data-pipeline/rerun-overview/)에서는 어떻게 기록되는지 연결해서 정리.

:::caution[TODO]
여기에 실제로 다루는 로봇/그리퍼 사양, 많이 참고하는 논문·레포 링크를 채워 넣기.
:::
