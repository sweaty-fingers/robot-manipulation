---
title: 지각과 제어
description: Manipulation 정책에서 사용하는 지각 입력과 제어 방식 정리.
---

## 지각 입력

- **RGB / RGB-D 카메라**: wrist camera vs. external (3rd-person) camera. 시점(viewpoint) 선택이 정책 일반화에 영향.
- **Proprioception**: joint position/velocity, end-effector pose/twist.
- **Force/Torque sensing**: contact-rich task (삽입, 조립)에서 중요.

## 제어 방식

| 방식 | 설명 | 장점 | 단점 |
| --- | --- | --- | --- |
| Joint position control | 목표 조인트 각도 추종 | 구현 단순 | task space 일반화 어려움 |
| End-effector impedance/velocity control | task space에서 delta pose/속도 명령 | 일반화, 안전성(컴플라이언스) | 모델링 복잡도 증가 |
| Learned policy (behavior cloning / RL) | 관측 → 액션 직접 매핑 | 복잡한 접촉 task 대응 | 데이터/학습 비용, 분포 밖 입력에 취약 |

## 정책 입출력 설계 체크리스트

- [ ] observation space에 포함할 modality 확정 (이미지 해상도, proprioception 차원)
- [ ] action space (joint vs. task space, delta vs. absolute, gripper 포함 여부)
- [ ] 제어 주기(control frequency)와 센서 주기가 일치하는지
- [ ] 좌표계/단위(미터·라디안 등) 일관성

:::note[TODO]
실제 사용 중인 observation/action 스펙을 여기에 코드 스니펫으로 명시.
:::
