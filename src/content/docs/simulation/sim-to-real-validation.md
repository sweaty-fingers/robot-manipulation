---
title: Sim-to-Real 검증
description: 시뮬레이션에서 학습한 policy를 실기에 배포하기 전 검증하는 절차.
---

## 검증 단계

1. **시뮬레이션 내 반복 평가**: 다양한 초기 조건(물체 위치, 조명 등 — 시각 입력을 쓰는 경우)에서 성공률 측정.
2. **Domain randomization 강도별 비교**: randomization 없이 학습한 policy와 비교해 강건성 확인.
3. **Sim-to-real gap 지표화**: 같은 task를 시뮬레이션과 실기에서 실행하고 trajectory를 [rerun.io](/data-pipeline/rerun-overview/)로 나란히 기록 → 시각적으로 비교.
4. **점진적 실기 배포**: 안전한 범위(낮은 속도, 제한된 workspace)에서 먼저 실기 테스트.

## 검증 지표 (예시)

| 지표 | 설명 |
| --- | --- |
| Task success rate | N회 시도 중 성공 비율 |
| Trajectory deviation | 시뮬레이션 대비 실기 end-effector 경로 편차 |
| Contact force mismatch | 접촉 시 힘/토크 센서 값 차이 |
| Recovery rate | 실패 후 복구 성공 비율 |

## 실패 모드 기록

실패한 롤아웃은 반드시 rerun.io 로그로 저장하고, [재학습 루프](/data-pipeline/retraining-loop.md) 문서에서 다루는 "실패 사례 재수집" 파이프라인으로 연결.

:::note[TODO]
실제 검증 결과(성공률 표, 실패 사례 스크린샷/rerun 링크)를 이 페이지에 채워 넣기.
:::
