---
title: 시뮬레이션 도구
description: Drake 접촉 모델(hydroelastic)을 중심으로, 이 프로젝트가 쓰는 시뮬레이터의 근거 자료.
---

## Drake와 접촉 모델

이 프로젝트가 Drake를 우선하는 이유는 접촉 물리 자체가 연구 대상이기 때문입니다.
Phase B 환경([`grasp_env.py`](https://github.com/sweaty-fingers/robot-manipulation/blob/main/robot_manipulation/envs/grasp_env.py))은
접촉 모델 설정이 기본값(`HydroelasticWithFallback`)이지만 도형에 hydroelastic 속성을 주지 않아, 실제로는 **point contact만** 쓰입니다(접촉 결과에서 point-pair 2개, hydroelastic 0개로 확인).

| 자료 | 내용 |
|---|---|
| [Elandt et al. 2019, arXiv 1904.11433](https://arxiv.org/abs/1904.11433) | hydroelastic의 기반인 pressure field 접촉 모델. 물체 내부 압력장이 같아지는 면이 접촉면이고, 그 위에서 견인력을 적분해 접촉 힘·모멘트를 얻음 |
| [Velocity-Level Approximation of Pressure Field Contact Patches, arXiv 2110.04157](https://arxiv.org/pdf/2110.04157) | pressure field 접촉 패치의 속도 수준 근사. 제목과 검색 결과만 확인, 초록은 읽지 않음 |
| [Contact-Implicit Trajectory Optimization with Hydroelastic Contact and iLQR, arXiv 2202.13986](https://arxiv.org/abs/2202.13986) | hydroelastic + iLQR로 접촉 리치 행동 합성. 전신 조작 궤적을 Kinova 팔로 개방 루프 재생해 물리 정확도 확인 (초록 확인) |
| [Drake 튜토리얼: hydroelastic contact basics](https://drake.mit.edu/tutorials/hydroelastic_contact_basics.html) | `compliant_hydroelastic`, `hydroelastic_modulus` 태그 사용법 |
| [TRI 블로그: Rethinking Contact Simulation for Robot Manipulation](https://medium.com/toyotaresearch/rethinking-contact-simulation-for-robot-manipulation-434a56b5ec88) | 접촉 모델링 배경 설명 |

(Drake 문서는 hydroelastic을 point contact보다 권장하되 시뮬레이션 속도가 제약이면 예외라고 안내합니다 — 검색 요약 기준, 원문 미확인.)

## 다음 확인 항목

- Phase B 환경을 point contact에서 hydroelastic으로 바꿨을 때 비동기 접촉 변위 결과가 얼마나 달라지는지. 시뮬레이터의 접촉 모델이 "밀림" 현상의 크기에 영향을 준다면 결론의 신뢰도에 직결됩니다.
- MuJoCo, Isaac Lab 등 병렬 시뮬레이터는 이번 세션에 조사하지 않았습니다. 이전 브리프의 기술은 원문 미확인.
