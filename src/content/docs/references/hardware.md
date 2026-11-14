---
title: 하드웨어 & 제품
description: 로봇 손과 촉각 센서의 종류, 대략적인 가격대, 이 프로젝트의 선택 근거.
---

> 가격은 판매처·블로그 표기라 불일치하고 변동합니다(2026-11 웹 검색). LEAP Hand 가격만 논문에서 확인했습니다.

## 이 프로젝트의 선택

- **목표 손: LEAP Hand.** 논문 기준 약 2,000달러, 4시간 조립, Allegro 대비 1/8 비용, 조립 설명과 Sim2Real 파이프라인 공개
  ([논문 항목](../papers/#leap-hand)). 촉각 센서가 없는 모델입니다.
- **입력: 비전 + proprioception.** 촉각 센서는 쓰지 않음. 선례는 [Visual Dexterity](../papers/#visual-dexterity),
  proprioception만으로 접촉력을 추정하는 연구는 [FEEL](../papers/#feel-finger-force-estimation).
- 촉각은 **첫 단계에서만 쓰지 않는 것**이고 이후 확장을 고려합니다([단계 계획](../../concepts/touchless-grasping/#6-촉각을-나중에-더하는-경로-제안)). 촉각 확장을 염두에 두면 LEAP Hand 자체는 촉각이 없으므로 손끝 센서 부착이나 다른 손으로의 이동이 필요할 수 있음(아래 표).
- 대가: 촉각이 파지 안정성 예측을 개선한다는 증거가 있음([Temporal Visuo-Tactile](../papers/#temporal-visuo-tactile-grasp-stability)). 무엇을 포기하는지 인지하고 가는 선택입니다.

## 로봇 손

| 손 | 촉각 | 가격 표기 | 출처 성격 |
|---|---|---|---|
| LEAP Hand (v1) | 없음 | 약 $2,000 (논문) / 스토어 $2,066 | 논문 + 벤더 |
| LEAP Hand v2 | 없음(기본) | $4,550 (한 가이드 표기) | 벤더 표기 |
| Allegro Hand | 기본 없음(커뮤니티 손끝 센서) | $13,000 (한 비교표) | 벤더 표기 |
| Shadow Hand | 선택 | 추정치가 $45,000–300,000으로 크게 갈림 | 벤더/블로그 |
| Inspire RH56 계열 | 변형별 상이 | 촉각형 $4,500–$10,900 | 리테일러 |
| BrainCo Revo2 Touch | 있음 | $14,999.99 + 운송 | 리테일러 |
| AgiBot OmniHand Tactile / Pro | 있음 | $6,399 / $15,999 | 한 구매 가이드 |
| ORCA Hand Touch | 있음(17 DoF) | $6.1k | 한 구매 가이드 |

Sharpa Wave 등은 이번 검색에서 자료를 찾지 못했습니다. 이전 리서치 브리프의 손 비교표(PSYONIC, Sanctuary 등)는 원문 미확인.

## 촉각 센서 (이번 조사에서 만난 것들)

| 센서 | 방식 | 사용 사례 |
|---|---|---|
| BioTac SP | 유체/전극형 | [Grasping in the Dark](../papers/#grasping-in-the-dark) |
| TacTip 계열 microTac | 광학, 소형 | [Shear-based Grasp Control](../papers/#shear-based-grasp-control) |
| Digit 360 | 고해상도 동적 촉각 | [Temporal Visuo-Tactile](../papers/#temporal-visuo-tactile-grasp-stability) |
| 촉각 피부(전단+법선) | 이진/삼진 힘 | [In-Hand Translation](../papers/#learning-in-hand-translation-with-tactile-skin) |

시뮬레이션과의 격차가 큰 쪽은 광학식 센서입니다(이전 결정 근거). 단 [Closing the Reality Gap](../papers/#closing-the-reality-gap)처럼
간단한 거리 기반 가상 촉각 유닛으로 전이한 사례도 있어, "촉각=무조건 불가"는 아닙니다.

## 확인이 필요한 것

- LEAP Hand의 자유도·액추에이터 사양과 URDF 라이선스를 공식 사이트에서 직접 확인(이번에 하지 않음).
- 실기 구매 시점에 가격 재조회.
