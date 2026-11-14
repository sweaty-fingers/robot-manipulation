---
title: 논문 목록
description: 무교란 파지 관련 논문 목록. 제목·연도·요약·읽은 범위를 표로 보고, 상세 페이지가 있는 항목은 링크로 이동.
---

목록 페이지입니다. 한 줄 요약만 담고, 자세한 정리는 항목별 **상세 페이지**(요약 끝의 `상세 →` 링크)에 둡니다.
상세 페이지는 필요한 항목만 [템플릿](https://github.com/sweaty-fingers/robot-manipulation/blob/main/src/content/docs/references/papers/_template.md)으로 작성합니다. 링크가 없는 항목은 아직 상세 페이지가 없는 것입니다.

- **읽은 범위**: `초록`(arXiv 초록만), `본문 일부`(본문 일부 섹션까지), `본문`(전체), `미확인`(원문 못 봄). 자세한 의미는 [사용법](../#검증-상태-표기).
- 요약의 `→`는 이 프로젝트 관점의 내 해석입니다.
- 조사 시점 2026-11-14. 연도는 arXiv 최초 제출 기준.


## 시뮬레이션 기반 손 학습

| 제목 | 연도·범위 | 요약 · 상세 |
|---|---|---|
| <span id="dactyl"></span>[Dactyl (Learning Dexterous In-Hand Manipulation)](https://arxiv.org/abs/1808.00177) | 2018<br>초록 | 도메인 랜덤화로 시뮬레이션에서만 학습, Shadow Hand 비전 기반 물체 재배치를 실기 전이. 손가락 보행·다지 협응이 자연 발생. → sim-to-real 레시피의 원조, 접촉 순간 교란은 주제 아님 |
| <span id="general-in-hand-reorientation"></span>[General In-Hand Reorientation](https://arxiv.org/abs/2111.03043) | 2021<br>초록 | 모델 프리 RL로 2000개+ 형상을 손바닥 위/아래에서 재배치, 실세계 관측으로 증류 |
| <span id="visual-dexterity"></span>[Visual Dexterity](https://arxiv.org/abs/2211.11744) | 2022<br>본문 일부 | 깊이 카메라 1대 + 관절 위치만으로 새 형상 물체 재배치(시뮬레이션 RL, 중앙값 약 7초). 5천 달러 미만 오픈소스 하드웨어. → 촉각 없이 접촉 리치 조작이 가능하다는 직접 선례 **[상세 →](visual-dexterity/)** |
| <span id="sim-to-real-humanoid-rl"></span>[Sim-to-Real Humanoid RL](https://arxiv.org/abs/2502.20396) | 2025<br>초록 | 휴머노이드의 비전 기반 접촉 리치 양손 조작을 sim-to-real RL로. 접촉·물체 목표 기반 일반화 보상, 정책 증류 |
| <span id="viserdex"></span>[ViserDex](https://arxiv.org/abs/2604.11138) | 2026<br>초록 | 단안 RGB 재배치. 3D Gaussian Splatting 공간에서 도메인 랜덤화해 시각 sim-to-real 격차 축소 |
| <span id="closing-the-reality-gap"></span>[Closing the Reality Gap](https://arxiv.org/abs/2601.02778) | 2026<br>초록 | 촉각 + 관절 토크를 쓰는 sim-to-real RL. 모터 전류→토크 보정으로 토크 센서 없이 힘 추종 파지·재배치를 제로샷 전이. → 시뮬레이션 접촉력·액추에이터 모델링 참고 |

## 파지 생성·데이터셋

| 제목 | 연도·범위 | 요약 · 상세 |
|---|---|---|
| <span id="dexgraspnet"></span>[DexGraspNet](https://arxiv.org/abs/2210.02697) | 2022<br>초록 | Shadow Hand 기준 5355개 물체에 132만 파지. 미분 가능 force-closure 추정기로 합성, Isaac Gym 검증 |
| <span id="get-a-grip"></span>[Get a Grip](https://arxiv.org/abs/2410.23701) | 2024<br>초록 | 4.3천 물체 350만 파지 + RGB/포인트클라우드/NeRF. 생성 모델보다 판별형 파지 평가기가 핵심이라는 주장 |
| <span id="crossdex"></span>[CrossDex](https://arxiv.org/abs/2410.02479) | 2024<br>초록 | 인간 eigengrasp 기반 공통 행동 공간, proprioception을 손끝·손바닥 위치로 단순화. YCB에서 4개 손 80% |
| <span id="clutterdexgrasp"></span>[ClutterDexGrasp](https://arxiv.org/abs/2506.14317) | 2025<br>초록 | 어지러운 장면 목표 지향 파지. 교사(시뮬레이션) → 3D diffusion policy 증류, 제로샷 sim-to-real |
| <span id="lightning-grasp"></span>[Lightning Grasp](https://arxiv.org/abs/2511.07418) | 2025<br>초록 | Contact Field 자료구조로 파지 합성을 대폭 가속(오픈소스) |
| <span id="d-grasp"></span>[D-Grasp](https://arxiv.org/abs/2112.03028) | 2021<br>초록 | 물리 시뮬레이션 + RL로 접근-파지-이동의 동적 손-물체 상호작용 합성 |

## 접촉 제어 — 닫는 동작과 순응성

| 제목 | 연도·범위 | 요약 · 상세 |
|---|---|---|
| <span id="grasping-by-interconnection"></span>[Grasping by Interconnection](https://arxiv.org/abs/2609.19228) | 2026<br>초록 | 거친 템플릿 + 순응적·미끄러지는 접촉 기반 닫는 동작 플래너. Shadow Hand에서 피드백 없이 크기 오차 약 1cm·자세 오차 수 cm 허용, 일상 물체 80개 중 82.5% 파지. → 베이스라인 ② 1순위 후보 |
| <span id="underactuated-hand-grasp-learning"></span>[Underactuated Hand Grasp Learning](https://arxiv.org/abs/1609.07592) | 2016<br>초록 | 언더액추에이티드 손은 접촉 회피를 보장할 수 없고 접촉을 이용해 평형에 도달. 접촉 변화 대신 평형으로 가는 모터 명령을 모델링 |
| <span id="transferring-contact-not-just-motion"></span>[Transferring Contact, Not Just Motion](https://arxiv.org/abs/2606.15516) | 2026<br>초록 | 손 종류가 달라도 쓰는 힘-위치 인터페이스. 노력 신호를 시스템 식별로 관절 토크로 보정, flow-matching 정책 학습 |
| <span id="whole-hand-force-regulation"></span>[Whole-Hand Force Regulation](https://arxiv.org/abs/2609.30082) | 2026<br>초록 | 추적된 물체 모델 + proprioception으로 손 전체 접촉을 촉각 없이 추정하고 접촉력 분배를 실시간 재계산(시뮬레이션은 무중력). → 베이스라인 ③ 1순위 후보 |

## 접촉 제어 — 센싱 기반 반응

| 제목 | 연도·범위 | 요약 · 상세 |
|---|---|---|
| <span id="grasping-in-the-dark"></span>[Grasping in the Dark](https://arxiv.org/abs/2011.00712) | 2020<br>초록 | 슬립 감지 후 보정하는 촉각 컨트롤러. Shadow Hand + BioTac SP, 성공률 73.5% |
| <span id="shear-based-grasp-control"></span>[Shear-based Grasp Control](https://arxiv.org/abs/2503.17501) | 2025<br>초록 | 언더액추에이티드 손 + 손끝 촉각 센서(microTac)의 접촉 자세·힘 피드백으로 외란 하 섬세 파지 |
| <span id="feel-finger-force-estimation"></span>[FEEL (Finger Force Estimation)](https://arxiv.org/abs/2610.38351) | 2026<br>초록 | proprioception 이력만으로 손가락별 외력 추정(0–20N, RMSE 0.251N). 상자 취급에서 파손 없는 결과 7/10 → 10/10. → 촉각 없이 접촉력 추정이 된다는 근거 |
| <span id="tactile-based-blind-grasping"></span>[Tactile-Based Blind Grasping](https://arxiv.org/abs/1709.02924) | 2017<br>초록 | 물체 정보 없이 proprioception+촉각만으로 하는 조작의 강건 제어 법칙과 안정성 해석 |
| <span id="learning-in-hand-translation-with-tactile-skin"></span>[In-Hand Translation with Tactile Skin](https://arxiv.org/abs/2407.07885) | 2024<br>초록 | 이진 법선력·삼진 전단력 센서 모델로 제로샷 sim-to-real 촉각 RL, 촉각 없는 baseline보다 우수 |
| <span id="temporal-visuo-tactile-grasp-stability"></span>[Temporal Visuo-Tactile Grasp Stability](https://arxiv.org/abs/2610.10283) | 2026<br>초록 | Digit 360 4개 달린 손으로 200개 물체 1만 번 파지. 촉각이 안정성 예측을 개선, 재파지 성공률 +10.5%p. → 촉각을 안 쓰는 선택의 비용 가늠 자료 |

## VLA와 접촉

| 제목 | 연도·범위 | 요약 · 상세 |
|---|---|---|
| <span id="pi0"></span>[π0](https://arxiv.org/abs/2410.24164) | 2024<br>초록 | 사전학습 VLM 위의 flow matching 범용 로봇 정책. 여러 로봇의 대규모 데이터, 빨래 개기 등 |
| <span id="openvla"></span>[OpenVLA](https://arxiv.org/abs/2406.09246) | 2024<br>초록 | 7B 오픈소스 VLA. 97만 개 실세계 시연, LoRA 미세조정 가능 |
| <span id="demystifying-vla-failures"></span>[Demystifying VLA Failures](https://arxiv.org/abs/2608.01402) | 2026<br>초록 | 접촉 리치 실패를 정밀도 실패·힘 실패로 분류, 후학습 기법 제안(5개 과제 평균 66%, 이전 최고 41%). → "VLA는 접촉에 약하다"의 직접 근거 |
| <span id="forcevla"></span>[ForceVLA](https://arxiv.org/abs/2505.22159) | 2025<br>초록 | 6축 힘을 VLA의 1급 입력으로 취급하는 force-aware MoE. π0 기반 baseline 대비 평균 +23.2% |
| <span id="fd-vla"></span>[FD-VLA](https://arxiv.org/abs/2602.02142) | 2026<br>초록 | 힘 센서 없이 비전·상태에서 힘 토큰을 증류해 VLM에 주입, 센서 직접 측정보다 좋았다고 보고 |
| <span id="t-rex"></span>[T-Rex](https://arxiv.org/abs/2606.17055) | 2026<br>초록 | 100시간 촉각 데이터셋 + 가변 속도 MoT + 시간적 촉각 VQ-VAE. 12개 과제 평균 성공률 +30% 이상 |
| <span id="decal"></span>[DeCAL](https://arxiv.org/abs/2609.09119) | 2026<br>초록 | 접촉 인식 게이팅으로 촉각을 융합하는 손재주 VLA, 평균 성공률 71% |
| <span id="tactile-wam"></span>[Tactile-WAM](https://arxiv.org/abs/2606.26663) | 2026<br>초록 | 촉각이 시각 모델링을 방해하는 "촉각 오염"을 비대칭 어텐션으로 완화한 World Action Model |

## 로봇 손·비파지

| 제목 | 연도·범위 | 요약 · 상세 |
|---|---|---|
| <span id="leap-hand"></span>[LEAP Hand](https://arxiv.org/abs/2309.06440) | 2023<br>초록 | 약 $2,000, 4시간 조립, Allegro 대비 1/8 비용, 손가락 자세와 무관한 최대 손재주. Sim2Real 파이프라인 공개. → 목표 하드웨어(촉각 없음) |
| <span id="gd2p"></span>[GD2P (Nonprehensile Pushing)](https://arxiv.org/abs/2509.18455) | 2025<br>초록 | 물체를 일부러 미는 비파지 조작용 접촉 전 손 자세 합성, Allegro·LEAP 실험. → 우리와 정반대 목표(대조 사례) |

## 인간 감각운동 제어

> 이 분류는 **원문 미확인**입니다. 이전 리서치 브리프와 웹 검색 스니펫에서 가져왔습니다.

| 제목 | 연도·범위 | 요약 · 상세 |
|---|---|---|
| [Johansson & Flanagan, *Nat. Rev. Neurosci.*](https://www.nature.com/articles/nrn2621) | 2009<br>미확인 | 촉각 정보가 물체 조작의 시점·힘 제어에 쓰이는 방식을 다룬 종설. 확인한 요지는 그립력이 피드포워드 성격이 강하고 감각운동 전달 지연이 힘 보정 속도를 제한한다는 정도 |
| Johansson & Westling, *Exp. Brain Res.* | 1984<br>미확인 | 정밀 파지에서 그립력과 들어올림 힘의 결합을 규명한 고전. 검색 스니펫상 검지·엄지 그립력 신호는 최초 접촉 순간을 제외하면 거의 같음. 첫 접촉 지연 수치는 못 봄 |

이전 브리프의 "접촉 후 약 70ms 반사", Santello & Soechting(1998)의 프리셰이핑 주장도 같은 상태(미확인).
