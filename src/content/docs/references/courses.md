---
title: 교재 · 강의
description: 논문이 아닌 교재, 강의 노트, 온라인 코스 자료 모음. 이 프로젝트가 쓰는 Drake 기반 자료를 중심으로.
---

논문이 아닌 **교재·강의 자료**를 모으는 페이지입니다. 항목마다 앵커(`#manipulation` 등)가 있어 개념 문서에서 링크할 수 있습니다.
새 항목은 아래 표에 한 줄 추가하고, 필요하면 항목 아래에 "관련 챕터"를 덧붙입니다.

- **읽은 범위**는 [논문 목록](../papers/)과 같은 기준입니다. 아래 두 항목은 **목차(메인 페이지)만** 확인했고 본문 챕터는 읽지 않았습니다.
- 두 노트 모두 저작권 표기만 있고 별도 라이선스는 명시돼 있지 않아, 내용을 옮기지 않고 **링크만** 둡니다.
- "→ 해석"은 챕터 제목만 보고 짠 내 판단이라, 실제로 읽은 뒤 고쳐야 합니다.

| 제목 | 저자 · 형태 | 요약 | 읽은 범위 |
|---|---|---|---|
| <span id="manipulation"></span>[Robotic Manipulation: Perception, Planning, and Control](https://manipulation.csail.mit.edu/index.html) | Russ Tedrake · MIT 강의 노트(교재 형식), 페이지상 2026년 가을 학기 기준으로 갱신 중 | 로봇 조작 전반(픽앤플레이스, 자세 추정, 모션 플래닝, 매니퓰레이터 제어, 딥 퍼셉션, 강화학습, 소프트 로봇·촉각)을 Drake 코드와 노트북으로 다룸. 총 12장 + 부록 | 목차만 |
| <span id="underactuated"></span>[Underactuated Robotics](https://underactuated.csail.mit.edu/index.html) | Russ Tedrake · MIT 강의 노트(교재 형식), 페이지상 2024년 봄 학기 기준 | 비선형 제어·최적화 기반의 동역학·제어 교재. 궤적 최적화, 정책 탐색, 접촉을 통한 계획·제어, 시스템 식별, 모방학습 등. 총 21장 + 부록, 강의 영상(YouTube)과 노트북 제공 | 목차만 |

## Robotic Manipulation

공통: Drake를 쓰며(부록 B), 온라인 Jupyter 노트북과 Google Colab 실행을 지원합니다. 전제 지식은 페이지에 명시돼 있지 않습니다.

우리 프로젝트와 연결될 만한 부분(챕터 제목 기준):

| 챕터 | 왜 볼 만한가 (→ 해석) |
|---|---|
| 8. Manipulator Control | 위치·임피던스 제어의 기초. 베이스라인 ①②를 정리할 때 |
| 11. Reinforcement Learning | 시뮬레이션 RL 레시피 |
| 12. Soft Robots and Tactile Sensing | 촉각 확장 단계(T2–T3)의 배경. [촉각 없이 접촉을 다루는 법](../../concepts/touchless-grasping/#6-촉각을-나중에-더하는-경로-제안) |
| 부록 B. Drake | Pydrake 사용법 |
| 부록 C. DrakeGym Environments | Drake 기반 환경을 gym 형태로 만드는 방식. 우리 `RobotEnv` 설계와 비교해 볼 가치 |
| 3. Basic Pick and Place, 5. Bin Picking | 파지 이전·이후 파이프라인의 전체 그림 |

## Underactuated Robotics

공통: Drake의 Python 바인딩 예제, 강의 영상, 온라인 노트북. 전제 지식은 명시돼 있지 않습니다.

| 챕터 | 왜 볼 만한가 (→ 해석) |
|---|---|
| 17. Planning and Control through Contact | 접촉을 다루는 계획·제어. 접촉 암묵(contact-implicit) 계열과 Drake 접촉 모델의 배경. [시뮬레이션 도구](../simulation-tools/) |
| 10. Trajectory Optimization | 베이스라인 ③(접촉력 최적화)을 구현할 때의 기반 |
| 8. Linear Quadratic Regulators, 9. Lyapunov Analysis | 제어 이론 기초 |
| 11. Policy Search, 20. Model-Free Policy Search | 정책 학습 이론 |
| 18. System Identification | 실기 매개변수를 시뮬레이션에 맞추는 방법. sim-to-real 단계 |
| 21. Imitation Learning | Phase 0 BC 파이프라인의 이론적 배경 |

## 열린 질문

- 두 노트의 접촉 관련 챕터(Underactuated 17장, Manipulation 12장)를 실제로 읽고 우리 문제(비동기 접촉)에 맞는 부분 추리기.
- Manipulation 부록 C의 DrakeGym과 이 레포 `RobotEnv`/`DrakeEnv`의 설계 차이 비교.
