---
title: Colab 실습 가이드
description: 문서의 코드 예제를 Google Colab에서 바로 실행하는 방법.
---

이 문서의 예제들은 로컬 설치 없이 Google Colab에서 바로 실행할 수 있도록 노트북(`.ipynb`)을 같은 레포의 `notebooks/` 아래 함께 관리합니다.

## 연동 방식

각 예제 페이지 상단의 **"Open in Colab"** 배지를 누르면, GitHub에 올라간 노트북이 Colab에서 열립니다. 별도 서버 없이 아래 URL 패턴만으로 동작합니다.

```
https://colab.research.google.com/github/<github-user>/<repo>/blob/main/notebooks/<notebook>.ipynb
```

## 새 예제를 Colab 연동으로 추가하는 절차

1. `notebooks/`에 `.ipynb` 작성 (첫 셀에 `pip install` 의존성 명시)
2. `src/content/docs/examples/`에 설명 페이지 작성, 아래 배지를 상단에 삽입:

   ```md
   [![Open in Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/<github-user>/<repo>/blob/main/notebooks/<notebook>.ipynb)
   ```

3. GitHub에 push하면 즉시 Colab에서 열림 (별도 빌드 불필요)

## Colab 환경에서 시뮬레이터 설치 시 주의

- MuJoCo: Colab에서는 `pip install mujoco`만으로 보통 충분 (렌더링은 headless/EGL 설정 필요할 수 있음)
- Drake: `pip install drake` 또는 Drake 공식 Colab 설치 스크립트 사용 — 버전에 따라 설치 시간이 길 수 있음
- GPU 런타임이 필요하면 Colab 메뉴에서 **런타임 > 런타임 유형 변경 > GPU** 선택

:::caution[TODO]
실제 사용하는 notebook 템플릿(의존성 설치 셀, 공통 유틸)을 `notebooks/_template.ipynb`로 만들어 재사용.
:::
