---
title: 로컬 개발 환경 설정
description: 이 문서 사이트와 예제 코드를 로컬에서 실행하는 방법.
---

## 문서 사이트 실행

```bash
npm install
npm run dev
```

`http://localhost:4321`에서 확인. `src/content/docs/` 아래 markdown 파일을 수정하면 즉시 반영됩니다.

## 새 문서 페이지 추가

1. `src/content/docs/<섹션>/<파일명>.md` 생성, 상단에 frontmatter(`title`, `description`) 작성
2. `astro.config.mjs`의 `sidebar`에 새 항목 추가 (또는 `autogenerate` 디렉터리 사용 중이면 자동 반영)
3. `npm run build`로 빌드 오류 확인

## 예제 코드 실행 환경

Python 예제는 가상환경에서 실행을 권장합니다.

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

로컬에 GPU/시뮬레이터 환경이 없다면 [Colab 실습 가이드](/guides/colab-quickstart/)를 사용하세요.
