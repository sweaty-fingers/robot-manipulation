---
description: 문서 사이트와 robot_manipulation 패키지를 로컬에서 실행하는 방법.
---

# G1. 로컬 개발 환경 설정

이 레포는 두 개의 독립된 툴체인을 같은 git 레포에서 관리합니다 — 문서 사이트(docs-kit, `.docs-kit/` 서브모듈)와
`robot_manipulation` 패키지(Python)는 서로 영향을 주지 않습니다.

=== "문서 사이트 (docs-kit)"
    ```bash
    git submodule update --init          # .docs-kit/ 이 비어 있을 때
    .docs-kit/bin/docs.sh start          # Docker가 없으면 DOCS_RUNNER=host (Node 22.18 이상)
    ```

    `http://localhost:8000/robot-manipulation/`에서 확인. `docs/` 아래 `.md` 파일이나 `docs-kit.yml`을
    수정하면 바로 반영됩니다. 끄기는 `.docs-kit/bin/docs.sh stop`.

=== "Python 프레임워크 (uv)"
    ```bash
    uv venv .venv
    uv pip install --python .venv/bin/python -e ".[dev]"
    pytest tests/
    ```

    GPU/시뮬레이터 환경이 로컬에 없다면 [Colab 실습 가이드](g2_colab.md)를 사용하세요.


## 새 문서 페이지 추가

1. 종류에 맞는 탭 폴더에 파일을 만듭니다([문서 규격](../docs-standard.md) §8): 개념·도구 설명은 `docs/background/NN_<이름>.md`(제목 `# NN. ...`), 실행 절차는 `docs/guides/gN_<이름>.md`(`# G<N>. ...`), 자료는 `docs/reference/rN_<이름>.md`(`# R<N>. ...`). 탭 폴더에 두면 메뉴에 저절로 들어갑니다
2. 내부 링크는 `.md` 상대 경로로 씁니다(`../background/07_fk_and_ik.md#절-제목`). 사이트에서는 `.html`로 바뀌고, GitHub 화면에서도 그대로 열립니다
3. 탭(`===`), 알림(`!!! note`), 수식(`$...$`), Mermaid는 [문서 사이트 사용법](../docs-kit.md) §2의 문법을 씁니다
4. `.docs-kit/bin/docs.sh build`로 빌드해 `WARNING`이 0인지 확인합니다(깨진 링크·앵커, 메뉴에 없는 문서도 경고)
