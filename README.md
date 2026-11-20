# Robot Manipulation Lab

Robot manipulation 개념, Drake/MuJoCo 시뮬레이션, 실제 제어 모델 학습·검증, rerun.io 기반
재학습 데이터 파이프라인을 정리하는 개인 학습 노트 사이트입니다. 문서는
[docs-kit](https://github.com/sweaty-fingers/docs-kit)(공통 문서 규격, `.docs-kit/` 서브모듈)으로 쓰고,
GitHub Pages로 배포됩니다.

## 문서 사이트

```bash
git clone --recurse-submodules https://github.com/sweaty-fingers/robot-manipulation.git
.docs-kit/bin/docs.sh start      # 미리보기 http://localhost:8000/robot-manipulation/ (Docker, 없으면 DOCS_RUNNER=host)
.docs-kit/bin/docs.sh build      # 정적 사이트 build/site/ (WARNING 0이 문서 작업 끝)
```

설정은 `docs-kit.yml` 하나(켠 탭, 사이트 이름·배포 경로)입니다. 문서는 종류별 탭 폴더에 둡니다.

```
docs/
  README.md        개요 (첫 페이지)
  background/      배경: 개념, 시뮬레이터·학습·데이터 파이프라인 설명 (NN_*.md, 제목 "NN.")
  guides/          실행 가이드: 설정, Colab, 시뮬레이션 환경, 예제 (gN_*.md, 제목 "G<N>.")
  reference/       참고: 자료 모음(연구 지형, 논문 목록·상세, 교재, 하드웨어, 도구) (rN_*.md, 제목 "R<N>.")
  glossary/        용어집
  _static/         사이트 스타일(그림)
notebooks/         Colab에서 여는 실제 .ipynb 파일
```

다이어그램은 ` ```mermaid ` 코드블록, 수식은 `$...$`로 씁니다. 작성 규칙은 사이트의 "문서 규격"·"문서 사이트 사용법"과
[AGENTS.md](./AGENTS.md)를 참고하세요.

## 코드 프레임워크 (`robot_manipulation/`)

사전학습 인코더 + 학습 가능한 action-expert + 교체 가능한 시뮬레이터 백엔드(Drake 우선)
+ rerun.io 데이터 파이프라인으로 구성된 Python 프레임워크입니다.

```bash
uv venv .venv && uv pip install --python .venv/bin/python -e ".[dev]"
python scripts/collect_demos.py --config configs/experiments/00_skeleton.yaml --n-episodes 50
python scripts/train.py --config configs/experiments/00_skeleton.yaml
python scripts/evaluate.py --config configs/experiments/00_skeleton.yaml
pytest tests/
```

구조와 설계 이유는 [AGENTS.md](./AGENTS.md)를 참고하세요.

## 배포

`main` 브랜치에 push하면 `.github/workflows/deploy.yml`이 docs-kit으로 빌드(`docs.sh build --strict`, 경고가 있으면 실패)한 뒤
GitHub Pages에 배포합니다. 배포 전 리포지토리 Settings → Pages에서 Source를
**GitHub Actions**로 설정해야 합니다.
