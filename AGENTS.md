## Project purpose

This repo has two parts in one git repo, deliberately kept decoupled:

1. Docs (`docs/`) written with **docs-kit** (`.docs-kit/`, git submodule of
   https://github.com/sweaty-fingers/docs-kit, branch `main` = Astro site engine): personal notes on robot
   manipulation — concepts, Drake simulation, model training & validation, a rerun.io-based retraining data pipeline,
   and a reference collection for the research goal (non-perturbing grasp, LEAP Hand target).
2. `robot_manipulation/` — the actual Python framework: pretrained perception encoders feeding a trainable
   action-expert, backed by a swappable sim backend (Drake first; MuJoCo designed-for but not yet built) and a
   rerun.io data pipeline.

When `robot_manipulation/` code changes in a way that affects a doc's claims (a schema, a script's CLI args, which
architecture is "current phase"), update the corresponding doc page in the same change — there is no automated sync.

## Editing `robot_manipulation/` (the Python framework)

- Env: `uv venv .venv && uv pip install --python .venv/bin/python -e ".[dev]"`
  (or `pip install -e ".[dev]"` with any Python >=3.10).
- Swapping any piece (encoder, action-expert architecture, sim backend) should
  only ever require: a new module implementing the relevant base class
  (`Encoder`, `ActionExpert`, `RobotEnv`), one registry entry in
  `robot_manipulation/factory.py`, and a config field — never a change to
  `scripts/train.py`/`evaluate.py`. If a change forces editing those scripts,
  that's a signal the abstraction leaked; fix the interface, don't special-case
  the script.
- rerun-sdk (pinned loosely, currently tested against 0.42) has no public
  `.rrd` read-back API — don't try to reconstruct training data from `.rrd`.
  `RerunLogger` is visualization/debugging-only; `scripts/collect_demos.py`
  saves the real training arrays directly to `.npz` via
  `data/dataset_builder.py` at collection time.
- Run `pytest tests/` after any change under `robot_manipulation/`.

## 문서 관리 (docs-kit)

이 프로젝트의 문서는 **docs-kit**(`.docs-kit/`, git 서브모듈, 원격 https://github.com/sweaty-fingers/docs-kit)으로 씁니다. 문서를 어떤 종류로 나누고 어디에 무엇을 쓰는지의 원본은 **문서 규격**(`.docs-kit/assets/docs-standard.md`, 사이트 메뉴 "문서 규격")입니다. 이 절은 그 요약과 이 프로젝트에서 정한 것이고, 규격과 다르면 규격이 우선입니다.

- 문서 사이트: 공유 https://sweaty-fingers.github.io/robot-manipulation/ (main push → GitHub Actions 배포), 개발 미리보기 `.docs-kit/bin/docs.sh start` → http://localhost:8000/robot-manipulation/ (Docker가 없으면 `DOCS_RUNNER=host`)
- 켠 탭: `docs-kit.yml` (`.docs-kit/bin/kit.py status`). 지금: `minimal` + 배경·참고 켬, ADR 끔(키트 Astro 판 M4 전까지)
- 답변·문서는 한국어, "~합니다"체. 사실과 추정을 구분("확인하지 않음", "(추정)", `→ 해석:`). 날짜는 `YYYY-MM-DD`
- 코드·주석·CLI 출력은 영어. 커밋 메시지는 영어

### D1. 세션을 시작하면

1. `.docs-kit/bin/kit.py check --fetch`를 실행합니다. `WARN`이 있으면 작업 전에 사용자에게 알립니다(새 키트 버전이면 `kit.py update` 제안).
2. `git status`로 작업 트리를 봅니다. 내가 만들지 않은 변경은 고치거나 되돌리거나 커밋하지 않습니다.

### D2. 어디에 쓰나 (규격 §1~§6)

| 층 | 종류 | 위치 | 생명 주기 |
| --- | --- | --- | --- |
| 운영 | 실행 가이드 G | `docs/guides/gN_*.md`, 제목 `# G<N>.` | 갱신. 단계마다 명령 → 기대 결과 → 안 될 때 |
| 운영 | 참고 R (자료 모음: 연구 지형, 논문 목록·상세, 교재, 하드웨어, 도구) | `docs/reference/rN_*.md`, 제목 `# R<N>.`. 논문 상세는 번호 11부터, 형식은 `docs/reference/template.md` | 갱신 |
| 지식 | 배경 (개념, 시뮬레이터·학습·데이터 파이프라인 설명) | `docs/background/NN_*.md`, 제목 `# NN.` | 가끔 갱신 |
| 지식 | 용어집 | `docs/glossary/README.md` | 새 용어마다 한 줄 |

- 탭 폴더에 두면 메뉴에 저절로 들어갑니다. 탭 폴더 밖 문서는 경고가 납니다.
- 번호는 재사용하지 않습니다(G<N>, R<N>, 배경 NN). 링크될 제목에 ` / `, ` → `, ` - `를 쓰지 않습니다.
- 문서 사이 링크는 `.md` 상대 경로(`../background/07_fk_and_ik.md#절`), `docs/` 밖은 코드 경로(`` `scripts/x.py` ``).
- 할 일은 GitHub 이슈(https://github.com/sweaty-fingers/robot-manipulation/issues)에 두고 문서에는 이슈 번호만.
- 그림: Mermaid는 ` ```mermaid `, 수식은 `$...$`, 계산으로 만드는 그림은 `scripts/*.py --figs`의 인라인 SVG(스타일 `docs/_static/site.css`).
- 결정을 남길 일이 생기면 ADR 탭을 켜고(`kit.py enable adr`) ADR로, 실험 결과는 실험(EXP)·검증 기록(VR) 탭을 켜서 씁니다(사용자와 먼저 정함).

### D3. 근거 먼저

1. 프로젝트 사실(값, 결정, 경로, 명령, 상태)을 쓰기 전에 `python3 .docs-kit/bin/docs_search.py "<질문>"`으로 찾고 그 절을 읽습니다. 코드·설정이 답이면 그 파일을 읽습니다.
2. 외부 자료(논문, 강의 노트)는 실제로 읽은 범위를 적습니다(초록 확인, 본문 일부, 원문 미확인).
3. 못 찾으면 "확인 못 함"이라고 쓰고 추측으로 채우지 않습니다.

### D4. 쓰고 나서 확인

```bash
.docs-kit/bin/docs.sh build 2>&1 | grep -iE "warn|error"     # 아무것도 안 나와야 함 (배포는 --strict)
```

- 바꿨다고 보고하는 것은 미리보기나 빌드한 사이트 화면에서 확인합니다.

### D6. docs-kit을 고칠 때

여러 프로젝트가 같이 쓰는 것(문서 규격, 템플릿, 사이트 엔진, 탭, `bin/` 도구)은 이 프로젝트의 `docs/`가 아니라 docs-kit 저장소에서 고칩니다(`.docs-kit/AGENTS-GUIDE.md` §7). `VERSION`·`CHANGELOG.md`를 올리고 `.docs-kit/bin/selftest.sh` → `selftest OK`, docs-kit 먼저 push한 뒤 이 프로젝트에서 `.docs-kit`을 커밋합니다.

### D7. git

- 커밋과 push는 **사용자가 요청할 때만** 합니다(이 프로젝트는 사용자가 요청하면 `main`에 바로 커밋·push).
- 내 변경만 경로로 하나씩 `git add` 합니다. 사용자가 일부러 남겨 둔 변경(`.gitignore`의 `context_tmp.md` 줄, `.claude/`)은 넣지 않습니다.
- 커밋은 주제별로 나눕니다(도구 / 문서 / docs-kit 갱신).
