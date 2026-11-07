# Robot Manipulation Lab

Robot manipulation 개념, Drake/MuJoCo 시뮬레이션, 실제 제어 모델 학습·검증, rerun.io 기반
재학습 데이터 파이프라인을 정리하는 개인 학습 노트 사이트입니다. [Starlight](https://starlight.astro.build/)
(Astro) 기반이며, GitHub Pages로 배포됩니다.

## 로컬 실행

```bash
npm install
npm run dev
```

`http://localhost:4321`에서 확인.

## 콘텐츠 구조

```
src/content/docs/
  concepts/        로봇 매니퓰레이션 개념
  simulation/       Drake / MuJoCo 시뮬레이션
  training/          모델 학습 & 검증
  data-pipeline/    rerun.io 기반 재학습 파이프라인
  guides/            설정 가이드, Colab 실습 가이드
  examples/         코드 예제 (Colab 연동)
notebooks/           Colab에서 여는 실제 .ipynb 파일
```

새 페이지 작성 규칙은 [AGENTS.md](./AGENTS.md)를 참고하세요.

## 배포

`main` 브랜치에 push하면 `.github/workflows/deploy.yml`이 자동으로 빌드 후
GitHub Pages에 배포합니다. 배포 전 리포지토리 Settings → Pages에서 Source를
**GitHub Actions**로 설정해야 합니다.
