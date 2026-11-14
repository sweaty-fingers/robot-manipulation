// @ts-check
import { defineConfig } from 'astro/config';
import starlight from '@astrojs/starlight';

// GitHub Pages serves this repo at https://sweaty-fingers.github.io/robot-manipulation/
// Update `site`/`base` here if the GitHub user or repo name changes.
export default defineConfig({
	site: 'https://sweaty-fingers.github.io',
	base: '/robot-manipulation',
	integrations: [
		starlight({
			title: 'Robot Manipulation Lab',
			social: [{ icon: 'github', label: 'GitHub', href: 'https://github.com/sweaty-fingers/robot-manipulation' }],
			customCss: ['./src/styles/custom.css'],
			components: {
				Header: './src/components/SectionTabsHeader.astro',
				Sidebar: './src/components/ScopedSidebar.astro',
			},
			head: [
				{ tag: 'link', attrs: { rel: 'preconnect', href: 'https://fonts.googleapis.com' } },
				{ tag: 'link', attrs: { rel: 'preconnect', href: 'https://fonts.gstatic.com', crossorigin: true } },
				{
					tag: 'link',
					attrs: {
						rel: 'stylesheet',
						href: 'https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&display=swap',
					},
				},
			],
			sidebar: [
				{
					label: '개념 (Concepts)',
					items: [
						{ label: 'Manipulation 기초', slug: 'concepts/manipulation-basics' },
						{ label: '지각과 제어', slug: 'concepts/perception-and-control' },
						{ label: '촉각 없이 접촉을 다루는 법', slug: 'concepts/touchless-grasping' },
						{ label: '용어집', slug: 'concepts/glossary' },
					],
				},
				{
					label: '시뮬레이션 (Drake / MuJoCo)',
					items: [
						{ label: '개요', slug: 'simulation/drake-mujoco-overview' },
						{ label: '환경 구축', slug: 'simulation/sim-environment-setup' },
						{ label: 'Sim-to-Real 검증', slug: 'simulation/sim-to-real-validation' },
					],
				},
				{
					label: '모델 학습 & 검증',
					items: [
						{ label: '모델 학습', slug: 'training/model-training' },
						{ label: '모델 검증', slug: 'training/validation' },
					],
				},
				{
					label: '데이터 파이프라인 (rerun.io)',
					items: [
						{ label: 'rerun.io 개요', slug: 'data-pipeline/rerun-overview' },
						{ label: '재학습 루프', slug: 'data-pipeline/retraining-loop' },
					],
				},
				{
					label: '자료 (References)',
					items: [
						{ label: '사용법', slug: 'references' },
						{ label: '연구 지형', slug: 'references/landscape' },
						{ label: '논문 목록', slug: 'references/papers' },
						{ label: '하드웨어 & 제품', slug: 'references/hardware' },
						{ label: '시뮬레이션 도구', slug: 'references/simulation-tools' },
					],
				},
				{
					label: '가이드',
					items: [
						{ label: '로컬 환경 설정', slug: 'guides/setup' },
						{ label: 'Colab 실습 가이드', slug: 'guides/colab-quickstart' },
					],
				},
				{
					label: '예제',
					items: [
						{ label: 'Point-Mass Reach (Drake)', slug: 'examples/example-01-reach' },
					],
				},
			],
		}),
	],
});
