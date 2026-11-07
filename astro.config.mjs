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
			sidebar: [
				{
					label: '개념 (Concepts)',
					items: [
						{ label: 'Manipulation 기초', slug: 'concepts/manipulation-basics' },
						{ label: '지각과 제어', slug: 'concepts/perception-and-control' },
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
