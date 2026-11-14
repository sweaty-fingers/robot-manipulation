import { defineCollection } from 'astro:content';
import { docsLoader } from '@astrojs/starlight/loaders';
import { docsSchema } from '@astrojs/starlight/schema';
import { z } from 'astro/zod';

export const collections = {
	docs: defineCollection({
		loader: docsLoader(),
		// `template`: references/papers 상세 페이지의 템플릿 버전(예: v0.1).
		// 템플릿을 바꿀 때 구버전 문서를 찾아내는 용도.
		schema: docsSchema({ extend: z.object({ template: z.string().optional() }) }),
	}),
};
