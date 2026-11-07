## Project purpose

This repo is a Starlight (Astro) docs site for personal notes on robot manipulation:
concepts, Drake/MuJoCo simulation, model training & validation, and a rerun.io-based
retraining data pipeline. Content pages live under `src/content/docs/`, grouped by
section (`concepts/`, `simulation/`, `training/`, `data-pipeline/`, `guides/`,
`examples/`). Executable notebooks live in `notebooks/` and are opened via Colab
badges linked from pages under `examples/`.

## Editing content

- Add/edit `.md` files under `src/content/docs/<section>/`. Every page needs
  `title` and `description` frontmatter.
- Register new pages in the `sidebar` array in `astro.config.mjs` (grouped to match
  the folder they live in) — pages not listed there won't appear in navigation.
- When adding a Colab-backed example, add the notebook to `notebooks/` and link it
  from the example page with the Colab badge pattern described in
  `src/content/docs/guides/colab-quickstart.md`.
- Run `npm run build` after content changes to catch broken links/frontmatter
  before pushing (GitHub Actions runs the same build on deploy).

## Development

When starting the dev server, use background mode:

```
astro dev --background
```

Manage the background server with `astro dev stop`, `astro dev status`, and `astro dev logs`.

## Documentation

Full documentation: https://docs.astro.build

Consult these guides before working on related tasks:

- [Adding pages, dynamic routes, or middleware](https://docs.astro.build/en/guides/routing/)
- [Working with Astro components](https://docs.astro.build/en/basics/astro-components/)
- [Using React, Vue, Svelte, or other framework components](https://docs.astro.build/en/guides/framework-components/)
- [Adding or managing content](https://docs.astro.build/en/guides/content-collections/)
- [Adding styles or using Tailwind](https://docs.astro.build/en/guides/styling/)
- [Supporting multiple languages](https://docs.astro.build/en/guides/internationalization/)
