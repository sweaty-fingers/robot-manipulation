## Project purpose

This repo has two parts in one git repo, deliberately kept decoupled:

1. A Starlight (Astro) docs site (`src/content/docs/`) for personal notes on robot
   manipulation — concepts, Drake simulation, model training & validation, and a
   rerun.io-based retraining data pipeline.
2. `robot_manipulation/` — the actual Python framework: pretrained perception
   encoders feeding a trainable action-expert, backed by a swappable sim backend
   (Drake first; MuJoCo designed-for but not yet built) and a rerun.io data
   pipeline. See the plan this was built from for the full architecture reasoning:
   module boundaries, phased build order, what's deferred.

## Editing docs content

- Add/edit `.md` files under `src/content/docs/<section>/`. Every page needs
  `title` and `description` frontmatter.
- **Internal links must be relative (`../section/page/`), never absolute
  root paths (`/section/page/`)**. This site deploys under a GitHub Pages
  subpath (`/robot-manipulation/`); Starlight's own sidebar/pagination links
  get that base prefixed automatically, but hand-written markdown links and
  `hero.actions[].link` in frontmatter do NOT — an absolute link 404s. This bit
  us once already; always use relative paths for in-content links.
- Register new pages in the `sidebar` array in `astro.config.mjs` (grouped to match
  the folder they live in) — pages not listed there won't appear in navigation.
- When adding a Colab-backed example, add the notebook to `notebooks/` and link it
  from the example page with the Colab badge pattern described in
  `src/content/docs/guides/colab-quickstart.md`.
- Run `npm run build` after content changes to catch broken links/frontmatter
  before pushing (GitHub Actions runs the same build on deploy).
- When `robot_manipulation/` code changes in a way that affects a doc's claims
  (e.g. a schema, a script's CLI args, which architecture is "current phase"),
  update the corresponding doc page in the same change — there is no automated
  sync between the two.

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
