# MkDocs documentation

The main website imports the developer usage guide from `docs/` on `main`.
Keep the aggregator import branch, installation commands, and source links aligned
with `main`.
Its path is: install and build, run the supplied example, choose a Map and
Planner, write a WorldView, assemble a control pipeline, and configure its
Behavior Tree. Getting Started follows the layered `finenav_example` source.
The `finenav_planner` and `finenav_map` sections catalogue the independent
component repositories and link to custom Planner and Map development.

The earlier detailed documentation remains in `docs-detailed/`, with its OpenWiki sources
in `openwiki/`. Both versions use the same Material theme.

See [the developer-documentation workflow](DOCS-WORKFLOW.md) for the agreed
organization-wide source layout, bilingual conventions, aggregation, and deployment.
Each code repository uses `docs/` with English `.md` and Chinese
`.zh.md` files, built by the public `FineNav.github.io` repository on a schedule
or manual trigger. Engine's source layout is ready for aggregation. The
[aggregator repository](https://github.com/FineNav/FineNav.github.io) owns the
build and deployment workflow; its Actions runs report deployment status. Individual code
repositories do not deploy separate sites. The retained `docs-user/` directory
is an inactive historical copy; edit `docs/` for future updates.
The historical copies and OpenWiki export assets described below are local
archives and are not required to build or import the active guide.

## Build and preview the user guide

From the repository root:

```bash
python -m venv .venv-docs
source .venv-docs/bin/activate
python -m pip install -r requirements-docs.txt
python -m mkdocs build --strict
python -m mkdocs serve --dev-addr 127.0.0.1:8008
```

In Windows PowerShell, use `.venv-docs\Scripts\python.exe` for the Python commands.
`mkdocs.yml` builds `docs/` into `.site-docs/`. English is the default language;
Chinese uses `.zh.md` suffixes and is served under `/zh/`. All current guide
pages have Chinese versions; future untranslated pages fall back to English.
The local preview is <http://127.0.0.1:8008/>.

## Build and preview the detailed version

```bash
python -m mkdocs build --strict -f mkdocs.detailed.yml
python -m mkdocs serve -f mkdocs.detailed.yml --dev-addr 127.0.0.1:8009
```

`mkdocs.detailed.yml` builds `docs-detailed/` into `.site-docs-detailed/`.
The local preview is <http://127.0.0.1:8009/> while that server is running.

A full snapshot of the detailed version was saved on 2026-10-10 at
`F:\FineNav-Engine-docs-versions\detailed-v1`. It contains the Markdown,
OpenWiki sources, export script, configuration, and built HTML. Its own
`mkdocs.yml` can be used to rebuild or preview that saved version independently.

## Refresh detailed pages after OpenWiki updates

```bash
python scripts/export_openwiki.py
python -m mkdocs build --strict -f mkdocs.detailed.yml
```

Exporting updates `docs-detailed/` only; it does not change the developer guide in `docs/`.
Do not edit exported pages directly, as subsequent exports overwrite them.
When OpenWiki adds or removes pages, update the detailed navigation and remove
obsolete exported pages deliberately. Source references link to a pinned Git
revision; uncommitted source changes are unavailable through those links.
## Plugin catalogue

The developer-facing plugin catalogue lives in `docs/guide/plugins/`, with a section
index and separate Map and Planner pages. Add integrated plugin repositories to
the appropriate category when their sources are available.

Each plugin entry should document its purpose, repository, supported environment,
installation instructions, package and registration names, input/output or Map
queries, recommended WorldView pairings, and a minimal usage example. Use the
plugin source and README to verify the entry. Keep framework implementation
details in the detailed documentation.
