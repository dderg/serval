# Vendored frontend libraries

No-build ESM bundles, fetched once and checked in so `servo-cal serve`
stays a single self-contained binary. Not wired into `app.js`/`index.html`
yet — this directory only stages the assets and their HTTP routes.

| File | Version | Upstream source |
| --- | --- | --- |
| `htm-preact-standalone-3.1.1.mjs` | htm 3.1.1 / preact 10.29.7 | https://unpkg.com/htm@3.1.1/preact/standalone.module.js |
| `uPlot-1.6.32.esm.js` | 1.6.32 | https://unpkg.com/uplot@1.6.32/dist/uPlot.esm.js |
| `uPlot-1.6.32.min.css` | 1.6.32 | https://unpkg.com/uplot@1.6.32/dist/uPlot.min.css |

`htm-preact-standalone-3.1.1.mjs` is htm's official combined build: it
bundles preact, preact/hooks, and htm's tagged-template `html` function
into one dependency-free ESM module, so a relative `import` pulls in both
libraries. `LICENSE-preact` and `LICENSE-htm` cover the code each
contributes to that bundle; `LICENSE-uplot` covers the uPlot files.

To update a pin, re-run the `curl` commands above against the new
version and update this table.
