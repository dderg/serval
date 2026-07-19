import { html, render } from "htm/preact";
import { mustEl } from "./api";
import { QueryRoot } from "./queries/client";
import { fetchDriveState } from "./queries/drive";
import { startRunsPolling, reconcileRuns } from "./runs";
import { App } from "./shell";

render(html`<${QueryRoot}><${App} /><//>`, mustEl("app"));
startRunsPolling((runs) => void reconcileRuns(runs));
fetchDriveState().catch((err) => console.error("drive state prefetch failed", err));
