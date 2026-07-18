import { el } from "./api.js";
import { fetchMacroHelp, loadCachedMacroHelp } from "./docs.js";
import { renderDriveBanner, loadDriveState } from "./drive.js";
import { pollMoonrakerHealth, emergencyStop } from "./moonraker.js";
import { refresh } from "./runs.js";
import { pageFromHash, bindAccordionToggle, renderPage } from "./shell.js";
import { REFRESH_MS, MOONRAKER_KEY, MOONRAKER_HEALTH_POLL_MS, state } from "./state.js";

// --- boot -------------------------------------------------------------------

function initShell() {
  el("estop-btn").addEventListener("click", emergencyStop);
  const input = el("moonraker-url");
  input.value = localStorage.getItem(MOONRAKER_KEY) || `http://${location.hostname}:7125`;
  input.addEventListener("change", () => {
    localStorage.setItem(MOONRAKER_KEY, input.value);
    pollMoonrakerHealth();
    fetchMacroHelp();
  });
  loadCachedMacroHelp();
  fetchMacroHelp();
  pollMoonrakerHealth();
  setInterval(pollMoonrakerHealth, MOONRAKER_HEALTH_POLL_MS);
  bindAccordionToggle();
  window.addEventListener("hashchange", () => {
    state.page = pageFromHash();
    renderPage();
  });
  state.page = pageFromHash();
  renderPage();
}

async function tick() {
  try {
    await refresh();
  } catch (e) {
    console.error(e);
  }
  renderDriveBanner();
}

initShell();
tick();
loadDriveState();
setInterval(tick, REFRESH_MS);
setInterval(renderDriveBanner, 1000);

export { initShell, tick };
