import { mustEl } from "./api";
import { fetchMacroHelp, loadCachedMacroHelp } from "./docs";
import { renderDriveBanner, fetchDriveState } from "./drive";
import { pollRtHealth } from "./live";
import { pollMoonrakerHealth, emergencyStop } from "./moonraker";
import { startRunsPolling } from "./runs";
import { pageFromHash, bindAccordionToggle, renderPage } from "./shell";
import { MOONRAKER_KEY, MOONRAKER_HEALTH_POLL_MS, RT_HEALTH_POLL_MS, state } from "./state";

function initShell() {
  mustEl("estop-btn").addEventListener("click", emergencyStop);
  const input = mustEl<HTMLInputElement>("moonraker-url");
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
  pollRtHealth();
  setInterval(pollRtHealth, RT_HEALTH_POLL_MS);
  bindAccordionToggle();
  window.addEventListener("hashchange", () => {
    state.page = pageFromHash();
    renderPage();
  });
  state.page = pageFromHash();
  renderPage();
}

initShell();
startRunsPolling();
fetchDriveState().catch((err) => console.error("drive state prefetch failed", err));
renderDriveBanner();
setInterval(renderDriveBanner, 1000);

export { initShell };
