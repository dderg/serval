import { LIVE_UNIT_KEY } from "./state";

type FerrUnit = "µm" | "counts";

function loadFerrUnit(): FerrUnit {
  return localStorage.getItem(LIVE_UNIT_KEY) === "counts" ? "counts" : "µm";
}

function setFerrUnit(unit: FerrUnit) {
  localStorage.setItem(LIVE_UNIT_KEY, unit);
}

export type { FerrUnit };
export { loadFerrUnit, setFerrUnit };
