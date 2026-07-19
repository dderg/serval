import { afterEach, beforeEach, expect, test } from "bun:test";
import { act } from "preact/test-utils";
import { registerDom } from "./dom";
import type { StrainMap } from "../src/api/runs";
import type { RunSummary } from "../src/api/runs";

registerDom();

const strain = await import("../src/strain");
const { queryClient, QueryRoot } = await import("../src/queries/client");
const { html, render } = await import("htm/preact");
const { strainKey, strainViewKey } = await import("../src/queries/strain");
const { runKeys } = await import("../src/queries/runs");
const { state, PAGE_DEFS } = await import("../src/state");

const STRAIN_DEF = PAGE_DEFS.strain;

function line(name: string, swept: Record<string, number>, scale: number): StrainMap["lines"][number] {
  return {
    name,
    swept,
    bin_centers: [10, 30, 50],
    belts: [
      {
        pair: "AB",
        elastic: [1 * scale, 2 * scale, null],
        friction: [0.1 * scale, 0.2 * scale, null],
      },
    ],
  };
}

function strainMap(scale = 1): StrainMap {
  return {
    lines: [
      line("xline_0", { y: 20 }, scale),
      line("xline_1", { y: 40 }, scale * 1.5),
      line("yline_0", { x: 15 }, scale * 0.7),
      line("yline_1", { x: 35 }, scale * 1.2),
    ],
  };
}

function mismatchedMap(): StrainMap {
  const m = strainMap();
  m.lines = m.lines.map((l) => ({ ...l, bin_centers: [5, 25, 45] }));
  return m;
}

function runSummary(name: string, i: number): RunSummary {
  return {
    name,
    experiment: "strain_map",
    tag: `map${i}`,
    axis: "X",
    mtime_utc: `2026-01-01T00:00:0${i}Z`,
    has_results: true,
  } as unknown as RunSummary;
}

function seedRuns(names: string[]): RunSummary[] {
  const runs = names.map((n, i) => runSummary(n, i));
  queryClient.setQueryData(runKeys.all, runs);
  return runs;
}

function seedStrain(run: RunSummary, data: StrainMap) {
  queryClient.setQueryData(strainKey(run.name), { mtime_utc: run.mtime_utc, data });
  queryClient.setQueryData(strainViewKey(run.name), data);
}

let container: HTMLElement;

async function mount() {
  container = document.createElement("div");
  document.body.appendChild(container);
  await act(async () => {
    render(html`<${QueryRoot}><${strain.StrainPage} def=${STRAIN_DEF} /><//>`, container);
  });
}

beforeEach(() => {
  queryClient.clear();
  state.strain.selected = null;
  state.strain.compare.clear();
  state.strain.field = "elastic";
});

afterEach(() => {
  if (container) {
    render(null, container);
    container.remove();
  }
});

test("StrainPage renders the run selector, summary, heatmaps, profiles, and DC bars for the selected run", async () => {
  const runs = seedRuns(["strain_a", "strain_b"]);
  seedStrain(runs[0], strainMap());
  state.strain.selected = "strain_a";
  await mount();

  expect(container.querySelector(".workspace")).toBeTruthy();
  expect(container.querySelector("main.analysis")).toBeTruthy();
  expect(container.querySelector("aside.controls")).toBeTruthy();

  const rows = container.querySelectorAll("#strain-run-body tr");
  expect(rows.length).toBe(2);
  expect(rows[0].classList.contains("selectable")).toBe(true);
  expect(rows[0].classList.contains("selected")).toBe(true);
  expect(rows[1].classList.contains("selected")).toBe(false);

  const summary = container.querySelector("#strain-summary")!;
  expect(summary.textContent).toContain("max |elastic|");
  expect(summary.textContent).toContain("mean |friction|");

  expect(container.querySelectorAll("#strain-heatmaps .chart-box canvas").length).toBe(2);
  expect(container.querySelectorAll("#strain-profiles .chart-box").length).toBe(2);
  expect(container.querySelector("#strain-profiles .uplot")).toBeTruthy();
  expect(container.querySelectorAll("#strain-dc .chart-box canvas").length).toBe(2);

  const scale = container.querySelector("#strain-scale .strain-scale");
  expect(scale).toBeTruthy();
  expect(scale!.querySelector(".hint")!.textContent).toContain("elastic differential torque");
});
test("the strain view query key changes with the run revision", () => {
  const [run] = seedRuns(["strain_a"]);
  const initialKey = strainViewKey(run.name);
  queryClient.setQueryData(runKeys.all, [{ ...run, mtime_utc: "2026-01-01T00:01:00Z" }]);
  expect(strainViewKey(run.name)).not.toEqual(initialKey);
});


test("selecting a run auto-falls back to the first run and prunes stale compares", async () => {
  const runs = seedRuns(["strain_a", "strain_b"]);
  state.strain.selected = "gone";
  state.strain.compare.add("also_gone");
  const changed = strain.reconcileStrainSelection(runs);
  expect(changed).toBe(true);
  expect(state.strain.selected).toBe("strain_a");
  expect(state.strain.compare.size).toBe(0);
});

test("the field toggle redraws on the friction scale without refetching the strain map", async () => {
  const runs = seedRuns(["strain_a"]);
  seedStrain(runs[0], strainMap());
  state.strain.selected = "strain_a";
  await mount();

  const viewKey = strainViewKey("strain_a");
  const before = queryClient.getQueryData(viewKey);

  const friction = container.querySelector<HTMLButtonElement>('button[data-field="friction"]')!;
  expect(friction.disabled).toBe(false);
  await act(async () => {
    friction.dispatchEvent(new MouseEvent("click", { bubbles: true }));
  });

  expect(state.strain.field).toBe("friction");
  expect(
    container.querySelector<HTMLButtonElement>('button[data-field="friction"]')!.disabled
  ).toBe(true);
  expect(
    container.querySelector<HTMLButtonElement>('button[data-field="elastic"]')!.disabled
  ).toBe(false);
  expect(container.querySelector("#strain-scale .hint")!.textContent).toContain(
    "friction (direction-dependent) differential torque"
  );
  expect(container.querySelectorAll("#strain-heatmaps .chart-box canvas").length).toBe(2);
  expect(queryClient.getQueryData(viewKey)).toBe(before);
});

test("shift-click adds a compare run and renders a diff heatmap; a plain click resets the selection", async () => {
  const runs = seedRuns(["strain_a", "strain_b"]);
  seedStrain(runs[0], strainMap(1));
  seedStrain(runs[1], strainMap(2));
  state.strain.selected = "strain_a";
  await mount();

  const rowB = container.querySelectorAll("#strain-run-body tr")[1];
  await act(async () => {
    rowB.dispatchEvent(new MouseEvent("click", { bubbles: true, shiftKey: true }));
  });

  expect(state.strain.selected).toBe("strain_a");
  expect(state.strain.compare.has("strain_b")).toBe(true);
  expect(container.querySelectorAll("#strain-run-body tr")[1].classList.contains("compare")).toBe(
    true
  );

  const heads = [...container.querySelectorAll("#strain-heatmaps .chart-box h3")].map(
    (h) => h.textContent ?? ""
  );
  expect(heads.some((t) => t.startsWith("Δ"))).toBe(true);
  expect(container.querySelectorAll("#strain-heatmaps canvas").length).toBe(4);
  expect(
    container.querySelector<HTMLElement>('#strain-heatmaps .strain-scale')!.style.gridColumn
  ).toBe("1 / -1");

  const rowBAgain = container.querySelectorAll("#strain-run-body tr")[1];
  await act(async () => {
    rowBAgain.dispatchEvent(new MouseEvent("click", { bubbles: true }));
  });
  expect(state.strain.selected).toBe("strain_b");
  expect(state.strain.compare.size).toBe(0);
});

test("computeStrainDiffs skips a compare whose binning differs and diffs one that matches", () => {
  seedRuns(["strain_a", "strain_b", "strain_c"]);
  state.strain.selected = "strain_a";
  state.strain.field = "elastic";
  const selected = strainMap(1);

  const mismatch = strain.computeStrainDiffs(selected, [
    { name: "strain_b", data: mismatchedMap(), error: null },
  ]);
  expect(mismatch.nodes.length).toBe(0);
  expect(mismatch.skips[0]).toContain("dimensions differ");

  const errored = strain.computeStrainDiffs(selected, [
    { name: "strain_b", data: null, error: new Error("boom") },
  ]);
  expect(errored.skips[0]).toContain("boom");

  const matched = strain.computeStrainDiffs(selected, [
    { name: "strain_b", data: strainMap(2), error: null },
  ]);
  expect(matched.skips.length).toBe(0);
  const heat = matched.nodes.filter((n) => n.kind === "heat");
  const scale = matched.nodes.filter((n) => n.kind === "scale");
  expect(heat.length).toBe(2);
  expect(scale.length).toBe(1);
  expect(heat[0].kind === "heat" && heat[0].title.startsWith("Δ")).toBe(true);
});

test("with no strain runs the summary prompts a capture and draws nothing", async () => {
  seedRuns([]);
  await mount();
  expect(container.querySelector("#strain-summary")!.textContent).toContain(
    "no strain_map runs yet"
  );
  expect(container.querySelectorAll("#strain-heatmaps .chart-box").length).toBe(0);
  expect(container.querySelector("#strain-scale")!.childElementCount).toBe(0);
});

test("a selected run with no lines reports it and draws no charts", async () => {
  const runs = seedRuns(["strain_empty"]);
  seedStrain(runs[0], { lines: [] });
  state.strain.selected = "strain_empty";
  await mount();
  expect(container.querySelector("#strain-summary")!.textContent).toBe("run has no lines");
  expect(container.querySelectorAll("#strain-heatmaps .chart-box").length).toBe(0);
  expect(container.querySelectorAll("#strain-profiles .chart-box").length).toBe(0);
});

test("unmounting tears down the page without leaving markup or throwing", async () => {
  const runs = seedRuns(["strain_a"]);
  seedStrain(runs[0], strainMap());
  state.strain.selected = "strain_a";
  await mount();
  expect(container.querySelector("#strain-profiles .uplot")).toBeTruthy();
  await act(async () => {
    render(null, container);
  });
  expect(container.childElementCount).toBe(0);
});
