import { expect, test } from "bun:test";
import { registerDom } from "./dom";

registerDom();

const { GROUP_ORDER, RETIRED_PARAMS, groupParams, paramGroupSection } = await import("../src/drive");
const fixture = JSON.parse(
  await Bun.file(new URL("./fixtures/drive_state.json", import.meta.url).pathname).text()
);

function param(name: string, group: string) {
  return {
    name,
    c_code: "C99.00",
    addr: "0x2063.0x01",
    type: "u16",
    unit: "",
    group,
    description: name,
    options: null,
  };
}

test("retired params never render, whatever group a stale payload puts them in", () => {
  const stale = [
    param("gain_mode", "gains"),
    param("stiffness_level", "gains"),
    param("adaptive_notch_mode", "notch"),
    param("inertia_ratio", "load"),
    param("position_gain", "gains"),
  ];
  const sections = groupParams(stale as any);
  const rendered = [...sections.values()].flat().map((p) => p.name);
  expect(rendered).toEqual(["position_gain"]);
  for (const name of RETIRED_PARAMS) expect(rendered).not.toContain(name);
});

test("the load group is gone from the section order", () => {
  expect(GROUP_ORDER).not.toContain("load");
  expect(paramGroupSection(param("inertia_ratio", "load") as any)).toBe("other");
});

test("the shipped fixture carries no retired params and no load group", () => {
  const names = fixture.params.map((p: any) => p.name);
  for (const name of RETIRED_PARAMS) expect(names).not.toContain(name);
  expect(fixture.params.every((p: any) => p.group !== "load")).toBe(true);
  const sections = groupParams(fixture.params);
  expect(sections.get("gains")!.map((p: any) => p.name)).toEqual([
    "position_gain",
    "speed_gain",
    "integral_time",
  ]);
  expect(sections.get("other")!).toEqual([]);
});
