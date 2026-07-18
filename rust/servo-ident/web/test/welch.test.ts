import { expect, test } from "bun:test";
import { registerDom } from "./dom";

registerDom();
const { fftPow2Js, welchPsdJs } = await import("../src/dynamics");

// Reference values printed by servo_ident::psd::welch_psd (Rust) for the
// same input: sin(2π·50·k/1000), k = 0..4096, fs = 1000. The JS port must
// match the analyzer bin-for-bin, not just qualitatively.
const FS = 1000;
const F0 = 50;
const N = 4096;
const RUST_PEAK_IDX = 51;
const RUST_PEAK_FREQ = 49.8046875;
const RUST_BINS = 513;
const RUST_PSD_AROUND_PEAK: [number, number][] = [
  [48, 1.336961779829425e-5],
  [49, 1.652897797097623e-4],
  [50, 4.301515876991693e-2],
  [51, 3.238422539096017e-1],
  [52, 1.441694036134009e-1],
  [53, 7.511376373153963e-4],
  [54, 3.353016827577241e-5],
];
const RUST_TOTAL_POWER = 0.500006731970238;

function sinusoid(): number[] {
  return Array.from({ length: N }, (_, k) => Math.sin((2 * Math.PI * F0 * k) / FS));
}

test("welchPsdJs matches Rust welch_psd bin-for-bin on a known sinusoid", () => {
  const out = welchPsdJs(sinusoid(), FS);
  if (!out) throw new Error("welchPsdJs returned null for a 4096-sample input");
  const { freqs, psd } = out;
  expect(freqs.length).toBe(RUST_BINS);
  expect(psd.length).toBe(RUST_BINS);

  let peak = 0;
  for (let i = 1; i < psd.length; i++) if (psd[i] > psd[peak]) peak = i;
  expect(peak).toBe(RUST_PEAK_IDX);
  expect(freqs[peak]).toBeCloseTo(RUST_PEAK_FREQ, 10);

  for (const [i, rustValue] of RUST_PSD_AROUND_PEAK) {
    expect(Math.abs(psd[i] - rustValue)).toBeLessThan(Math.abs(rustValue) * 1e-9);
  }

  const df = freqs[1] - freqs[0];
  const total = psd.reduce((a, b) => a + b, 0) * df;
  expect(total).toBeCloseTo(RUST_TOTAL_POWER, 9);
  expect(total).toBeCloseTo(0.5, 4);
});

test("welchPsdJs refuses inputs too short for a 64-sample segment", () => {
  expect(welchPsdJs(sinusoid().slice(0, 40), FS)).toBeNull();
});

test("fftPow2Js recovers a single tone at the exact bin", () => {
  const n = 256;
  const bin = 17;
  const re = Array.from({ length: n }, (_, k) => Math.cos((2 * Math.PI * bin * k) / n));
  const im = new Array(n).fill(0);
  fftPow2Js(re, im);
  const mags = re.map((r, i) => Math.hypot(r, im[i]));
  expect(mags[bin]).toBeCloseTo(n / 2, 6);
  expect(mags[n - bin]).toBeCloseTo(n / 2, 6);
  for (let i = 0; i < mags.length; i++) {
    if (i === bin || i === n - bin) continue;
    expect(mags[i]).toBeLessThan(1e-6);
  }
});
