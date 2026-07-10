/* tslint:disable */
/* eslint-disable */

export function init(): void;

/**
 * Plans the pasted gcode under the given config and returns the snapshot
 * JSON — the same schema the snapshot baselines use, directly consumable by
 * the snapshot-viewer `TrajectoryData`.
 */
export function plan(gcode_text: string, config_json: string): string;

/**
 * Like [`plan`] (byte-identical final JSON), but invokes `on_partial` with
 * the JSON string of a schema-complete partial snapshot — the trajectory
 * pieces produced so far — every [`plan_core::PARTIAL_BATCH_SEGMENTS`]
 * shaped segments, so the UI can draw the trajectory as it grows.
 */
export function plan_streaming(gcode_text: string, config_json: string, on_partial: Function): string;

export type InitInput = RequestInfo | URL | Response | BufferSource | WebAssembly.Module;

export interface InitOutput {
    readonly memory: WebAssembly.Memory;
    readonly plan: (a: number, b: number, c: number, d: number) => [number, number, number, number];
    readonly plan_streaming: (a: number, b: number, c: number, d: number, e: any) => [number, number, number, number];
    readonly init: () => void;
    readonly __wbindgen_exn_store: (a: number) => void;
    readonly __externref_table_alloc: () => number;
    readonly __wbindgen_externrefs: WebAssembly.Table;
    readonly __wbindgen_free: (a: number, b: number, c: number) => void;
    readonly __wbindgen_malloc: (a: number, b: number) => number;
    readonly __wbindgen_realloc: (a: number, b: number, c: number, d: number) => number;
    readonly __externref_table_dealloc: (a: number) => void;
    readonly __wbindgen_start: () => void;
}

export type SyncInitInput = BufferSource | WebAssembly.Module;

/**
 * Instantiates the given `module`, which can either be bytes or
 * a precompiled `WebAssembly.Module`.
 *
 * @param {{ module: SyncInitInput }} module - Passing `SyncInitInput` directly is deprecated.
 *
 * @returns {InitOutput}
 */
export function initSync(module: { module: SyncInitInput } | SyncInitInput): InitOutput;

/**
 * If `module_or_path` is {RequestInfo} or {URL}, makes a request and
 * for everything else, calls `WebAssembly.instantiate` directly.
 *
 * @param {{ module_or_path: InitInput | Promise<InitInput> }} module_or_path - Passing `InitInput` directly is deprecated.
 *
 * @returns {Promise<InitOutput>}
 */
export default function __wbg_init (module_or_path?: { module_or_path: InitInput | Promise<InitInput> } | InitInput | Promise<InitInput>): Promise<InitOutput>;
