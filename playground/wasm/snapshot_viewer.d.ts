/* tslint:disable */
/* eslint-disable */

export class TrajectoryData {
    free(): void;
    [Symbol.dispose](): void;
    a_cent(): Float64Array;
    a_scalar(): Float64Array;
    a_tang(): Float64Array;
    accel_impulse_mag(): Float64Array;
    accel_impulse_t(): Float64Array;
    ae(): Float64Array;
    ax(): Float64Array;
    ay(): Float64Array;
    az(): Float64Array;
    curvature_class(): Float64Array;
    constructor(json: string);
    has_toolhead(): boolean;
    j_cent(): Float64Array;
    j_scalar(): Float64Array;
    j_tang(): Float64Array;
    je(): Float64Array;
    jerk_impulse_mag(): Float64Array;
    jerk_impulse_t(): Float64Array;
    jx(): Float64Array;
    jy(): Float64Array;
    jz(): Float64Array;
    kappa(): Float64Array;
    kin_x(): Float64Array;
    kin_y(): Float64Array;
    point_count(): number;
    raw_x(): Float64Array;
    raw_y(): Float64Array;
    seam_max_da(): Float64Array;
    seam_max_dp(): Float64Array;
    seam_max_dv(): Float64Array;
    t(): Float64Array;
    th_a_cent(): Float64Array;
    th_a_scalar(): Float64Array;
    th_a_tang(): Float64Array;
    th_ax(): Float64Array;
    th_ay(): Float64Array;
    th_j_cent(): Float64Array;
    th_j_scalar(): Float64Array;
    th_j_tang(): Float64Array;
    th_jx(): Float64Array;
    th_jy(): Float64Array;
    th_kappa(): Float64Array;
    th_v_scalar(): Float64Array;
    th_vx(): Float64Array;
    th_vy(): Float64Array;
    th_x(): Float64Array;
    th_y(): Float64Array;
    traversal_time(): number;
    v_scalar(): Float64Array;
    ve(): Float64Array;
    vx(): Float64Array;
    vy(): Float64Array;
    vz(): Float64Array;
    worst_seams_json(): string;
}

export type InitInput = RequestInfo | URL | Response | BufferSource | WebAssembly.Module;

export interface InitOutput {
    readonly memory: WebAssembly.Memory;
    readonly __wbg_trajectorydata_free: (a: number, b: number) => void;
    readonly trajectorydata_from_json: (a: number, b: number) => [number, number, number];
    readonly trajectorydata_raw_x: (a: number) => any;
    readonly trajectorydata_raw_y: (a: number) => any;
    readonly trajectorydata_kin_x: (a: number) => any;
    readonly trajectorydata_kin_y: (a: number) => any;
    readonly trajectorydata_t: (a: number) => any;
    readonly trajectorydata_vx: (a: number) => any;
    readonly trajectorydata_vy: (a: number) => any;
    readonly trajectorydata_v_scalar: (a: number) => any;
    readonly trajectorydata_ax: (a: number) => any;
    readonly trajectorydata_ay: (a: number) => any;
    readonly trajectorydata_a_scalar: (a: number) => any;
    readonly trajectorydata_jx: (a: number) => any;
    readonly trajectorydata_jy: (a: number) => any;
    readonly trajectorydata_j_scalar: (a: number) => any;
    readonly trajectorydata_jerk_impulse_t: (a: number) => any;
    readonly trajectorydata_jerk_impulse_mag: (a: number) => any;
    readonly trajectorydata_accel_impulse_t: (a: number) => any;
    readonly trajectorydata_accel_impulse_mag: (a: number) => any;
    readonly trajectorydata_vz: (a: number) => any;
    readonly trajectorydata_ve: (a: number) => any;
    readonly trajectorydata_az: (a: number) => any;
    readonly trajectorydata_ae: (a: number) => any;
    readonly trajectorydata_jz: (a: number) => any;
    readonly trajectorydata_je: (a: number) => any;
    readonly trajectorydata_a_tang: (a: number) => any;
    readonly trajectorydata_a_cent: (a: number) => any;
    readonly trajectorydata_j_tang: (a: number) => any;
    readonly trajectorydata_j_cent: (a: number) => any;
    readonly trajectorydata_has_toolhead: (a: number) => number;
    readonly trajectorydata_th_x: (a: number) => any;
    readonly trajectorydata_th_y: (a: number) => any;
    readonly trajectorydata_th_vx: (a: number) => any;
    readonly trajectorydata_th_vy: (a: number) => any;
    readonly trajectorydata_th_ax: (a: number) => any;
    readonly trajectorydata_th_ay: (a: number) => any;
    readonly trajectorydata_th_jx: (a: number) => any;
    readonly trajectorydata_th_jy: (a: number) => any;
    readonly trajectorydata_th_v_scalar: (a: number) => any;
    readonly trajectorydata_th_a_scalar: (a: number) => any;
    readonly trajectorydata_th_j_scalar: (a: number) => any;
    readonly trajectorydata_th_a_tang: (a: number) => any;
    readonly trajectorydata_th_a_cent: (a: number) => any;
    readonly trajectorydata_th_j_tang: (a: number) => any;
    readonly trajectorydata_th_j_cent: (a: number) => any;
    readonly trajectorydata_th_kappa: (a: number) => any;
    readonly trajectorydata_seam_max_dp: (a: number) => any;
    readonly trajectorydata_seam_max_dv: (a: number) => any;
    readonly trajectorydata_seam_max_da: (a: number) => any;
    readonly trajectorydata_worst_seams_json: (a: number) => [number, number];
    readonly trajectorydata_traversal_time: (a: number) => number;
    readonly trajectorydata_point_count: (a: number) => number;
    readonly trajectorydata_kappa: (a: number) => any;
    readonly trajectorydata_curvature_class: (a: number) => any;
    readonly __wbindgen_externrefs: WebAssembly.Table;
    readonly __wbindgen_malloc: (a: number, b: number) => number;
    readonly __wbindgen_realloc: (a: number, b: number, c: number, d: number) => number;
    readonly __externref_table_dealloc: (a: number) => void;
    readonly __wbindgen_free: (a: number, b: number, c: number) => void;
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
