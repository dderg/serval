# Wrapper around C helper code
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import os

import cffi

######################################################################
# c_helper.so compiling
######################################################################

GCC_CMD = "gcc"
COMPILE_ARGS = (
    "-Wall -g -O2 -shared -fPIC"
    " -flto -fwhole-program -fno-use-linker-plugin"
    " -o %s %s"
)
NATIVE_FLAGS = "-march=native -mtune=native"
SOURCE_FILES = [
    "pyhelper.c",
    "serialqueue.c",
    "stepcompress.c",
    "itersolve.c",
    "trapq.c",
    "linear_quintic.c",
    "pollreactor.c",
    "msgblock.c",
    "trdispatch.c",
    "kin_cartesian.c",
    "kin_corexy.c",
    "kin_corexz.c",
    "kin_delta.c",
    "kin_deltesian.c",
    "kin_polar.c",
    "kin_rotary_delta.c",
    "kin_winch.c",
    "kin_extruder.c",
    "kin_idex.c",
    "integrate.c",
    "bs_compose.c",
    "fir_compose.c",
    "linear_pa_compose.c",
    "cheb_fit.c",
]
DEST_LIB = "c_helper.so"
OTHER_FILES = [
    "list.h",
    "serialqueue.h",
    "stepcompress.h",
    "itersolve.h",
    "pyhelper.h",
    "trapq.h",
    "linear_quintic.h",
    "pollreactor.h",
    "msgblock.h",
    "integrate.h",
    "bs_compose.h",
    "fir_compose.h",
    "linear_pa_compose.h",
    "cheb_fit.h",
]

defs_stepcompress = """
    struct pull_history_steps {
        uint64_t first_clock, last_clock;
        int64_t start_position;
        int step_count, interval, add;
    };

    struct stepcompress *stepcompress_alloc(uint32_t oid);
    void stepcompress_fill(struct stepcompress *sc, uint32_t max_error
        , int32_t queue_step_msgtag, int32_t set_next_step_dir_msgtag);
    void stepcompress_set_invert_sdir(struct stepcompress *sc
        , uint32_t invert_sdir);
    void stepcompress_free(struct stepcompress *sc);
    int stepcompress_reset(struct stepcompress *sc, uint64_t last_step_clock);
    int stepcompress_set_last_position(struct stepcompress *sc
        , uint64_t clock, int64_t last_position);
    int64_t stepcompress_find_past_position(struct stepcompress *sc
        , uint64_t clock);
    int stepcompress_queue_msg(struct stepcompress *sc
        , uint32_t *data, int len);
    int stepcompress_queue_mq_msg(struct stepcompress *sc, uint64_t req_clock
        , uint32_t *data, int len);
    int stepcompress_extract_old(struct stepcompress *sc
        , struct pull_history_steps *p, int max
        , uint64_t start_clock, uint64_t end_clock);

    struct steppersync *steppersync_alloc(struct serialqueue *sq
        , struct stepcompress **sc_list, int sc_num, int move_num);
    void steppersync_free(struct steppersync *ss);
    void steppersync_set_time(struct steppersync *ss
        , double time_offset, double mcu_freq);
    int steppersync_flush(struct steppersync *ss, uint64_t move_clock
        , uint64_t clear_history_clock);
"""

defs_itersolve = """
    int32_t itersolve_generate_steps(struct stepper_kinematics *sk
        , double flush_time);
    double itersolve_check_active(struct stepper_kinematics *sk
        , double flush_time);
    int32_t itersolve_is_active_axis(struct stepper_kinematics *sk, char axis);
    void itersolve_set_trapq(struct stepper_kinematics *sk, struct trapq *tq);
    void itersolve_set_stepcompress(struct stepper_kinematics *sk
        , struct stepcompress *sc, double step_dist);
    double itersolve_calc_position_from_coord(struct stepper_kinematics *sk
        , double x, double y, double z);
    void itersolve_set_position(struct stepper_kinematics *sk
        , double x, double y, double z);
    double itersolve_get_commanded_pos(struct stepper_kinematics *sk);
"""

defs_trapq = """
    struct pull_move {
        double print_time, move_t;
        double start_v, accel;
        double start_x, start_y, start_z;
        double x_r, y_r, z_r;
    };

    struct trapq *trapq_alloc(void);
    void trapq_free(struct trapq *tq);
    void trapq_append_quintic(struct trapq *tq, double print_time
        , int n_phases, const double *phase_t_ends
        , double move_t, double arc_length, double v_cap_min
        , int shape_disabled
        , double start_pos_x, double start_pos_y, double start_pos_z
        , const double *coeff_buf);
    void build_linear_as_quintic_coeffs(
        double accel_t, double cruise_t, double decel_t,
        double start_v, double cruise_v, double accel,
        double axes_r_x, double axes_r_y, double axes_r_z,
        double start_pos_x, double start_pos_y, double start_pos_z,
        double coeff_buf[180]);
    void trapq_finalize_moves(struct trapq *tq, double print_time
        , double clear_history_time);
    void trapq_set_position(struct trapq *tq, double print_time
        , double pos_x, double pos_y, double pos_z);
    int trapq_extract_old(struct trapq *tq, struct pull_move *p, int max
        , double start_time, double end_time);
"""

defs_compose = """
    int bs_compose(
        int n_input_phases,
        const double *input_phase_t_ends,
        const double *input_coeffs,
        int bs_order,
        double shaper_freq,
        double damping_ratio,
        int out_capacity,
        double *out_phase_t_ends,
        double *out_coeffs);
    int fir_compose(
        int n_input_phases,
        const double *input_phase_t_ends,
        const double *input_coeffs,
        int n_impulses,
        const double *impulse_amplitudes,
        const double *impulse_delays,
        int out_capacity,
        double *out_phase_t_ends,
        double *out_coeffs);
    void linear_pa_compose(
        int n_phases,
        double *coeff_buf,
        double axis_n_x,
        double axis_n_y,
        double axis_n_z,
        double extr_r,
        double k_pa);
    void cheb_fit_degree4_nodes(
        double v_lo, double v_hi, double *out_nodes);
    double cheb_fit_degree4_interval(
        const double *samples,
        double *out_cheb_coeffs,
        double *out_mono_coeffs);
    int cheb_fit_degree4_piecewise(
        double v_lo, double v_hi,
        int n_breaks,
        const double *breaks,
        const double *samples,
        double *out_mono_coeffs,
        double *out_piece_v_bounds);
    double cheb_fit_degree4_eval_mono(
        const double *mono_coeffs,
        double v_lo, double v_hi,
        double v);
"""

defs_kin_cartesian = """
    struct stepper_kinematics *cartesian_stepper_alloc(char axis);
"""

defs_kin_corexy = """
    struct stepper_kinematics *corexy_stepper_alloc(char type);
"""

defs_kin_corexz = """
    struct stepper_kinematics *corexz_stepper_alloc(char type);
"""

defs_kin_delta = """
    struct stepper_kinematics *delta_stepper_alloc(double arm2
        , double tower_x, double tower_y);
"""

defs_kin_deltesian = """
    struct stepper_kinematics *deltesian_stepper_alloc(double arm2
        , double arm_x);
"""

defs_kin_polar = """
    struct stepper_kinematics *polar_stepper_alloc(char type);
"""

defs_kin_rotary_delta = """
    struct stepper_kinematics *rotary_delta_stepper_alloc(
        double shoulder_radius, double shoulder_height
        , double angle, double upper_arm, double lower_arm);
"""

defs_kin_winch = """
    struct stepper_kinematics *winch_stepper_alloc(double anchor_x
        , double anchor_y, double anchor_z);
"""

defs_kin_extruder = """
    struct stepper_kinematics *extruder_stepper_alloc(void);
    void extruder_set_pressure_advance(struct stepper_kinematics *sk
        , int n_params, double params[], double time_offset);
    struct pressure_advance_params;
    double pressure_advance_linear_model_func(double position
        , double pa_velocity, struct pressure_advance_params *pa_params);
    double pressure_advance_tanh_model_func(double position
        , double pa_velocity, struct pressure_advance_params *pa_params);
    double pressure_advance_recipr_model_func(double position
        , double pa_velocity, struct pressure_advance_params *pa_params);
    void extruder_set_pressure_advance_model_func(struct stepper_kinematics *sk
        , double (*func)(double, double, struct pressure_advance_params *));
    double extruder_get_step_gen_window(struct stepper_kinematics *sk);
"""

# Plan 8 Chunk 2 Task 13: defs_kin_shaper retired along with kin_shaper.c.
# The post-hoc step-generator shaper cascade has no successor on this
# fork — shaping is baked into the planner polynomial by
# blendplanner._bake_shaper_polynomial.

defs_kin_idex = """
    void dual_carriage_set_sk(struct stepper_kinematics *sk
        , struct stepper_kinematics *orig_sk);
    int dual_carriage_set_transform(struct stepper_kinematics *sk
        , char axis, double scale, double offs);
    struct stepper_kinematics * dual_carriage_alloc(void);
"""

defs_serialqueue = """
    #define MESSAGE_MAX 64
    struct pull_queue_message {
        uint8_t msg[MESSAGE_MAX];
        int len;
        double sent_time, receive_time;
        uint64_t notify_id;
    };

    struct serialqueue *serialqueue_alloc(int serial_fd, char serial_fd_type
        , int client_id);
    void serialqueue_exit(struct serialqueue *sq);
    void serialqueue_free(struct serialqueue *sq);
    struct command_queue *serialqueue_alloc_commandqueue(void);
    void serialqueue_free_commandqueue(struct command_queue *cq);
    void serialqueue_send(struct serialqueue *sq, struct command_queue *cq
        , uint8_t *msg, int len, uint64_t min_clock, uint64_t req_clock
        , uint64_t notify_id);
    void serialqueue_pull(struct serialqueue *sq
        , struct pull_queue_message *pqm);
    void serialqueue_set_wire_frequency(struct serialqueue *sq
        , double frequency);
    void serialqueue_set_receive_window(struct serialqueue *sq
        , int receive_window);
    void serialqueue_set_clock_est(struct serialqueue *sq, double est_freq
        , double conv_time, uint64_t conv_clock, uint64_t last_clock);
    void serialqueue_get_stats(struct serialqueue *sq, char *buf, int len);
    int serialqueue_extract_old(struct serialqueue *sq, int sentq
        , struct pull_queue_message *q, int max);
"""

defs_trdispatch = """
    void trdispatch_start(struct trdispatch *td, uint32_t dispatch_reason);
    void trdispatch_stop(struct trdispatch *td);
    struct trdispatch *trdispatch_alloc(void);
    struct trdispatch_mcu *trdispatch_mcu_alloc(struct trdispatch *td
        , struct serialqueue *sq, struct command_queue *cq, uint32_t trsync_oid
        , uint32_t set_timeout_msgtag, uint32_t trigger_msgtag
        , uint32_t state_msgtag);
    void trdispatch_mcu_setup(struct trdispatch_mcu *tdm
        , uint64_t last_status_clock, uint64_t expire_clock
        , uint64_t expire_ticks, uint64_t min_extend_ticks);
"""

defs_pyhelper = """
    void set_python_logging_callback(void (*func)(const char *));
    double get_monotonic(void);
"""

defs_std = """
    void free(void*);
"""

defs_all = [
    defs_pyhelper,
    defs_serialqueue,
    defs_std,
    defs_stepcompress,
    defs_itersolve,
    defs_trapq,
    defs_trdispatch,
    defs_kin_cartesian,
    defs_kin_corexy,
    defs_kin_corexz,
    defs_kin_delta,
    defs_kin_deltesian,
    defs_kin_polar,
    defs_kin_rotary_delta,
    defs_kin_winch,
    defs_kin_extruder,
    defs_kin_idex,
    defs_compose,
]


# Update filenames to an absolute path
def get_abs_files(srcdir, filelist):
    return [os.path.join(srcdir, fname) for fname in filelist]


# Return the list of file modification times
def get_mtimes(filelist):
    out = []
    for filename in filelist:
        try:
            t = os.path.getmtime(filename)
        except os.error:
            continue
        out.append(t)
    return out


# Check if the code needs to be compiled
def check_build_code(sources, target):
    src_times = get_mtimes(sources)
    obj_times = get_mtimes([target])
    return not obj_times or max(src_times) > min(obj_times)


# Check if the current gcc version supports a particular command-line option
def check_gcc_option(option):
    cmd = "%s %s -S -o /dev/null -xc /dev/null > /dev/null 2>&1" % (
        GCC_CMD,
        option,
    )
    res = os.system(cmd)
    return res == 0


# Check if the current gcc version supports a particular command-line option
def do_build_code(cmd):
    res = os.system(cmd)
    if res:
        msg = "Unable to build C code module (error=%s)" % (res,)
        logging.error(msg)
        raise Exception(msg)


FFI_main = None
FFI_lib = None
pyhelper_logging_callback = None


# Hepler invoked from C errorf() code to log errors
def logging_callback(msg):
    logging.error(FFI_main.string(msg))


# Return the Foreign Function Interface api to the caller
def get_ffi():
    global FFI_main, FFI_lib, pyhelper_logging_callback
    if FFI_lib is None:
        srcdir = os.path.dirname(os.path.realpath(__file__))
        srcfiles = get_abs_files(srcdir, SOURCE_FILES)
        ofiles = get_abs_files(srcdir, OTHER_FILES)
        destlib = get_abs_files(srcdir, [DEST_LIB])[0]
        if check_build_code(srcfiles + ofiles + [__file__], destlib):
            if check_gcc_option(NATIVE_FLAGS):
                cmd = "%s %s %s" % (GCC_CMD, NATIVE_FLAGS, COMPILE_ARGS)
            else:
                cmd = "%s %s" % (GCC_CMD, COMPILE_ARGS)
            logging.info("Building C code module %s", DEST_LIB)
            do_build_code(cmd % (destlib, " ".join(srcfiles)))
        FFI_main = cffi.FFI()
        for d in defs_all:
            FFI_main.cdef(d)
        FFI_lib = FFI_main.dlopen(destlib)
        # Setup error logging
        pyhelper_logging_callback = FFI_main.callback(
            "void func(const char *)", logging_callback
        )
        FFI_lib.set_python_logging_callback(pyhelper_logging_callback)
    return FFI_main, FFI_lib


######################################################################
# hub-ctrl hub power controller
######################################################################

HC_COMPILE_CMD = "gcc -Wall -g -O2 -o %s %s -lusb"
HC_SOURCE_FILES = ["hub-ctrl.c"]
HC_SOURCE_DIR = "../../lib/hub-ctrl"
HC_TARGET = "hub-ctrl"
HC_CMD = "sudo %s/hub-ctrl -h 0 -P 2 -p %d"


def run_hub_ctrl(enable_power):
    srcdir = os.path.dirname(os.path.realpath(__file__))
    hubdir = os.path.join(srcdir, HC_SOURCE_DIR)
    srcfiles = get_abs_files(hubdir, HC_SOURCE_FILES)
    destlib = get_abs_files(hubdir, [HC_TARGET])[0]
    if check_build_code(srcfiles, destlib):
        logging.info("Building C code module %s", HC_TARGET)
        do_build_code(HC_COMPILE_CMD % (destlib, " ".join(srcfiles)))
    os.system(HC_CMD % (hubdir, enable_power))


if __name__ == "__main__":
    get_ffi()
