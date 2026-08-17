#ifndef __SAMPLE_WIRE_H
#define __SAMPLE_WIRE_H
// Wire contract for the sample-stream transport, mirrored from
// rust/runtime/src/sample_wire.rs. Use these macros in the DECL_COMMAND /
// DECL_ENCODER strings so the two sides cannot drift; runtime's
// sample_wire_tests.rs parses this header and asserts they match.

#define SAMPLE_ANCHOR_ARGS "sample_anchor oid=%c clock=%u position=%i"
#define SAMPLE_RUN_ARGS "sample_run oid=%c interval=%u count=%c data=%*s"
#define SAMPLE_OVERLAY_ARGS \
    "sample_overlay oid=%c clock=%u interval=%u count=%c data=%*s"
#define SAMPLE_GET_POSITION_ARGS "sample_get_position oid=%c"
#define SAMPLE_POSITION_ARGS "sample_position oid=%c clock=%u position=%i"

// Mirrors SAMPLE_RUN_DATA_MAX / SAMPLE_RUN_COUNT_MAX in sample_run.rs: a
// Klipper block payload is 59 bytes and the header fields claim the rest.
#define SAMPLE_RUN_DATA_MAX 48
#define SAMPLE_RUN_COUNT_MAX 48

#endif // sample_wire.h
