// Standalone AddressSanitizer/UBSan memory gate for the piece_sink parser. Built
// by scripts/fuzz-piece-sink.sh together with src/piece_sink.c and the seam stub.
// Throws random byte streams at feed()+commit(); ASan traps any out-of-bounds
// access. Define PIECE_SINK_LIBFUZZER to build a libFuzzer entry instead.
#include <stddef.h>
#include <stdint.h>
#include "mcu_transport_dispatch.h"

static void
drive(const uint8_t *data, size_t n)
{
    piece_sink_begin();
    for (size_t i = 0; i < n; i++)
        piece_sink_feed(data[i]);
    piece_sink_commit();
}

#ifdef PIECE_SINK_LIBFUZZER
int
LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    drive(data, size);
    return 0;
}
#else
int
main(void)
{
    uint64_t s = 0x9E3779B97F4A7C15ull;
    for (long t = 0; t < 2000000; t++) {
        s = s * 6364136223846793005ull + 1442695040888963407ull;
        size_t n = (size_t)((s >> 33) % 400);
        uint8_t buf[400];
        for (size_t i = 0; i < n; i++) {
            s = s * 6364136223846793005ull + 1442695040888963407ull;
            buf[i] = (uint8_t)(s >> 40);
        }
        drive(buf, n);
    }
    return 0;
}
#endif
