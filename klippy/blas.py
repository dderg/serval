import os


def limit_blas_to_one_thread():
    """Keeps numpy's OpenBLAS to the importing thread unless the environment
    already chose a thread count.

    Importing numpy starts one OpenBLAS thread per core, each with an 8 MiB
    stack, and reserves a 32 MiB buffer per core. The host process runs
    under mlockall, so all of it stays resident: about 120 MB on a
    four-core Pi 4, 600 MB on a 16-core host. Klippy's numpy work is small
    arrays that gain nothing from BLAS threads. OpenBLAS reads the variable
    once, when numpy loads it, so this must run before the first numpy
    import."""
    os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
