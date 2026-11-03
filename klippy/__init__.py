from .blas import limit_blas_to_one_thread

APP_NAME = "Serval"

limit_blas_to_one_thread()

from .printer import *  # noqa: E402, F403
