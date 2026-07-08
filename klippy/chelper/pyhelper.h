#ifndef PYHELPER_H
#define PYHELPER_H

double get_monotonic(void);
void set_python_logging_callback(void (*func)(const char *));
void errorf(const char *fmt, ...) __attribute__ ((format (printf, 1, 2)));
void report_errno(char *where, int rc);

#endif // pyhelper.h
