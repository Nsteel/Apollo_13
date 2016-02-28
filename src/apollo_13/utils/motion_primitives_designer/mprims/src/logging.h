#ifndef logging_h
#define logging_h

#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(fmt, ...) printf("[INFO] " fmt "\n", ##__VA_ARGS__); fflush(stdout)
#else
#define DEBUG_PRINT(fmt, ...)
#endif

#endif
