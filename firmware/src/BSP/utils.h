#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

#define min(a,b)             \
({                           \
    __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    (_a < _b) ? _a : _b;       \
})

#define max(a,b)             \
({                           \
    __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    (_a > _b) ? _a : _b;     \
})


int32_t fastAbs(int32_t v);

#endif // UTILS_H
