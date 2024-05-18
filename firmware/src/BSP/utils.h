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


#define clip(x, low, high) ({\
  __typeof__(x) _x = (x); \
  __typeof__(low) _low = (low);\
  __typeof__(high) _high = (high);\
  (_x > _high) ? _high : ((_x < _low) ? _low : _x);\
  })

uint32_t fastAbs(int32_t v);

#define PI_X1024 3217U
#define TWO_PI_X1024 6434U

#endif // UTILS_H
