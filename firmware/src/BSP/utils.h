#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

// cppcheck-suppress-macro misra-c2012-1.2
#define min(a,b)             \
__extension__ ({             \
    __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    (_a < _b) ? _a : _b;     \
})

// cppcheck-suppress-macro misra-c2012-1.2
#define max(a,b)             \
__extension__ ({             \
    __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    (_a > _b) ? _a : _b;     \
})


// cppcheck-suppress-macro misra-c2012-1.2
#define clip(x, low, high) \
__extension__ ({\
  __typeof__(x) _x = (x); \
  __typeof__(low) _low = (low);\
  __typeof__(high) _high = (high);\
  (_x > _high) ? _high : ((_x < _low) ? _low : _x);\
  })

uint32_t fastAbs(int32_t v);
#define PI 3.1415f

#define PI_X1024 3217U
#define TWO_PI_X1024 6434U

#define Ohm_to_mOhm 1000
#define H_to_uH 1000000
#define V_to_mV 1000

#endif // UTILS_H
