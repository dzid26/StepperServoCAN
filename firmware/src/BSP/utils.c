#include "utils.h"
//
#define UINT32_BIT_SIZE 32U
int32_t fastAbs(int32_t v)
{
	uint32_t u = (uint32_t)v;
	uint32_t mask = ~(u >> (UINT32_BIT_SIZE - 1U)) + 1U;
    uint32_t u_abs_value = (u ^ mask) - mask;

    /* When v is INT32_MIN, u_abs_value will be 2147483648 (as unsigned).
       This line will turn it into INT32_MAX instead (as unsigned). */
    u_abs_value &= ~(mask & (u_abs_value >> (UINT32_BIT_SIZE - 1U)));

    int32_t abs_value = (int32_t)u_abs_value;
    return abs_value;
}