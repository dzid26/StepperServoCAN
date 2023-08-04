#include "utils.h"

int32_t fastAbs(int32_t v)
{
	int32_t t;
	t = v >> 31;
	return (v ^ t) - t;
}
