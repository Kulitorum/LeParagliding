#include <math.h>

#include "f2c.h"

#ifdef __cplusplus
extern "C" {
#endif

int
signbit_f2c(double *value)
{
#ifdef _MSC_VER
	return value != 0 && _copysign(1., *value) < 0.;
#else
	return value != 0 && signbit(*value) != 0;
#endif
}

#ifdef __cplusplus
}
#endif
