#include "LowDiscrepancy.h"

GYT_NAMESPACE_BEGIN

int IsPrime(int a) noexcept {
	GYT_DEBUG_ASSERT(a >= 2);
	for (int i = 2; i * i <= a; i++) {
		if (a % i == 0)
			return false;
	}
	return true;
}

GYT_NAMESPACE_END