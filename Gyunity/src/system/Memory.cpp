#include "Memory.h"
#include <memory>

GYT_NAMESPACE_BEGIN

void* AllocAligned(size_t size) {

    return _aligned_malloc(size, GYT_L1_CACHE_LINE_SIZE);

}


void FreeAligned(void* ptr) {
    if (!ptr) return;
    _aligned_free(ptr);
}

GYT_NAMESPACE_END