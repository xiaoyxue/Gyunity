#pragma once
#define HAVE_ALIGNAS
#define L1_CACHE_LINE_SIZE 64

#include "common/Core.h"

GYT_NAMESPACE_BEGIN

#define MEMORY_POOL_ALLOC(arena, Type) new ((arena).Alloc(sizeof(Type))) Type


void* AllocAligned(size_t size);

template <typename T>
T* AllocAligned(size_t count) {
    return (T*)AllocAligned(count * sizeof(T));
}

void FreeAligned(void*);


class
#ifdef HAVE_ALIGNAS
    alignas(L1_CACHE_LINE_SIZE)
#endif
    MemoryPool {
public:
    MemoryPool(size_t blockSize = 262144) : blockSize(blockSize) {}

    ~MemoryPool() {
        FreeAligned(currentBlock);
        for (auto& block : usedBlocks) FreeAligned(block.second);
        for (auto& block : availableBlocks) FreeAligned(block.second);
    }

    void* Alloc(size_t nBytes) {
        // Round up _nBytes_ to minimum machine alignment

#if !defined(HAVE_ALIGNOF)
        const int align = 16;
#else
        const int align = alignof(std::max_align_t);
#endif

        //assert(IsPowerOf2(align));

        nBytes = (nBytes + align - 1) & ~(align - 1);
        if (currentBlockPos + nBytes > currentAllocSize) {
            // Add current block to _usedBlocks_ list
            if (currentBlock) {
                usedBlocks.push_back(
                    std::make_pair(currentAllocSize, currentBlock));
                currentBlock = nullptr;
                currentAllocSize = 0;
            }

            // Get new block of memory for _MemoryArena_

            // Try to get memory block from _availableBlocks_
            for (auto iter = availableBlocks.begin();
                iter != availableBlocks.end(); ++iter) {
                if (iter->first >= nBytes) {
                    currentAllocSize = iter->first;
                    currentBlock = iter->second;
                    availableBlocks.erase(iter);
                    break;
                }
            }
            if (!currentBlock) {
                currentAllocSize = std::max(nBytes, blockSize);
                currentBlock = AllocAligned<uint8_t>(currentAllocSize);
            }
            currentBlockPos = 0;
        }
        void* ret = currentBlock + currentBlockPos;
        currentBlockPos += nBytes;
        return ret;
    }

	template <typename T>
	T* Alloc(size_t n = 1, bool runConstructor = true) {
		T* ret = (T*)Alloc(n * sizeof(T));
		if (runConstructor)
			for (size_t i = 0; i < n; ++i) new (&ret[i]) T();
		return ret;
	}

	void Reset() {
		currentBlockPos = 0;
		availableBlocks.splice(availableBlocks.begin(), usedBlocks);
	}

	size_t TotalAllocated() const {
		size_t total = currentAllocSize;
		for (const auto& alloc : usedBlocks) total += alloc.first;
		for (const auto& alloc : availableBlocks) total += alloc.first;
		return total;
	}
private:
    MemoryPool(const MemoryPool&) = delete;
    MemoryPool& operator=(const MemoryPool&) = delete;
    // MemoryArena Private Data
    const size_t blockSize;
    size_t currentBlockPos = 0, currentAllocSize = 0;
    uint8_t* currentBlock = nullptr;
    std::list<std::pair<size_t, uint8_t*>> usedBlocks, availableBlocks;
};

GYT_NAMESPACE_END