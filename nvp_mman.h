#ifndef LEDGER_SRC_MMAN_H_
#define LEDGER_SRC_MMAN_H_

// Some functions for tinkering with the cache

#include <emmintrin.h>
#include "perf_delay/add_delay.h"

/*
void _mm_clflush(void const*p) - Cache line containing p is flushed and invalidated from all caches in the coherency domain.

void _mm_lfence(void) - Guarantees that every load instruction that precedes, in program order, the load fence instruction is globally visible before any load instruction that follows the fence in program order.

_mm_sfence(void) - Guarantees that every preceding store is globally visible before any subsequent store.

void _mm_mfence(void) - Guarantees that every memory access that precedes, in program order, the memory fence instruction is globally visible before any memory instruction that follows the fence in program order.
*/

// TODO: what should this size be?
#define CLFLUSH_SIZE 64
#define _mm_clflushopt(addr)\
	asm volatile("clflushopt %0" : "+m" (*(volatile char *)(addr)))

static inline void do_cflush_len(volatile void* addr, size_t length)
{
	// note: it's necessary to do an mfence before and after calling this function

	size_t i;
	//unsigned long long ctr = 0;
	for (i = 0; i < length; i += CLFLUSH_SIZE) {
		//ctr++;
		//printf("%s: addr = %lu\n", __func__, addr + i);
		_mm_clflush((void const*)(addr + i));
	}

	//printf("%s: number of cache flushes = %llu\n", __func__, ctr);
}


static inline void do_cflushopt_len(volatile void* addr, size_t length)
{
	// note: it's necessary to do an mfence before and after calling this function
	size_t i;
	for (i = 0; i < length; i += CLFLUSH_SIZE) {
		_mm_clflushopt((void *)(addr + i));
#if NVM_DELAY
		perfmodel_add_delay(0, CLFLUSH_SIZE);
#endif
	}
}

/*
// these functions might be useful, but the intrinsics in emmintrin.h are almost certainly better.

static inline void do_mfence(void)
{
	asm  volatile ( "mfence \n" 
	: 
	: 
	: "memory");
}

static inline void do_clflush(volatile void *vaddr)
{
	//asm volatile("clflush %0" : "+m" (*(char *)vaddr));
	asm volatile("clflush (%0)" :: "r" (vaddr));
}
*/

#endif
