// Header file for nvmfileops.c

#ifndef __NV_FILEOPS_H_
#define __NV_FILEOPS_H_

#include "nv_common.h"
#include "nvp_lock.h"
#include "liblfds711/inc/liblfds711.h"

#define ENV_NV_FOP "NVP_NV_FOP"

#define _NVP_USE_DEFAULT_FILEOPS NULL

// Declare the real_close function, as it will be called by bg thread
RETT_CLOSE _nvp_REAL_CLOSE(INTF_CLOSE, ino_t serialno, int async_file_closing);

/******************* Data Structures ********************/

struct NVFile
{
	NVP_LOCK_DECL;
	volatile bool valid;
	int fd;
	volatile size_t* offset;
	bool canRead;
	bool canWrite;
	bool append;
	bool aligned;
	ino_t serialno; // duplicated so that iterating doesn't require following every node*
	struct NVNode* node;
	bool posix;
	bool debug;
	char padding[200];
};

struct NVNode
{
	ino_t serialno;
	ino_t backup_serialno;
	NVP_LOCK_DECL;
//	char* volatile data; // the pointer itself is volatile
	unsigned long true_length;
	unsigned long true_length_for_read;
	volatile size_t length;
	volatile size_t maplength;
	unsigned long *root;
	unsigned long *anonymous_root;
	unsigned long *anonymous_offset_start;
	unsigned long *anonymous_offset_end;
	unsigned long *merkle_root;
	//struct merkleBtreeNode **merkle_root;
	int free_list_idx;
	int async_file_close;
	int anonymous_root_dirty_start;
	int anonymous_root_dirty_end;
	int num_alloc_anon_mmaps;
	unsigned long *root_dirty_cache;
	int root_dirty_num;
	int total_dirty_mmaps;
	unsigned int height;
	volatile int reference;
	int isRootSet;
	int isAnonMapReset;
	uint64_t anon_mem_used;
	volatile int usingAnonMem;
	int index_in_free_list;
	//	volatile int maxPerms;
	//	volatile int valid; // for debugging purposes
};

struct backupRoots {
	unsigned long *root;
	unsigned long *anonymous_root;
	unsigned long *anonymous_offset_start;
	unsigned long *anonymous_offset_end;
	unsigned long *merkle_root;
	unsigned long *root_dirty_cache;
};

struct free_anon_pool
{
	struct lfds711_queue_umm_element qe;
	unsigned long addr;
};

struct InodeToMapping
{
	ino_t serialno;
	unsigned long *root;
	unsigned long *merkle_root;
	//struct merkleBtreeNode **merkle_root;
	unsigned long *root_dirty_cache;
	int root_dirty_num;
	int total_dirty_mmaps;
	unsigned int height;
	char buffer[16];
};

/******************* Locking ********************/

#define BG_CLOSING 0
#define ASYNC_CLOSING async_close_enable
#define SEQ_LIST 0
#define RAND_LIST 1
#define DIRTY_TRACKING 1
#define PASS_THROUGH_CALLS 0
#define NON_TEMPORAL_WRITES 1
#define NUM_NODE_LISTS 1
#define PRINT_DEBUG_MSGS 0
#define PRINT_CONTENTION_MSGS 0
#define TOTAL_CLOSED_INODES 4096
#define ANON_MEM_HANDLE 1

volatile int async_close_enable;
unsigned int num_async_close;

#define GLOBAL_LOCKING 0
#if GLOBAL_LOCKING

#define GLOBAL_LOCK_WR()            {pthread_spin_lock(&global_lock);}
#define GLOBAL_UNLOCK_WR()          {pthread_spin_unlock(&global_lock);}

#else

#define GLOBAL_LOCK_WR()            {(void)(global_lock);}
#define GLOBAL_UNLOCK_WR()          {(void)(global_lock);}

#endif
#define GLOBAL_CLOSE_LOCKING 0
#if GLOBAL_CLOSE_LOCKING

#define GLOBAL_LOCK_CLOSE_WR()    {pthread_spin_lock(&global_lock_closed_files);}
#define GLOBAL_UNLOCK_CLOSE_WR()  {pthread_spin_unlock(&global_lock_closed_files);}

#else

#define GLOBAL_LOCK_CLOSE_WR()    {(void)(global_lock_closed_files);}
#define GLOBAL_UNLOCK_CLOSE_WR()  {(void)(global_lock_closed_files);}

#endif
#define LRU_HEAD_LOCKING 0
#if LRU_HEAD_LOCKING

#define LRU_LOCK_HEAD_WR()    {pthread_spin_lock(&global_lock_lru_head);}
#define LRU_UNLOCK_HEAD_WR()  {pthread_spin_unlock(&global_lock_lru_head);}

#else

#define LRU_LOCK_HEAD_WR()    {(void)(global_lock_lru_head);}
#define LRU_UNLOCK_HEAD_WR()  {(void)(global_lock_lru_head);}

#endif
#define LRU_NODE_LOCKING 1
#if LRU_NODE_LOCKING

#define LRU_NODE_LOCK_WR(cnode) NVP_LOCK_WR(cnode->lock)
#define LRU_NODE_UNLOCK_WR(cnode) NVP_LOCK_UNLOCK_WR(cnode->lock)

#else

#define LRU_NODE_LOCK_WR(cnode) {(void)(cnode->lock);}
#define LRU_NODE_UNLOCK_WR(cnode) {(void)(cnode->lock);}

#endif
#define HASH_TABLE_LOCKING 0
#if HASH_TABLE_LOCKING

#define NVP_LOCK_HASH_TABLE_RD(tbl, cpuid)   NVP_LOCK_RD(tbl->lock, cpuid)
#define NVP_UNLOCK_HASH_TABLE_RD(tbl, cpuid) NVP_LOCK_UNLOCK_RD(tbl->lock, cpuid)
#define NVP_LOCK_HASH_TABLE_WR(tbl)          NVP_LOCK_WR(tbl->lock)
#define NVP_UNLOCK_HASH_TABLE_WR(tbl)        NVP_LOCK_UNLOCK_WR(tbl->lock)

#else

#define NVP_LOCK_HASH_TABLE_RD(tbl, cpuid)   {(void)(cpuid);}
#define NVP_UNLOCK_HASH_TABLE_RD(tbl, cpuid) {(void)(cpuid);}
#define NVP_LOCK_HASH_TABLE_WR(tbl)          {(void)(tbl->lock);}
#define NVP_UNLOCK_HASH_TABLE_WR(tbl)        {(void)(tbl->lock);}

#endif
#define STACK_LOCKING 0
#if STACK_LOCKING

#define STACK_LOCK_WR()    {pthread_spin_lock(&stack_lock);}
#define STACK_UNLOCK_WR()  {pthread_spin_unlock(&stack_lock);}

#else

#define STACK_LOCK_WR()    {(void)(stack_lock);}
#define STACK_UNLOCK_WR()  {(void)(stack_lock);}

#endif
#define CLOSE_LOCKING 0
#if CLOSE_LOCKING

#define NVP_LOCK_CLOSE_RD(clf, cpuid)   NVP_LOCK_RD(clf->lock, cpuid)
#define NVP_UNLOCK_CLOSE_RD(clf, cpuid) NVP_LOCK_UNLOCK_RD(clf->lock, cpuid)
#define NVP_LOCK_CLOSE_WR(clf)          NVP_LOCK_WR(clf->lock)
#define NVP_UNLOCK_CLOSE_WR(clf)        NVP_LOCK_UNLOCK_WR(clf->lock)

#else

#define NVP_LOCK_CLOSE_RD(clf, cpuid)   {(void)(cpuid);}
#define NVP_UNLOCK_CLOSE_RD(clf, cpuid) {(void)(cpuid);}
#define NVP_LOCK_CLOSE_WR(clf)          {(void)(clf->lock);}
#define NVP_UNLOCK_CLOSE_WR(clf)        {(void)(clf->lock);}

#endif
#define FD_LOCKING 1
#if FD_LOCKING

#define NVP_LOCK_FD_RD(nvf, cpuid)	NVP_LOCK_RD(nvf->lock, cpuid)
#define NVP_UNLOCK_FD_RD(nvf, cpuid)	NVP_LOCK_UNLOCK_RD(nvf->lock, cpuid)
#define NVP_LOCK_FD_WR(nvf)		NVP_LOCK_WR(	   nvf->lock)
#define NVP_UNLOCK_FD_WR(nvf)		NVP_LOCK_UNLOCK_WR(nvf->lock)

#else

#define NVP_LOCK_FD_RD(nvf, cpuid) {(void)(cpuid);}
#define NVP_UNLOCK_FD_RD(nvf, cpuid) {(void)(cpuid);}
#define NVP_LOCK_FD_WR(nvf) {(void)(nvf->lock);}
#define NVP_UNLOCK_FD_WR(nvf) {(void)(nvf->lock);}

#endif
#define NODE_LOCKING 1
#if NODE_LOCKING	

#define NVP_LOCK_NODE_RD(nvf, cpuid)	NVP_LOCK_RD(nvf->node->lock, cpuid)
#define NVP_UNLOCK_NODE_RD(nvf, cpuid)	NVP_LOCK_UNLOCK_RD(nvf->node->lock, cpuid)
#define NVP_LOCK_NODE_WR(nvf)		NVP_LOCK_WR(	   nvf->node->lock)
#define NVP_UNLOCK_NODE_WR(nvf)		NVP_LOCK_UNLOCK_WR(nvf->node->lock)

#else

#define NVP_LOCK_NODE_RD(nvf, cpuid) {(void)(cpuid);}
#define NVP_UNLOCK_NODE_RD(nvf, cpuid) {(void)(cpuid);}
#define NVP_LOCK_NODE_WR(nvf) {(void)(nvf->node->lock);}
#define NVP_UNLOCK_NODE_WR(nvf)	{(void)(nvf->node->lock);}

#endif
/******************* MMAP ********************/

#define IS_ERR(x) ((unsigned long)(x) >= (unsigned long)-4095)

#define MAP_SIZE 16

#if MAP_SIZE == 512
#define MAX_MMAP_SIZE 536870912
#elif MAP_SIZE == 256
#define MAX_MMAP_SIZE 268435456
#elif MAP_SIZE == 128
#define MAX_MMAP_SIZE 134217728
#elif MAP_SIZE == 64
#define MAX_MMAP_SIZE 67108864
#elif MAP_SIZE == 32
#define MAX_MMAP_SIZE 33554432
#elif MAP_SIZE == 16
#define MAX_MMAP_SIZE 16777216
#elif MAP_SIZE == 8
#define MAX_MMAP_SIZE 8388608
#elif MAP_SIZE == 4
#define MAX_MMAP_SIZE 4194304
#elif MAP_SIZE == 2
#define MAX_MMAP_SIZE 2097152
#else
#define MAX_MMAP_SIZE 536870912
#endif

#define ANON_MAP_SIZE 16

#if ANON_MAP_SIZE == 512
#define ANON_MAX_MMAP_SIZE 536870912
#elif ANON_MAP_SIZE == 256
#define ANON_MAX_MMAP_SIZE 268435456
#elif ANON_MAP_SIZE == 128
#define ANON_MAX_MMAP_SIZE 134217728
#elif ANON_MAP_SIZE == 64
#define ANON_MAX_MMAP_SIZE 67108864
#elif ANON_MAP_SIZE == 32
#define ANON_MAX_MMAP_SIZE 33554432
#elif ANON_MAP_SIZE == 16
#define ANON_MAX_MMAP_SIZE 16777216
#elif ANON_MAP_SIZE == 8
#define ANON_MAX_MMAP_SIZE 8388608
#elif ANON_MAP_SIZE == 4
#define ANON_MAX_MMAP_SIZE 4194304
#elif ANON_MAP_SIZE == 2
#define ANON_MAX_MMAP_SIZE 2097152
#else
#define ANON_MAX_MMAP_SIZE 536870912
#endif


//#define MAX_MMAP_SIZE 2097152
//#define MAX_MMAP_SIZE 536870912

#define	ALIGN_MMAP_DOWN(addr)	((addr) & ~(MAX_MMAP_SIZE - 1))

#define DO_ALIGNMENT_CHECKS 0

#define DO_MSYNC(nvf) do{ \
	DEBUG("NOT doing a msync\n"); }while(0)
/*
	DEBUG("Doing a msync on fd %i (node %p)\n", nvf->fd, nvf->node); \
	if(msync(nvf->node->data, nvf->node->maplength, MS_SYNC|MS_INVALIDATE)) { \
		ERROR("Failed to msync for fd %i\n", nvf->fd); \
		assert(0); \
	} }while(0)
*/

/******************* Checking ********************/

#define NOSANITYCHECK 1
#if NOSANITYCHECK
	#define SANITYCHECK(x)
#else
	#define SANITYCHECK(x) if(UNLIKELY(!(x))) { ERROR("NVP_SANITY("#x") failed!\n"); exit(101); }
#endif

#define NVP_CHECK_NVF_VALID(nvf) do{ \
	if(UNLIKELY(!nvf->valid)) { \
		DEBUG("Invalid file descriptor: %i\n", file); \
		errno = 0;		      \
		return -1; \
	} \
	else \
	{ \
		DEBUG("this function is operating on node %p\n", nvf->node); \
		SANITYCHECKNVF(nvf); \
		DO_MSYNC(nvf); \
	} \
	} while(0)

#define NVP_CHECK_NVF_VALID_WR(nvf) do{ \
	if(UNLIKELY(!nvf->valid)) { \
		DEBUG("Invalid file descriptor: %i\n", file); \
		errno = 0;	       \
		return -1; \
	} \
	else \
	{ \
		DEBUG("this function is operating on node %p\n", nvf->node); \
		SANITYCHECKNVF(nvf);					\
		DO_MSYNC(nvf);						\
	} \
	} while(0)

#define SANITYCHECKNVF(nvf) \
		SANITYCHECK(nvf->valid); \
		SANITYCHECK(nvf->node != NULL); \
		SANITYCHECK(nvf->fd >= 0); \
		SANITYCHECK(nvf->fd < OPEN_MAX); \
		SANITYCHECK(nvf->offset != NULL); \
		SANITYCHECK(*nvf->offset >= 0); \
		SANITYCHECK(nvf->node->length >=0); \
		SANITYCHECK(nvf->node->maplength >= nvf->node->length); \
		SANITYCHECK(nvf->node->data != NULL)

/*
#define SANITYCHECKNVF(nvf) \
		SANITYCHECK(nvf->valid); \
		SANITYCHECK(nvf->node != NULL); \
		SANITYCHECK(nvf->fd >= 0); \
		SANITYCHECK(nvf->fd < OPEN_MAX); \
		SANITYCHECK(nvf->offset != NULL); \
		SANITYCHECK(*nvf->offset >= 0); \
		SANITYCHECK(nvf->serialno != 0); \
		SANITYCHECK(nvf->serialno == nvf->node->serialno); \
		SANITYCHECK(nvf->node->length >=0); \
		SANITYCHECK(nvf->node->maplength >= nvf->node->length); \
		SANITYCHECK(nvf->node->data != NULL)
*/

#endif
