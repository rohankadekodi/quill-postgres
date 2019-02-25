// a module which repalces the standart POSIX functions with memory mapped equivalents

#include "nv_common.h"
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <limits.h>
#include <pthread.h>
#include <signal.h>
#include <linux/kernel.h>
#include <sys/syscall.h>
#include <unistd.h>
#include "perfcount.h"

#include "nvp_mman.h"
//#include "nvp_lock.h"
#include "fileops_nvp.h"
#include "merkleLogicalBtree.h"
#include "thread_handle.h"
#include "stack.h"
#include "lru_cache.h"
#include "timers.h"

#if NON_TEMPORAL_WRITES
#include "non_temporal.h"
#endif

BOOST_PP_SEQ_FOR_EACH(DECLARE_WITHOUT_ALIAS_FUNCTS_IWRAP, _nvp_, ALLOPS_WPAREN)
BOOST_PP_SEQ_FOR_EACH(DECLARE_WITHOUT_ALIAS_FUNCTS_IWRAP, _nvp_, SHM_WPAREN)

int MMAP_PAGE_SIZE;

void* _nvp_zbuf; // holds all zeroes.  used for aligned file extending. TODO: does sharing this hurt performance?

pthread_spinlock_t	node_lookup_lock[NUM_NODE_LISTS];

struct NVFile* _nvp_fd_lookup;
int execve_fd_passing[1024];

//struct free_anon_pool *free_pool_of_anonymous_mmaps, *temp_free_pool_of_anonymous_mmaps;
struct lfds711_queue_umm_element *qe, qe_dummy;
struct lfds711_queue_umm_state qs;

//unsigned long* free_pool_of_anonymous_mmaps;
//int num_free_anonymous_mmaps;
int _nvp_free_list_head;

int _nvp_ino_lookup[1024];
struct InodeToMapping* _nvp_ino_mapping;

void _nvp_init2(void);

MODULE_REGISTRATION_F("nvp", _nvp_, _nvp_init2(); );

#define NVP_WRAP_HAS_FD(op) \
	RETT_##op _nvp_##op ( INTF_##op ) {				\
		CHECK_RESOLVE_FILEOPS(_nvp_);				\
		DEBUG("_nvp_"#op" is just wrapping %s->"#op"\n", _nvp_fileops->name); \
		if(UNLIKELY(file>=OPEN_MAX)) { DEBUG("file descriptor too large (%i > %i)\n", file, OPEN_MAX-1); errno = EBADF; return (RETT_##op) -1; } \
		if(UNLIKELY(file<0)) { DEBUG("file < 0 (file = %i).  return -1;\n", file); errno = EBADF; return (RETT_##op) -1; } \
		if(UNLIKELY(!_nvp_fd_lookup[file].valid)) { DEBUG("That file descriptor (%i) is invalid\n", file); errno = EBADF; return -1; } \
		DEBUG("_nvp_" #op " is calling %s->" #op "\n", _nvp_fileops->name); \
		return (RETT_##op) _nvp_fileops->op( CALL_##op );	\
	}

#define NVP_WRAP_NO_FD(op)						\
	RETT_##op _nvp_##op ( INTF_##op ) {				\
		CHECK_RESOLVE_FILEOPS(_nvp_);				\
		DEBUG("_nvp_"#op" is just wrapping %s->"#op"\n", _nvp_fileops->name); \
		return _nvp_fileops->op( CALL_##op );			\
	}

#define NVP_WRAP_HAS_FD_IWRAP(r, data, elem) NVP_WRAP_HAS_FD(elem)
#define NVP_WRAP_NO_FD_IWRAP(r, data, elem) NVP_WRAP_NO_FD(elem)

BOOST_PP_SEQ_FOR_EACH(NVP_WRAP_HAS_FD_IWRAP, placeholder, (ACCEPT))
BOOST_PP_SEQ_FOR_EACH(NVP_WRAP_NO_FD_IWRAP, placeholder, (PIPE) (FORK) (SOCKET))


/* ============================= memcpy =============================== */

extern long copy_user_nocache(void *dst, const void *src, unsigned size, int zerorest);

static inline int copy_from_user_inatomic_nocache(void *dst, const void *src, unsigned size) {
	return copy_user_nocache(dst, src, size, 0);
}

static inline void* my_memcpy_nocache(void* dst, const void* src, unsigned size) {
	if(copy_from_user_inatomic_nocache(dst, src, size)) {
		return dst;
	} else { 
		return 0;
	}
}

static inline void *intel_memcpy(void * __restrict__ b, const void * __restrict__ a, size_t n){
	char *s1 = b;
	const char *s2 = a;
	for(; 0<n; --n)*s1++ = *s2++;
	return b;
}

void *(*import_memcpy)(void * __restrict__ b, const void * __restrict__ a, size_t n);

extern void * __memcpy(void * __restrict__ to, const void * __restrict__ from, size_t len);

#define MMX2_MEMCPY_MIN_LEN 0x40
#define MMX_MMREG_SIZE 8

// ftp://ftp.work.acer-euro.com/gpl/AS1800/xine-lib/src/xine-utils/memcpy.c
static void * mmx2_memcpy(void * __restrict__ to, const void * __restrict__ from, size_t len)
{
  void *retval;
  //retval = to;

  /* PREFETCH has effect even for MOVSB instruction ;) */
  /*__asm__ __volatile__ (
    "   prefetchnta (%0)\n"
    "   prefetchnta 32(%0)\n"
    "   prefetchnta 64(%0)\n"
    "   prefetchnta 96(%0)\n"
    "   prefetchnta 128(%0)\n"
    "   prefetchnta 160(%0)\n"
    "   prefetchnta 192(%0)\n"
    "   prefetchnta 224(%0)\n"
    "   prefetchnta 256(%0)\n"
    "   prefetchnta 288(%0)\n"
    : : "r" (from) );

  if(len >= MMX2_MEMCPY_MIN_LEN)
  {
    register unsigned long int delta;
  */ /* Align destinition to MMREG_SIZE -boundary */
  /*  delta = ((unsigned long int)to)&(MMX_MMREG_SIZE-1);
    if(delta)
    {
      delta=MMX_MMREG_SIZE-delta;
      len -= delta;
      memcpy(to, from, delta);
    }
    i = len >> 6; */ /* len/64 */
  /*len&=63;
    for(; i>0; i--)
    {
      __asm__ __volatile__ (
      "prefetchnta 320(%0)\n"
      "prefetchnta 352(%0)\n"
      "movq (%0), %%mm0\n"
      "movq 8(%0), %%mm1\n"
      "movq 16(%0), %%mm2\n"
      "movq 24(%0), %%mm3\n"
      "movq 32(%0), %%mm4\n"
      "movq 40(%0), %%mm5\n"
      "movq 48(%0), %%mm6\n"
      "movq 56(%0), %%mm7\n"
      "movntq %%mm0, (%1)\n"
      "movntq %%mm1, 8(%1)\n"
      "movntq %%mm2, 16(%1)\n"
      "movntq %%mm3, 24(%1)\n"
      "movntq %%mm4, 32(%1)\n"
      "movntq %%mm5, 40(%1)\n"
      "movntq %%mm6, 48(%1)\n"
      "movntq %%mm7, 56(%1)\n"
      :: "r" (from), "r" (to) : "memory");
      //((const unsigned char *)from)+=64;
      //((unsigned char *)to)+=64;
      from = (void*)(((const unsigned char *)from) + 64);
      to = (void*)(((unsigned char *)to) + 64);
      }*/
     /* since movntq is weakly-ordered, a "sfence"
     * is needed to become ordered again. */
  /*__asm__ __volatile__ ("sfence":::"memory");
    __asm__ __volatile__ ("emms":::"memory");
    }*/
  /*
   *	Now do the tail of the block
   */

  if(len)
	  retval = memcpy(to, from, len);
  else
	  retval = to;
 
  return retval;
}

/*
static char* memcpy1(char *to, char *from, size_t n)
{
	long esi, edi;
	int ecx;
	esi = (long)from;
	edi = (long)to;
	asm volatile("rep ; movsl"
		: "=&c" (ecx), "=&D" (edi), "=&S" (esi)
		: "0" (n / 4), "1" (edi), "2" (esi)
		: "memory"
		);
	return to;
}
*/
/* ============================= IO stats =============================== */

unsigned int num_open;
unsigned int num_close;
unsigned int num_read;
unsigned int num_write;
unsigned int num_stat;
unsigned int num_unlink;
unsigned int num_mmap;
unsigned int num_appendfsync;
unsigned int num_memcpy_read;
unsigned int num_memcpy_write;
unsigned int num_posix_read;
unsigned int num_posix_write;
unsigned int num_fsync;
unsigned int num_mfence;
unsigned int num_write_nontemporal;
unsigned int num_write_temporal;
unsigned int num_clflushopt;
unsigned long long appendfsync_size;
unsigned long long non_temporal_write_size;
unsigned long long temporal_write_size;
unsigned long long read_size;
unsigned long long write_size;
unsigned long long memcpy_read_size;
unsigned long long memcpy_write_size;
unsigned long long posix_read_size;
unsigned long long posix_write_size;
unsigned long long total_syscalls;
volatile size_t _nvp_wr_extended;
volatile size_t _nvp_wr_total;

void nvp_print_io_stats(void)
{
	printf("====================== NVP IO stats: ======================\n");
	printf("open %u, close %u, async close %u\n", num_open, num_close, num_async_close);
	printf("mmap %u, unlink %u, stat %u\n", num_mmap, num_unlink, num_stat);
	printf("fsync %u, appendfsync: count %u size %llu average %llu\n", num_fsync, num_appendfsync, appendfsync_size, num_appendfsync ? appendfsync_size / num_appendfsync : 0);
	printf("READ: count %u, size %llu, average %llu\n", num_read,
		read_size, num_read ? read_size / num_read : 0);
	printf("WRITE: count %u, size %llu, average %llu\n", num_write,
		write_size, num_write ? write_size / num_write : 0);
	printf("memcpy READ: count %u, size %llu, average %llu\n",
		num_memcpy_read, memcpy_read_size,
		num_memcpy_read ? memcpy_read_size / num_memcpy_read : 0);
	printf("memcpy WRITE: count %u, size %llu, average %llu\n",
		num_memcpy_write, memcpy_write_size,
		num_memcpy_write ? memcpy_write_size / num_memcpy_write : 0);
	printf("posix READ: count %u, size %llu, average %llu\n",
		num_posix_read, posix_read_size,
		num_posix_read ? posix_read_size / num_posix_read : 0);
	printf("posix WRITE: count %u, size %llu, average %llu\n",
		num_posix_write, posix_write_size,
		num_posix_write ? posix_write_size / num_posix_write : 0);
	printf("write extends %lu, total %lu\n", _nvp_wr_extended,
		_nvp_wr_total);
	printf("MFENCE: count %u\n",
	       num_mfence);
	printf("CLFLUSHOPT: count %u\n",
	       num_clflushopt);
	printf("NON_TEMPORAL_WRITES: count %u, size %llu, average %llu\n",
	       num_write_nontemporal, non_temporal_write_size,
	       num_write_nontemporal ? non_temporal_write_size / num_write_nontemporal : 0);
	printf("TEMPORAL WRITES: count %u, size %llu, average %llu\n",
	       num_write_temporal, temporal_write_size,
	       num_write_temporal ? temporal_write_size / num_write_temporal : 0);
	printf("TOTAL SYSCALLS (open + close + read + write + fsync): count %u\n",
	       num_open + num_close + num_posix_read + num_posix_write);
}

void msg_nvp_print_io_stats(void)
{
	MSG("====================== NVP IO stats: ======================\n");
	MSG("open %u, close %u, async close %u\n", num_open, num_close, num_async_close);
	MSG("mmap %u, unlink %u, stat %u\n", num_mmap, num_unlink, num_stat);
	MSG("fsync %u, appendfsync: count %u size %llu average %llu\n", num_fsync, num_appendfsync, appendfsync_size, num_appendfsync ? appendfsync_size / num_appendfsync : 0);
	MSG("READ: count %u, size %llu, average %llu\n", num_read,
		read_size, num_read ? read_size / num_read : 0);
	MSG("WRITE: count %u, size %llu, average %llu\n", num_write,
		write_size, num_write ? write_size / num_write : 0);
	MSG("memcpy READ: count %u, size %llu, average %llu\n",
		num_memcpy_read, memcpy_read_size,
		num_memcpy_read ? memcpy_read_size / num_memcpy_read : 0);
	MSG("memcpy WRITE: count %u, size %llu, average %llu\n",
		num_memcpy_write, memcpy_write_size,
		num_memcpy_write ? memcpy_write_size / num_memcpy_write : 0);
	MSG("posix READ: count %u, size %llu, average %llu\n",
		num_posix_read, posix_read_size,
		num_posix_read ? posix_read_size / num_posix_read : 0);
	MSG("posix WRITE: count %u, size %llu, average %llu\n",
		num_posix_write, posix_write_size,
		num_posix_write ? posix_write_size / num_posix_write : 0);
	MSG("write extends %lu, total %lu\n", _nvp_wr_extended,
		_nvp_wr_total);
	MSG("MFENCE: count %u\n",
	       num_mfence);
	MSG("CLFLUSHOPT: count %u\n",
	       num_clflushopt);
	MSG("NON_TEMPORAL_WRITES: count %u, size %llu, average %llu\n",
	       num_write_nontemporal, non_temporal_write_size,
	       num_write_nontemporal ? non_temporal_write_size / num_write_nontemporal : 0);
	MSG("TEMPORAL WRITES: count %u, size %llu, average %llu\n",
	       num_write_temporal, temporal_write_size,
	       num_write_temporal ? temporal_write_size / num_write_temporal : 0);
	MSG("TOTAL SYSCALLS (open + close + read + write + fsync): count %u\n",
	       num_open + num_close + num_posix_read + num_posix_write);
}


void result_nvp_print_io_stats(void)
{
	RESULT_FILE("====================== NVP IO stats: ======================\n");
	RESULT_FILE("open %u, close %u, async close %u\n", num_open, num_close, num_async_close);
	RESULT_FILE("mmap %u, unlink %u, stat %u\n", num_mmap, num_unlink, num_stat);
	RESULT_FILE("fsync %u, appendfsync: count %u size %llu average %llu\n", num_fsync, num_appendfsync, appendfsync_size, num_appendfsync ? appendfsync_size / num_appendfsync : 0);
	RESULT_FILE("READ: count %u, size %llu, average %llu\n", num_read,
		read_size, num_read ? read_size / num_read : 0);
	RESULT_FILE("WRITE: count %u, size %llu, average %llu\n", num_write,
		write_size, num_write ? write_size / num_write : 0);
	RESULT_FILE("memcpy READ: count %u, size %llu, average %llu\n",
		num_memcpy_read, memcpy_read_size,
		num_memcpy_read ? memcpy_read_size / num_memcpy_read : 0);
	RESULT_FILE("memcpy WRITE: count %u, size %llu, average %llu\n",
		num_memcpy_write, memcpy_write_size,
		num_memcpy_write ? memcpy_write_size / num_memcpy_write : 0);
	RESULT_FILE("posix READ: count %u, size %llu, average %llu\n",
		num_posix_read, posix_read_size,
		num_posix_read ? posix_read_size / num_posix_read : 0);
	RESULT_FILE("posix WRITE: count %u, size %llu, average %llu\n",
		num_posix_write, posix_write_size,
		num_posix_write ? posix_write_size / num_posix_write : 0);
	RESULT_FILE("write extends %lu, total %lu\n", _nvp_wr_extended,
		_nvp_wr_total);
	RESULT_FILE("MFENCE: count %u\n",
	       num_mfence);
	RESULT_FILE("CLFLUSHOPT: count %u\n",
	       num_clflushopt);
	RESULT_FILE("NON_TEMPORAL_WRITES: count %u, size %llu, average %llu\n",
	       num_write_nontemporal, non_temporal_write_size,
	       num_write_nontemporal ? non_temporal_write_size / num_write_nontemporal : 0);
	RESULT_FILE("TEMPORAL WRITES: count %u, size %llu, average %llu\n",
	       num_write_temporal, temporal_write_size,
	       num_write_temporal ? temporal_write_size / num_write_temporal : 0);
	RESULT_FILE("TOTAL SYSCALLS (open + close + read + write + fsync): count %u\n",
	       num_open + num_close + num_posix_read + num_posix_write);
}





/* ====================== Memory operation policy ======================= */

// modifications to support different FSYNC policies
//#define MEMCPY memcpy
//define MEMCPY intel_memcpy
//#define MEMCPY (void*)copy_from_user_inatomic_nocache
//#define MEMCPY my_memcpy_nocache
#define MEMCPY mmx2_memcpy
//#define MEMCPY memcpy1
#define MMAP mmap
//#define FSYNC fsync

#define FSYNC_POLICY_NONE 0
#define FSYNC_POLICY_FLUSH_ON_FSYNC 1
#define FSYNC_POLICY_UNCACHEABLE_MAP 2
#define FSYNC_POLICY_NONTEMPORAL_WRITES 3
#define FSYNC_POLICY_FLUSH_ON_WRITE 4

#define FSYNC_POLICY FSYNC_POLICY_FLUSH_ON_FSYNC

#if FSYNC_POLICY == FSYNC_POLICY_NONE
	#define FSYNC_MEMCPY MEMCPY
	#define FSYNC_MMAP MMAP
	#define FSYNC_FSYNC 
#elif FSYNC_POLICY == FSYNC_POLICY_FLUSH_ON_FSYNC
	#define FSYNC_MEMCPY MEMCPY
	#define FSYNC_MMAP MMAP
        #define FSYNC_FSYNC(nvf,cpuid,close,fdsync) fsync_flush_on_fsync(nvf,cpuid,close,fdsync)
#elif FSYNC_POLICY == FSYNC_POLICY_UNCACHEABLE_MAP
	#define FSYNC_MEMCPY MEMCPY
	#define FSYNC_MMAP mmap_fsync_uncacheable_map
	#define FSYNC_FSYNC 
#elif FSYNC_POLICY == FSYNC_POLICY_NONTEMPORAL_WRITES
	#define FSYNC_MEMCPY memcpy_fsync_nontemporal_writes
	#define FSYNC_MMAP MMAP
	#define FSYNC_FSYNC _mm_mfence()
#elif FSYNC_POLICY == FSYNC_POLICY_FLUSH_ON_WRITE
	#define FSYNC_MEMCPY memcpy_fsync_flush_on_write
	#define FSYNC_MMAP MMAP
	#define FSYNC_FSYNC _mm_mfence()
#endif


/* ============================= Fsync =============================== */

static unsigned long calculate_capacity(unsigned int height);
static unsigned long calculate_anon_capacity(unsigned int height);

static inline void copy_appends_to_file(struct NVFile* nvf, int close, int fdsync)
{
	unsigned long capacity = ANON_MAX_MMAP_SIZE;
	unsigned long *root = nvf->node->anonymous_root;
	unsigned long start_addr;
	off_t start_offset, curr_offset;
	int index;
	unsigned long anonymous_offset_start, anonymous_offset_end;
	size_t len_to_write;
	size_t posix_write;

	capacity = calculate_anon_capacity(0);

	_mm_mfence();
	
	start_offset = nvf->node->anonymous_root_dirty_start * ANON_MAX_MMAP_SIZE;	
	
	while (1) {		
		curr_offset = start_offset;
		index = curr_offset / capacity;
		DEBUG("index %d\n", index);
		if (index > nvf->node->anonymous_root_dirty_end)
			break;
		/* Rohan, traversing tree to find dirty data */

		if(root[index] != 0) {
			start_addr = root[index];
			anonymous_offset_start = nvf->node->anonymous_offset_start[index];
			anonymous_offset_end = nvf->node->anonymous_offset_end[index];
			len_to_write = anonymous_offset_end - anonymous_offset_start + 1;
			if(!close) {

				num_mfence++;
				num_write_nontemporal++;
				
				if(fdsync) {
					posix_write = _nvp_fileops->PWRITE(nvf->fd, (const void *)start_addr + anonymous_offset_start, len_to_write, nvf->node->true_length + start_offset + anonymous_offset_start);
				} else {
					posix_write = syscall(333, nvf->fd, start_addr + anonymous_offset_start, len_to_write, nvf->node->true_length + start_offset + anonymous_offset_start);
				}
				num_mfence++;
				non_temporal_write_size += posix_write;
				
			} else {
				if(nvf->node->reference != 1)
					goto out;

				num_mfence++;
				num_write_nontemporal++;
				
				posix_write = _nvp_fileops->PWRITE(nvf->fd, (const void *)start_addr + anonymous_offset_start, len_to_write, nvf->node->true_length + start_offset + anonymous_offset_start);

				num_mfence++;
				non_temporal_write_size += posix_write;
			}
				
			num_appendfsync++;
			appendfsync_size += len_to_write;
			
			/* printf("%s: nvf->node->true_length = %lu, nvf->node->length = %lu, offset in file = %lu, start_offset = %lu, index = %d, size of write = %lu, fd = %d, inode = %lu\n", __func__, nvf->node->true_length, nvf->node->length, nvf->node->true_length + start_offset + anonymous_offset_start, start_offset, index, len_to_write, nvf->fd, nvf->node->serialno); */
		       						
			if(posix_write != len_to_write) {
				ERROR("%s: write failed! Reason = %s\n", __func__, strerror(errno));			
				ERROR("%s: nvf->node->true_length = %lu, nvf->node->length = %lu, offset in file = %lu, start_offset = %lu, index = %d, size of write = %lu, fd = %d, inode = %lu, return value = %lu\n", __func__, nvf->node->true_length, nvf->node->length, nvf->node->true_length + start_offset + anonymous_offset_start, start_offset, index, len_to_write, nvf->fd, nvf->node->serialno, posix_write);
				fflush(NULL);
				
				assert(0);				     
			}								
			nvf->node->isAnonMapReset = 0;
		}

		curr_offset = curr_offset % capacity;
		start_offset += ANON_MAX_MMAP_SIZE;
	}

	nvf->node->true_length = nvf->node->length;
	if(close)
		nvf->node->true_length_for_read = nvf->node->length;
	
 out:
	return;
}

/* FIXME: untested */
static inline void fsync_flush_on_fsync(struct NVFile* nvf, int cpuid, int close, int fdsync)
{
	instrumentation_type nvnode_lock_time;

#if NON_TEMPORAL_WRITES
	goto out;
#else //NON_TEMPORAL_WRITES
	unsigned int height = nvf->node->height;
	unsigned long capacity = MAX_MMAP_SIZE;
	unsigned long *root = nvf->node->root;
	unsigned long *merkle_root = nvf->node->merkle_root;
	uint64_t num_cachelines = 0;
	off_t start_offset = 0, curr_offset;
	int index;
#if DIRTY_TRACKING
	unsigned long start_addr;
#endif // DIRTY_TRACKING
	if(close)
		goto out;

	capacity = calculate_capacity(height);
       
	_mm_mfence();
	num_mfence++;
	
	while (1) {
		height = nvf->node->height;
		curr_offset = start_offset;
		do {
			index = curr_offset / capacity;
			DEBUG("index %d\n", index);
			if (index >= 1024 || root[index] == 0 || merkle_root[index] == 0) {
				goto out;
			}
			if (height) {
				root = (unsigned long *)root[index];
				merkle_root = (unsigned long *)merkle_root[index];
				DEBUG("%p\n", root);
			} else {
				/* Rohan, traversing tree to find dirty data */
#if DIRTY_TRACKING			
				start_addr = root[index];
				num_cachelines = traverseTree((struct merkleBtreeNode *)merkle_root[index], start_addr);
				num_clflushopt += num_cachelines;
				
#endif
			}
			curr_offset = curr_offset % capacity;
		} while(height--);
		start_offset += MAX_MMAP_SIZE;
	}

	_mm_mfence();
	num_mfence++;
	
#endif //NON_TEMPORAL_WRITES
 out:
	if(nvf->node->true_length != nvf->node->length) {
#if ANON_MEM_HANDLE
		DEBUG_FILE("%s: File desc = %d, true length = %lu, fake length = %lu\n", __func__, nvf->fd, nvf->node->true_length, nvf->node->length);
		START_TIMING(nvnode_lock_t, nvnode_lock_time);
		NVP_LOCK_NODE_WR(nvf);		
		END_TIMING(nvnode_lock_t, nvnode_lock_time);
		copy_appends_to_file(nvf, close, fdsync);
		NVP_UNLOCK_NODE_WR(nvf);
#else
		_nvp_fileops->FSYNC(nvf->fd);
#endif
	}
}

void *mmap_fsync_uncacheable_map(void *start, size_t length, int prot, int flags, int fd, off_t offset)
{
	void* result = MMAP( start, length, prot, flags, fd, offset );
	// mark the result as uncacheable // TODO
	// not_implemented++;

	return result;
}

void *memcpy_fsync_nontemporal_writes(void *dest, const void *src, size_t n)
{
	// TODO: an asm version of memcpy, with movdqa replaced by movntdq
//	void* result; = FSYNC_MEMCPY( not_implemented );

	_mm_mfence();

	return NULL;
//	return result;
}

void *memcpy_fsync_flush_on_write(void *dest, const void *src, size_t n)
{
	// first, perform the memcpy as usual
	void* result = MEMCPY(dest, src, n);

	// then, flush all the pages which were just modified.
	_mm_mfence();
	do_cflush_len(dest, n);
	_mm_mfence();

	return result;
}


/* ============================= Timing =============================== */

/*
enum timing_category {
	do_pread_t = 0,
	do_pwrite_t,
	memcpyr_t,
	memcpyw_t,
	lookup_t,
	insert_t,
	read_t,
	write_t,
	pread_t,
	pwrite_t,
	open_t,
	close_t,
	posix_open_t,
	posix_close_t,
	get_node_t,
	alloc_node_t,
	fsync_t,
	fdsync_t,
	mmap_t,
	get_mmap_t,
	TIMING_NUM,	// Keep as last entry
};

unsigned long long Countstats[TIMING_NUM];
unsigned long long Timingstats[TIMING_NUM];
const char *Timingstring[TIMING_NUM] = 
{
	"do_pread",
	"do_pwrite",
	"memcpy_read",
	"memcpy_write",
	"Tree_lookup",
	"Tree_insert",
	"READ",
	"WRITE",
	"PREAD",
	"PWRITE",
	"OPEN",
	"CLOSE",
	"Posix OPEN",
	"Posix CLOSE",
	"get_node",
	"alloc_node",
	"Fsync",
	"Fdsync",
	"mmap",
	"get_mmap_addr",
};

typedef struct timespec timing_type;

#if MEASURE_TIMING

#define	NVP_START_TIMING(name, start) \
	clock_gettime(CLOCK_MONOTONIC, &start)

#define	NVP_END_TIMING(name, start) \
	{timing_type end; \
	 clock_gettime(CLOCK_MONOTONIC, &end); \
	 Countstats[name]++; \
	 Timingstats[name] += (end.tv_sec - start.tv_sec) * 1e9 \
				+ (end.tv_nsec - start.tv_nsec); \
	}

void nvp_print_time_stats(void)
{
	int i;

	printf("==================== NVP timing stats: ====================\n");
	for (i = 0; i < TIMING_NUM; i++)
		printf("%s: count %llu, timing %llu, average %llu\n",
			Timingstring[i], Countstats[i], Timingstats[i],
			Countstats[i] ? Timingstats[i] / Countstats[i] : 0);

}

#else

#define NVP_START_TIMING(name, start) {(void)(start);}

#define  NVP_END_TIMING(name, start) \
	{Countstats[name]++;}

void nvp_print_time_stats(void)
{
	int i;

	printf("==================== NVP timing stats: ====================\n");
	for (i = 0; i < TIMING_NUM; i++)
		printf("%s: count %llu\n", Timingstring[i], Countstats[i]);
}

#endif
*/

/* ========================== Internal methods =========================== */

//void nvp_free_btree(unsigned long *root, struct merkleBtreeNode **merkle_root, unsigned long height, unsigned long *dirty_cache, int root_dirty_num, int total_dirty_mmaps);

void nvp_free_btree(unsigned long *root, unsigned long *merkle_root, unsigned long height, unsigned long *dirty_cache, int root_dirty_num, int total_dirty_mmaps);
void nvp_free_anonymous_mmaps();
void nvp_cleanup_node(struct NVNode *node, int free_root, int unmap_btree);

void nvp_cleanup(void)
{
	int i, j;

	//printf("starting cleanup..\n");
	//fflush(NULL);
	
#if BG_CLOSING
	while(!waiting_for_signal)
		sleep(1);
	
	//cancel thread
	cancelBgThread();	
	exit_bgthread = 1;
	cleanup = 1;
	bgCloseFiles(1);
#endif	
	nvp_free_anonymous_mmaps();

	free(_nvp_fd_lookup);

	for (i = 0; i < NUM_NODE_LISTS; i++) {
		pthread_spin_lock(&node_lookup_lock[i]);
       
		for (j = 0; j< OPEN_MAX; j++) {		
			nvp_cleanup_node(&_nvp_node_lookup[i][j], 1, 1); 
		}
	
		pthread_spin_unlock(&node_lookup_lock[i]);
	
		free(_nvp_node_lookup[i]);
	}

	//printf("%s: cleaning ino mapping\n", __func__);
	//fflush(NULL);
	
	for (i = 0; i < OPEN_MAX; i++) {
			nvp_free_btree(_nvp_ino_mapping[i].root, _nvp_ino_mapping[i].merkle_root, _nvp_ino_mapping[i].height, _nvp_ino_mapping[i].root_dirty_cache, _nvp_ino_mapping[i].root_dirty_num, _nvp_ino_mapping[i].total_dirty_mmaps);
	}
	
	free(_nvp_ino_mapping);
	//printf("cleanup complete..\n");
}

void nvp_exit_handler(void)
{
	MSG("exit handler\n");
	//MSG("Exit: print stats\n");
	//nvp_print_time_stats();
	//msg_nvp_print_io_stats();
	result_nvp_print_io_stats();
	PRINT_TIME();
	
	//MSG("calling cleanup\n");
	nvp_cleanup();
}

void _nvp_SIGUSR1_handler(int sig)
{
	MSG("SIGUSR1: print stats\n");
	//nvp_print_time_stats();
	//msg_nvp_print_io_stats();
        result_nvp_print_io_stats();
	//printf("%s: BG THREAD CALLED %d TIMES\n", __func__, calledBgThread);

	PRINT_TIME();
}

void _nvp_SIGBUS_handler(int sig)
{
	ERROR("We got a SIGBUS (sig %i)! "
		"This almost certainly means someone tried to access an area "
		"inside an mmaped region but past the length of the mmapped "
		"file.\n", sig);
	printf("%s: sigbus got\n", __func__);
	fflush(NULL);
	
	assert(0);
}

void _nvp_SHM_COPY() {

        int exec_ledger_fd = -1;
	int i,j;
	unsigned long offset_in_map = 0;
	
	exec_ledger_fd = shm_open("exec-ledger", O_RDONLY, 0666);

	if (exec_ledger_fd == -1) {
		printf("%s: shm_open failed. Err = %s\n", __func__, strerror(errno));
		assert(0);
	}
	
	char *shm_area = mmap(NULL, 10*1024*1024, PROT_READ, MAP_SHARED, exec_ledger_fd, 0);
	if (shm_area == NULL) {
		printf("%s: mmap failed. Err = %s\n", __func__, strerror(errno));
		assert(0);
	}

	if (memcpy(_nvp_fd_lookup, shm_area + offset_in_map, 1024 * sizeof(struct NVFile)) == NULL) {
		printf("%s: memcpy of fd lookup failed. Err = %s\n", __func__, strerror(errno));		
		assert(0);
	}
	
	offset_in_map += (1024 * sizeof(struct NVFile));

	if (memcpy(execve_fd_passing, shm_area + offset_in_map, 1024 * sizeof(int)) == NULL) {
		printf("%s: memcpy of offset passing failed. Err = %s\n", __func__, strerror(errno));
	}

	offset_in_map += (1024 * sizeof(int));

	for (i = 0; i < 1024; i++) {
		_nvp_fd_lookup[i].offset = (size_t*)calloc(1, sizeof(int));
		*(_nvp_fd_lookup[i].offset) = execve_fd_passing[i];
	}

	if (memcpy(_nvp_node_lookup[0], shm_area + offset_in_map, 1024*sizeof(struct NVNode)) == NULL) {
		printf("%s: memcpy of node lookup failed. Err = %s\n", __func__, strerror(errno));
		assert(0);
	}

	for (i = 0; i < 1024; i++) {
		_nvp_fd_lookup[i].node = NULL;
		_nvp_node_lookup[0][i].root_dirty_num = 0;
		_nvp_node_lookup[0][i].total_dirty_mmaps = 0;
		_nvp_node_lookup[0][i].usingAnonMem = 0;
		_nvp_node_lookup[0][i].isRootSet = 0;
		_nvp_node_lookup[0][i].isAnonMapReset = 1;
		_nvp_node_lookup[0][i].anon_mem_used = 0;
		_nvp_node_lookup[0][i].height = 0;
		_nvp_node_lookup[0][i].root_dirty_num = 0;
		
		_nvp_node_lookup[0][i].root = _nvp_backup_roots[0][i].root;
		_nvp_node_lookup[0][i].anonymous_root = _nvp_backup_roots[0][i].anonymous_root;
		_nvp_node_lookup[0][i].anonymous_offset_start = _nvp_backup_roots[0][i].anonymous_offset_start;
		_nvp_node_lookup[0][i].anonymous_offset_end = _nvp_backup_roots[0][i].anonymous_offset_end;
		_nvp_node_lookup[0][i].merkle_root = _nvp_backup_roots[0][i].merkle_root;
	}

	offset_in_map += (1024*sizeof(struct NVNode));
	
	for (i = 0; i < 1024; i++) {
		if (_nvp_fd_lookup[i].fd != -1) {
			for (j = 0; j < 1024; j++) {
				if (_nvp_fd_lookup[i].serialno == _nvp_node_lookup[0][j].serialno) {
					if (_nvp_fd_lookup[i].fd == 1)
						MSG("%s: here\n", __func__);
					_nvp_fd_lookup[i].node = &_nvp_node_lookup[0][j];
					break;
				}			
			}
		}
	}
	
	if (memcpy(_nvp_ino_lookup, shm_area + offset_in_map, 1024 * sizeof(int)) == NULL) {
		printf("%s: memcpy of ino lookup failed. Err = %s\n", __func__, strerror(errno));
		assert(0);
	}

	offset_in_map += (1024 * sizeof(int));

	if (memcpy(_nvp_free_node_list[0], shm_area + offset_in_map, 1024*sizeof(struct StackNode)) == NULL) {
		printf("%s: memcpy of free node list failed. Err = %s\n", __func__, strerror(errno));
		assert(0);
	}

	munmap(shm_area, 10*1024*1024);
	shm_unlink("exec-ledger");
}

void _nvp_init2(void)
{
	int i, j;
	struct InodeToMapping *tempMapping;

	assert(!posix_memalign(((void**)&_nvp_zbuf), 4096, 4096));
	

	ASYNC_CLOSING = 0;

	/* 
	   Allocating and Initializing NVFiles. Total number of NVFiles = 1024. _nvp_fd_lookup is an array of struct NVFile 
	*/
	_nvp_fd_lookup = (struct NVFile*)calloc(OPEN_MAX,
						sizeof(struct NVFile));	 
	
	if (!_nvp_fd_lookup)
		assert(0);

	// Initializing the valid bits and locks of each NVFile
	for(i = 0; i < OPEN_MAX; i++) {
		_nvp_fd_lookup[i].valid = 0;
		execve_fd_passing[i] = 0;
		NVP_LOCK_INIT(_nvp_fd_lookup[i].lock);
	}

	/* Initializing the closed file descriptor array */
	_nvp_closed_files = (struct ClosedFiles*)calloc(TOTAL_CLOSED_INODES, sizeof(struct ClosedFiles));
	for(i = 0; i < TOTAL_CLOSED_INODES; i++) {
		_nvp_closed_files[i].fd = -1;
		_nvp_closed_files[i].serialno = 0;
		_nvp_closed_files[i].index_in_free_list = -1;
		_nvp_closed_files[i].next_closed_file = -1;
		_nvp_closed_files[i].prev_closed_file = -1;
		NVP_LOCK_INIT(_nvp_closed_files[i].lock);
	}

	lru_head = -1;
	lru_tail = -1;
	lru_tail_serialno = 0;
	
	if(!_nvp_closed_files)
		assert(0);

	/* Initialize and allocate hash table for closed file descriptor array */
	inode_to_closed_file = (struct InodeClosedFile *)calloc(OPEN_MAX, sizeof(struct InodeClosedFile));
	for(i = 0; i < OPEN_MAX; i++) {
		inode_to_closed_file[i].index = -1;
		NVP_LOCK_INIT(inode_to_closed_file[i].lock);
	}

	if(!inode_to_closed_file)
		assert(0);
	
	/* 
	   Allocate and initialize the free list for nodes
	*/
	for (i = 0; i < NUM_NODE_LISTS; i++) {
		_nvp_free_node_list[i] = (struct StackNode*)calloc(OPEN_MAX,
							sizeof(struct StackNode));
		for(j = 0; j < OPEN_MAX; j++) {
			_nvp_free_node_list[i][j].free_bit = 1;
			_nvp_free_node_list[i][j].next_free_idx = j+1;
		}
		_nvp_free_node_list[i][OPEN_MAX - 1].next_free_idx = -1;
	}
	
	_nvp_free_lru_list = (struct StackNode*)calloc(OPEN_MAX,
						       sizeof(struct StackNode));
	for(i = 0; i < OPEN_MAX; i++) {
		_nvp_free_lru_list[i].free_bit = 1;
		_nvp_free_lru_list[i].next_free_idx = i+1;
	}
       	_nvp_free_lru_list[OPEN_MAX - 1].next_free_idx = -1;

	for (i = 0; i < NUM_NODE_LISTS; i++) {
		if (!_nvp_free_node_list[i])
			assert(0);
	}
	
	if(!_nvp_free_lru_list)
		assert(0);

	for (i = 0; i < NUM_NODE_LISTS; i++) {
		_nvp_free_node_list_head[i] = 0;
	}
	_nvp_free_lru_list_head = 0;
	
	/* 
	   Allocating and Initializing mmap cache. Can hold mmaps, merkle trees and dirty mmap caches belonging to 1024 files. _nvp_ino_mapping is an array of struct InodeToMapping 
	*/	
	_nvp_ino_mapping = (struct InodeToMapping*)calloc(OPEN_MAX, sizeof(struct InodeToMapping));
	memset(_nvp_ino_mapping, 0, OPEN_MAX * sizeof(struct InodeToMapping));		
	if (!_nvp_ino_mapping)
		assert(0);

	for(i=0; i<OPEN_MAX; i++) {
		tempMapping = &_nvp_ino_mapping[i];

		// Allocating region to store mmap() addresses
		tempMapping->root = malloc(1024 * sizeof(unsigned long));
		memset(tempMapping->root, 0, 1024 * sizeof(unsigned long));

		tempMapping->merkle_root = malloc(1024 * sizeof(unsigned long));
		memset(tempMapping->merkle_root, 0, 1024 * sizeof(unsigned long));
		
		// Allocating region to store dirty mmap caches
		tempMapping->root_dirty_cache = malloc(20 * sizeof(unsigned long));
		memset(tempMapping->root_dirty_cache, 0, 20 * sizeof(unsigned long));

		tempMapping->root_dirty_num = 0;
		tempMapping->total_dirty_mmaps = 0;
		
		// Initializing the inode numbers = keys to 0
		_nvp_ino_mapping[i].serialno = 0;				
	}
	

	/*
	  Allocating and Initializing NVNode. Number of NVNodes = 1024. _nvp_node_lookup is an array of struct NVNode 
	*/
	for (i = 0; i < NUM_NODE_LISTS; i++) {
		_nvp_node_lookup[i] = (struct NVNode*)calloc(OPEN_MAX,
							     sizeof(struct NVNode));
		_nvp_backup_roots[i] = (struct backupRoots*)calloc(OPEN_MAX,
								   sizeof(struct backupRoots));
		if (!_nvp_node_lookup[i])
			assert(0);
		memset(_nvp_node_lookup[i], 0, OPEN_MAX * sizeof(struct NVNode));
	
		// Allocating and initializing all the dynamic structs inside struct NVNode 
		for(j = 0; j < OPEN_MAX; j++) {

			// Initializing lock associated with NVNode
			NVP_LOCK_INIT(_nvp_node_lookup[i][j].lock);

			// Allocating and Initializing mmap() roots associated with NVNode 
			_nvp_node_lookup[i][j].root = malloc(1024 * sizeof(unsigned long));
			memset(_nvp_node_lookup[i][j].root, 0, 1024 * sizeof(unsigned long));
			_nvp_backup_roots[i][j].root = _nvp_node_lookup[i][j].root;
			
			// Allocating and Initializing merkle tree roots associated with NVNode 
			_nvp_node_lookup[i][j].merkle_root = malloc(1024 * sizeof(unsigned long));
			memset(_nvp_node_lookup[i][j].merkle_root, 0, 1024 * sizeof(unsigned long));
			_nvp_backup_roots[i][j].merkle_root = _nvp_node_lookup[i][j].merkle_root;

			// Allocating and Initializing the dirty mmap cache associated with NVNode
			_nvp_node_lookup[i][j].root_dirty_cache = malloc(20 * sizeof(unsigned long));
			memset(_nvp_node_lookup[i][j].root_dirty_cache, 0, 20 * sizeof(unsigned long));
			_nvp_backup_roots[i][j].root_dirty_cache = _nvp_node_lookup[i][j].root_dirty_cache;

			_nvp_node_lookup[i][j].root_dirty_num = 0;
			_nvp_node_lookup[i][j].total_dirty_mmaps = 0;

			// Allocating and Initializing anonymous root of the node
			_nvp_node_lookup[i][j].anonymous_root = malloc(1024 * sizeof(unsigned long));
			memset(_nvp_node_lookup[i][j].anonymous_root, 0, 1024 * sizeof(unsigned long));
			_nvp_backup_roots[i][j].anonymous_root = _nvp_node_lookup[i][j].anonymous_root;
			
			_nvp_node_lookup[i][j].anonymous_offset_start = malloc(1024 * sizeof(unsigned long));
			memset(_nvp_node_lookup[i][j].anonymous_offset_start, UINT32_MAX, 1024 * sizeof(unsigned long));
			_nvp_backup_roots[i][j].anonymous_offset_start = _nvp_node_lookup[i][j].anonymous_offset_start;

			
			_nvp_node_lookup[i][j].anonymous_offset_end = malloc(1024 * sizeof(unsigned long));
			memset(_nvp_node_lookup[i][j].anonymous_offset_end, 0, 1024 * sizeof(unsigned long));
			_nvp_backup_roots[i][j].anonymous_offset_end = _nvp_node_lookup[i][j].anonymous_offset_end;			
			
		}
	}
	
	// Initializing global lock for accessing NVNode
	for (i = 0; i < NUM_NODE_LISTS; i++) {
		pthread_spin_init(&node_lookup_lock[i], PTHREAD_PROCESS_SHARED);
	}
	pthread_spin_init(&global_lock, PTHREAD_PROCESS_SHARED);
	pthread_spin_init(&global_lock_closed_files, PTHREAD_PROCESS_SHARED);
	pthread_spin_init(&global_lock_lru_head, PTHREAD_PROCESS_SHARED);
	pthread_spin_init(&stack_lock, PTHREAD_PROCESS_SHARED);
	
	/*
	  Allocating and Initializing the free pool of anonymous mmap()s. Total number of mmap()s allowed = 1024.
	*/

	lfds711_queue_umm_init_valid_on_current_logical_core( &qs, &qe_dummy, NULL );

	MMAP_PAGE_SIZE = getpagesize();
	SANITYCHECK(MMAP_PAGE_SIZE > 100);

	INITIALIZE_TIMERS();

	/*
	  Setting up variables and initialization for background thread
	*/

	cleanup = 0;
	waiting_for_signal = 0;
	started_bgthread = 0;
	exit_bgthread = 0;
	lim_num_files = 100;
	lim_anon_mem = (5ULL) * 1024 * 1024 * 1024;
	lim_anon_mem_closed = 500 * 1024 * 1024;
	run_background_thread = 0;
	initEnvForBg();
	//printf("%s: initialized environment, OPEN_MAX = %d\n", __func__, OPEN_MAX);
	anon_mem_allocated = 0;
	anon_mem_closed_files = 0;
#if BG_CLOSING	
	calledBgThread = 0;
	startBgThread();
#endif
	
	/*
	  Setting up signal handlers: SIGBUS and SIGUSR 
	*/
	DEBUG("Installing signal handler.\n");
	signal(SIGBUS, _nvp_SIGBUS_handler);
	/* For filebench */
	signal(SIGUSR1, _nvp_SIGUSR1_handler);

	/*
	  Setting up the exit handler to print stats 
	*/
	atexit(nvp_exit_handler);

	if (access("/dev/shm/exec-ledger", F_OK ) != -1) 
		execv_done = 1;
	else
		execv_done = 0;
	
}

void nvp_transfer_to_free_anonymous_pool(unsigned long *root, unsigned long *offset_start, unsigned long *offset_end, int anonymous_root_dirty_start, int anonymous_root_dirty_end, struct NVNode *node) {
	
	int i, num_free_anonymous_mmaps;
	struct free_anon_pool *free_pool_of_anonymous_mmaps;
	
	if(anonymous_root_dirty_start != -1) {

		free_pool_of_anonymous_mmaps = malloc(sizeof(struct free_anon_pool)*(anonymous_root_dirty_end - anonymous_root_dirty_start + 1));
		num_free_anonymous_mmaps = 0;
		
		for (i=anonymous_root_dirty_start; i<=anonymous_root_dirty_end; i++) {
			if(root[i]) {				
				offset_start[i] = UINT32_MAX;
				offset_end[i] = 0;					
				
				free_pool_of_anonymous_mmaps[num_free_anonymous_mmaps].addr = root[i];
				LFDS711_QUEUE_UMM_SET_VALUE_IN_ELEMENT( free_pool_of_anonymous_mmaps[num_free_anonymous_mmaps].qe, &free_pool_of_anonymous_mmaps[num_free_anonymous_mmaps] );
				lfds711_queue_umm_enqueue( &qs, &free_pool_of_anonymous_mmaps[num_free_anonymous_mmaps].qe );
				num_free_anonymous_mmaps++;
				root[i] = 0;
				
				__atomic_fetch_sub(&anon_mem_allocated, ANON_MAX_MMAP_SIZE, __ATOMIC_SEQ_CST);
				if (node->async_file_close)
					__atomic_fetch_sub(&anon_mem_closed_files, ANON_MAX_MMAP_SIZE, __ATOMIC_SEQ_CST);				
				
			}
		}

		//free(free_pool_of_anonymous_mmaps);
	}

	node->anonymous_root_dirty_start = -1;
	node->anonymous_root_dirty_end = -1;
	node->isAnonMapReset = 1;
	node->usingAnonMem = 0;
 
	//node->num_alloc_anon_mmaps = 0;
	return;
}

void nvp_free_anonymous_mmaps()
{
	unsigned long addr;
	struct free_anon_pool *temp_free_pool_of_anonymous_mmaps;

	while( lfds711_queue_umm_dequeue(&qs, &qe) ) {
		temp_free_pool_of_anonymous_mmaps = LFDS711_QUEUE_UMM_GET_VALUE_FROM_ELEMENT( *qe );
		addr = temp_free_pool_of_anonymous_mmaps->addr;
		munmap((void *)addr, ANON_MAX_MMAP_SIZE);		
	}

	lfds711_queue_umm_cleanup( &qs, NULL );

}

void nvp_free_btree(unsigned long *root, unsigned long *merkle_root, unsigned long height, unsigned long *dirty_cache, int root_dirty_num, int total_dirty_mmaps)
{
	int i, dirty_index;

	dirty_index = 0;
	if (height == 0) {
		for(i = 0; i < root_dirty_num; i++) {
			dirty_index = dirty_cache[i];
			if(root && root[dirty_index]) {
				munmap((void *)root[dirty_index], MAX_MMAP_SIZE);
				root[dirty_index] = 0;
				merkle_root[dirty_index] = 0;
			}
		}

		root_dirty_num = 0;

		if(total_dirty_mmaps) {
			for (i = 0; i < 1024; i++) {
				if (root && root[i]) {
					DEBUG("munmap: %d, addr 0x%lx\n",
					      i, root[i]);
					munmap((void *)root[i], MAX_MMAP_SIZE);
					root[i] = 0;
					merkle_root[i] = 0;
				}
			}
		}
		return;
	}

	for (i = 0; i < 1024; i++) {
		if (root[i] && merkle_root[i]) {
			nvp_free_btree((unsigned long *)root[i], (unsigned long *)merkle_root[i], height - 1, NULL, 0, 1);
			root[i] = 0;
			merkle_root[i] = 0;
		}
	}

	free(root);
	free(merkle_root);
}


void nvp_add_to_inode_mapping(struct NVNode *node, ino_t serialno)
{
	struct InodeToMapping *mappingToBeAdded;
	
	int index = serialno % 1024;
	int i, dirty_index;

	if (serialno == 0)
		return;

	DEBUG("Cleanup: root 0x%x, height %u\n", root, height);

	// Check the entry in the global file backed mmap() cache to see if the entry for this inode is present or not
	mappingToBeAdded = &_nvp_ino_mapping[index];

	//printf("%s: adding node of address = %lu, inode = %lu, index = %d\n", __func__, node->root[0], serialno, index);

	//printf("%s: start: adding mapping of address = %lu, node->root[0] = %lu, root_dirty_num = %d, inode = %lu, index = %d, node reference = %d, thread id = %lu\n", __func__, mappingToBeAdded->root[0], node->root[0], node->root_dirty_num, mappingToBeAdded->serialno, index, node->reference, pthread_self());

	if(mappingToBeAdded->serialno != 0 && mappingToBeAdded->serialno != serialno) {
		// Replacing some mmap() in that global mmap() cache. So must munmap() all the mmap() ranges in that cache. 
		nvp_free_btree(mappingToBeAdded->root, mappingToBeAdded->merkle_root, mappingToBeAdded->height, mappingToBeAdded->root_dirty_cache, mappingToBeAdded->root_dirty_num, mappingToBeAdded->total_dirty_mmaps);		

		mappingToBeAdded->serialno = 0;
	}

	// Check if many mmap()s need to be copied. If total_dirty_mmaps is set, that means all the mmap()s need to be copied. 
	if(node->total_dirty_mmaps) {
		memcpy(mappingToBeAdded->root, node->root, 1024 * sizeof(unsigned long));
		memcpy(mappingToBeAdded->merkle_root, node->merkle_root, 1024 * sizeof(unsigned long));
		
	} else {
		// Only copy the dirty mmaps. The indexes can be found in the root_dirty_cache. 
		for(i = 0; i < node->root_dirty_num; i++) {
			dirty_index = node->root_dirty_cache[i];
			if(node->root && node->root[dirty_index])
				mappingToBeAdded->root[dirty_index] = node->root[dirty_index];

			if(node->merkle_root && node->merkle_root[dirty_index])
				mappingToBeAdded->merkle_root[dirty_index] = node->merkle_root[dirty_index];
		}
	}

	mappingToBeAdded->serialno = serialno;
	
	if(node->root_dirty_num)
		memcpy(mappingToBeAdded->root_dirty_cache, node->root_dirty_cache, 20 * sizeof(unsigned long));

	mappingToBeAdded->root_dirty_num = node->root_dirty_num;
	mappingToBeAdded->total_dirty_mmaps = node->total_dirty_mmaps;
	mappingToBeAdded->height = node->height;      	

	//printf("%s: end: added mapping of address = %lu, node->root[0] = %lu, root_dirty_num = %d, inode = %lu, index = %d, node reference = %d, thread id = %lu\n", __func__, mappingToBeAdded->root[0], node->root[0], node->root_dirty_num, mappingToBeAdded->serialno, index, node->reference, pthread_self());

}

/* 
 * This function is responsible for copying all the mapping from the global mmap() cache on to the mmap tree of the node. 
 */
int nvp_retrieve_inode_mapping(struct NVNode *node)
{

	struct InodeToMapping *mappingToBeRetrieved;
	int index = node->serialno % 1024;
	int dirty_index, i;
	
	DEBUG("Cleanup: root 0x%x, height %u\n", root, height);

	/* 
	 * Get the mapping from the global mmap() cache, based on the inode number of the node whose mapping it should
         * be retrieved from. 
	 */
	mappingToBeRetrieved = &_nvp_ino_mapping[index];
	
	if(mappingToBeRetrieved->serialno == node->serialno) {

		/* 
		 * Copy the file backed mmap()s and the merkle roots. total_dirty_mmaps suggests that there are more than
		 * 20 mmaps that need to be copied.
		 */
		if(mappingToBeRetrieved->total_dirty_mmaps) {
			memcpy(node->root, mappingToBeRetrieved->root, 1024 * sizeof(unsigned long));
			memcpy(node->merkle_root, mappingToBeRetrieved->merkle_root, 1024 * sizeof(unsigned long));
			
		} else {
	
			for(i = 0; i < mappingToBeRetrieved->root_dirty_num; i++) {
				dirty_index = mappingToBeRetrieved->root_dirty_cache[i];
				if(mappingToBeRetrieved->root && mappingToBeRetrieved->root[dirty_index])
					node->root[dirty_index] = mappingToBeRetrieved->root[dirty_index];

				if(mappingToBeRetrieved->merkle_root && mappingToBeRetrieved->merkle_root[dirty_index])
					node->merkle_root[dirty_index] = mappingToBeRetrieved->merkle_root[dirty_index];
			}
		}
		
		// Copy the root_dirty_cache from the global mmap() cache on to the node mmap() cache
		if(mappingToBeRetrieved->root_dirty_num)
			memcpy(node->root_dirty_cache, mappingToBeRetrieved->root_dirty_cache, 20 * sizeof(unsigned long));
		
		node->root_dirty_num = mappingToBeRetrieved->root_dirty_num;
		node->total_dirty_mmaps = mappingToBeRetrieved->total_dirty_mmaps;
		node->height = mappingToBeRetrieved->height;      	

		//printf("%s: end: node->root[0] = %lu, mapping root = %lu, mapping root dirty num = %d, node->serialno = %lu, index = %d, node reference = %d, thread_id = %lu\n", __func__, node->root[0], mappingToBeRetrieved->root[0], mappingToBeRetrieved->root_dirty_num, node->serialno, index, node->reference, pthread_self());

		goto out;
	}

	return -1;
 out:
	return 0;
}

void nvp_reset_mappings(struct NVNode *node)
{
	int i, dirty_index;
	
	DEBUG("Cleanup: root 0x%x, height %u\n", root, height);

	if(node->root_dirty_num) {		
		// Check if many mmap()s need to be memset. If total_dirty_mmaps is set, that means all the mmap()s need to be copied 
		if(node->total_dirty_mmaps) {
			memset(node->root, 0, 1024 * sizeof(unsigned long));		
			memset(node->merkle_root, 0, 1024 * sizeof(unsigned long));	
		} else {
			// Only copy the dirty mmaps. The indexes can be found in the root_dirty_cache. 
			for(i = 0; i < node->root_dirty_num; i++) {
				dirty_index = node->root_dirty_cache[i];
				if(node->root && node->root[dirty_index]) {
					node->root[dirty_index] = 0;
					node->merkle_root[dirty_index] = 0;
				}
			}
		}

		if(node->root_dirty_num)
			memset(node->root_dirty_cache, 0, 20 * sizeof(unsigned long));	
	}

	node->isRootSet = 0;
	node->height = 0;
	node->total_dirty_mmaps = 0;
	node->root_dirty_num = 0;

}

void nvp_cleanup_node(struct NVNode *node, int free_root, int unmap_btree)
{

	unsigned int height = node->height;
	unsigned long *root = node->root;
	unsigned long *merkle_root = node->merkle_root;
	unsigned long *dirty_cache;
	int total_dirty_mmaps = node->total_dirty_mmaps;
	int root_dirty_num = node->root_dirty_num;
	
	DEBUG("Cleanup: root 0x%x, height %u\n", root, height);

	if(root_dirty_num > 0)
		dirty_cache = node->root_dirty_cache;
	else
		dirty_cache = NULL;
     
	if(unmap_btree && node->root_dirty_num) {
		// munmap() all the file backed mmap()s of this file. 
		nvp_free_btree(root, merkle_root, height, dirty_cache, root_dirty_num, total_dirty_mmaps);
	}
		
	/* 
	 * Deallocate everything related to NVNode. This should be done at the end when Ledger is exiting. 
	 */
	if (free_root && node->root[0]) {
		free(node->root);
		free(node->merkle_root);
		free(node->root_dirty_cache);
		node->root = NULL;
		node->merkle_root = NULL;
		node->root_dirty_cache = NULL;
		return;
	}

	// Copy all the anonymous mmap()s linked to this node, to the global pool of anonymous mmap()s

	/*
	 * Resetting the file backed mmap addresses, merkle tree addresses and the dirty file backed mmap cache of this node to 0. 
	 */
	if(!unmap_btree)
		nvp_reset_mappings(node);
}

void nvp_init_anonymous(struct NVNode *node)
{
	if(!node->anonymous_root) {
		node->anonymous_root = malloc(1024 * sizeof(unsigned long));
		memset(node->anonymous_root, 0, 1024 * sizeof(unsigned long));
	}
	if(!node->anonymous_offset_start) {
		node->anonymous_offset_start = malloc(1024 * sizeof(unsigned long));
		memset(node->anonymous_offset_start, UINT32_MAX, 1024 * sizeof(unsigned long));
	}
	if(!node->anonymous_offset_end) {
		node->anonymous_offset_end = malloc(1024 * sizeof(unsigned long));
		memset(node->anonymous_offset_end, 0, 1024 * sizeof(unsigned long));
	}
}

void nvp_init_node(struct NVNode *node)
{
	int i;
	if (!node->root) {
		node->root = malloc(1024 * sizeof(unsigned long));
		memset(node->root, 0, 1024 * sizeof(unsigned long));
	}
	if(!node->merkle_root) {
		node->merkle_root = malloc(1024 * sizeof(struct merkleBtreeNode *));
		for(i=0; i<1024; i++)
			node->merkle_root[i] = 0;
	}
	if(!node->root_dirty_cache) {
		node->root_dirty_cache = malloc(20 * sizeof(unsigned long));
		memset(node->root_dirty_cache, 0, 20 * sizeof(unsigned long));
	}					
}

struct NVNode * nvp_allocate_node(int list_idx)
{
	struct NVNode *node = NULL;
	int idx_in_list = -1;
	int i, candidate = -1;

#if PRINT_DEBUG_MSGS
	printf("%s: popping from stack, index = %d\n", __func__, list_idx);
	fflush(NULL);
#endif

	idx_in_list = pop_from_stack(1, 0, list_idx);

#if PRINT_DEBUG_MSGS
	printf("%s: popped from stack, index = %d\n", __func__, list_idx);
	fflush(NULL);
#endif
	
	if(idx_in_list != -1) {
		node = &_nvp_node_lookup[list_idx][idx_in_list];
		node->index_in_free_list = idx_in_list;
#if PRINT_DEBUG_MSGS
		printf("%s: returning node, index = %d\n", __func__, list_idx);
		fflush(NULL);
#endif
		return node;
	}	
	
	/*
	 * Get the first unused NVNode from the global array of 1024 NVNodes. If the node is not unusued but the reference number is
	 * 0, meaning that there is no thread that has this file open, it can be used for holding info of the new file
	 */	
	for (i = 0; i < 1024; i++) {
		if (_nvp_node_lookup[list_idx][i].serialno == 0) {
			DEBUG("Allocate unused node %d\n", i);
			_nvp_free_node_list[list_idx][i].free_bit = 0;
			node->index_in_free_list = i;		
			_nvp_free_node_list_head[list_idx] = _nvp_free_node_list[list_idx][node->index_in_free_list].next_free_idx;
			node = &_nvp_node_lookup[list_idx][i];
			break;
		}
		if (candidate == -1 && _nvp_node_lookup[list_idx][i].reference == 0)
			candidate = i;
	}

	if (node) {
		return node;
	}

	if (candidate != -1) {
		node = &_nvp_node_lookup[list_idx][candidate];
		DEBUG("Allocate unreferenced node %d\n", candidate);
		node->index_in_free_list = candidate;		
		_nvp_free_node_list[list_idx][candidate].free_bit = 0;
		_nvp_free_node_list_head[list_idx] = _nvp_free_node_list[list_idx][candidate].next_free_idx;
		return node;
	}

	return NULL;	
}

struct NVNode * nvp_get_node(const char *path, struct stat *file_st, int result)
{
	int i, index, ret;
	struct NVNode *node = NULL;
	int node_list_idx = pthread_self() % NUM_NODE_LISTS;
	instrumentation_type node_lookup_lock_time, nvnode_lock_time;
	
#if PRINT_DEBUG_MSGS
	printf("%s: acquiring node lookup lock, index = %d, thread = %lu\n", __func__, node_list_idx, pthread_self());
	fflush(NULL);
#endif
	
	//START_TIMING(node_lookup_lock_t, node_lookup_lock_time);
	pthread_spin_lock(&node_lookup_lock[node_list_idx]);

#if PRINT_DEBUG_MSGS
	printf("%s: acquired node lookup lock, index = %d, thread = %lu\n", __func__, node_list_idx, pthread_self());
	fflush(NULL);
#endif
	
	/* 
	 *  Checking to see if the file is already open by another thread. In this case, the same NVNode can be used by this thread            
	 *  too. But it will have its own separate NVFile, since the fd is unique per thread 
	 */
	index = file_st->st_ino % 1024;
	if (_nvp_ino_lookup[index]) {
		i = _nvp_ino_lookup[index];
		if ( _nvp_fd_lookup[i].node &&
		     _nvp_fd_lookup[i].node->serialno == file_st->st_ino) {
			DEBUG("File %s is (or was) already open in fd %i "
			      "(this fd hasn't been __open'ed yet)! "
			      "Sharing nodes.\n", path, i);
			
			node = _nvp_fd_lookup[i].node;
			SANITYCHECK(node != NULL);			

		        //START_TIMING(nvnode_lock_t, nvnode_lock_time);
			NVP_LOCK_WR(node->lock);
			//END_TIMING(nvnode_lock_t, nvnode_lock_time);
			node->reference++;			
			NVP_LOCK_UNLOCK_WR(node->lock);

			pthread_spin_unlock(&node_lookup_lock[node_list_idx]);
			//END_TIMING(node_lookup_lock_t, node_lookup_lock_time);

			goto out;
		}
	}

#if PRINT_DEBUG_MSGS
	printf("%s: in the area where node = NULL, index = %d\n", __func__, node_list_idx);
	fflush(NULL);
#endif
	/*
	 * This is the first time the file is getting opened. The first unused NVNode is assigned here to hold info of the file.  
	 */
	if(node == NULL) {
		DEBUG("File %s is not already open. "
		      "Allocating new NVNode.\n", path);
		node = nvp_allocate_node(node_list_idx);
		//START_TIMING(nvnode_lock_t, nvnode_lock_time);
		NVP_LOCK_WR(node->lock);
		//END_TIMING(nvnode_lock_t, nvnode_lock_time);
		node->serialno = file_st->st_ino;
		node->reference++;
		NVP_LOCK_UNLOCK_WR(node->lock);
		if(UNLIKELY(!node)) {
			ERROR("Node is null\n");
			assert(100);
		}			
		//assert(node);
	}

#if PRINT_DEBUG_MSGS
	printf("%s: releasing node lookup lock, index = %d, thread = %lu\n", __func__, node_list_idx, pthread_self());
	fflush(NULL);
#endif

	index = file_st->st_ino % 1024;
	if (_nvp_ino_lookup[index] == 0)
		_nvp_ino_lookup[index] = result;
	
	node->free_list_idx = node_list_idx;
	
	pthread_spin_unlock(&node_lookup_lock[node_list_idx]);
	//END_TIMING(node_lookup_lock_t, node_lookup_lock_time);

#if PRINT_DEBUG_MSGS
	printf("%s: released node lookup lock, index = %d, thread = %lu\n", __func__, node_list_idx, pthread_self());
	fflush(NULL);
#endif
	
	//START_TIMING(nvnode_lock_t, nvnode_lock_time);
	NVP_LOCK_WR(node->lock);
	//END_TIMING(nvnode_lock_t, nvnode_lock_time);

	/* 
	 * Checking if the mapping exists in the global mmap() cache for this inode number. If it does, copy all the mapping
	 * from the global mmap() cache on to the NVNode mmap()
         */  
	nvp_add_to_inode_mapping(node, node->backup_serialno);
	nvp_reset_mappings(node);	
	ret = nvp_retrieve_inode_mapping(node);
	
	if(ret != 0) {
		/* 
		 * If the height is not 0, that means that there exist levels in the file backed mmap() tree. So need to free
		 * the file backed mmap() tree completely. 
		 */
		if(node->height != 0) 
			nvp_cleanup_node(node, 0, 1);		
	}

	node->length = file_st->st_size;
	node->maplength = 0;
	node->true_length = node->length;
	node->true_length_for_read = node->true_length;
	node->usingAnonMem = 0;
	node->anon_mem_used = 0;

	GLOBAL_LOCK_WR();

	if(node->anonymous_root) {
		nvp_transfer_to_free_anonymous_pool(node->anonymous_root, node->anonymous_offset_start, node->anonymous_offset_end, node->anonymous_root_dirty_start, node->anonymous_root_dirty_end, node);
	}

	node->async_file_close = 0;
	node->backup_serialno = node->serialno;
	
	GLOBAL_UNLOCK_WR();
	
	NVP_LOCK_UNLOCK_WR(node->lock);

out:
	return node;
}

static unsigned long calculate_anon_capacity(unsigned int height)
{
	unsigned long capacity = ANON_MAX_MMAP_SIZE;

	while (height) {
		capacity *= 1024;
		height--;
	}

	return capacity;
}

static unsigned long calculate_capacity(unsigned int height)
{
	unsigned long capacity = MAX_MMAP_SIZE;

	while (height) {
		capacity *= 1024;
		height--;
	}

	return capacity;
}

static unsigned int calculate_new_height(off_t offset)
{
	unsigned int height = 0;
	off_t temp_offset = offset / ((unsigned long)1024 * MAX_MMAP_SIZE);

	while (temp_offset) {
		temp_offset /= 1024;
		height++;
	}

	return height;
}

static int nvp_get_mmap_address(struct NVFile *nvf, off_t offset, size_t count, unsigned long *mmap_addr, unsigned long *bitmap_root, off_t *offset_within_mmap, size_t *extent_length, int wr_lock, int cpuid)
{
	int i;
	int index;
	unsigned int height = nvf->node->height;
	unsigned int new_height;
	unsigned long capacity = MAX_MMAP_SIZE;
	unsigned long *root = nvf->node->root;
	unsigned long *merkle_root = nvf->node->merkle_root;
	unsigned long start_addr;
	unsigned long merkle_start_addr;
	off_t start_offset = offset;
	instrumentation_type nvnode_lock_time;
	
	DEBUG("Get mmap address: offset 0x%lx, height %u\n", offset, height);
	DEBUG("root @ %p\n", root);

	//NVP_START_TIMING(lookup_t, lookup_time);
	do {
		capacity = calculate_capacity(height);
		index = start_offset / capacity;

		DEBUG("index %d\n", index);
		if (index >= 1024 || root[index] == 0 || merkle_root[index] == 0) {
			//NVP_END_TIMING(lookup_t, lookup_time);
			goto not_found;
		}
		if (height) {
			root = (unsigned long *)root[index];
			merkle_root = (unsigned long *)merkle_root[index];
			DEBUG("%p\n", root);
		} else {
			start_addr = root[index];
			merkle_start_addr = merkle_root[index];
			DEBUG("addr 0x%lx\n", start_addr);
		}
		start_offset = start_offset % capacity;
	} while(height--);
	//NVP_END_TIMING(lookup_t, lookup_time);

	if (IS_ERR(start_addr) || start_addr == 0 || merkle_start_addr == 0) {
		MSG("ERROR!\n");
		fflush(NULL);
		assert(0);
	}

        (*mmap_addr) = (start_addr + (offset % MAX_MMAP_SIZE));
	*offset_within_mmap = offset % MAX_MMAP_SIZE;
	*bitmap_root = merkle_start_addr;
	(*extent_length) = (MAX_MMAP_SIZE - (offset % MAX_MMAP_SIZE));

	DEBUG("Found: mmap addr 0x%lx, extent length %lu\n",
			*mmap_addr, *extent_length);
	return 0;

not_found:
	DEBUG("Not found, perform mmap\n");

	if (offset >= ALIGN_MMAP_DOWN(nvf->node->true_length)) {
		DEBUG("File length smaller than offset: "
			"length 0x%lx, offset 0x%lx\n",
			nvf->node->length, offset);
		return 1;
	}
		
	if (!wr_lock) {
		NVP_UNLOCK_NODE_RD(nvf, cpuid);
		START_TIMING(nvnode_lock_t, nvnode_lock_time);
		NVP_LOCK_NODE_WR(nvf);
	        END_TIMING(nvnode_lock_t, nvnode_lock_time);
	}

	start_offset = ALIGN_MMAP_DOWN(offset);	
		
	if (start_offset + MAX_MMAP_SIZE > nvf->node->true_length) {
		ERROR("File length smaller than offset: "
			"length 0x%lx, offset 0x%lx\n",
			nvf->node->length, offset);
		printf("%s: file length smaller than offset\n", __func__);
		fflush(NULL);
		return 1;
	}
	
	//NVP_START_TIMING(mmap_t, mmap_time);
	
	int max_perms = ((nvf->canRead) ? PROT_READ : 0) | 
			((nvf->canWrite) ? PROT_WRITE : 0);

	start_addr = (unsigned long) FSYNC_MMAP
	(
		NULL,
		MAX_MMAP_SIZE,
		max_perms, //max_perms,
		MAP_SHARED | MAP_POPULATE,
//		MAP_SHARED,
		nvf->fd, //fd_with_max_perms,
		start_offset
		//0
	);

	//printf("%s: created mapping of address = %lu, inode = %lu, thread id = %lu\n", __func__, start_addr, nvf->node->serialno, pthread_self());
	
	/* Bitmap Tree creation */
	createTree((struct merkleBtreeNode **)&merkle_start_addr);
	//NVP_END_TIMING(mmap_t, mmap_time);
	num_mmap++;

	if (IS_ERR(start_addr) || start_addr == 0 || merkle_start_addr == 0)
	{
		MSG("mmap failed for fd %i: %s, mmap count %d, addr %lu, errno is %lu\n",
		    nvf->fd, strerror(errno), num_mmap, start_addr, errno);
		MSG("Open count %d, close count %d\n", num_open, num_close);
		MSG("Use posix operations for fd %i instead.\n", nvf->fd);
		nvf->posix = 1;
		fflush(NULL);
		assert(0);
	}

	DEBUG("mmap offset 0x%lx, start_offset 0x%lx\n", offset, start_offset);

	height = nvf->node->height;
	new_height = calculate_new_height(offset);

	if (height < new_height) {
		MSG("Increase height from %u to %u\n", height, new_height);

		while (height < new_height) {
			unsigned long old_root = (unsigned long)nvf->node->root;
			unsigned long old_merkle_root = (unsigned long)nvf->node->merkle_root;
			nvf->node->root = malloc(1024 * sizeof(unsigned long));
			nvf->node->merkle_root = malloc(1024 * sizeof(unsigned long));
			DEBUG("Malloc new root @ %p\n", nvf->node->root);
			for (i = 0; i < 1024; i++) {
				nvf->node->root[i] = 0;
				nvf->node->merkle_root[i] = 0;
			}
			nvf->node->root[0] = (unsigned long)old_root;
			nvf->node->merkle_root[0] = (unsigned long)old_merkle_root;
			DEBUG("Old root 0x%lx\n", nvf->node->root[0]);
			height++;
		}

		nvf->node->height = new_height;
		height = new_height;
	}

	root = nvf->node->root;
	merkle_root = nvf->node->merkle_root;
	do {
		capacity = calculate_capacity(height);
		index = start_offset / capacity;
		DEBUG("index %d\n", index);
		if (height) {
			if (root[index] == 0) {
				root[index] = (unsigned long)malloc(1024 *
						sizeof(unsigned long));
				merkle_root[index] = (unsigned long)malloc(1024 * sizeof(unsigned long));
				DEBUG("Malloc new leaf @%p, height %u, "
				      "index %u\n",
					root[index], height, index);
				root = (unsigned long *)root[index];
				merkle_root = (unsigned long *)merkle_root[index];
				for (i = 0; i < 1024; i++) {
					root[i] = 0;
					merkle_root[i] = 0;
				}
			} else {
				root = (unsigned long *)root[index];
				merkle_root = (unsigned long *)merkle_root[index];
			}
		} else {
			root[index] = start_addr;
			nvf->node->root_dirty_cache[nvf->node->root_dirty_num] = index;
			if(!nvf->node->total_dirty_mmaps) {
				nvf->node->root_dirty_num++;
				if(nvf->node->root_dirty_num == 20)
					nvf->node->total_dirty_mmaps = 1;
			}
			merkle_root[index] = merkle_start_addr;
			DEBUG("mmap for fd %i: %d, addr 0x%lx\n",
				nvf->fd, index, start_addr);
		}
		start_offset = start_offset % capacity;
	} while(height--);

	nvf->node->isRootSet = 1;
	(*mmap_addr) = (start_addr + (offset % MAX_MMAP_SIZE));
	*offset_within_mmap = offset % MAX_MMAP_SIZE;
	*bitmap_root = merkle_start_addr;
	(*extent_length) = (MAX_MMAP_SIZE - (offset % MAX_MMAP_SIZE));

	if (!wr_lock) {
		NVP_UNLOCK_NODE_WR(nvf);
		NVP_LOCK_NODE_RD(nvf, cpuid);
	}

	DEBUG("mmap addr 0x%lx, extent length %lu\n",
			*mmap_addr, *extent_length);

	return 0;
}

static void nvp_manage_anonymous_memory(struct NVFile *nvf, uint64_t *extent_length, uint64_t len_to_write, off_t start_offset, int index) {

	int i;
	/* 
	 * Check if the reads are being served from anonymous DRAM. If yes, then all the future reads should
	 * be performed through the file backed memory, for the appended and fsync()ed region. 
	 */
	if(!nvf->node->isAnonMapReset) {
		// Clear all the anonymous mmaps. This is the first write to the appended region after fsync(). 
		for(i=nvf->node->anonymous_root_dirty_start; i<=nvf->node->anonymous_root_dirty_end; i++) {
			nvf->node->anonymous_offset_end[i] = 0;
			nvf->node->anonymous_offset_start[i] = UINT32_MAX;
		}

		// Initialize the anonymous mappings in this node. 
		nvf->node->anonymous_root_dirty_start = -1;
		nvf->node->anonymous_root_dirty_end = -1;
		nvf->node->isAnonMapReset = 1;
	}

	if(nvf->node->anonymous_offset_start[index] > start_offset) 			
		// Update the portion from which the dirty anon region starts. 
		nvf->node->anonymous_offset_start[index] = start_offset;
	if(*extent_length > len_to_write) {
		if(nvf->node->anonymous_offset_end[index] < (start_offset + len_to_write))
			// Update the portion till which the dirty anon region exists
			nvf->node->anonymous_offset_end[index] = (start_offset + len_to_write - 1) % ANON_MAX_MMAP_SIZE;
	}
	else {
		// It is a large write. So finish writing to this mmap. 
		if(nvf->node->anonymous_offset_end[index] < (start_offset + *extent_length))			       
			nvf->node->anonymous_offset_end[index] = (start_offset + *extent_length - 1) % ANON_MAX_MMAP_SIZE;
	}

	if(nvf->node->anonymous_root_dirty_start == -1) {
		// If this is the first append after fsync(). 
		nvf->node->anonymous_root_dirty_start = index;
		nvf->node->anonymous_root_dirty_end = index;
	} else {
		if(index > nvf->node->anonymous_root_dirty_end)
			// Update the end index of the pool. 
			nvf->node->anonymous_root_dirty_end = index;
		else if(index < nvf->node->anonymous_root_dirty_start)
			// Update the start index of the pool. 
			nvf->node->anonymous_root_dirty_start = index;
		else {
			//Do Nothing.
		}
	}

}


static int nvp_get_anonymous_mmap_address(struct NVFile *nvf, off_t offset, size_t len_to_write, size_t count, unsigned long *mmap_addr, off_t *offset_within_mmap, size_t *extent_length, int wr_lock,int cpuid, int iswrite)
{
	int index;
	unsigned long capacity = ANON_MAX_MMAP_SIZE;
	unsigned long *root = nvf->node->anonymous_root;
	unsigned long start_addr;
	off_t start_offset = offset;
	instrumentation_type nvnode_lock_time, anon_mem_queue_time;
	struct free_anon_pool *temp_free_pool_of_anonymous_mmaps;
	
	DEBUG("Get mmap address: offset 0x%lx, height %u\n", offset, height);
	DEBUG("root @ %p\n", root);

	//NVP_START_TIMING(lookup_t, lookup_time);

	// The index of the mmap in the global anonymous pool. Max number of entries = 1024. 
	index = start_offset / capacity;	
	DEBUG("index %d\n", index);
	if (index >= 1024 || root[index] == 0) {
		//NVP_END_TIMING(lookup_t, lookup_time);
		if(iswrite)
			// Have to get the mmap from the global anonymous pool. 
			goto not_found;
		else {
			// If it is a read, then the anonymous mmap must be found. Otherwise something is wrong. 
			ERROR("anon mmap not found\n");
			printf("%s: anon mmap not found\n", __func__);
			fflush(NULL);
			exit(100);

			//assert(0);
		}
	}
	// Anonymous mmap at that index is present for the file. So get the start address. 
	start_addr = root[index];
	
	DEBUG("addr 0x%lx\n", start_addr);
	// Get the offset in the mmap to which the memcpy must be performed. 
	start_offset = start_offset % capacity;
	
	//NVP_END_TIMING(lookup_t, lookup_time);

	if (IS_ERR(start_addr) || start_addr == 0) {
		MSG("ERROR!\n");
		fflush(NULL);
		assert(0);
	}

	// address we want to perform memcpy(). The start_offset is the offset with relation to node->true_length. 
	*mmap_addr = start_addr + start_offset;
	*offset_within_mmap = start_offset;
	// This gives how much free space is remaining in the current anonymous mmap. 
	*extent_length = ANON_MAX_MMAP_SIZE - start_offset;

	// The mmap for that index was not found. Performing mmap in this section. 	
	if (!wr_lock) {
		NVP_UNLOCK_NODE_RD(nvf, cpuid);
		START_TIMING(nvnode_lock_t, nvnode_lock_time);
		NVP_LOCK_NODE_WR(nvf);
		END_TIMING(nvnode_lock_t, nvnode_lock_time);
	}

	if(iswrite) {
		nvp_manage_anonymous_memory(nvf, extent_length, len_to_write, start_offset, index);		
	}

	if (!wr_lock && !iswrite) {
		NVP_UNLOCK_NODE_WR(nvf);
		NVP_LOCK_NODE_RD(nvf, cpuid);
	}

	return 0;

not_found:

	// The mmap for that index was not found. Performing mmap in this section. 	
	if (!wr_lock) {
		NVP_UNLOCK_NODE_RD(nvf, cpuid);
		START_TIMING(nvnode_lock_t, nvnode_lock_time);
		NVP_LOCK_NODE_WR(nvf);
		END_TIMING(nvnode_lock_t, nvnode_lock_time);
	}
       
	//NVP_START_TIMING(mmap_t, mmap_time);

	
	GLOBAL_LOCK_WR();

	START_TIMING(anon_mem_queue_t, anon_mem_queue_time);
	if( lfds711_queue_umm_dequeue(&qs, &qe) ) {
		temp_free_pool_of_anonymous_mmaps = LFDS711_QUEUE_UMM_GET_VALUE_FROM_ELEMENT( *qe );
		start_addr = temp_free_pool_of_anonymous_mmaps->addr;
		
		GLOBAL_UNLOCK_WR();

		//nvf->node->num_alloc_anon_mmaps++;
		nvf->node->usingAnonMem = 1;
		__atomic_fetch_add(&anon_mem_allocated, ANON_MAX_MMAP_SIZE, __ATOMIC_SEQ_CST);
		nvf->node->anon_mem_used += ANON_MAX_MMAP_SIZE;
		
	} else {
		GLOBAL_UNLOCK_WR();

		int max_perms = ((nvf->canRead) ? PROT_READ : 0) | 
			((nvf->canWrite) ? PROT_WRITE : 0);

		// There is no free mmap in the global anonymous mmap pool. Perform anonymous memory backed mmap here. 
		start_addr = (unsigned long) FSYNC_MMAP
			(
			 NULL,
			 ANON_MAX_MMAP_SIZE,
			 max_perms, //max_perms,
			 MAP_PRIVATE | MAP_ANONYMOUS,
			 -1, //fd_with_max_perms,
			 0
			 );

		num_mmap++;		
		
		nvf->node->usingAnonMem = 1;
		__atomic_fetch_add(&anon_mem_allocated, ANON_MAX_MMAP_SIZE, __ATOMIC_SEQ_CST);
		nvf->node->anon_mem_used += ANON_MAX_MMAP_SIZE;
	}
	END_TIMING(anon_mem_queue_t, anon_mem_queue_time);

	if (IS_ERR(start_addr) || start_addr == 0)
	{
		MSG("mmap failed for  %s, mmap count %d, addr %lu, errno is %lu\n",
		    strerror(errno), num_mmap, start_addr, errno);
		MSG("Open count %d, close count %d\n", num_open, num_close);
		nvf->posix = 1;
		fflush(NULL);
		assert(0);
	}
	
	
	root = nvf->node->anonymous_root;
	// Get the index of the mmap from the size of mmap and from the offset. 
	index = start_offset / capacity;
	root[index] = start_addr;       	
	start_offset = start_offset % capacity;
	
	//NVP_END_TIMING(insert_t, insert_time);
	
	*mmap_addr = start_addr + start_offset;
	*offset_within_mmap = start_offset;
	*extent_length = ANON_MAX_MMAP_SIZE - start_offset;

	if(iswrite) 
		nvp_manage_anonymous_memory(nvf, extent_length, len_to_write, start_offset, index);		
	
	if (!wr_lock && !iswrite) {
		NVP_UNLOCK_NODE_WR(nvf);
		NVP_LOCK_NODE_RD(nvf, cpuid);
	}

	return 0;
}

RETT_PREAD _nvp_read_beyond_true_length(INTF_PREAD, int wr_lock, int cpuid, struct NVFile *nvf)
{
	size_t len_to_read, extent_length, len_to_read_beyond_true_length, read_count;
	//timing_type do_pread_time, memcpyr_time, get_mmap_time;
	unsigned long mmap_addr;
	off_t read_offset_beyond_true_length, offset_within_mmap;

	//printf("%s: here beyond true length\n", __func__);
	read_count = 0;
	len_to_read = count;

	read_offset_beyond_true_length = offset - nvf->node->true_length_for_read;
	len_to_read_beyond_true_length = len_to_read;

	while (len_to_read > 0) {

		//NVP_START_TIMING(get_mmap_t, get_mmap_time);

		// Get the anonymous mmap address from which read is to be performed. 
		nvp_get_anonymous_mmap_address(nvf, read_offset_beyond_true_length, len_to_read_beyond_true_length, read_count, &mmap_addr, &offset_within_mmap, &extent_length, wr_lock, cpuid, 0);
		//NVP_END_TIMING(get_mmap_t, get_mmap_time);
		
		if(extent_length > len_to_read_beyond_true_length)
			extent_length = len_to_read_beyond_true_length;

		//NVP_START_TIMING(memcpyr_t, memcpyr_time);

		if(FSYNC_MEMCPY(buf, (char *)mmap_addr, extent_length) != buf) {
			printf("%s: memcpy read failed\n", __func__);
			fflush(NULL);
			assert(0);
		}
		//NVP_END_TIMING(memcpyr_t, memcpyr_time);

		num_memcpy_read++;
		memcpy_read_size += extent_length;

		len_to_read_beyond_true_length -= extent_length;
		read_offset_beyond_true_length += extent_length;
		read_count  += extent_length;
		buf += extent_length;
		len_to_read -= extent_length;
		
	}

	return read_count;	
}

RETT_PREAD _nvp_do_pread(INTF_PREAD, int wr_lock, int cpuid, struct NVFile *nvf)
{
	SANITYCHECKNVF(nvf);
	int ret;
	off_t read_offset_within_true_length = 0;
	size_t read_count, extent_length, read_count_beyond_true_length;
	size_t len_to_read_within_true_length;
	size_t posix_read = 0;
	unsigned long mmap_addr = 0;
	unsigned long bitmap_root = 0;
	off_t offset_within_mmap;
	ssize_t available_length = (nvf->node->length) - offset;
	void *buf2 = buf;

	if (UNLIKELY(!nvf->canRead)) {
		DEBUG("FD not open for reading: %i\n", file);
		errno = EBADF;
		return -1;
	}

	else if (UNLIKELY(offset < 0))
	{
		DEBUG("Requested read at negative offset (%li)\n", offset);
		errno = EINVAL;
		return -1;
	}

	if(nvf->aligned)
	{
		DEBUG("This read must be aligned.  Checking alignment.\n");

		if(UNLIKELY(available_length <= 0))
		{
			DEBUG("Actually there weren't any bytes available "
				"to read.  Bye! (length %li, offset %li, "
				"available_length %li)\n", nvf->node->length,
				offset, available_length);
			return 0;
		}

		if(UNLIKELY(count % 512))
		{
			DEBUG("cout is not aligned to 512 (count was %i)\n",
				count);

			errno = EINVAL;
			return -1;
		}
		if(UNLIKELY(offset % 512))
		{
			DEBUG("offset was not aligned to 512 (offset was %i)\n",
				offset);

			errno = EINVAL;
			return -1;
		}
		if(UNLIKELY(((long long int)buf & (512-1)) != 0))
		{
			DEBUG("buffer was not aligned to 512 (buffer was %p, "
				"mod 512=%i)\n", buf, (long long int)buf % 512);
			errno = EINVAL;
			return -1;
		}
	}

	ssize_t len_to_read = count;
	if (count > available_length)
	{
		len_to_read = available_length;
		DEBUG("Request read length was %li, but only %li bytes "
			"available. (filelen = %li, offset = %li, "
			"requested %li)\n", count, len_to_read,
			nvf->node->length, offset, count);
	}

	if(UNLIKELY( (len_to_read <= 0) || (available_length <= 0) ))
	{
		//NVP_END_TIMING(do_pread_t, do_pread_time);
		return 0; // reading 0 bytes is easy!
	}

	DEBUG("mmap is length %li, len_to_read is %li\n", nvf->node->maplength,
		len_to_read);

	SANITYCHECK(len_to_read + offset <= nvf->node->length);
		
	read_count = 0;

	/*
	 * if data to be read <= true_length_for_read, then it can be read from file backed mmap. Otherwise, it can be
	 * read from anonymous mmap 
	 * len_to_read_within_true_length = amount of data that can be read using file backed mmap. 
	 */
	read_offset_within_true_length = (offset > nvf->node->true_length_for_read) ? -1 : offset;
		
	if(read_offset_within_true_length == -1)
		len_to_read_within_true_length = 0;
	else {
		len_to_read_within_true_length = (len_to_read + offset > nvf->node->true_length_for_read) ? nvf->node->true_length_for_read - offset : len_to_read;
	}

	while (len_to_read_within_true_length > 0) {
		// Get the file backed mmap address from which the read is to be performed. 
		ret = nvp_get_mmap_address(nvf, read_offset_within_true_length, read_count, &mmap_addr, &bitmap_root, &offset_within_mmap, &extent_length, wr_lock,cpuid);
		//NVP_END_TIMING(get_mmap_t, get_mmap_time);

		DEBUG("Pread: get_mmap_address returned %d, length %llu\n",
			ret, extent_length);

		switch (ret) {
 		case 0: // Mmaped. Do memcpy.
			break;
		case 1: // Not mmaped. Calling Posix pread.
			posix_read = _nvp_fileops->PREAD(nvf->fd, buf2,
							 len_to_read_within_true_length, read_offset_within_true_length);

			read_count += posix_read;
			len_to_read -= posix_read;
			offset += posix_read;
			num_posix_read++;
			posix_read_size += posix_read;
		     
			goto next;
		default:
			break;
		}

		if (extent_length > len_to_read_within_true_length)
			extent_length = len_to_read_within_true_length;		
		
		//NVP_START_TIMING(memcpyr_t, memcpyr_time);

		if(FSYNC_MEMCPY(buf2, (const void * restrict)mmap_addr, extent_length) != buf2) {
			printf("%s: memcpy read failed\n", __func__);
			fflush(NULL);
			assert(0);
		}

		// Add the NVM read latency
#if NVM_DELAY
		perfmodel_add_delay(1, extent_length);
#endif		
		//NVP_END_TIMING(memcpyr_t, memcpyr_time);

		num_memcpy_read++;
		memcpy_read_size += extent_length;
		len_to_read -= extent_length;
		len_to_read_within_true_length -= extent_length;
		read_offset_within_true_length += extent_length;
		read_count  += extent_length;
		buf2 += extent_length;
		offset += extent_length;
	}

 next:
	if(!len_to_read) {
		return read_count;
	}

	// If we need to read from anonymous memory, call _nvp_read_beyond_true_length
	if(buf == buf2)
		buf2 += posix_read;
	read_count_beyond_true_length = _nvp_read_beyond_true_length(nvf->fd, buf2, len_to_read, offset, wr_lock, cpuid, nvf);
	read_count += read_count_beyond_true_length;
	
	return read_count;
}


/* 
 * _nvp_extend_write gets called whenever there is an append to a file. The write first goes to the
 * anonymous memory region through memcpy. During fsync() time, the data is copied non-temporally from
 * anonymous DRAM to the file. 
 */
RETT_PWRITE _nvp_extend_write(INTF_PWRITE, int wr_lock, int cpuid, struct NVFile *nvf) {

	size_t len_to_write, write_count;
	off_t write_offset;
	unsigned long mmap_addr;
	off_t offset_within_mmap, write_offset_wrt_true_length;
	size_t extent_length, extension_with_node_length;	
	instrumentation_type get_map_time, copy_data_time;
	//timing_type do_pwrite_time, memcpyw_time, get_mmap_time;

	// Increment counter for append
	_nvp_wr_extended++;
	
	DEBUG("Request write length %li will extend file. "
	      "(filelen=%li, offset=%li, count=%li, extension=%li)\n",
	      count, nvf->node->length, offset, count, extension);
		
	len_to_write = count;
	write_count = 0;
	write_offset = offset;
	
	extension_with_node_length = 0;
	
	while (len_to_write > 0) {
		// This is used mostly to check if the write is not an append, but is way beyond the length of the file. 
		write_offset_wrt_true_length = write_offset - nvf->node->true_length;

		// The address to perform the memcpy to is got from this function. 
		START_TIMING(get_map_t, get_map_time);

		nvp_get_anonymous_mmap_address(nvf, write_offset_wrt_true_length, len_to_write, write_count, &mmap_addr, &offset_within_mmap, &extent_length, wr_lock, cpuid, 1);
		if (extent_length > len_to_write)
			extent_length = len_to_write;

		if((extent_length + write_offset) > nvf->node->length)
			extension_with_node_length = extent_length + write_offset - nvf->node->length;

		nvf->node->length += extension_with_node_length;

		END_TIMING(get_map_t, get_map_time);

		NVP_UNLOCK_NODE_WR(nvf);
		NVP_LOCK_NODE_RD(nvf, cpuid);
		
		// Write to anonymous DRAM. No dirty tracking to be performed here. 

		START_TIMING(copy_data_t, copy_data_time);

		if(FSYNC_MEMCPY((char *)mmap_addr, buf, extent_length) != (char *)mmap_addr) {
			printf("%s: memcpy failed\n", __func__);
			fflush(NULL);
			assert(0);
		}
		
		num_memcpy_write++;
		memcpy_write_size += extent_length;

		len_to_write -= extent_length;
		write_offset += extent_length;
		write_count  += extent_length;
		buf += extent_length;			

		END_TIMING(copy_data_t, copy_data_time);
	}

	return write_count;
}

RETT_PWRITE _nvp_do_pwrite(INTF_PWRITE, int wr_lock, int cpuid, struct NVFile *nvf)
{
	CHECK_RESOLVE_FILEOPS(_nvp_);
	instrumentation_type get_map_time, copy_data_time, write_syscall_time, nvp_do_pwrite_time, nvp_do_pwrite_overwrite_time, nvp_do_pwrite_append_time;	

	int ret;
        off_t write_offset, offset_within_mmap;
	size_t write_count, extent_length;
	size_t posix_write;
	unsigned long mmap_addr = 0;
	unsigned long bitmap_root = 0;
	uint64_t extendFileReturn;

	START_TIMING(nvp_do_pwrite_t, nvp_do_pwrite_time);

	DEBUG("_nvp_do_pwrite\n");
		
	_nvp_wr_total++;

	SANITYCHECKNVF(nvf);
	
	if(UNLIKELY(!nvf->canWrite)) {
		DEBUG("FD not open for writing: %i\n", file);
		errno = EBADF;

		END_TIMING(nvp_do_pwrite_t, nvp_do_pwrite_time);
		return -1;
	}

	if(nvf->aligned)
	{
		DEBUG("This write must be aligned.  Checking alignment.\n");
		if(UNLIKELY(count % 512))
		{
			DEBUG("count is not aligned to 512 (count was %li)\n",
				count);
			errno = EINVAL;
			
			END_TIMING(nvp_do_pwrite_t, nvp_do_pwrite_time);
			return -1;
		}
		if(UNLIKELY(offset % 512))
		{
			DEBUG("offset was not aligned to 512 "
				"(offset was %li)\n", offset);
			errno = EINVAL;

			END_TIMING(nvp_do_pwrite_t, nvp_do_pwrite_time);
			return -1;
		}

		if(UNLIKELY(((long long int)buf & (512-1)) != 0))
		{
			DEBUG("buffer was not aligned to 512 (buffer was %p, "
				"mod 512 = %li)\n", buf,
				(long long int)buf % 512);
			errno = EINVAL;

			END_TIMING(nvp_do_pwrite_t, nvp_do_pwrite_time);
			return -1;
		}
	}

	if(nvf->append)
	{
		DEBUG("this fd (%i) is O_APPEND; setting offset from the "
			"passed value (%li) to the end of the file (%li) "
			"prior to writing anything\n", nvf->fd, offset,
			nvf->node->length);
		offset = nvf->node->length;
	}

	ssize_t len_to_write;
	ssize_t extension_with_true_length, extension_with_read_length;
	DEBUG("time for a Pwrite. file length %li, offset %li, extension %li, count %li\n", nvf->node->length, offset, extension, count);

	len_to_write = count;

	SANITYCHECK(nvf->valid);
	SANITYCHECK(nvf->node != NULL);
	SANITYCHECK(buf > 0);
	SANITYCHECK(count >= 0);

	write_count = 0;
	write_offset = offset;
		
	while (len_to_write > 0) {

		// Find if we need to write to anonymous DRAM or file backed DRAM. 
		extension_with_true_length = len_to_write + write_offset - (nvf->node->true_length);
		extension_with_read_length = len_to_write + write_offset - (nvf->node->true_length_for_read);
		
		if((nvf->node->true_length_for_read < nvf->node->true_length) && extension_with_read_length > 0)
			nvf->node->true_length_for_read = nvf->node->true_length;

		if(write_offset < nvf->node->true_length) {

			START_TIMING(nvp_do_pwrite_overwrite_t, nvp_do_pwrite_overwrite_time);

			// Get the file backed mmap address to which the write is to be performed. 

			START_TIMING(get_map_t, get_map_time);

			ret = nvp_get_mmap_address(nvf, write_offset, write_count, &mmap_addr, &bitmap_root, &offset_within_mmap, &extent_length, wr_lock, cpuid);

			END_TIMING(get_map_t, get_map_time);

			DEBUG("Pwrite: get_mmap_address returned %d, length %llu\n",
			      ret, extent_length);

			switch (ret) {
			case 0: // Mmaped. Do memcpy.
				break;
			case 1: // Not mmaped. Calling Posix pread. 

				num_mfence++;
				if(extension_with_true_length > 0) {
					START_TIMING(write_syscall_t, write_syscall_time);
					posix_write = _nvp_fileops->PWRITE(file, buf,
									   (nvf->node->true_length - write_offset) , write_offset);
					END_TIMING(write_syscall_t, write_syscall_time);

				}
				else {
					START_TIMING(write_syscall_t, write_syscall_time);
					posix_write = _nvp_fileops->PWRITE(file, buf, len_to_write, write_offset);
					END_TIMING(write_syscall_t, write_syscall_time);
				}
				num_mfence++;
				num_write_nontemporal++;
				non_temporal_write_size += posix_write;
				write_count += posix_write;
				num_posix_write++;
				posix_write_size += posix_write;
				len_to_write -= posix_write;
				buf += posix_write;
				write_offset += posix_write;

				END_TIMING(nvp_do_pwrite_overwrite_t, nvp_do_pwrite_overwrite_time);
				continue;
			default:
				break;
			}

			if(extent_length > len_to_write)
				extent_length = len_to_write;
			if(extent_length > nvf->node->true_length - write_offset)
				extent_length = (nvf->node->true_length - write_offset);
			
			// The write is performed to file backed mmap
#if NON_TEMPORAL_WRITES

			START_TIMING(copy_data_t, copy_data_time);
			
			_mm_mfence();
			num_mfence++;
			if(memmove_nodrain_movnt_granularity((char *)mmap_addr, buf, extent_length) == NULL) {
				DEBUG_FILE("%s: non-temporal memcpy failed\n", __func__);

				END_TIMING(nvp_do_pwrite_overwrite_t, nvp_do_pwrite_overwrite_time);
				END_TIMING(nvp_do_pwrite_t, nvp_do_pwrite_time);
				assert(0);
			}

#if NVM_DELAY
			perfmodel_add_delay(0, extent_length);
#endif
			_mm_mfence();
			num_mfence++;
			num_write_nontemporal++;
			non_temporal_write_size += extent_length;
			
			END_TIMING(copy_data_t, copy_data_time);

#else //NON_TEMPORAL_WRITES
			
			if(FSYNC_MEMCPY((char *)mmap_addr, buf, extent_length) != (char *)mmap_addr) {
				DEBUG_FILE("%s: memcpy failed\n", __func__);

				END_TIMING(nvp_do_pwrite_overwrite_t, nvp_do_pwrite_overwrite_time);
			        END_TIMING(nvp_do_pwrite_loop_t, nvp_do_pwrite_loop_time);
				END_TIMING(nvp_do_pwrite_t, nvp_do_pwrite_time);
				assert(0);
			}
			// Add the NVM write latency
#if NVM_DELAY
			perfmodel_add_delay(0, extent_length);
#endif
			num_write_temporal++;
			temporal_write_size += extent_length;
#if DIRTY_TRACKING			
			// The dirty tracking tree tracks the addresses of the cachelines that have been dirtied. 
			modifyBmap((struct merkleBtreeNode *)bitmap_root, offset_within_mmap, extent_length);
#endif //DIRTY_TRACKING
#endif //NON_TEMPORAL_WRITES		

			num_memcpy_write++;
			memcpy_write_size += extent_length;
			len_to_write -= extent_length;
			write_offset += extent_length;
			write_count  += extent_length;
			buf += extent_length;

			END_TIMING(nvp_do_pwrite_overwrite_t, nvp_do_pwrite_overwrite_time);
				
		} else {

			START_TIMING(nvp_do_pwrite_append_t, nvp_do_pwrite_append_time);

			// If we need to append data, we should call _nvp_extend_write to write to anonymous mmap. 
#if ANON_MEM_HANDLE
			extendFileReturn = _nvp_extend_write(file, buf, len_to_write, write_offset, wr_lock, cpuid, nvf);
#else

			START_TIMING(write_syscall_t, write_syscall_time);

			extendFileReturn = _nvp_fileops->PWRITE(file, buf, len_to_write, write_offset);
			END_TIMING(write_syscall_t, write_syscall_time);

			num_posix_write++;
			posix_write_size += extendFileReturn;
			num_mfence++;
			num_write_nontemporal++;
			_nvp_wr_extended++;
			nvf->node->length += extendFileReturn;
			nvf->node->true_length += extendFileReturn;
			nvf->node->true_length_for_read += extendFileReturn;
			
#endif
			len_to_write -= extendFileReturn;
			write_count += extendFileReturn;
			write_offset += extendFileReturn;
			buf += extendFileReturn;
			
			END_TIMING(nvp_do_pwrite_append_t, nvp_do_pwrite_append_time);
		}
	}
				
	/*
	DEBUG_FILE("About to return from _nvp_PWRITE with ret val %li.  file len: "
	      "%li, file off: %li, map len: %li, node %p\n",
	      count, nvf->node->length, nvf->offset,
	      nvf->node->maplength, nvf->node);
	*/
	
	END_TIMING(nvp_do_pwrite_t, nvp_do_pwrite_time);
	return write_count;
}

void _nvp_test_invalidate_node(struct NVFile* nvf)
{
	struct NVNode* node = nvf->node;

	DEBUG("munmapping temporarily diabled...\n"); // TODO

	return;

	SANITYCHECK(node!=NULL);

	pthread_spin_lock(&node_lookup_lock[(int) (pthread_self() % NUM_NODE_LISTS)]);
	NVP_LOCK_NODE_WR(nvf);
	node->reference--;
	NVP_UNLOCK_NODE_WR(nvf);
	if (node->reference == 0) {
		NVP_LOCK_NODE_WR(nvf);
		int index = nvf->serialno % 1024;
		_nvp_ino_lookup[index] = 0;
		// FIXME: Also munmap?
		nvp_cleanup_node(nvf->node, 0, 1);
		node->serialno = 0;
		NVP_UNLOCK_NODE_WR(nvf);
	}
	pthread_spin_unlock(&node_lookup_lock[(int) (pthread_self() % NUM_NODE_LISTS)]);

}

RETT_SEEK64 _nvp_do_seek64(INTF_SEEK64, struct NVFile *nvf)
{
	CHECK_RESOLVE_FILEOPS(_nvp_);

	DEBUG("_nvp_do_seek64\n");

	//struct NVFile* nvf = &_nvp_fd_lookup[file];
	
	DEBUG("_nvp_do_seek64: file len %li, map len %li, current offset %li, "
		"requested offset %li with whence %li\n", 
		nvf->node->length, nvf->node->maplength, *nvf->offset,
		offset, whence);

	switch(whence)
	{
		case SEEK_SET:
			if(offset < 0)
			{
				DEBUG("offset out of range (would result in "
					"negative offset).\n");
				errno = EINVAL;
				return -1;
			}
			*(nvf->offset) = offset ;
			//if(offset == 0)
				//INITIALIZE_TIMER();
			return *(nvf->offset);

		case SEEK_CUR:
			if((*(nvf->offset) + offset) < 0)
			{
				DEBUG("offset out of range (would result in "
					"negative offset).\n");
				errno = EINVAL;
				return -1;
			}
			*(nvf->offset) += offset ;
			return *(nvf->offset);

		case SEEK_END:
			if( nvf->node->length + offset < 0 )
			{
				DEBUG("offset out of range (would result in "
					"negative offset).\n");
				errno = EINVAL;
				return -1;
			}

			*(nvf->offset) = nvf->node->length + offset;
			return *(nvf->offset);

		default:
			DEBUG("Invalid whence parameter.\n");
			errno = EINVAL;
			return -1;
	}
     
	assert(0); // unreachable
	return -1;
}

static ssize_t _nvp_check_read_size_valid(size_t count)
{ 
	if(count == 0)
	{
		DEBUG("Requested a read of 0 length.  No problem\n");
		return 0;
	}
	else if(count < 0)
	{
		DEBUG("Requested read of negative bytes (%li)\n", count);
		errno = EINVAL;
		return -1;
	}

	return count;
}

static ssize_t _nvp_check_write_size_valid(size_t count)
{
	if(count == 0)
	{
		DEBUG("Requested a write of 0 bytes.  No problem\n");
		return 0;
	}

	if(((signed long long int)count) < 0)
	{
		DEBUG("Requested a write of %li < 0 bytes.\n",
			(signed long long int)count);
		errno = EINVAL;
		return -1;
	}

	return count;
}

/* ========================== POSIX API methods =========================== */


RETT_MKSTEMP _nvp_MKSTEMP(INTF_MKSTEMP)
{
	CHECK_RESOLVE_FILEOPS(_nvp_);

	char* suffix = file + strlen(file) - 6; // char* suffix = strstr(file, "XXXXXX");

	DEBUG("Called _nvp_mkstemp with template %s; making a new filename...\n", file);

	if(suffix == NULL) {
		DEBUG("Invalid template string (%s) passed to mkstemp\n", file);
		errno = EINVAL;

		return -1;
	}

	// generate a valid file name
	int i;
	int success = 0;
	for(i=0; i<1000000; i++)
	{
		sprintf(suffix, "%.6i", i);

		int fs = access(file, F_OK);

		if(fs == 0) { // file doesn't exist; we're good
			success = 1;
			break;
		}
	}

	if(!success) {
		DEBUG("No available file names!\n");
		return -1;
	}

	DEBUG("Generated filename %s.  Calling (regular) open...\n", file);
	
	return open(file, O_CREAT | O_EXCL, 0600);
}

RETT_CLOSE _nvp_REAL_CLOSE(INTF_CLOSE, ino_t serialno, int async_file_closing) {

	RETT_CLOSE result;
	CHECK_RESOLVE_FILEOPS(_nvp_);
	instrumentation_type node_lookup_lock_time, nvnode_lock_time, close_syscall_time, copy_to_anon_pool_time, copy_to_mmap_cache_time, give_up_node_time;
	int cpuid;
	int node_list_idx;
		
	DEBUG("_nvp_CLOSE(%i)\n", file);
	
	// Get the struct NVFile from the file descriptor

	//START_TIMING(close_t, close_time);
	
	struct NVFile* nvf = &_nvp_fd_lookup[file];
	node_list_idx = nvf->node->free_list_idx;

	num_close++;
	
	if (nvf->posix) {
		nvf->valid = 0;
		nvf->posix = 0;
		NVP_LOCK_NODE_WR(nvf);
		nvf->node->reference--;
		NVP_UNLOCK_NODE_WR(nvf);
		if (nvf->node->reference == 0) {
			nvf->node->serialno = 0;
			int index = nvf->serialno % 1024;
			_nvp_ino_lookup[index] = 0;
			push_in_stack(1, 0, nvf->node->index_in_free_list, node_list_idx);
		}
		nvf->serialno = 0;
		DEBUG("Call posix CLOSE for fd %d\n", nvf->fd);
		result = _nvp_fileops->CLOSE(CALL_CLOSE);

		//END_TIMING(close_t, close_time);
		return result;
	}

#if ANON_MEM_HANDLE	
	cpuid = GET_CPUID();
	fsync_flush_on_fsync(nvf, cpuid, 1, 0);
#endif
	
	//PRINT_TIME();
	//msg_nvp_print_io_stats();
	
	/* 
	 * nvf->node->reference contains how many threads have this file open. 
	 */

#if PRINT_DEBUG_MSGS
	printf("%s: acquiring node lookup lock, index = %d, thread = %lu\n", __func__, node_list_idx, pthread_self());
	fflush(NULL);
#endif

	//START_TIMING(node_lookup_lock_t, node_lookup_lock_time);
	pthread_spin_lock(&node_lookup_lock[node_list_idx]);

#if PRINT_DEBUG_MSGS
	printf("%s: acquired node lookup lock, index = %d\n", __func__, node_list_idx);
	fflush(NULL);	
#endif
	
	if(nvf->valid == 0) {
		pthread_spin_unlock(&node_lookup_lock[node_list_idx]);
		//END_TIMING(close_t, close_time);
		return -1;
	}
	if(nvf->node->reference <= 0) {
		pthread_spin_unlock(&node_lookup_lock[node_list_idx]);
		//END_TIMING(close_t, close_time);
		return -1;
	}
	if(nvf->node->serialno != serialno) {
		pthread_spin_unlock(&node_lookup_lock[node_list_idx]);
		//END_TIMING(close_t, close_time);
		return -1;
	}
	
	//START_TIMING(nvnode_lock_t, nvnode_lock_time);
	NVP_LOCK_NODE_WR(nvf);
	//END_TIMING(nvnode_lock_t, nvnode_lock_time);
	nvf->node->reference--;
	NVP_UNLOCK_NODE_WR(nvf);

	if (nvf->node->reference == 0) {
		nvf->node->serialno = 0;				
		//nvp_add_to_inode_mapping(nvf->node, nvf->serialno);
		//START_TIMING(give_up_node_t, give_up_node_time);
		push_in_stack(1, 0, nvf->node->index_in_free_list, node_list_idx);
		//END_TIMING(give_up_node_t, give_up_node_time);
	}		

	if (async_file_closing) {
		nvf->node->async_file_close = 1;
	}

	pthread_spin_unlock(&node_lookup_lock[node_list_idx]);
	//END_TIMING(node_lookup_lock_t, node_lookup_lock_time);
		
	NVP_LOCK_FD_WR(nvf);
	NVP_CHECK_NVF_VALID_WR(nvf);	
	//START_TIMING(nvnode_lock_t, nvnode_lock_time);
	NVP_LOCK_NODE_WR(nvf);
	//END_TIMING(nvnode_lock_t, nvnode_lock_time);

	// setting valid to 0 means that this fd is not open. So can be used for a subsequent open of same or different file.
	if(nvf->valid == 0) {
		NVP_UNLOCK_NODE_WR(nvf);
		NVP_UNLOCK_FD_WR(nvf);
		//END_TIMING(close_t, close_time);
		return -1;
	}
	if(nvf->node->reference < 0) {
		NVP_UNLOCK_NODE_WR(nvf);
		NVP_UNLOCK_FD_WR(nvf);
		//END_TIMING(close_t, close_time);
		return -1;
	}
	if(nvf->serialno != serialno) {
		NVP_UNLOCK_NODE_WR(nvf);
		NVP_UNLOCK_FD_WR(nvf);
		//END_TIMING(close_t, close_time);
		return -1;
	}

	nvf->valid = 0;
	// Checking the reference bit means checking whether this file is not open by any thread.

	if (nvf->node->reference == 0) {

		//START_TIMING(copy_to_mmap_cache_t, copy_to_mmap_cache_time);
		nvp_add_to_inode_mapping(nvf->node, nvf->serialno);
		//END_TIMING(copy_to_mmap_cache_t, copy_to_mmap_cache_time);
		nvf->node->backup_serialno = 0;
		
		int index = nvf->serialno % 1024;
		_nvp_ino_lookup[index] = 0;
		DEBUG("Close Cleanup node for %d\n", file);

		GLOBAL_LOCK_WR();
		if(nvf->node->anonymous_root) {
			//START_TIMING(copy_to_anon_pool_t, copy_to_anon_pool_time);

			nvp_transfer_to_free_anonymous_pool(nvf->node->anonymous_root, nvf->node->anonymous_offset_start, nvf->node->anonymous_offset_end, nvf->node->anonymous_root_dirty_start, nvf->node->anonymous_root_dirty_end, nvf->node);

			//END_TIMING(copy_to_anon_pool_t, copy_to_anon_pool_time);
		}
		nvf->node->async_file_close = 0;
		GLOBAL_UNLOCK_WR();
	
		nvp_cleanup_node(nvf->node, 0, 0);

		// Copy all the mmap()s of this node to the global mmap() cache. 
		// reset the NVNode, since this file is not open by any thread. 
	}
	nvf->serialno = 0;

	NVP_UNLOCK_NODE_WR(nvf);
	NVP_UNLOCK_FD_WR(nvf);

	// close() system call of the file is done here. 
	//START_TIMING(close_syscall_t, close_syscall_time);
	result = _nvp_fileops->CLOSE(CALL_CLOSE);
	//END_TIMING(close_syscall_t, close_syscall_time);
	
	//END_TIMING(close_t, close_time);
	return result;

}

RETT_OPEN _nvp_OPEN(INTF_OPEN)
{	

	int result;	
#if BG_CLOSING
	int closed_filedesc = -1, fd = -1, hash_index = -1;
#if SEQ_LIST || RAND_LIST
	struct ClosedFiles *clf = NULL;
#else // SEQ_LIST || RAND_LIST	
	struct InodeClosedFile *tbl = NULL;
#endif // SEQ_LIST || RAND_LISTOB
#endif // BG_CLOSING
	instrumentation_type open_time, clf_lock_time, nvnode_lock_time;

	START_TIMING(open_t, open_time);
	
#if PASS_THROUGH_CALLS
	num_open++;
	if (FLAGS_INCLUDE(oflag,O_CREAT))
	{
		va_list arg;
		va_start(arg, oflag);
		int mode = va_arg(arg, int);
		va_end(arg);
		
		// Open system call is done here  
		result = _nvp_fileops->OPEN(path, oflag & (~O_APPEND), mode);
	} else { 
		result = _nvp_fileops->OPEN(path, oflag & (~O_APPEND));
	}
	
	if(result<0)
	{
		DEBUG("_nvp_OPEN->%s_OPEN failed: %s\n",
			_nvp_fileops->name, strerror(errno));

		DEBUG_FILE("%s: open failed!, num_files_closed = %lu, \n", __func__, num_files_closed);		

		END_TIMING(open_t, open_time);
		return result;
	}	

	END_TIMING(open_t, open_time);
	return result;
#endif // PASS_THROUGH_CALLS

	CHECK_RESOLVE_FILEOPS(_nvp_);
	
	if(path==NULL) {
		DEBUG("Invalid path.\n");
		errno = EINVAL;
		END_TIMING(open_t, open_time);
		return -1;
	}

	DEBUG("_nvp_OPEN(%s)\n", path);
	num_open++;
	
	DEBUG("Attempting to _nvp_OPEN the file \"%s\" with the following "
		"flags (0x%X): ", path, oflag);
	
	/*
	 *  Print all the flags passed to open() 
	 */


	if (FLAGS_INCLUDE(oflag,O_WRONLY)) {
		oflag |= O_RDONLY;
	} 

	if((oflag&O_RDWR)||((oflag&O_RDONLY)&&(oflag&O_WRONLY))) {
		DEBUG_P("O_RDWR ");
	} else if(FLAGS_INCLUDE(oflag, O_RDONLY)) {
		DEBUG_P("O_RDONLY ");
	}

/*
  else if(FLAGS_INCLUDE(oflag,O_WRONLY)) {
  DEBUG_P("O_WRONLY ");
*/
	DUMP_FLAGS(oflag,O_APPEND);
	DUMP_FLAGS(oflag,O_CREAT);
	DUMP_FLAGS(oflag,O_TRUNC);
	DUMP_FLAGS(oflag,O_EXCL);
	DUMP_FLAGS(oflag,O_SYNC);
	DUMP_FLAGS(oflag,O_ASYNC);
	DUMP_FLAGS(oflag,O_DSYNC);
	DUMP_FLAGS(oflag,O_FSYNC);
	DUMP_FLAGS(oflag,O_RSYNC);
	DUMP_FLAGS(oflag,O_NOCTTY);
	DUMP_FLAGS(oflag,O_NDELAY);
	DUMP_FLAGS(oflag,O_NONBLOCK);
	DUMP_FLAGS(oflag,O_DIRECTORY);
	DUMP_FLAGS(oflag,O_LARGEFILE);
	DUMP_FLAGS(oflag,O_NOATIME);
	DUMP_FLAGS(oflag,O_DIRECT);
	DUMP_FLAGS(oflag,O_NOFOLLOW);
	DEBUG_P("\n");

	struct stat file_st;	
	
	// Initialize NVNode 
	struct NVNode* node = NULL;

#if BG_CLOSING		
	if (async_close_enable) {
		if(num_files_closed >= 800 || (anon_mem_closed_files >= ((5ULL) * 1024 * 1024 * 1024))) {
			ASYNC_CLOSING = 0;
			checkAndActivateBgThread();		
		}
	}
#endif // BG_CLOSING
	
	/*
	 * If creation of the file is involved, 3 parameters must be passed to open(). Otherwise, 2 parameters must be passed
	 */	

	if (FLAGS_INCLUDE(oflag,O_CREAT))
	{
		va_list arg;
		va_start(arg, oflag);
		int mode = va_arg(arg, int);
		va_end(arg);
		
		// Open system call is done here  
		result = syscall(334, path, oflag & (~O_APPEND), mode, &file_st.st_ino, &file_st.st_size);
		//result = _nvp_fileops->OPEN(path, oflag & (~O_APPEND), mode);
	} else { 
		result = syscall(334, path, oflag & (~O_APPEND), 0666, &file_st.st_ino, &file_st.st_size);

		//result = _nvp_fileops->OPEN(path, oflag & (~O_APPEND));

		//if (!S_ISREG(*inode_mode) && !S_ISBLK(*inode_mode)) {
		//	*inode_mode = 50;
		//	END_TIMING(open_t, open_time);
		//	return result;
		//}
	}
	
	if(result<0)
	{
		DEBUG("_nvp_OPEN->%s_OPEN failed: %s\n",
			_nvp_fileops->name, strerror(errno));

		DEBUG_FILE("_nvp_OPEN->%s_OPEN failed: %s\n",
			_nvp_fileops->name, strerror(errno));		
		END_TIMING(open_t, open_time);

		return result;
	}	
	
	SANITYCHECK(&_nvp_fd_lookup[result] != NULL);	

	struct NVFile* nvf = NULL;

#if BG_CLOSING	
	if (async_close_enable)
		checkAndActivateBgThread();
	GLOBAL_LOCK_CLOSE_WR();
	hash_index = file_st.st_ino % TOTAL_CLOSED_INODES;
#if SEQ_LIST || RAND_LIST
	clf = &_nvp_closed_files[hash_index];

#if PRINT_CONTENTION_MSGS		
	printf("%s: acquiring lock for clf entry = %d, thread id = %lu\n", __func__, hash_index, pthread_self());
	fflush(NULL);
#endif

	//START_TIMING(clf_lock_t, clf_lock_time);
	LRU_NODE_LOCK_WR(clf);
	//END_TIMING(clf_lock_t, clf_lock_time);

	fd = remove_from_seq_list_hash(clf, file_st.st_ino);
#else // SEQ_LIST || RAND_LIST
	tbl = &inode_to_closed_file[hash_index];
	NVP_LOCK_HASH_TABLE_WR(tbl);	
	fd = remove_from_lru_list_hash(file_st.st_ino, 0);
#endif // SEQ_LIST || RAND_LIST		
	if(fd >= 0) {
		if ((oflag & O_RDWR) || FLAGS_INCLUDE(oflag, O_RDONLY)) {	
			num_close++;
			closed_filedesc = fd;			
			__atomic_fetch_sub(&num_files_closed, 1, __ATOMIC_SEQ_CST);
#if SEQ_LIST || RAND_LIST

#if PRINT_CONTENTION_MSGS		
		MSG("%s: releasing lock for clf entry = %d, thread id = %lu\n", __func__, hash_index, pthread_self());
#endif
			LRU_NODE_UNLOCK_WR(clf);
#else // SEQ_LIST || RAND_LIST
			NVP_UNLOCK_HASH_TABLE_WR(tbl);
#endif // SEQ_LIST || RAND_LIST
			GLOBAL_UNLOCK_CLOSE_WR();
			
			_nvp_fileops->CLOSE(result);			
			result = closed_filedesc;
			nvf = &_nvp_fd_lookup[result];			
			node = nvf->node;
			__atomic_fetch_sub(&anon_mem_closed_files, nvf->node->anon_mem_used, __ATOMIC_SEQ_CST);
			NVP_LOCK_FD_WR(nvf);
			//START_TIMING(nvnode_lock_t, nvnode_lock_time);
			NVP_LOCK_NODE_WR(nvf);
		        //END_TIMING(nvnode_lock_t, nvnode_lock_time);
			nvf->valid = 0;			
			goto initialize_nvf;
		}
	}

#if SEQ_LIST || RAND_LIST
	LRU_NODE_UNLOCK_WR(clf);
#else // SEQ_LIST || RAND_LIST
	NVP_UNLOCK_HASH_TABLE_WR(tbl);
#endif // SEQ_LIST || RAND_LIST
	GLOBAL_UNLOCK_CLOSE_WR();
	
#endif	// BG_CLOSING

	// Retrieving the NVFile corresponding to the file descriptor returned by open() system call

	nvf = &_nvp_fd_lookup[result];
	
	DEBUG_FILE("_nvp_OPEN succeeded for path %s: fd %i returned. "
		"filling in file info\n", path, result);

	NVP_LOCK_FD_WR(nvf);
	
	// Check if the file descriptor is already open. If open, something is wrong and return error
	if(nvf->valid)
	{
		DEBUG_FILE("There is already a file open with that FD (%i)!\n", result);
		assert(0);
		END_TIMING(open_t, open_time);
		return result;
	}

	/* 
	 * stat() system call is done here. Stat() is required to get the inode number and the size of the file. The inode number is 
	 * required to see if the NVNode for the particular file is already present (in case of other threads having this file open
	 * already, by checking through the _nvp_ino_lookup[] global array). The size of the file is stored in 
	 * NVNode (nvf->node->length)	   
	 */
	/*
	if(stat(path, &file_st))
	{
		ERROR("Failed to stat opened file %s: %s\n",
			path, strerror(errno));
		assert(0);
	}
	*/
	
	/*
	 * NVNode is retrieved here. Keeping this check because in quill it was required. Not necessary in Ledger
	 */
	if(node == NULL)
	{
		//DEBUG("We created the file.  Let's check and make sure "
		//	"someone else hasn't already created the node.\n");

		// Find or allocate a NVNode

		node = nvp_get_node(path, &file_st, result);
		NVP_LOCK_WR(node->lock);
	}

#if BG_CLOSING
 initialize_nvf:
#endif // BG_CLOSING
	nvf->fd = result;	
	nvf->node = node;
	nvf->posix = 0;
	nvf->serialno = file_st.st_ino;
	
	/* 
	 * Write the entry of this file into the global inode number struct. This contains the fd of the thread that first 
	 * opened this file. 
	 */
	/*
	index = nvf->serialno % 1024;
	if (_nvp_ino_lookup[index] == 0)
		_nvp_ino_lookup[index] = result;
	*/
	
	// Set FD permissions
	if((oflag & O_RDWR)||((oflag & O_RDONLY) && (oflag & O_WRONLY))) {
		DEBUG("oflag (%i) specifies O_RDWR for fd %i\n", oflag, result);
		nvf->canRead = 1;
		nvf->canWrite = 1;
	} else if(oflag&O_WRONLY) {
		DEBUG("oflag (%i) specifies O_WRONLY for fd %i\n", oflag,
			result);
		//MSG("File %s is opened O_WRONLY.\n", path);
		//MSG("Does not support mmap, use posix instead.\n");
		//nvf->posix = 1;
		nvf->canRead = 1;
		nvf->canWrite = 1;
		//NVP_UNLOCK_NODE_WR(nvf);
		//NVP_UNLOCK_FD_WR(nvf);
		//END_TIMING(open_t, open_time);
		//return nvf->fd;
	} else if(FLAGS_INCLUDE(oflag, O_RDONLY)) {
		DEBUG("oflag (%i) specifies O_RDONLY for fd %i\n",
			oflag, result);
		nvf->canRead = 1;
		nvf->canWrite = 0;
	} else {
		DEBUG_FILE("File permissions don't include read or write!\n");
		nvf->canRead = 0;
		nvf->canWrite = 0;		
		assert(0);
	}
	
	if(FLAGS_INCLUDE(oflag, O_APPEND)) {
		nvf->append = 1;
	} else {
		nvf->append = 0;
	}

	SANITYCHECK(nvf->node != NULL);

	if(FLAGS_INCLUDE(oflag, O_TRUNC) && nvf->node->length)
	{
		DEBUG("We just opened a file with O_TRUNC that was already "
			"open with nonzero length %li.  Updating length.\n",
			nvf->node->length);
		nvf->node->length = 0;
	}

	nvf->posix = 0;
	nvf->debug = 0;

	/* This is a nasty workaround for FIO */
	if (path[0] == '/' && path[1] == 's'
			&& path[2] == 'y' && path[3] == 's') {
		nvf->posix = 1;
		DEBUG_FILE("A Posix Path: %s\n", path);
	}

	/* For BDB log file, workaround the fdsync issue */
	if (path[29] == 'l' && path[30] == 'o' && path[31] == 'g') {
		nvf->debug = 1;
	}

	nvf->offset = (size_t*)calloc(1, sizeof(int));
	*nvf->offset = 0;

	if(FLAGS_INCLUDE(oflag, O_DIRECT) && (DO_ALIGNMENT_CHECKS)) {
		nvf->aligned = 1;
	} else {
		nvf->aligned = 0;
	}

	nvf->valid = 1;
	
	NVP_UNLOCK_NODE_WR(nvf);
	NVP_UNLOCK_FD_WR(nvf);
	
	errno = 0;
	END_TIMING(open_t, open_time);

	DEBUG_FILE("%s: Returning fd = %d\n", __func__, nvf->fd);
	return nvf->fd;
}

#ifdef TRACE_FP_CALLS

/*
RETT_FDOPEN _nvp_FDOPEN(INTF_FDOPEN) {
	//Do Nothing.
}
*/

RETT_FOPEN _nvp_FOPEN(INTF_FOPEN)
{
	int oflag = 0;
	int fd = -1;
	RETT_FOPEN fp = NULL;
	

#if PASS_THROUGH_CALLS
	num_open++;
	fp = _nvp_fileops->FOPEN(path, mode);
	return fp;
#endif // PASS_THROUGH_CALLS
	
	if (!strcmp(mode,"w+") || !strcmp(mode,"a+")) {
		oflag |= O_RDWR;
		oflag |= O_CREAT;
	}
	else if (!strcmp(mode,"r+"))
		oflag |= O_RDWR;
	else if (!strcmp(mode, "w") || !strcmp(mode, "a")) {
		oflag |= O_WRONLY;
		oflag |= O_CREAT;
	} else if (!strcmp(mode, "r"))
		oflag |= O_RDONLY;
	else {
		ERROR("%s: unknown mode %s\n", __func__, mode);
		assert(0);
	}
	
	if (mode[0] == 'a') 
		oflag |= O_APPEND;

	if (FLAGS_INCLUDE(oflag,O_CREAT)) 
		fd = _nvp_OPEN(path, oflag, 0666);
	else
		fd = _nvp_OPEN(path, oflag);

	fp = fdopen(fd, mode);
	if (!fp) {
		ERROR("%s: fdopen failed! error = %s, fd = %d, mode = %s\n", __func__, strerror(errno), fd, mode);
		fflush(NULL);
		assert(0);
	}
	
	return fp;
}

RETT_FOPEN64 _nvp_FOPEN64(INTF_FOPEN64) {

	//Do Nothing.
}

#endif

RETT_CLOSE _nvp_CLOSE(INTF_CLOSE)
{
	RETT_CLOSE result;
	ino_t serialno;
	struct NVFile* nvf = NULL;
	instrumentation_type close_time;

	START_TIMING(close_t, close_time);

#if PASS_THROUGH_CALLS	
	num_close++;
	result = _nvp_fileops->CLOSE(CALL_CLOSE);

	END_TIMING(close_t, close_time);

	return result;	
#endif // PASS_THROUGH_CALLS
	
	if (!async_close_enable)
		goto sync_close_bg_enabled;
			
#if BG_CLOSING
	CHECK_RESOLVE_FILEOPS(_nvp_);
	instrumentation_type clf_lock_time;
	int previous_closed_filedesc = -1;
	ino_t previous_closed_serialno = 0, stale_serialno = 0;
	int cpuid, stale_fd = -1;
	int hash_index = -1;
	int node_list_idx = 0;
#if SEQ_LIST || RAND_LIST
	struct ClosedFiles *clf = NULL;
#else //SEQ_LIST || RAND_LIST	
	struct InodeClosedFile *tbl = NULL;
#endif	//SEQ_LIST || RAND_LIST
		
	DEBUG("_nvp_CLOSE(%i)\n", file);
	//num_close++;
	// Get the struct NVFile from the file descriptor
	
        nvf = &_nvp_fd_lookup[file];
	node_list_idx = nvf->node->free_list_idx;
	
	if (nvf->posix) {
		nvf->valid = 0;
		nvf->posix = 0;
		NVP_LOCK_NODE_WR(nvf);
		nvf->node->reference--;
		NVP_UNLOCK_NODE_WR(nvf);
		if (nvf->node->reference == 0) {
			nvf->node->serialno = 0;
			int index = nvf->serialno % 1024;
			_nvp_ino_lookup[index] = 0;
			push_in_stack(1, 0, nvf->node->index_in_free_list, node_list_idx);
		}
		nvf->serialno = 0;
		DEBUG("Call posix CLOSE for fd %d\n", nvf->fd);
		result = _nvp_fileops->CLOSE(CALL_CLOSE);

		END_TIMING(close_t, close_time);

		return result;
	}
	
	serialno = nvf->node->serialno;	
	GLOBAL_LOCK_CLOSE_WR();

	hash_index = serialno % TOTAL_CLOSED_INODES;

#if SEQ_LIST || RAND_LIST
	clf = &_nvp_closed_files[hash_index];

#if PRINT_CONTENTION_MSGS		
	printf("%s: acquiring lock for clf entry = %d, thread id = %lu\n", __func__, hash_index, pthread_self());
	fflush(NULL);
#endif //PRINT_CONTENTION_MSGS

	//START_TIMING(clf_lock_t, clf_lock_time);
	LRU_NODE_LOCK_WR(clf);
	//END_TIMING(clf_lock_t, clf_lock_time);
#else //SEQ_LIST || RAND_LIST
	tbl = &inode_to_closed_file[hash_index];
	NVP_LOCK_HASH_TABLE_WR(tbl);
#endif	//SEQ_LIST || RAND_LIST
	cpuid = GET_CPUID();
	NVP_LOCK_NODE_RD(nvf, cpuid);

	if(nvf->node->reference == 1) {
		NVP_UNLOCK_NODE_RD(nvf, cpuid);		
		__atomic_fetch_add(&anon_mem_closed_files, nvf->node->anon_mem_used, __ATOMIC_SEQ_CST);
		
#if SEQ_LIST || RAND_LIST
		stale_fd = insert_in_seq_list(clf, &stale_serialno, file, serialno);
#else //SEQ_LIST || RAND_LIST
		stale_fd = insert_in_lru_list(file, serialno, &stale_serialno);
#endif	//SEQ_LIST || RAND_LIST			
		if(stale_fd >= 0 && stale_serialno > 0) {
			previous_closed_filedesc = stale_fd;
			previous_closed_serialno = stale_serialno;
		}		
		
		if(previous_closed_filedesc != -1) {
			_nvp_REAL_CLOSE(previous_closed_filedesc, previous_closed_serialno, 1);
		}
		else 
			__atomic_fetch_add(&num_files_closed, 1, __ATOMIC_SEQ_CST);
			
#if SEQ_LIST || RAND_LIST
#if PRINT_CONTENTION_MSGS		
		printf("%s: releasing lock for clf entry = %d, thread id = %lu\n", __func__, hash_index, pthread_self());
		fflush(NULL);
#endif
		LRU_NODE_UNLOCK_WR(clf);
#else //SEQ_LIST || RAND_LIST
		NVP_UNLOCK_HASH_TABLE_WR(tbl);
#endif //SEQ_LIST || RAND_LIST		
		GLOBAL_UNLOCK_CLOSE_WR();			

		END_TIMING(close_t, close_time);

		return 0;
	}

	NVP_UNLOCK_NODE_RD(nvf, cpuid);
#if SEQ_LIST || RAND_LIST
	LRU_NODE_UNLOCK_WR(clf);
#else //SEQ_LIST || RAND_LIST
	NVP_UNLOCK_HASH_TABLE_WR(tbl);
#endif //SEQ_LIST || RAND_LIST
	GLOBAL_UNLOCK_CLOSE_WR();
#else //BG_CLOSING
	nvf = &_nvp_fd_lookup[file];
	serialno = nvf->node->serialno;	
#endif //BG_CLOSING
	result = _nvp_REAL_CLOSE(CALL_CLOSE, serialno, 0);	

	END_TIMING(close_t, close_time);

	return result;	

 sync_close_bg_enabled:

	nvf = &_nvp_fd_lookup[file];

	if (file == 0 || file == 1 || file == 2) {
		nvf->valid = 0;
		nvf->serialno = 0;
		result = _nvp_fileops->CLOSE(CALL_CLOSE);
		END_TIMING(close_t, close_time);
		return result;				
	}

	serialno = nvf->node->serialno;	
	result = _nvp_REAL_CLOSE(CALL_CLOSE, serialno, 0);	

	END_TIMING(close_t, close_time);
	return result;		
}

RETT_OPENAT _nvp_OPENAT(INTF_OPENAT) {
	return _nvp_fileops->OPENAT(CALL_OPENAT);
}

RETT_EXECVE _nvp_EXECVE(INTF_EXECVE) {

	int exec_ledger_fd = -1, i = 0;
	unsigned long offset_in_map = 0;


	//MSG("%s: serialno = %d, node = %lu, node serialno = %lu\n", __func__, _nvp_fd_lookup[1].serialno, _nvp_fd_lookup[1].node, _nvp_fd_lookup[1].node->serialno);

		
	for (i = 0; i < 1024; i++) {
		if (_nvp_fd_lookup[i].offset != NULL)
			execve_fd_passing[i] = *(_nvp_fd_lookup[i].offset);
		else
			execve_fd_passing[i] = 0;
	}
		
	exec_ledger_fd = shm_open("exec-ledger", O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
	if (exec_ledger_fd == -1) {
		printf("%s: %s\n", __func__, strerror(errno));
		assert(0);
	}

	int res = _nvp_fileops->TRUNC64(exec_ledger_fd, (10*1024*1024));
	if (res == -1) {
		printf("%s: ftruncate failed. Err = %s\n", __func__, strerror(errno));
		assert(0);
	}

	char *shm_area = mmap(NULL, 10*1024*1024, PROT_READ | PROT_WRITE, MAP_SHARED, exec_ledger_fd, 0);
	if (shm_area == NULL) {
		printf("%s: mmap failed. Err = %s\n", __func__, strerror(errno));
		assert(0);
	}

	if (memcpy(shm_area + offset_in_map, _nvp_fd_lookup, 1024 * sizeof(struct NVFile)) == NULL) {
		printf("%s: memcpy of fd lookup failed. Err = %s\n", __func__, strerror(errno));
		assert(0);
	}

	offset_in_map += (1024 * sizeof(struct NVFile));

	if (memcpy(shm_area + offset_in_map, execve_fd_passing, 1024 * sizeof(int)) == NULL) {
		printf("%s: memcpy of execve offset failed. Err = %s\n", __func__, strerror(errno));
		assert(0);
	}

	offset_in_map += (1024 * sizeof(int));
	
	
	if (memcpy(shm_area + offset_in_map, _nvp_node_lookup[0], 1024*sizeof(struct NVNode)) == NULL) {
		printf("%s: memcpy of node lookup failed. Err = %s\n", __func__, strerror(errno));
		assert(0);
	}
	
	offset_in_map += (1024*sizeof(struct NVNode));
	
	if (memcpy(shm_area + offset_in_map, _nvp_ino_lookup, 1024 * sizeof(int)) == NULL) {
		printf("%s: memcpy of ino lookup failed. Err = %s\n", __func__, strerror(errno));
		assert(0);
	}

	offset_in_map += (1024 * sizeof(int));

	if (memcpy(shm_area + offset_in_map, _nvp_free_node_list[0], 1024*sizeof(struct StackNode)) == NULL) {
		printf("%s: memcpy of free node list failed. Err = %s\n", __func__, strerror(errno));
		assert(0);
	}

	offset_in_map += (1024 * sizeof(struct StackNode));
	//munmap(shm_area, 10*1024*1024);
	
	return _nvp_fileops->EXECVE(CALL_EXECVE);

}

RETT_EXECVP _nvp_EXECVP(INTF_EXECVP) {

	int exec_ledger_fd = -1, i = 0;
	unsigned long offset_in_map = 0;

	for (i = 0; i < 1024; i++) {
		if (_nvp_fd_lookup[i].offset != NULL)
			execve_fd_passing[i] = *(_nvp_fd_lookup[i].offset);
		else
			execve_fd_passing[i] = 0;
	}
		
	exec_ledger_fd = shm_open("exec-ledger", O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
	if (exec_ledger_fd == -1) {
		printf("%s: %s\n", __func__, strerror(errno));
		assert(0);
	}

	int res = _nvp_fileops->TRUNC64(exec_ledger_fd, (10*1024*1024));
	if (res == -1) {
		printf("%s: ftruncate failed. Err = %s\n", __func__, strerror(errno));
		assert(0);
	}

	char *shm_area = mmap(NULL, 10*1024*1024, PROT_READ | PROT_WRITE, MAP_SHARED, exec_ledger_fd, 0);
	if (shm_area == NULL) {
		printf("%s: mmap failed. Err = %s\n", __func__, strerror(errno));
		assert(0);
	}

	if (memcpy(shm_area + offset_in_map, _nvp_fd_lookup, 1024 * sizeof(struct NVFile)) == NULL) {
		printf("%s: memcpy of fd lookup failed. Err = %s\n", __func__, strerror(errno));
		assert(0);
	}

	offset_in_map += (1024 * sizeof(struct NVFile));

	if (memcpy(shm_area + offset_in_map, execve_fd_passing, 1024 * sizeof(int)) == NULL) {
		printf("%s: memcpy of execve offset failed. Err = %s\n", __func__, strerror(errno));
		assert(0);
	}

	offset_in_map += (1024 * sizeof(int));
	
	
	if (memcpy(shm_area + offset_in_map, _nvp_node_lookup[0], 1024*sizeof(struct NVNode)) == NULL) {
		printf("%s: memcpy of node lookup failed. Err = %s\n", __func__, strerror(errno));
		assert(0);
	}
	
	offset_in_map += (1024*sizeof(struct NVNode));
	
	if (memcpy(shm_area + offset_in_map, _nvp_ino_lookup, 1024 * sizeof(int)) == NULL) {
		printf("%s: memcpy of ino lookup failed. Err = %s\n", __func__, strerror(errno));
		assert(0);
	}

	offset_in_map += (1024 * sizeof(int));

	if (memcpy(shm_area + offset_in_map, _nvp_free_node_list[0], 1024*sizeof(struct StackNode)) == NULL) {
		printf("%s: memcpy of free node list failed. Err = %s\n", __func__, strerror(errno));
		assert(0);
	}

	offset_in_map += (1024 * sizeof(struct StackNode));
	//munmap(shm_area, 10*1024*1024);
	
	return _nvp_fileops->EXECVP(CALL_EXECVP);
}

RETT_EXECV _nvp_EXECV(INTF_EXECV) {

	int exec_ledger_fd = -1, i = 0;
	unsigned long offset_in_map = 0;

	for (i = 0; i < 1024; i++) {
		if (_nvp_fd_lookup[i].offset != NULL)
			execve_fd_passing[i] = *(_nvp_fd_lookup[i].offset);
		else
			execve_fd_passing[i] = 0;
	}
		
	exec_ledger_fd = shm_open("exec-ledger", O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
	if (exec_ledger_fd == -1) {
		printf("%s: %s\n", __func__, strerror(errno));
		assert(0);
	}

	int res = _nvp_fileops->TRUNC64(exec_ledger_fd, (10*1024*1024));
	if (res == -1) {
		printf("%s: ftruncate failed. Err = %s\n", __func__, strerror(errno));
		assert(0);
	}

	char *shm_area = mmap(NULL, 10*1024*1024, PROT_READ | PROT_WRITE, MAP_SHARED, exec_ledger_fd, 0);
	if (shm_area == NULL) {
		printf("%s: mmap failed. Err = %s\n", __func__, strerror(errno));
		assert(0);
	}

	if (memcpy(shm_area + offset_in_map, _nvp_fd_lookup, 1024 * sizeof(struct NVFile)) == NULL) {
		printf("%s: memcpy of fd lookup failed. Err = %s\n", __func__, strerror(errno));
		assert(0);
	}

	offset_in_map += (1024 * sizeof(struct NVFile));

	if (memcpy(shm_area + offset_in_map, execve_fd_passing, 1024 * sizeof(int)) == NULL) {
		printf("%s: memcpy of execve offset failed. Err = %s\n", __func__, strerror(errno));
		assert(0);
	}

	offset_in_map += (1024 * sizeof(int));
	
	
	if (memcpy(shm_area + offset_in_map, _nvp_node_lookup[0], 1024*sizeof(struct NVNode)) == NULL) {
		printf("%s: memcpy of node lookup failed. Err = %s\n", __func__, strerror(errno));
		assert(0);
	}
	
	offset_in_map += (1024*sizeof(struct NVNode));
	
	if (memcpy(shm_area + offset_in_map, _nvp_ino_lookup, 1024 * sizeof(int)) == NULL) {
		printf("%s: memcpy of ino lookup failed. Err = %s\n", __func__, strerror(errno));
		assert(0);
	}

	offset_in_map += (1024 * sizeof(int));

	if (memcpy(shm_area + offset_in_map, _nvp_free_node_list[0], 1024*sizeof(struct StackNode)) == NULL) {
		printf("%s: memcpy of free node list failed. Err = %s\n", __func__, strerror(errno));
		assert(0);
	}

	offset_in_map += (1024 * sizeof(struct StackNode));
	//munmap(shm_area, 10*1024*1024);
	
	return _nvp_fileops->EXECV(CALL_EXECV);
}

RETT_EXECL _nvp_EXECL(INTF_EXECL) {

	int exec_ledger_fd = -1, i = 0;
	unsigned long offset_in_map = 0;
	const char *argv1 = NULL;
	const char *argv2 = NULL;
	
	va_list arglist;
	va_start(arglist, arg);
	argv1 = va_arg(arglist, const char *);
	argv2 = va_arg(arglist, const char *);
	va_end(arglist);


	DEBUG_FILE("%s: argument 1 = %s\n", __func__, argv1);
	DEBUG_FILE("%s: argument 2 = %s\n", __func__, argv2);
	
	for (i = 0; i < 1024; i++) {
		if (_nvp_fd_lookup[i].offset != NULL)
			execve_fd_passing[i] = *(_nvp_fd_lookup[i].offset);
		else
			execve_fd_passing[i] = 0;
	}
		
	exec_ledger_fd = shm_open("exec-ledger", O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
	if (exec_ledger_fd == -1) {
		printf("%s: %s\n", __func__, strerror(errno));
		assert(0);
	}

	int res = _nvp_fileops->TRUNC64(exec_ledger_fd, (10*1024*1024));
	if (res == -1) {
		printf("%s: ftruncate failed. Err = %s\n", __func__, strerror(errno));
		assert(0);
	}

	char *shm_area = mmap(NULL, 10*1024*1024, PROT_READ | PROT_WRITE, MAP_SHARED, exec_ledger_fd, 0);
	if (shm_area == NULL) {
		printf("%s: mmap failed. Err = %s\n", __func__, strerror(errno));
		assert(0);
	}

	if (memcpy(shm_area + offset_in_map, _nvp_fd_lookup, 1024 * sizeof(struct NVFile)) == NULL) {
		printf("%s: memcpy of fd lookup failed. Err = %s\n", __func__, strerror(errno));
		assert(0);
	}

	offset_in_map += (1024 * sizeof(struct NVFile));

	if (memcpy(shm_area + offset_in_map, execve_fd_passing, 1024 * sizeof(int)) == NULL) {
		printf("%s: memcpy of execve offset failed. Err = %s\n", __func__, strerror(errno));
		assert(0);
	}

	offset_in_map += (1024 * sizeof(int));
	
	
	if (memcpy(shm_area + offset_in_map, _nvp_node_lookup[0], 1024*sizeof(struct NVNode)) == NULL) {
		printf("%s: memcpy of node lookup failed. Err = %s\n", __func__, strerror(errno));
		assert(0);
	}
	
	offset_in_map += (1024*sizeof(struct NVNode));
	
	if (memcpy(shm_area + offset_in_map, _nvp_ino_lookup, 1024 * sizeof(int)) == NULL) {
		printf("%s: memcpy of ino lookup failed. Err = %s\n", __func__, strerror(errno));
		assert(0);
	}

	offset_in_map += (1024 * sizeof(int));

	if (memcpy(shm_area + offset_in_map, _nvp_free_node_list[0], 1024*sizeof(struct StackNode)) == NULL) {
		printf("%s: memcpy of free node list failed. Err = %s\n", __func__, strerror(errno));
		assert(0);
	}

	offset_in_map += (1024 * sizeof(struct StackNode));
	//munmap(shm_area, 10*1024*1024);
	
	return _nvp_fileops->EXECL(CALL_EXECL, argv1, argv2, (char *) NULL);
}




#ifdef TRACE_FP_CALLS
RETT_FCLOSE _nvp_FCLOSE(INTF_FCLOSE)
{
	CHECK_RESOLVE_FILEOPS(_nvp_);
	RETT_FCLOSE result;
	int fd = -1;
	instrumentation_type close_time;

	START_TIMING(close_t, close_time);
	
#if PASS_THROUGH_CALLS
	result = _nvp_fileops->FCLOSE(CALL_FCLOSE);	
	END_TIMING(close_t, close_time);
	return result;
#endif
	
	fd = fileno(fp);
	result = _nvp_CLOSE(fd);
	return result;
}

RETT_FREAD _nvp_FREAD(INTF_FREAD)
{
	DEBUG("_nvp_READ %d\n", fileno(fp));
	RETT_READ result;
	instrumentation_type read_time;
	
	START_TIMING(read_t, read_time);
	
#if PASS_THROUGH_CALLS
	num_read++;
	result = _nvp_fileops->FREAD(CALL_FREAD);
	END_TIMING(read_t, read_time);
	return result;
#endif
	
	//result = _nvp_fileops->FREAD(CALL_FREAD);
	//return result;

	struct NVFile* nvf = &_nvp_fd_lookup[fileno(fp)];
	
	if (nvf->posix) {
		DEBUG("Call posix READ for fd %d\n", nvf->fd);
		result = _nvp_fileops->FREAD(CALL_FREAD);
		read_size += result;
		num_posix_read++;
		posix_read_size += result;
		return result;
	}

	result = _nvp_check_read_size_valid(length);
	if (result <= 0) {
		return result;
	}

	int cpuid = GET_CPUID();

	NVP_LOCK_FD_RD(nvf, cpuid); // TODO
	NVP_CHECK_NVF_VALID_WR(nvf);
	NVP_LOCK_NODE_RD(nvf, cpuid);
	
	result = _nvp_do_pread(fileno(fp), buf, length*nmemb, 
			       __sync_fetch_and_add(nvf->offset, length), 0, cpuid, nvf);
	
	NVP_UNLOCK_NODE_RD(nvf, cpuid);

	if(result == length)	{
		DEBUG("PREAD succeeded: extending offset from %li to %li\n",
			*nvf->offset - result, *nvf->offset);
	}
	else if (result <= 0){
		DEBUG("_nvp_READ: PREAD failed; not changing offset. "
			"(returned %i)\n", result);		
		//assert(0); // TODO: this is for testing only
		__sync_fetch_and_sub(nvf->offset, length);
	} else {
		DEBUG("_nvp_READ: PREAD failed; Not fully read. "
			"(returned %i)\n", result);
		// assert(0); // TODO: this is for testing only
		__sync_fetch_and_sub(nvf->offset, length - result);
	}

	NVP_UNLOCK_FD_RD(nvf, cpuid);

	num_read++;
	read_size += result;

	return result;
}
#endif

RETT_READ _nvp_READ(INTF_READ)
{
	DEBUG("_nvp_READ %d\n", file);
	num_read++;
	RETT_READ result;
	instrumentation_type read_time;
	unsigned long pid, ppid;

	START_TIMING(read_t, read_time);
	
#if PASS_THROUGH_CALLS	
	result = _nvp_fileops->READ(CALL_READ);
	END_TIMING(read_t, read_time);
	return result;
#endif

	//result = _nvp_fileops->READ(CALL_READ);
	//return result;
	
	struct NVFile* nvf = &_nvp_fd_lookup[file];

	if(nvf->posix) {
		DEBUG("Call posix READ for fd %d\n", nvf->fd);
		result = _nvp_fileops->READ(CALL_READ);
		read_size += result;
		num_posix_read++;
		posix_read_size += result;
		END_TIMING(read_t, read_time);
		return result;
	}

	result = _nvp_check_read_size_valid(length);
	if (result <= 0) {
		END_TIMING(read_t, read_time);
		return result;
	}

	int cpuid = GET_CPUID();
		
	NVP_LOCK_FD_RD(nvf, cpuid); // TODO
	NVP_CHECK_NVF_VALID_WR(nvf);
	NVP_LOCK_NODE_RD(nvf, cpuid);

	result = _nvp_do_pread(CALL_READ,
			       __sync_fetch_and_add(nvf->offset, length), 0, cpuid, nvf);
	
	NVP_UNLOCK_NODE_RD(nvf, cpuid);

	if(result == length)	{
		DEBUG("PREAD succeeded: extending offset from %li to %li\n",
			*nvf->offset - result, *nvf->offset);
	}
	else if (result <= 0){
		DEBUG("_nvp_READ: PREAD failed; not changing offset. "
			"(returned %i)\n", result);		
		//assert(0); // TODO: this is for testing only
		__sync_fetch_and_sub(nvf->offset, length);
	} else {
		DEBUG("_nvp_READ: PREAD failed; Not fully read. "
			"(returned %i)\n", result);
		// assert(0); // TODO: this is for testing only
		__sync_fetch_and_sub(nvf->offset, length - result);
	}

	NVP_UNLOCK_FD_RD(nvf, cpuid);

	read_size += result;

	END_TIMING(read_t, read_time);
	return result;
}


#ifdef TRACE_FP_CALLS
RETT_FWRITE _nvp_FWRITE(INTF_FWRITE)
{
	DEBUG("_nvp_WRITE %d\n", fileno(fp));
	num_write++;
	RETT_FWRITE result;
	instrumentation_type write_time;
	
	DEBUG_FILE("%s: Calling fwrite with fp = %p, fd = %d, length = %lu, nmemb = %d\n", __func__, fp, fileno(fp), length, nmemb);

	START_TIMING(write_t, write_time);
	
#if PASS_THROUGH_CALLS
	//result = _nvp_fileops->FWRITE(CALL_FWRITE);
	result = _nvp_fileops->WRITE(fileno(fp), buf, (length*nmemb)); 
	DEBUG_FILE("%s: Result of the write = %d\n", __func__, result);
	write_size += result;
	END_TIMING(write_t, write_time);
	return result;
#endif
	
	//result = _nvp_fileops->WRITE(fileno(fp), buf, (length*nmemb)); 
	//DEBUG_FILE("%s: Result of the write = %d\n", __func__, result);
	//return result;

	if (fileno(fp) == 1) {
		result = _nvp_fileops->WRITE(fileno(fp), buf, (length*nmemb));
		DEBUG_FILE("%s: Result of the write = %d\n", __func__, result);
		write_size += result;
		num_posix_write++;
		posix_write_size += result;
		END_TIMING(write_t, write_time);
		return result;
	}
	
	struct NVFile* nvf = &_nvp_fd_lookup[fileno(fp)];
	
	if (nvf->posix) {
		DEBUG("Call posix WRITE for fd %d\n", nvf->fd);
		result = _nvp_fileops->FWRITE(CALL_FWRITE);
		write_size += result;
		num_posix_write++;
		posix_write_size += result;
		END_TIMING(write_t, write_time);
		return result;
	}

	int cpuid = GET_CPUID();

	result = _nvp_check_write_size_valid(length);
	if (result <= 0) {
		END_TIMING(write_t, write_time);
		return result;
	}

	NVP_LOCK_FD_RD(nvf, cpuid); // TODO
	NVP_CHECK_NVF_VALID_WR(nvf);
	NVP_LOCK_NODE_RD(nvf, cpuid); //TODO

	result = _nvp_do_pwrite(fileno(fp), buf, length*nmemb,
				__sync_fetch_and_add(nvf->offset, length), 0, cpuid, nvf);

	NVP_UNLOCK_NODE_RD(nvf, cpuid);

	if(result >= 0)
	{
		if(nvf->append)
		{
			DEBUG("PWRITE succeeded and append == true. "
				"Setting offset to end...\n"); 
			assert(_nvp_do_seek64(nvf->fd, 0, SEEK_END, nvf)
				!= (RETT_SEEK64)-1);
		}
		else
		{
			DEBUG("PWRITE succeeded: extending offset "
				"from %li to %li\n",
				*nvf->offset - result, *nvf->offset);
		}
	}
	else {
		DEBUG("_nvp_WRITE: PWRITE failed; not changing offset. "
			"(returned %i)\n", result);
	}

	DEBUG_FILE("About to return from _nvp_WRITE with ret val %i (errno %i). "
		"file len: %li, file off: %li\n",
		result, errno, nvf->node->length, nvf->offset);

	NVP_UNLOCK_FD_RD(nvf, cpuid);
	write_size += result;
	END_TIMING(write_t, write_time);
	return result;
}
#endif

#ifdef TRACE_FP_CALLS
RETT_FSEEK _nvp_FSEEK(INTF_FSEEK)
{
	RETT_WRITE result;
	int fd = -1;

	fd = fileno(fp);
	result = _nvp_SEEK(fd, offset, whence);
	return result;
}
#endif

#ifdef TRACE_FP_CALLS
RETT_FTELL _nvp_FTELL(INTF_FTELL)
{
	RETT_FTELL result;
	int fd = -1;
	struct NVFile *nvf = NULL;
	
	fd = fileno(fp);
	nvf = &_nvp_fd_lookup[fd];
	
	result = _nvp_SEEK(fd, *(nvf->offset), SEEK_CUR);
	return result;	
}
#endif

#ifdef TRACE_FP_CALLS
RETT_FTELLO _nvp_FTELLO(INTF_FTELLO)
{
	RETT_FTELLO result;

	result = _nvp_FTELL(CALL_FTELLO);
	return result;
}
#endif

RETT_WRITE _nvp_WRITE(INTF_WRITE)
{
	DEBUG("_nvp_WRITE %d\n", file);
	num_write++;
	RETT_WRITE result;
	instrumentation_type write_time, write_syscall_time;

	START_TIMING(write_t, write_time);
	
#if PASS_THROUGH_CALLS
	result = _nvp_fileops->WRITE(CALL_WRITE);
	write_size += result;
	END_TIMING(write_t, write_time);
	return result;
#endif

	//result = _nvp_fileops->WRITE(CALL_WRITE);
	//return result;
	
	struct NVFile* nvf = &_nvp_fd_lookup[file];

	if (nvf->posix) {
		DEBUG("Call posix WRITE for fd %d\n", nvf->fd);
		result = _nvp_fileops->WRITE(CALL_WRITE);
		write_size += result;
		num_posix_write++;
		posix_write_size += result;
		END_TIMING(write_t, write_time);
		return result;
	}

	int cpuid = GET_CPUID();

	result = _nvp_check_write_size_valid(length);
	if (result <= 0) {
		END_TIMING(write_t, write_time);
		return result;
	}

	NVP_LOCK_FD_RD(nvf, cpuid); // TODO
	NVP_CHECK_NVF_VALID_WR(nvf);
	NVP_LOCK_NODE_RD(nvf, cpuid); //TODO

	// If write is beyond the size of file, do write system call. 
#if !ANON_MEM_HANDLE
	if ((*(nvf->offset) + length) > nvf->node->true_length) {

		_nvp_wr_total++;
		
		NVP_UNLOCK_NODE_RD(nvf, cpuid);
		NVP_LOCK_NODE_WR(nvf);
		
		nvf->node->length = *(nvf->offset) + length;
		nvf->node->true_length = *(nvf->offset) + length;
		nvf->node->true_length_for_read = *(nvf->offset) + length;
		
		START_TIMING(write_syscall_t, write_syscall_time);

		result = _nvp_fileops->PWRITE(CALL_WRITE, __sync_fetch_and_add(nvf->offset, length));

		END_TIMING(write_syscall_t, write_syscall_time);

		num_posix_write++;
		posix_write_size += result;
		num_mfence++;
		num_write_nontemporal++;
		_nvp_wr_extended++;

		NVP_UNLOCK_NODE_WR(nvf);
		NVP_UNLOCK_FD_RD(nvf, cpuid);

		END_TIMING(write_t, write_time);
		return result;
	}
#endif
	
	result = _nvp_do_pwrite(CALL_WRITE,
				__sync_fetch_and_add(nvf->offset, length), 0, cpuid, nvf);
	NVP_UNLOCK_NODE_RD(nvf, cpuid);
	if(result >= 0)
	{
		if(nvf->append)
		{
			DEBUG("PWRITE succeeded and append == true. "
				"Setting offset to end...\n"); 
			assert(_nvp_do_seek64(nvf->fd, 0, SEEK_END, nvf)
				!= (RETT_SEEK64)-1);
		}
		else
		{
			DEBUG("PWRITE succeeded: extending offset "
				"from %li to %li\n",
				*nvf->offset - result, *nvf->offset);
		}

		write_size += result;
	}

	DEBUG("About to return from _nvp_WRITE with ret val %i (errno %i). "
		"file len: %li, file off: %li, map len: %li\n",
		result, errno, nvf->node->length, nvf->offset,
		nvf->node->maplength);

	NVP_UNLOCK_FD_RD(nvf, cpuid);

	END_TIMING(write_t, write_time);
	return result;
}

RETT_PREAD _nvp_PREAD(INTF_PREAD)
{
	CHECK_RESOLVE_FILEOPS(_nvp_);
	DEBUG("_nvp_PREAD %d\n", file);
	num_read++;
	instrumentation_type read_time;
	RETT_PREAD result;

	START_TIMING(pread_t, read_time);

#if PASS_THROUGH_CALLS
	result = _nvp_fileops->PREAD(CALL_PREAD);

	END_TIMING(pread_t, read_time);
	return result;
#endif

	//result = _nvp_fileops->PREAD(CALL_PREAD);
	//return result;
	
	struct NVFile* nvf = &_nvp_fd_lookup[file];

	if (nvf->posix) {
		DEBUG("Call posix PREAD for fd %d\n", nvf->fd);
		result = _nvp_fileops->PREAD(CALL_PREAD);
		read_size += result;
		num_posix_read++;
		posix_read_size += result;

		END_TIMING(pread_t, read_time);
		return result;
	}

	result = _nvp_check_read_size_valid(count);
	if (result <= 0) {
		END_TIMING(pread_t, read_time);
		return result;
	}

	int cpuid = GET_CPUID();

	NVP_LOCK_FD_RD(nvf, cpuid);
	NVP_CHECK_NVF_VALID(nvf);
	NVP_LOCK_NODE_RD(nvf, cpuid);

	result = _nvp_do_pread(CALL_PREAD, 0, cpuid, nvf);

	NVP_UNLOCK_NODE_RD(nvf, cpuid);
	NVP_UNLOCK_FD_RD(nvf, cpuid);

	read_size += result;

	END_TIMING(pread_t, read_time);
	return result;
}

RETT_PWRITE _nvp_PWRITE(INTF_PWRITE)
{
	CHECK_RESOLVE_FILEOPS(_nvp_);
	DEBUG("_nvp_PWRITE %d\n", file);
	num_write++;
	instrumentation_type write_time;
	RETT_PWRITE result;

	START_TIMING(pwrite_t, write_time);
	
#if PASS_THROUGH_CALLS
	result = _nvp_fileops->PWRITE(CALL_PWRITE);

	write_size += result;
	END_TIMING(pwrite_t, write_time);
	return result;
#endif
	
	//result = _nvp_fileops->PWRITE(CALL_PWRITE);
	//return result;
	
	struct NVFile* nvf = &_nvp_fd_lookup[file];

	if (nvf->posix) {
		DEBUG("Call posix PWRITE for fd %d\n", nvf->fd);
		result = _nvp_fileops->PWRITE(CALL_PWRITE);
		write_size += result;
		num_posix_write++;
		posix_write_size += result;
		END_TIMING(pwrite_t, write_time);
		return result;
	}

	result = _nvp_check_write_size_valid(count);
	if (result <= 0) {
		END_TIMING(pwrite_t, write_time);
		return result;
	}
	int cpuid = GET_CPUID();
	NVP_LOCK_FD_RD(nvf, cpuid);
	NVP_CHECK_NVF_VALID(nvf);
	NVP_LOCK_NODE_RD(nvf, cpuid);
	
	ssize_t available_length = (nvf->node->length) - offset;

	/*
	if(count > available_length) {
		DEBUG("Promoting PWRITE lock to WRLOCK\n");
		NVP_UNLOCK_NODE_RD(nvf, cpuid);
		NVP_LOCK_NODE_WR(nvf);
		
		result = _nvp_do_pwrite(CALL_PWRITE, 1, cpuid, nvf);
		NVP_UNLOCK_NODE_WR(nvf);
	}
	else {
		result = _nvp_do_pwrite(CALL_PWRITE, 0, cpuid, nvf);
		NVP_UNLOCK_NODE_RD(nvf, cpuid);
	}
	*/

	result = _nvp_do_pwrite(CALL_PWRITE, 0, cpuid, nvf);
	NVP_UNLOCK_NODE_RD(nvf, cpuid);
	
	NVP_UNLOCK_FD_RD(nvf, cpuid);

	write_size += result;

	END_TIMING(pwrite_t, write_time);
	return result;
}


RETT_SEEK _nvp_SEEK(INTF_SEEK)
{
	DEBUG("_nvp_SEEK\n");

	return _nvp_SEEK64(CALL_SEEK);
}

RETT_SEEK64 _nvp_SEEK64(INTF_SEEK64)
{

	CHECK_RESOLVE_FILEOPS(_nvp_);
	instrumentation_type seek_time;
	RETT_SEEK64 ret = 0;
	DEBUG("_nvp_SEEK64 %d\n", file);

	START_TIMING(seek_t, seek_time);

#if PASS_THROUGH_CALLS
	ret = _nvp_fileops->SEEK64(CALL_SEEK);
	END_TIMING(seek_t, seek_time);
	return ret;
#endif	
	
	struct NVFile* nvf = &_nvp_fd_lookup[file];

	if (nvf->posix) {
		DEBUG("Call posix SEEK64 for fd %d\n", nvf->fd);
		END_TIMING(seek_t, seek_time);
		ret = _nvp_fileops->SEEK64(CALL_SEEK64);

		END_TIMING(seek_t, seek_time);
		return ret;
	}

	int cpuid = GET_CPUID();

	NVP_LOCK_FD_WR(nvf);
	NVP_CHECK_NVF_VALID_WR(nvf);
	NVP_LOCK_NODE_RD(nvf, cpuid);

	DEBUG_FILE("%s: Calling seek with file = %d, offset = %lu, whence = %d\n", __func__, file, offset, whence); 
	RETT_SEEK64 result =  _nvp_do_seek64(CALL_SEEK64, nvf);	
	
	NVP_UNLOCK_NODE_RD(nvf, cpuid);
	NVP_UNLOCK_FD_WR(nvf);

	END_TIMING(seek_t, seek_time);
	return result;
}

RETT_TRUNC _nvp_TRUNC(INTF_TRUNC)
{
	CHECK_RESOLVE_FILEOPS(_nvp_);

#if PASS_THROUGH_CALLS
	return _nvp_fileops->TRUNC(CALL_TRUNC);
#endif	

	DEBUG("_nvp_TRUNC\n");

	return _nvp_TRUNC64(CALL_TRUNC);
}

RETT_TRUNC64 _nvp_TRUNC64(INTF_TRUNC64)
{
	int i = 0;
	CHECK_RESOLVE_FILEOPS(_nvp_);

#if PASS_THROUGH_CALLS
	return _nvp_fileops->TRUNC64(CALL_TRUNC);
#endif	

	DEBUG("_nvp_TRUNC64\n");

	struct NVFile* nvf = &_nvp_fd_lookup[file];

	if (nvf->posix) {
		DEBUG("Call posix TRUNC64 for fd %d\n", nvf->fd);
		return _nvp_fileops->TRUNC64(CALL_TRUNC64);
	}

	int cpuid = GET_CPUID();
	NVP_LOCK_FD_RD(nvf, cpuid);
	NVP_CHECK_NVF_VALID(nvf);
	NVP_LOCK_NODE_WR(nvf);

	if(!nvf->canWrite) {
		DEBUG("FD not open for writing: %i\n", file);
		errno = EINVAL;
		NVP_UNLOCK_NODE_WR(nvf);
		NVP_UNLOCK_FD_RD(nvf, cpuid);
		return -1;
	}

	if(length == nvf->node->length)
	{
		DEBUG("_nvp_TRUNC64: requested length was the same as old "
			"length (%li).\n", nvf->node->length);
		NVP_UNLOCK_NODE_WR(nvf);
		NVP_UNLOCK_FD_RD(nvf, cpuid);
		return 0;
	}

	DO_MSYNC(nvf);

	int result = _nvp_fileops->TRUNC64(CALL_TRUNC64);

	if(result != 0)
	{
		ERROR("%s->TRUNC64 failed (returned %li, requested %li): %s\n",
			_nvp_fileops->name, result, length, strerror(errno));
		assert(0);
	}

	if(length > nvf->node->length)
	{
		DEBUG_FILE("TRUNC64 extended file from %li to %li\n",
			nvf->node->length, length);
	}
	else 
	{
		DEBUG_FILE("TRUNC64 shortened file from %li to %li\n",
			nvf->node->length, length);
	}

	nvf->node->length = length;
	nvf->node->true_length = length;
	nvf->node->true_length_for_read = length;
	nvf->node->root_dirty_num = 0;

	for(i=nvf->node->anonymous_root_dirty_start; i<=nvf->node->anonymous_root_dirty_end; i++) {
		nvf->node->anonymous_offset_end[i] = 0;
		nvf->node->anonymous_offset_start[i] = UINT32_MAX;
	}	

	for (i=0; i<1024; i++) {
		nvf->node->root[i] = 0;
	}

	if (*(nvf->offset) > nvf->node->true_length)
		*(nvf->offset) = nvf->node->true_length;

	DO_MSYNC(nvf);

	NVP_UNLOCK_NODE_WR(nvf);
	NVP_UNLOCK_FD_RD(nvf, cpuid);

	return result;
}

RETT_READV _nvp_READV(INTF_READV)
{
	CHECK_RESOLVE_FILEOPS(_nvp_);

	DEBUG("CALL: _nvp_READV\n");

	//TODO: opportunities for optimization exist here

	int fail = 0;

	int i;
	for(i=0; i<iovcnt; i++)
	{
		fail |= _nvp_READ(file, iov[i].iov_base, iov[i].iov_len);
		if(fail) { break; }
	}

	if(fail != 0) {
		DEBUG("_nvp_READV failed on iov %i\n", i);
		return -1;
	}

	return 0;
}

RETT_WRITEV _nvp_WRITEV(INTF_WRITEV)
{
	CHECK_RESOLVE_FILEOPS(_nvp_);

	DEBUG("CALL: _nvp_WRITEV\n");

	//TODO: opportunities for optimization exist here

	int fail = 0;

	int i;
	for(i=0; i<iovcnt; i++)
	{
		fail |= _nvp_WRITE(file, iov[i].iov_base, iov[i].iov_len);
		if(fail) { break; }
	}

	if(fail != 0) {
		DEBUG("_nvp_WRITEV failed on iov %i\n", i);
		return -1;
	}

	return 0;
}

RETT_FCNTL _nvp_FCNTL(INTF_FCNTL)
{
	DEBUG("%s: fcntl called. Fd = %d\n", __func__, file);
	struct NVFile *nvf = NULL;
	int result = 0;
	 
	va_list arg;
	va_start(arg, cmd);
	int arg3 = va_arg(arg, int);
	va_end(arg);

#if PASS_THROUGH_CALLS
	result = _nvp_fileops->FCNTL(file, cmd, arg3);
	return result;
#endif	 
	nvf = &_nvp_fd_lookup[file];
	if (nvf->fd == 0)
		nvf->fd = file;
	 
	result = _nvp_fileops->FCNTL(file, cmd, arg3);
	if (result == -1) {
		DEBUG_FILE("%s: negative result. Err = %s\n", __func__, strerror(errno));
	}
}


RETT_DUP _nvp_DUP(INTF_DUP)
{
	//return _nvp_fileops->DUP(CALL_DUP);
	
	DEBUG("_nvp_DUP(" PFFS_DUP ")\n", CALL_DUP);

#if PASS_THROUGH_CALLS
	return _nvp_fileops->DUP(CALL_DUP);
#endif
	
	//CHECK_RESOLVE_FILEOPS(_nvp_);
	if(file < 0) {
		return _nvp_fileops->DUP(CALL_DUP);
	}

	
	struct NVFile* nvf = &_nvp_fd_lookup[file];

	NVP_LOCK_FD_WR(nvf);
	NVP_CHECK_NVF_VALID_WR(nvf);	
	NVP_LOCK_NODE_WR(nvf); // TODO

	int result = _nvp_fileops->DUP(CALL_DUP);
	
	if(result < 0) 
	{
		DEBUG("Call to _nvp_DUP->%s->DUP failed: %s\n",
			_nvp_fileops->name, strerror(errno));
		NVP_UNLOCK_NODE_WR(nvf);
		NVP_UNLOCK_FD_WR(nvf);
		return result;
	}

	struct NVFile* nvf2 = &_nvp_fd_lookup[result];

	nvf->valid = 0;
	nvf2->valid = 0;
		
	if (nvf->posix) {
		DEBUG("Call posix DUP for fd %d\n", nvf->fd);
		nvf2->posix = nvf->posix;
		NVP_UNLOCK_NODE_WR(nvf);
		NVP_UNLOCK_FD_WR(nvf);
		return result;
	}

	NVP_LOCK_FD_WR(nvf2);
	
	if(nvf2->valid) {
		ERROR("fd %i was already in use!\n", result);
		assert(!nvf2->valid);
	} else {
		//free(nvf2->offset); // TODO: free this iff it's not in use anymore to avoid memory leaks
	}
	
	nvf2->fd 	= result;
	nvf2->offset 	= nvf->offset;
	nvf2->canRead 	= nvf->canRead;
	nvf2->canWrite 	= nvf->canWrite;
	nvf2->append 	= nvf->append;
	nvf2->aligned   = nvf->aligned;
	nvf2->serialno 	= nvf->serialno;
	nvf2->node 	= nvf->node;
	nvf2->posix 	= nvf->posix;

	SANITYCHECK(nvf2->node != NULL);
	
	nvf->node->reference++;
	nvf->valid      = 1;
	nvf2->valid 	= 1;

	DO_MSYNC(nvf);
	DO_MSYNC(nvf2);

	NVP_UNLOCK_NODE_WR(nvf); // nvf2->node->lock == nvf->node->lock since nvf and nvf2 share a node
	NVP_UNLOCK_FD_WR(nvf);
	NVP_UNLOCK_FD_WR(nvf2);

	return nvf2->fd;
}

RETT_DUP2 _nvp_DUP2(INTF_DUP2)
{
	CHECK_RESOLVE_FILEOPS(_nvp_);
	DEBUG("_nvp_DUP2(" PFFS_DUP2 ")\n", CALL_DUP2);
	
#if PASS_THROUGH_CALLS
	return _nvp_fileops->DUP2(CALL_DUP2);
#endif

	if(file<0) {
		return _nvp_fileops->DUP(CALL_DUP);
	}

	if(fd2<0) {
		DEBUG("Invalid fd2\n");
		errno = EBADF;
		return -1;
	}

	if(file == fd2)
	{
		DEBUG("Input and output files were the same (%i)\n", file);
		return file;
	}

	struct NVFile* nvf = &_nvp_fd_lookup[file];
	struct NVFile* nvf2 = &_nvp_fd_lookup[fd2];

	if (nvf->posix) {
		DEBUG("Call posix DUP2 for fd %d\n", nvf->fd);
		nvf2->posix = nvf->posix;
		int result = _nvp_fileops->DUP2(CALL_DUP2);
		nvf2->fd = result;
		return result;
	}

	//int iter;

	if(file > fd2)
	{
		NVP_LOCK_FD_WR(nvf);
		NVP_LOCK_FD_WR(nvf2);
	} else {
		NVP_LOCK_FD_WR(nvf2);
		NVP_LOCK_FD_WR(nvf);
	}

	if( (!nvf->valid)||(!nvf2->valid) ) {
		errno = EBADF;
		DEBUG("Invalid FD1 %i or FD2 %i\n", file, fd2);
//		NVP_UNLOCK_FD_WR(nvf);
//		NVP_UNLOCK_FD_WR(nvf2);
	}

	if(nvf->node == nvf2->node || !nvf2->node) {
		NVP_LOCK_NODE_WR(nvf);
	} else {
		if(nvf->node > nvf2->node) {
			NVP_LOCK_NODE_WR(nvf);
			NVP_LOCK_NODE_WR(nvf2);
		} else {
			NVP_LOCK_NODE_WR(nvf2);
			NVP_LOCK_NODE_WR(nvf);
		}
	}

	int result = _nvp_fileops->DUP2(CALL_DUP2);

	if(result < 0)
	{
		DEBUG("_nvp_DUP2 failed to %s->DUP2(%i, %i) "
			"(returned %i): %s\n", _nvp_fileops->name, file,
			fd2, result, strerror(errno));
		NVP_UNLOCK_NODE_WR(nvf);
		if(nvf->node != nvf2->node) { NVP_UNLOCK_NODE_WR(nvf2); }
		NVP_UNLOCK_FD_WR(nvf);
		NVP_UNLOCK_FD_WR(nvf2);
		return result;
	}
	else
	{
		//free(nvf2->offset); // TODO: free this iff it's not in use anymore to avoid memory leaks
	}

	nvf2->valid = 0;
	
	if(nvf2->node && nvf->node != nvf2->node) { NVP_UNLOCK_NODE_WR(nvf2); }

	_nvp_test_invalidate_node(nvf2);

	if(result != fd2)
	{
		WARNING("result of _nvp_DUP2(%i, %i) didn't return the fd2 "
			"that was just closed.  Technically this doesn't "
			"violate POSIX, but I DON'T LIKE IT. "
			"(Got %i, expected %i)\n",
			file, fd2, result, fd2);
		assert(0);

		NVP_UNLOCK_FD_WR(nvf2);

		nvf2 = &_nvp_fd_lookup[result];

		NVP_LOCK_FD_WR(nvf2);

		if(nvf2->valid)
		{
			DEBUG("%s->DUP2 returned a result which corresponds "
				"to an already open NVFile! dup2(%i, %i) "
				"returned %i\n", _nvp_fileops->name,
				file, fd2, result);
			assert(0);
		}
	}

	nvf2->fd = result;
	nvf2->offset = nvf->offset;
	nvf2->canRead = nvf->canRead;
	nvf2->canWrite = nvf->canWrite;
	nvf2->append = nvf->append;
	nvf2->aligned = nvf->aligned;
	nvf2->serialno = nvf->serialno;
	nvf2->node = nvf->node;
	nvf2->node->reference++;
	nvf2->valid = nvf->valid;
	nvf2->posix = nvf->posix;

	SANITYCHECK(nvf2->node != NULL);
	SANITYCHECK(nvf2->valid);

	DEBUG("fd2 should now match fd1. "
		"Testing to make sure this is true.\n");

	NVP_CHECK_NVF_VALID_WR(nvf2);

	DO_MSYNC(nvf);
	DO_MSYNC(nvf2);

	NVP_UNLOCK_NODE_WR(nvf); // nvf2 was already unlocked.  old nvf2 was not the same node, but new nvf2 shares a node with nvf1
	NVP_UNLOCK_FD_WR(nvf2);
	NVP_UNLOCK_FD_WR(nvf);
	
	return nvf2->fd;
}

RETT_IOCTL _nvp_IOCTL(INTF_IOCTL)
{
	CHECK_RESOLVE_FILEOPS(_nvp_);

	DEBUG("CALL: _nvp_IOCTL\n");

	va_list arg;
	va_start(arg, request);
	int* third = va_arg(arg, int*);

	RETT_IOCTL result = _nvp_fileops->IOCTL(file, request, third);

	return result;
}

RETT_UNLINK _nvp_UNLINK(INTF_UNLINK)
{
	struct stat file_st;
	int index;
	struct InodeToMapping* mappingToBeRemoved;
	instrumentation_type unlink_time, clf_lock_time;
	RETT_UNLINK result = 0;

	START_TIMING(unlink_t, unlink_time);

#if PASS_THROUGH_CALLS
	num_unlink++;
        result = _nvp_fileops->UNLINK(CALL_UNLINK);
	END_TIMING(unlink_t, unlink_time);
	return result;
#endif

#if BG_CLOSING
	int hash_index = -1, closed_filedesc = -1, stale_fd = 0;
	ino_t closed_serialno = 0;
#if SEQ_LIST || RAND_LIST
	struct ClosedFiles *clf = NULL;
#else //SEQ_LIST || RAND_LIST
	struct InodeClosedFile *tbl = NULL;
#endif //SEQ_LIST || RAND_LIST
#endif //BG_CLOSING
	
	num_stat++;
	num_unlink++;

	
	CHECK_RESOLVE_FILEOPS(_nvp_);
	DEBUG("CALL: _nvp_UNLINK\n");

	if (stat(path, &file_st) == 0) {					
		index = file_st.st_ino % 1024;
#if BG_CLOSING
		GLOBAL_LOCK_CLOSE_WR();		
		hash_index = file_st.st_ino % TOTAL_CLOSED_INODES;
#if SEQ_LIST || RAND_LIST
		clf = &_nvp_closed_files[hash_index];

#if PRINT_CONTENTION_MSGS		
		printf("%s: acquiring lock for clf entry = %d, thread id = %lu\n", __func__, hash_index, pthread_self());
		fflush(NULL);
#endif
		//START_TIMING(clf_lock_t, clf_lock_time);
		LRU_NODE_LOCK_WR(clf);
		//END_TIMING(clf_lock_t, clf_lock_time);

		stale_fd = remove_from_seq_list_hash(clf, file_st.st_ino);
#else //SEQ_LIST || RAND_LIST
		tbl = &inode_to_closed_file[hash_index];
		NVP_LOCK_HASH_TABLE_WR(tbl);		
		stale_fd = remove_from_lru_list_hash(file_st.st_ino, 0);
#endif	//SEQ_LIST || RAND_LIST	
		if(stale_fd >= 0) {
			closed_filedesc = stale_fd;
			closed_serialno = file_st.st_ino;

			if(!_nvp_REAL_CLOSE(closed_filedesc, closed_serialno, 1)) 
				__atomic_fetch_sub(&num_files_closed, 1, __ATOMIC_SEQ_CST);
		}

#if SEQ_LIST || RAND_LIST
#if PRINT_CONTENTION_MSGS		
		printf("%s: releasing lock for clf entry = %d, thread id = %lu\n", __func__, hash_index, pthread_self());
		fflush(NULL);
#endif
		LRU_NODE_UNLOCK_WR(clf);
#else //SEQ_LIST || RAND_LIST
		NVP_UNLOCK_HASH_TABLE_WR(tbl);
#endif //SEQ_LIST || RAND_LIST
		GLOBAL_UNLOCK_CLOSE_WR();
#endif //BG_CLOSING

		mappingToBeRemoved = &_nvp_ino_mapping[index];

		if(file_st.st_ino == mappingToBeRemoved->serialno && mappingToBeRemoved->root_dirty_num) {
			nvp_free_btree(mappingToBeRemoved->root, mappingToBeRemoved->merkle_root, mappingToBeRemoved->height, mappingToBeRemoved->root_dirty_cache, mappingToBeRemoved->root_dirty_num, mappingToBeRemoved->total_dirty_mmaps);		
			
			mappingToBeRemoved->serialno = 0;
		}
	}
	
	result = _nvp_fileops->UNLINK(CALL_UNLINK);

	END_TIMING(unlink_t, unlink_time);

	return result;
}

RETT_UNLINKAT _nvp_UNLINKAT(INTF_UNLINKAT)
{
	CHECK_RESOLVE_FILEOPS(_nvp_);

	DEBUG("CALL: _nvp_UNLINKAT\n");

	RETT_UNLINKAT result = _nvp_fileops->UNLINKAT(CALL_UNLINKAT);

	return result;
}

RETT_FSYNC _nvp_FSYNC(INTF_FSYNC)
{
	CHECK_RESOLVE_FILEOPS(_nvp_);
	RETT_FSYNC result = 0;
	int cpuid = -1;
	instrumentation_type fsync_time;

	START_TIMING(fsync_t, fsync_time);

#if PASS_THROUGH_CALLS
	num_fsync++;
	result = _nvp_fileops->FSYNC(file);
	END_TIMING(fsync_t, fsync_time);
	return 0;
#endif
	
	// Retrieve the NVFile from the global array of NVFiles
	cpuid = GET_CPUID();
	struct NVFile* nvf = &_nvp_fd_lookup[file];

	// This goes to fsync_flush_on_fsync()	
	FSYNC_FSYNC(nvf, cpuid, 0, 0);
	num_fsync++;
	
	END_TIMING(fsync_t, fsync_time);
	
	return result;
}

RETT_FDSYNC _nvp_FDSYNC(INTF_FDSYNC)
{
	CHECK_RESOLVE_FILEOPS(_nvp_);
	RETT_FDSYNC result = 0;
	int cpuid = -1;
	instrumentation_type fsync_time;

	START_TIMING(fsync_t, fsync_time);

#if PASS_THROUGH_CALLS
	num_fsync++;
	result = _nvp_fileops->FSYNC(file);
	END_TIMING(fsync_t, fsync_time);
	return 0;
#endif
	
	struct NVFile* nvf = &_nvp_fd_lookup[file];

	cpuid = GET_CPUID();
	FSYNC_FSYNC(nvf, cpuid, 0, 1);
	num_fsync++;

	END_TIMING(fsync_t, fsync_time);

	return result;
}

