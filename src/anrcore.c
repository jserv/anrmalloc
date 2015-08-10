/* -*- mode: c; indent-tabs-mode: nil; c-basic-offset: 4; -*- */
/* vim: set expandtab shiftwidth=4 softtabstop=4 : */
/* anrmalloc 
 *
 * Copyright © 2013 Lexmark International
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License version 2.1 as published by the Free Software Foundation
 * (the "LGPL").
 *
 * You should have received a copy of the LGPL along with this library
 * in the file COPYING; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Suite 500, Boston, MA 02110-1335, USA
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY
 * OF ANY KIND, either express or implied.
 *
 * The Original Code is the anrmalloc and gmalloc API, with anrcore library.
 *
 * The Initial Developer of the Original Code is Lexmark International, Inc.
 * Author: Ian Watkins
 *
 * Commercial licensing is available. See the file COPYING for contact
 * information.
 */
/*
 * anr_malloc is a custom allocator focused primarly on being as efficient as
 * possible, while not having a huge impact on run time.  Some enhancements
 * have been specifically to improve performance.
 *
 * Other enhancements over a typical malloc implimentation like dlmalloc are
 * in the debugging area.
 *
 *   * debug features
 *      - anr_malloc maintains a circular buffer of events that have been
 *      processed.  This buffer is maintained outside of the address space
 *      being partitioned for clients.
 *      - anr_malloc implements a mark/sweep garbage collection, and so can
 *          detect memory leaks.
 *      - anr_malloc can optionally maintain statistical data, so memory
 *      consumption can be tracked with some reasonable granularity
 *      - anr_malloc can optionally log an allocation stream, sutiable for post
 *           processing by various other tools.
 *      - anr_malloc can optionally detect buffer over/under run errors.
 *      - anr_malloc can optionally capture debug data for every allocation.
 *
 *    * performance enhancements
 *      - anr_malloc has a built-in slab allocation scheme for small allocations.
 *
 * anr_malloc is conceived as an incremental improvement of p4mem and dlmalloc.
 * Other allocators were surveyed for possible benefits that could be
 * incorporated into anr_malloc include, but are not limited to: jemalloc,
 * umem, hoard, TCmalloc, TLSF malloc, and everything Knuth wrote about dynamic
 * memory allocation in The Art of Computer Programming.
 *
 * anr_malloc was named by Ian Watkins after the collection of people who
 * informed its design and development: Ian Watkins, Scott Arrington, Steven
 * Walter and Howard Cochran.  The letters A, N, and R appear most frequently
 * amongst the group's surnames, and ties were decided by sorting in
 * alphabetical order.
 */

#define _GNU_SOURCE 
#include "anrcore.h"
#include <features.h> /* all include files use this.  We need _XOPEN_SOURCE >=
                       * 500 for pthread_mutexattr_settype, and we need
                       * _BSD_SOURCE or _SVID_SOURCE to be defined to get mmap
                       * working (MAP_ANONYMOUS is undefined if one of those is
                       * not).
                       *
                       * _GNU_SOURCE pulls all that in, but we only resort to
                       * that if we wouldn't get what we need from the compile
                       * environment.
                       */
#include <pthread.h>
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h> /* memset,vnsprintf */
#include <stdarg.h>
#include <sys/mman.h>
#include <unistd.h> /* write */
#include <sys/param.h> /* EXEC_PAGESIZE */
#include <stddef.h>
#include <stdio.h>
#include <sys/types.h>
#include <fcntl.h>


#ifdef HAVE_VALGRIND
#include <valgrind/valgrind.h>
#include <valgrind/memcheck.h>
#include <valgrind/callgrind.h>
#endif

/*****************************************************************************
 * Preliminaries
 ****************************************************************************/
#define BITS_PER_BYTE 8
#define BITS_PER_WORD (BITS_PER_BYTE * sizeof (bitmap_t))
#define SLAB_DEBUG 0 /* enables slab prints that may be helpful. */

/*
 * Macros for alignment, converting bytes to words, and padding the request
 * size
 */

#define INTALIGN_MASK (INTBYTES - 1)

#define OVERHEAD (BYTES_PER_WORD)
#define WOVERHEAD (bytes_to_words (OVERHEAD))

#define NEAREST_N_WORDS(n_bytes) \
    (((n_bytes) + BYTES_PER_WORD - 1) / BYTES_PER_WORD)

#define ALIGNMENT_MASK (WORDBYTES - 1)

#define pad_size(self, size) \
    (((size) + self->overhead + ALIGNMENT_MASK) & ~ALIGNMENT_MASK)

#define misaligned_chunk(ptr) \
    ((((uintptr_t)ptr) & ALIGNMENT_MASK) == (uintptr_t)ptr)


#define bytes_to_words(bytes) \
        ((bytes) / BYTES_PER_WORD)
#define words_to_bytes(words) \
        ((words) * BYTES_PER_WORD)

#define max(a,b) ((a) > (b) ? a : b)
#define min(a,b) ((a) < (b) ? a : b)
/* implementation limits */
#define MIN_BLOCK_SIZE (sizeof (binchunk_t))
#define SMALLBLOCK_MAX (words_to_bytes(SMALLBINS) + MIN_BLOCK_SIZE)
#define MAX_ALLOC (0xFFFFFFFF & ~(INTERNAL_BITS))

#define MIN_LARGEBLOCK_SIZE (SMALLBLOCK_MAX + WORDBYTES)

/****************************************************************************
 * Internal Common Structures.  
 ***************************************************************************/

#define LIST_LINKAGE(type)\
    struct type * next;\
    struct type * prev;

/*
 * The chunk is the basic working structure in the allocator.  Every free
 * allocation has one of these structures at its head. 
 *
 * If you're into C++ and objects, think of this as the base class.
 *
 */
struct chunk{
    uint32_t prev_size;
    uint32_t head;
};
typedef struct chunk chunk_t;

enum tree_color{
    RED = 0,
    BLACK = 1,
    NON_TREE_NODE
};
typedef enum tree_color tree_color_t;

/*
 * The binchunk is used to represent small allocations -- those under 1040
 * bytes in size.
 *
 * These chunks are all in the smallbins.  Each bin holds only 1 size.
 * binchunks are ordered within the bin temporally.  
 */
typedef struct binchunk binchunk_t;
struct binchunk{
    uint32_t prev_size;
    uint32_t head;
    struct {
        LIST_LINKAGE(binchunk);
    } link;
};

/*
 * The largebin struct holds the root of the rb tree for the size at this bin.
 */

/*
 * The treechunk is used to represent large allocations -- from 1040 bytes to 2GB.
 *
 * Like binchunks, treechunks of the same size are linked together in a list
 * sorted temporally.
 *
 * In each treebin, there is a red-black tree of treechunks.
 */

typedef struct treechunk treechunk_t;
struct treechunk{
    uint32_t prev_size;
    uint32_t head;
    struct {
        LIST_LINKAGE (treechunk);
    } list;
    treechunk_t * link[2];
    treechunk_t * parent;
    tree_color_t color;
    struct{
        LIST_LINKAGE (treechunk);
    } dirty;
};

/*
 * The slab pool manages a set of slabs for a particular size class.
 * slab pools pretend to be binchunks and exist in the small bin for the size
 * class they service, some of the time.
 *
 * When a slab pool is exhausted and can service no more allocations (i.e., the
 * last slice from the last slab the pool owns has been allocated)the slab
 * pool is inserted into the searchbin at the tail of the list.
 *
 * As allocs of the size class are made, the slab pool moves nearer to the front.
 *
 * When it reaches the front, if it still has no allocations available, a new
 * slab will be created to service the allocation.  This slab will be inserted
 * into the list at the pools location, and the pool will be taken out of the list.
 *
 * If, when it reaches the head of the list, it can service allocations, the
 * slab with the fewest free spaces will be inserted into the list in its
 * place.
 *
 * ALTERNATE POLICY: the slab that is next temporally.
 */

typedef struct slabpool slabpool_t;
struct slabpool{
    uint32_t n_available;
    uint32_t head;
    struct{
        LIST_LINKAGE (binchunk);
    } link;
    unsigned short slice_size;
    unsigned short slab_size;
    unsigned short n_slices_per_slab;
    unsigned short n_slice_words;
    unsigned short n_bitmap_words;
    uint16_t magic;
    uint16_t shift;
    uint16_t debug_count;
};

/*
 * Slabs manage a set of allocations for a particular class.  They have very
 * low overhead -- 2 bits per allocation + this header / allocations per slab.
 *
 * slabs also pretend to be binchunks and exist in the smallbins.
 *
 * The bitmaps in the slab are dynamically sized and exist directly after the
 * slab header.
 */
struct slab{
    uint32_t prev_size;
    uint32_t head;
    struct {
        LIST_LINKAGE (binchunk);
    }link;
    slabpool_t * pool;
    unsigned short n_free;
    unsigned short last_bitmap_word;
};
typedef struct slab slab_t;
struct lbin{
    treechunk_t * root;
    uint32_t max;
};

typedef struct lbin lbin_t;
/* 
 * The freechunk structure has 1 purpose: allow calls to free to be
 * reasonably non-blocking
 */
typedef struct freechunk freechunk_t;
struct freechunk{
    freechunk_t * next;
};
/* 
 * The mapping structure holds meta data about every mmap()'d area being managed.
 * It is used to do error checking and also holds the slabmap, which indicates
 * that a particular page is being used as a slab 
 *
 * Mappings are linked in a singly linked list.
 *
 */ 
typedef struct mapping mapping_t;
struct mapping{
    mapping_t * next;  
    chunk_t * fence; /* the top chunk for this mapping */
    void * top;      /* address returned by mmap() */
    void * bottom;   /* fencepost at the bottom of the mapping */
    uint32_t mapwords; /* number of words in the slab map */
    bitmap_t * slabmap; /* pointer to the slab map for this mapping */
    bitmap_t * dirtymap;
    uint32_t map_pages; /* number of pages in this mapping */
    uint32_t total_words; /* total words in this mapping that are usable */
    uint32_t size;      /* size (in words) of this mapping */
    uint32_t fake;      /* fakeout word for check bounds */
};

typedef struct event event_t;
struct event{
    void * return_function;
    void * address;
    uint32_t count;
    size_t size;
    event_t * next;
};

enum __event_types{
    ALLOC_BIT = 1,
    FREE_BIT = 1 << 1,
    EVENT_BITS = ALLOC_BIT | FREE_BIT    
};


#define ADDRESS(ptr)\
    ((void*)(((uintptr_t)(ptr)) & ~(EVENT_BITS)))

#define IS_ALLOC_EVENT(ptr)\
    ((void *)(((uintptr_t)(ptr)) & (ALLOC_BIT)))

#define IS_FREE_EVENT(ptr)\
    ((void *)(((uintptr_t)(ptr)) & (FREE_BIT)))

#define ALLOC_EVENT(ptr)\
    ((void *)(((uintptr_t)(ptr)) | (ALLOC_BIT)))

#define FREE_EVENT(ptr)\
    ((void *)(((uintptr_t)(ptr)) | (FREE_BIT)))


/*
 * Malloc state holds all of our internal state.
 * 
 * TODO DIAGRAM!!!!!
 *
 * The state has 2 bitmaps - 1 for each searchbin catagory.  If the bin has
 * anything in it, the corresponding bit will be 1, and if it is empty, the bit
 * will be 0.
 *
 * This lets us do best fit with bitmath, avoiding cacheline misses that would
 * be incurred searching through all the bins.
 *
 * The accounting for the allocator is also here.
 *
 * NOTE: we treat the malloc state as a tree chunk in the dirty list.
 * This means that the treechunk list linkage needs to be in exactly the same place
 * in tree chunks and in the malloc state.  If not, crashes WILL occur.
 *
 * DO NOT MOVE MEMBERS OF THIS STRUCT.
 *
 */

#define NEAREST_N_BITS(cnt)((cnt + BITS_PER_WORD - 1) / BITS_PER_WORD)
#define SMALLBINS 256
#define LARGEBINS 44

struct malloc_state{
    uint32_t flags;                              /* internal state - no memory, 
                                                    performing giveback, etc */
    freechunk_t * free_list;                     /* the free list */
    int32_t available_pages;                    /* number of available pages */
    int32_t reclaimable_pages;                  /* subset of available pages that
                                                    can easily be undirtied */
    int32_t total_pages;                        /* total pages */
    uint32_t available_words;                    /* available words */
    uint32_t total_words;                        /* total words */
    mapping_t * mappings;                        /* mappings being managed */
    treechunk_t dirty;   
    bitmap_t smallmap[NEAREST_N_BITS(SMALLBINS)];  /* The searchbins and their bitmasks  */
    bitmap_t largemap[NEAREST_N_BITS(LARGEBINS)]; 
    binchunk_t smallbins[SMALLBINS];            
    lbin_t largebins[LARGEBINS];                
                                             
    slab_t * slab_pool;                          /* slab for slab pool structures */
    event_t * history_head;                      /* event history head. */
    event_t * history_tail;                      /* event history tail */
    MoreMemoryFunction _more_memory;             /* more mem function and context */
    void * cb_context;
    uint32_t malloccompare;                      /* malloc compare */
    uint32_t malloccount;                        /* malloc count */
    uint32_t freecount;                          /* free count */
    int32_t dirty_page_ceiling;                 /* when we have this many dirty pages, we clean some */
    uint32_t overhead;                           /* overhead per allocation */
    pthread_mutex_t heap_lock;                   /* locks for the heap and freelist */
    pthread_mutex_t freelist_lock;
    pthread_cond_t list_populated;
#ifdef HAVE_VALGRIND
    int32_t def_count;
#endif
};


enum __malloc_state_flags{
    MALLOC_ERROR_BIT = 1,
    BOUNDS_CHECK_BIT = 1<< 1,
    VERIFY_BIT = 1 << 2,
    TRASH_BIT = 1 << 3
};
#define set_malloc_error(mstate)\
    ((mstate)->flags |= MALLOC_ERROR_BIT)
#define clear_malloc_error(state)\
    ((state)->flags &= ~MALLOC_ERROR_BIT)
#define error_state(state)\
    ((state)->flags & MALLOC_ERROR_BIT)
#define check_bounds(state)\
    ((state)->flags & BOUNDS_CHECK_BIT)
#define enable_bounds_check(state)\
    ((state)->flags |= BOUNDS_CHECK_BIT)
#define verify(state)\
    ((state)->flags & VERIFY_BIT)
#if OS_IS_WS > 0
#define enable_verify(state) \
    do {\
        ((state)->flags |= VERIFY_BIT);\
        do_verify = 1;\
    }while (0)
#else
#define enable_verify(state) \
    ((state)->flags |= VERIFY_BIT)
#endif

#define enable_trash(state) \
    ((state)->flags |= TRASH_BIT)
#define disable_trash(state) \
    ((state)->flags &= ~TRASH_BIT)
#define fill_with_trash(state) \
    ((state)->flags & TRASH_BIT)

/* Macros to simplify header state management */

enum __block_head_bits{
    IN_USE_BIT = 1 << ((BITS_PER_WORD)- 1),
    PREV_IN_USE_BIT = 1 << ((BITS_PER_WORD)- 2),
    MARK_BIT = 1 << ((BITS_PER_WORD)- 3),
    SLAB_BIT = 1 << ((BITS_PER_WORD)- 4),
    INTERNAL_BITS = IN_USE_BIT | PREV_IN_USE_BIT | MARK_BIT | SLAB_BIT
};

#define chunk_is_slab(block)\
    ((block)->head & SLAB_BIT)

#define chunk_in_use(block)\
    ((block)->head & IN_USE_BIT)

#define prev_chunk_in_use(block)\
    ((block)->head & PREV_IN_USE_BIT)

#define next_chunk_in_use(block)\
    (next_chunk ((block))->head & IN_USE_BIT)

#define chunk_marked(block)\
    ((block)->head & MARK_BIT)

#define mark_block(block)\
    ((block)->head |= MARK_BIT)

#define clear_mark(block)\
    ((block)->head &= ~MARK_BIT)

#define set_slab(block)\
    ((block)->head |= SLAB_BIT)


/* conversion from block pointers to user space and back */
#define chunk_to_mem(block)\
    (void *)(((uintptr_t)block) + sizeof (chunk_t))

#define mem_to_chunk(mem)\
    (chunk_t *)(((uintptr_t)mem)- sizeof (chunk_t))

/*****************************************************************************
 * Internal Functions --  Macros
 ****************************************************************************/

#define chunk_cast(ptr)\
    ((chunk_t *)(ptr))

#define bin_cast(chunk)\
    ((binchunk_t *)(chunk))

#define tree_cast(ptr)\
    ((treechunk_t *)(ptr))

#define slab_cast(ptr)\
    ((slab_t *)(ptr))

#define prev_chunk_words(block)\
    ((block)->prev_size)
#define prev_chunk_bytes(block)\
    words_to_bytes (prev_chunk_words (block))

#define chunk_words(block)\
    ((block)->head & ~(INTERNAL_BITS))

#define user_words(self, block)\
    (bytes_to_words (user_bytes (self, block)))

#define user_bytes(self, block)\
    (chunk_bytes (block)- self->overhead)

#define chunk_bytes(block)\
    words_to_bytes (chunk_words (block))

/* get the next chunk in the address space */
#define next_chunk(block)\
    ((chunk_t *)(((uintptr_t)(block)) + (chunk_bytes (block))))

/* get the previous chunk in the address space */
#ifdef HAVE_VALGRIND
#define prev_chunk(block)\
    ((chunk_t *)(RUNNING_ON_VALGRIND && prev_chunk_in_use (block) ? chunk_cast (NULL): \
    (chunk_cast (((uintptr_t)(block))- (prev_chunk_bytes (block))))))
#else
#define prev_chunk(block)\
    ((chunk_t *)(((uintptr_t)(block))- (prev_chunk_bytes (block))))
#endif

/* get the next binchunk in the list */
#define next_binchunk(block)\
    bin_cast (block)->link.next

#define prev_binchunk(block)\
    bin_cast (block)->link.prev

#define set_prev_in_use(block)\
    (block)->head |= (PREV_IN_USE_BIT)

/* set a chunk in use.  also sets the prev in use bit of the next block */
#define set_in_use(block)\
    do { \
        chunk_t * next = next_chunk (block); \
        set_prev_in_use (next); \
        (block)->head |= (IN_USE_BIT); \
    } while (0)  

#define set_prev_available(block)\
    (block)->head &= ~(PREV_IN_USE_BIT)

/* set a chunk as available.  also unsets the proper bit of the next block */
#define set_available(block)\
    do { \
        chunk_t * next = next_chunk(block); \
        set_prev_available (next); \
        next->prev_size = chunk_words (block); \
        (block)->head &= ~(IN_USE_BIT | MARK_BIT); \
    } while (0)

#define new_block_from_offset(block, offset)\
    ((void *)(((uintptr_t)block) + (offset)))


#define int_aligned_adder_from_offset(addr, offset)\
    ((void *)((uintptr_t)addr + (((offset) + INTALIGN_MASK) & ~INTALIGN_MASK)))

#define word_aligned_adder_from_offset(addr, offset)\
    ((void *)((uintptr_t)addr + (((offset) + ALIGNMENT_MASK) & ~ALIGNMENT_MASK)))


#define slice_to_slab(ptr)\
    ((slab_t *)(((uintptr_t)(ptr)) & ~PAGE_MASK))

#define ptr_to_page(ptr)\
   ((uintptr_t)(((uintptr_t)(ptr)) & ~PAGE_MASK))

#define ptr_to_slab(ptr)\
    ((slab_t *)ptr_to_page(ptr))

#define ptr_plus_offset_crosses_page(ptr, offset)\
    (((uint32_t)(PAGESIZE_BYTES - (((uintptr_t)(ptr)) & PAGE_MASK))) < (offset))

#define new_chunk_overhead(bytes)\
    ((bytes) > SMALLBLOCK_MAX ? sizeof (treechunk_t): sizeof (binchunk_t))

#define index2word(index)((index) >> 5)
#define index_mask(index)((index) & ~(BITS_PER_WORD - 1))
#define index2bit(index)((index - (index_mask (index))))

/* these two defines are the source of some confusion (even for the author).
 * Here's the deal: they mask the bits right or left of the index (inclusive),
 * respectively.
 *
 * So, mask_right (0xffffffff, 12) = 0xfffff000 -- it masked OFF the bits right
 * of the 12th bit.   Note that 1 << 12 = 0x1000, so the 12th bit is ON. 
 *
 * mask_left (0xffffffff, 12) = 0x00001fff -- it masked OFF
 * the bits LEFT of the 12th bit.  Here, the 12th bit is ON
 *
 * Note that this means that for all indexes, 
 * mask_right (val, [index]) & mask_left (val, [index]) == (1 << [index]) & val
 */
#if 1
#define mask_right(map, idx)((map) & (-((1U << (idx)))))
#define mask_left(map, idx)((map) & (~mask_right (0xffffffff, (idx)) | 1U << (idx)))
#else
/* these functions should produce the same results as the above macros.  They
 * exist for performace checking, and validation in a debugger.
 */
uint32_t mask_right(bitmap_t map, uint32_t index) {
    return map & (-(1U << index));
}
uint32_t mask_left(bitmap_t map, uint32_t index) {
    return map & (((~mask_right (map, index))) | (1U << index)); 
}
#endif



#ifndef MALLOC_DEBUG
#define MALLOC_DEBUG 0
#endif

#if MALLOC_DEBUG

static inline void dirty_list_unlink(treechunk_t *node) ALWAYS_INLINE;
static inline void
dirty_list_unlink(treechunk_t *node)
{
    treechunk_t * next = node->dirty.next;
    treechunk_t * prev = node->dirty.prev;
    next->dirty.prev = prev;
    prev->dirty.next = next;
    node->dirty.next = node;
    node->dirty.prev = node;
}

static inline void dirty_list_link(treechunk_t *, treechunk_t *) ALWAYS_INLINE;
static inline void
dirty_list_link(treechunk_t *node, treechunk_t * next)
{
    treechunk_t * prev = next->dirty.prev;
    node->dirty.next = next;
    next->dirty.prev = node;
    prev->dirty.next = node;
    node->dirty.prev = prev;
}
#else
#define dirty_list_unlink(node)\
    do { \
        treechunk_t * next = node->dirty.next; \
        treechunk_t * prev = node->dirty.prev; \
        next->dirty.prev = prev; \
        prev->dirty.next = next; \
        node->dirty.next = node; \
        node->dirty.prev = node; \
    } while (0)

#define dirty_list_link(node, __next)\
    do { \
        treechunk_t * prev = (__next)->dirty.prev; \
        (node)->dirty.next = __next; \
        (__next)->dirty.prev = node; \
        (prev)->dirty.next = node; \
        (node)->dirty.prev = prev; \
    } while (0)

#endif

#if MALLOC_DEBUG > 0 
#define address_ok(mstate, ptr)(find_mapping (mstate, ptr)!= NULL)
#else
#define address_ok(state, ptr)1
#endif
/****************************************************************************
 * Lock and Signal management
 **************************************************************************/

#define LOCK(lock)\
    pthread_mutex_lock (lock)

#define UNLOCK(lock)\
    pthread_mutex_unlock (lock)

#define TRYLOCK(lock)\
    pthread_mutex_trylock (lock)

#define WAIT(cond, lock)\
    pthread_cond_wait (cond, lock)

#define SIGNAL(cond)\
    pthread_cond_signal (cond)

/***************************************************************************
 * Internal Function Declarations.
 **************************************************************************/


#define debug(...)if (SLAB_DEBUG)malloc_printf (1, __VA_ARGS__)

static chunk_t * find_treechunk (malloc_state_t *self, size_t); 
static void insert_chunk (malloc_state_t * self, chunk_t * block);
static inline void local_abort (const char *, unsigned int, const char *)__attribute__((noreturn));
AbortFunction _abort = (AbortFunction)local_abort;

#define assert(a)anr_assert(a)

#define crash(...)anr_crash(__VA_ARGS__)


#if MALLOC_DEBUG > 0
#define debug_assert(a)assert((a))
#else
#define debug_assert(a)
#endif

#ifndef HEAP_VERIFY
#define HEAP_VERIFY 0
#endif

#if HEAP_VERIFY > 0
#define memory_debug_assert(a) assert((a))
#elif OS_IS_WS > 0
static int do_verify = 0;
#define memory_debug_assert(a)\
    do{\
        if (RARELY(1 == do_verify)) {\
            assert((a));\
        }\
    }while (0)
#else
#define memory_debug_assert(a)
#endif

/*****************************************************************************
 * UTILITY Functions
 ****************************************************************************/

/**
 * Find the mapping that owns a pointer.
 * @param self the malloc state.
 * @param block address to find.
 * @return the mapping that contains the pointer, or NULL if the pointer is unknown
 */

static inline mapping_t * find_mapping (malloc_state_t * self, 
                                        void * block)ALWAYS_INLINE;
static inline mapping_t *
find_mapping (malloc_state_t * self, void * block)
{
    mapping_t * mapping;

    /* Most of the time, there is 1 mapping.
     * So the for loop below will only execute once.
     */
    for (mapping = self->mappings; mapping != NULL; mapping = mapping->next) {
        if (USUALLY ((uintptr_t)block >= (uintptr_t)mapping->fence 
                     && (uintptr_t)mapping->bottom  >= (uintptr_t)block)) {
            return mapping;
        }
    }

    return NULL;
}
    
static inline bool alloc_is_slice (malloc_state_t * self, void * ptr)ALWAYS_INLINE;
static inline bool
alloc_is_slice(malloc_state_t * self, void * ptr)
{
    mapping_t * mapping = find_mapping (self, ptr);
    uintptr_t page_addr = ptr_to_page (ptr);
    uint32_t offset = page_addr - ((uintptr_t)mapping->top);

    offset /= PAGESIZE_BYTES;

    memory_debug_assert (index2word (offset) < mapping->mapwords);

    if (mapping->slabmap[index2word (offset)] & 1 << index2bit (offset)) {
        slab_t * slab = (slab_t *)page_addr;
        if (USUALLY (slab->pool->slab_size > ((uintptr_t)ptr)- page_addr))
            return true;
    }
    return false;
}
/*
 * clz - count leading zeros.
 * ctz - count trailing zeros.
 * large_index - compute the index into a tree bin.
 *
 * Each of these functions uses the appropriate asm call when possible.
 *
 * clz was pulled from lexlibc.
 *
 * ctz(x)is equilivant to 32 - clz (~x & (x - 1))
 *
 * large_index is clz, followed by some logical ops on the returned value.
 */
static inline unsigned int clz (unsigned int x)PURE_INLINE;
static inline unsigned int large_index (unsigned int)PURE_INLINE;
static inline unsigned int ctz (unsigned int x)PURE_INLINE;

#if defined (__GNUC__) && defined (i386)
static inline unsigned int
clz (unsigned int x)
{
    unsigned int i;
    static const int negone = -1;
    __asm__ ("bsrl %1, %0\n\t"
             "cmovz %2,%0" : "=&r" (i): "rm" (x), "r"(negone)); 
    return  31 - i ;
}
static inline unsigned int
ctz (unsigned int x)
{
    unsigned int i;
    static const unsigned int thirtytwo = BITS_PER_WORD;
    __asm__ ("bsf %1, %0\n\t"
             "cmovz %2,%0" : "=&r" (i): "rm" (x), "r"(thirtytwo)); 
    return i;

}
static inline unsigned int 
large_index (unsigned int x)
{
    unsigned int i = BITS_PER_WORD - clz (x)- 1; 

    return ((i - 8U) << 1U) + ((x >> (i - 1U)) & 1U);
}
#elif defined(_PPC)
static inline unsigned int 
clz (unsigned int x)
{
    unsigned int i;
    __asm__ ("cntlzw %0,%1" : "=r" (i) : "r" (x));
    return i;
}
static inline unsigned int
ctz (unsigned int x)
{
    unsigned int i = ((~x) & (x - 1));
    __asm__ ("cntlzw %0,%1" : "=r" (i): "r" (i));
    return BITS_PER_WORD - i;

}
static inline unsigned int 
large_index (unsigned int x)
{
    unsigned int i;
    __asm__ ("cntlzw %0,%1" : "=r" (i) : "r" (x));
    i = 31 - i;
    return ((i -8U) << 1U) + ((x >> (i-1U)) &1U);
}
#elif defined(_ARM) && defined(CONFIG_USE_ARM_CLZ)
static inline unsigned int 
clz (unsigned int x)
{
    unsigned int i;
    __asm__ ("clz %0,%1": "=r"(i): "r"(x));
    return i;
}
static inline unsigned int
ctz (unsigned int x)
{
    unsigned int i = ((~x) & (x - 1));
    __asm__ ("clz %0,%1": "=r"(i): "r"(i));
    return  BITS_PER_WORD - i;
}
static inline unsigned int 
large_index(unsigned int x)
{
    unsigned int i;
    __asm__ ("clz %0,%1": "=r"(i): "r"(x));
    i = 31 - i;
    return  ((i - 8U) << 1U) + ((x >> (i - 1U)) & 1U);
}
#else
static inline unsigned int
clz (unsigned int x)
{
    union _uf{
        double f;
        unsigned u[2];
    } uf;

    if (RARELY(x == 0)) {
        return 32; 
    } else {
        uf.f = (double)x;
        return 31 - ((uf.u[1] >> 20)- 1023);
    }
}
static inline unsigned int
ctz (unsigned int x)
{
    unsigned int i = ((~x) & (x - 1));

    return BITS_PER_WORD - clz (i);
}
static inline unsigned int 
large_index(unsigned int x)
{
    int y, m, n;
    unsigned int i;
    unsigned int z = x;

    y = -(x >> 16);
    m = (y >> 16) & 16;
    n = 16 - m;
    x >>= m;

    y = x - 0x100;
    m = (y >> 16) & 8;
    n+= m;
    x <<= m;

    y = x - 0x1000;
    m = (y >> 16) & 4;
    n += m;
    x <<= m;

    y = x - 0x4000;
    m = (y >> 16) & 2;
    n += m;
    x <<= m;

    y = x >> 14;
    m = y & ~(y >> 1);
    i  = 31 - (unsigned)(n +2 -m);
    return ((i - 8U) << 1U) + ((z >> (i - 1U)) & 1U);
}
#endif

/*!
 * population count
 * Returns the number of bits that are on in a 32 bit word.
 */
static inline unsigned int population_count (uint32_t val)PURE_INLINE;

#if defined(CONFIG_USE_ARM_POPCNT)
static inline unsigned int 
population_count (unsigned int val)
{
  unsigned int rc;
  __asm__ ("mrc p6,2,%0,c%1,c%1,0" :"=r"(rc): "r"(val));

  return rc;
}
#else
static inline unsigned int 
population_count (unsigned int val)
{

    uint32_t result;

    result = (val >> 1) & 0x77777777;
    val -= result;
    result = (result >> 1) & 0x77777777;
    val -= result;
    result = (result >> 1) & 0x77777777;
    val -= result;
    val = (val + (val >> 4)) & 0x0f0f0f0f;
    val *=  0x01010101;

    return val >> 24;
}
#endif /* CCXF_USE_ARM_POPCNT */
/**
 * @internal
 * Round a number up to the next power of 2.
 * Used to do internal aligned allocations.
 *
 * From Hacker's Delight by Harry S. Warren.
 */
static inline unsigned int 
pow2ceil(unsigned int x)
{
    x -= 1;
    x = x | (x >> 1);
    x = x | (x >> 2);
    x = x | (x >> 4);
    x = x | (x >> 8);
    x = x | (x >> 16);
    return x+1;
}
/*
rb comparisions => largebin index: start size -> (end size) 

    8 =>[ 0]:  1.00 KiB -> (1.50 KiB)
    8 =>[ 1]:  1.50 KiB -> (2.00 KiB)
    9 =>[ 2]:  2.00 KiB -> (3.00 KiB)
    9 =>[ 3]:  3.00 KiB -> (4.00 KiB)
    10=>[ 4]:  4.00 KiB -> (6.00 KiB)
    10=>[ 5]:  6.00 KiB -> (8.00 KiB)
    11=>[ 6]:  8.00 KiB -> (12.00 KiB)
    11=>[ 7]: 12.00 KiB -> (16.00 KiB)
    12=>[ 8]: 16.00 KiB -> (24.00 KiB)
    12=>[ 9]: 24.00 KiB -> (32.00 KiB)
    13=>[10]: 32.00 KiB -> (48.00 KiB)
    13=>[11]: 48.00 KiB -> (64.00 KiB)
    14=>[12]: 64.00 KiB -> (96.00 KiB)
    14=>[13]: 96.00 KiB -> (128.00 KiB)
    15=>[14]: 128.00 KiB -> (192.00 KiB)
    15=>[15]: 192.00 KiB -> (256.00 KiB)
    16=>[16]: 256.00 KiB -> (384.00 KiB)
    16=>[17]: 384.00 KiB -> (512.00 KiB)
    17=>[18]: 512.00 KiB -> (768.00 KiB)
    17=>[19]: 768.00 KiB -> (1024.00 KiB)
    18=>[20]:  1.00 MiB -> (1.50 MiB)
    18=>[21]:  1.50 MiB -> (2.00 MiB)
    19=>[22]:  2.00 MiB -> (3.00 MiB)
    19=>[23]:  3.00 MiB -> (4.00 MiB)
    20=>[24]:  4.00 MiB -> (6.00 MiB)
    20=>[25]:  6.00 MiB -> (8.00 MiB)
    21=>[26]:  8.00 MiB -> (12.00 MiB)
    21=>[27]: 12.00 MiB -> (16.00 MiB)
    22=>[28]: 16.00 MiB -> (24.00 MiB)
    22=>[29]: 24.00 MiB -> (32.00 MiB)
    23=>[30]: 32.00 MiB -> (48.00 MiB)
    23=>[31]: 48.00 MiB -> (64.00 MiB)
    24=>[32]: 64.00 MiB -> (96.00 MiB)
    24=>[33]: 96.00 MiB -> (128.00 MiB)
    25=>[34]: 128.00 MiB -> (192.00 MiB)
    25=>[35]: 192.00 MiB -> (256.00 MiB)
    26=>[36]: 256.00 MiB -> (384.00 MiB)
    26=>[37]: 384.00 MiB -> (512.00 MiB)
    27=>[38]: 512.00 MiB -> (768.00 MiB)
    27=>[39]: 768.00 MiB -> (1024.00 MiB)
    28=>[40]: 1024.00 MiB -> (1536.00 MiB)
    28=>[41]: 1536.00 MiB -> (2048.00 MiB)
    29=>[42]: 2048.00 MiB -> (3072.00 MiB)
    29=>[43]: 3072.00 MiB -> (4096.00 MiB)

    This is in the worst case-- that is, if each tree at each index is full.
    The rationale is this: if the free chunk is big, there probably aren't many
    of them in the heap, so more comparsons for a potentially larger tree are fine.

    For the 1MiB case (18 comparisons)that means we have 262,143 (2^18 -1)blocks
    between 1MiB and 1.5MiB, or over 260 MiB in the heap.  That's pretty fragmented, if
    that happens.

    The tree looks something like this:

                 .----------.
                 |  Parent  |
                 |   Left   |
                 |  Right   |\
                 | Previous | \
                 |   Next   |  \
                 '----------'   \
                                 \
                                  v
        .----------.         .----------.          .----------.
        |  Parent  |         |  Parent  |          |  Parent  |
        |   Left   |         |   Left   |          |   Left   |
        |  Right   |         |  Right   |          |  Right   |
  .---->| Previous |<------->| Previous |<-------->| Previous |<---.
  |     |   Next   |         |   Next   |          |   Next   |    |
  |     '----------'         '----------'          '----------'    |
  |                          /         \                           |
  '-------------------------/-----------\--------------------------'
                           /             \
                          /               \
                         v                 v
                  .----------.             .----------.
                  |  Parent  |             |  Parent  |
                  |   Left   |             |   Left   |
                  |  Right   |             |  Right   |
                  | Previous |             | Previous |
                  |   Next   |             |   Next   |
                  '----------'             '----------'

Each size is in the tree only once.  Blocks of the same size are linked into
the prev/next pointers.  Blocks smaller/larger than the current node are the
left/right links respectively

 */
/********************************************************************
 * Red Black Tree Code
 * Based heavily on samples in the public domain on eternallyconfuzzled.com
 *******************************************************************/
#define largeblock_initialize(block)\
do { \
    block.prev_size = 0; \
    block.head = 0; \
    block.list.next = 0; \
    block.list.prev = 0; \
    block.link[0] = 0; \
    block.link[1] = 0; \
    block.color = 0; \
} while (0)

/*!
 * Is a node red?
 */
static inline bool red_node (treechunk_t *)PURE_INLINE;
static inline bool
red_node (treechunk_t * block)
{
    return (block != NULL && block->color == RED);
}

/*!
 * Single rotate
 */
static inline treechunk_t *
rb_single_rotate (treechunk_t * root, int dir)
{
    treechunk_t * temp = root->link[!dir];

    root->link[!dir] = temp->link[dir];
    if (root->link[!dir]) {
        root->link[!dir]->parent = root;
    }
    temp->link[dir] = root;
    temp->parent = root->parent;
    root->parent = temp;
    root->color = RED;
    temp->color = BLACK;

    return temp;
}
/*!
 * double rotate
 */

static inline treechunk_t *
rb_double_rotate(treechunk_t * root, int dir)
{
    root->link[!dir] = rb_single_rotate (root->link[!dir], !dir);
    return rb_single_rotate(root, dir);
}
/*!
 * insert treechunk block into a tree.
 * @param block the treechunk to insert.
 * @param tree the root of the tree to insert into.
 * @return 0 for success, 1 for failure.
 */
static inline int
rb_insert(struct lbin * tree, treechunk_t * block)
{

    if (!block)return 0;

    block->list.next = block->list.prev = block;
    block->link[0] = block->link[1] = block->parent = NULL;
    block->color = RED;

    if (! tree->root) {
        tree->root = block;
        tree->max = chunk_words(block);
    }else{
        treechunk_t head;
        treechunk_t * parent;
        treechunk_t * local_root;
        treechunk_t * iter;
        treechunk_t * grand;
        int direction = 0;
        int last = 0;
    
        largeblock_initialize(head);

        local_root = &head;
        grand = parent = NULL;
        iter = local_root->link[1] = tree->root;

        iter->parent = local_root;

        while (1) {
            if (iter == NULL) {
                parent->link[ direction ] = block;
                iter = block;
                block->parent = parent;
            } else if (red_node(iter->link[0])
                       && red_node(iter->link[1])) {
                iter->color = RED;
                iter->link[0]->color = iter->link[1]->color = BLACK;
            }

            if (red_node (iter)
                && red_node (parent)) {
                int dir = (local_root->link[1] == grand); 

                if (iter == parent->link[ last ]) {
                    local_root->link[dir] = rb_single_rotate (grand, !last);
                } else {
                    local_root->link[dir] = rb_double_rotate (grand, !last);
                }
            }

            last = direction;
            direction = chunk_words (iter) < chunk_words (block);

            if (iter == block) { 
                tree->max = max (tree->max, chunk_words (block));   
                break;
            }
            if (chunk_words (iter) == chunk_words (block)) {
                treechunk_t * next = iter->list.next;
                iter->list.next = block; 
                next->list.prev = block;
                block->list.next = next;
                block->list.prev = iter;
                block->parent = 0;
                block->color = NON_TREE_NODE;
            

                break;
            }

            if (grand)
                local_root = grand;
            grand = parent; parent = iter;
            iter = iter->link[direction];
        }

        tree->root = head.link[1];
    }

    tree->root->color = BLACK;
    tree->root->parent = 0;
    return 1;
}
/*! Unlink a treechunk
 * @param chunk the chunk to unlink.
 * Helper function that keeps parent, children and siblings straight as we
 * unlink a node.
 */

static inline void unlink_treechunk(treechunk_t * chunk)ALWAYS_INLINE;
static inline void
unlink_treechunk(treechunk_t * chunk)
{
    treechunk_t * next = chunk->list.next;
    treechunk_t * prev = chunk->list.prev;

    prev->list.next = next;
    next->list.prev = prev;

    if (chunk->color != NON_TREE_NODE) {
        treechunk_t * parent = chunk->parent;
        treechunk_t * left  = chunk->link[0];
        treechunk_t * right = chunk->link[1];
        prev->parent = parent;
        prev->link[0] = left;
        prev->link[1] = right;

        prev->color = chunk->color;

        if (parent)
            parent->link[ chunk == parent->link[1]] = prev;

        if (left)
            left->parent = prev;
        if (right)
            right->parent = prev;

    }
}

/*! Remove a block of the specified size from a tree.
 * @param tree the tree to search.
 * @param words the minimum size needed.
 * @return NULL if no chunk large enough found.  Chunk if successfull.
 */
static inline treechunk_t *
rb_remove_size(struct lbin * tree, uint32_t words)
{
    treechunk_t local_root;
    treechunk_t * parent;
    treechunk_t * grand;
    treechunk_t * exact_fit;
    treechunk_t * best_fit;
    treechunk_t * iter;
    int direction=1 ;

    exact_fit = best_fit = NULL;
    debug_assert(tree->root != NULL);

    /* easy case where the root is what we're looking for */
    if (chunk_words(tree->root) >= words 
        && tree->root->link[0] == 0 
        && tree->root->link[1] == 0) {
        dirty_list_unlink(tree->root);
        exact_fit = tree->root;

        if (exact_fit->list.next != exact_fit) {
            unlink_treechunk(exact_fit);
            tree->root = exact_fit->list.prev;
        } else {
            tree->root = NULL;
        }
        return exact_fit;
    }

    largeblock_initialize(local_root);

    iter = &local_root;

    grand = parent = NULL;
    iter->link[1] = tree->root;

    /* this probably seems weird, but it is key to keeping sanity and
     * eliminating special case code all over the place.  we clean it up at the
     * end if we still have a root.
     */
    tree->root->parent = iter;

    while (iter->link[ direction ]) {
        int last = direction;

        grand = parent;
        parent = iter;
        iter = iter->link[direction];

        direction = (chunk_words(iter) < words);

        if (chunk_words(iter) > words)
            best_fit = iter;


        if (chunk_words(iter) == words) {
            exact_fit = iter;
        }

        if (!red_node(iter)
            && !red_node(iter->link[direction])) {
            if (red_node(iter->link[!direction])) {
                parent->link[ last ] = rb_single_rotate(iter, direction);
                parent = parent->link[last];
            } else if (!red_node(iter->link[ !direction ])) {

                treechunk_t * temp = parent->link[ !last ];
                if (temp) {
                    if (! red_node (temp->link[! last])
                        && ! red_node(temp->link[last])) {
                        parent->color = BLACK;
                        temp->color = RED;
                        iter->color = RED;
                    }
                    else {
                        int dir2 = grand->link[1] == parent;

                        if (red_node (temp->link[last ])) {
                            grand->link[dir2] = rb_double_rotate(parent, last);
                        } else if (red_node (temp->link[ !last ])) {
                            grand->link[dir2] = rb_single_rotate(parent, last);
                        }

                        iter->color = RED;
                        grand->link[dir2]->color = RED;
                        grand->link[dir2]->link[0]->color = BLACK;
                        grand->link[dir2]->link[1]->color = BLACK;
                    }
                }
            }
        }
    }

    
    exact_fit = exact_fit ? exact_fit:best_fit;

    /* either we've found an exact fit, and it is the only such chunk in the
     * tree, or we've found a decent (best)fit, and have no idea what it's
     * state is.  unlink_treechunk()is used if we have a list.  otherwise, we
     * have more work to do. 
     *
     * exact fit is the node we want to remove.  we're sneaky and push iter
     * into exact_fits spot.
     */

    if (exact_fit) {

        /* first check to see if exact fit is in a list of nodes.
         * if it is, we're done.
         */

        if (exact_fit != exact_fit->list.next)
            unlink_treechunk(exact_fit);
        else {
            /* not a list.  so, we actually have to pull it out of the tree.
            */

            /* first, we make the parent of the exact fit point at the replacement.
            */
            if (exact_fit->parent) {
                int dir = exact_fit == exact_fit->parent->link[1];
                exact_fit->parent->link[ dir ] = iter;
            } 

            /* next, we make the iter's parent point at the only child of iter --
             * that is, we replace iter with it's child. */
            parent->link[ iter == parent->link[1]] = iter->link[ iter->link[0] == NULL ];

            /* now, we move iter to exact_fit's spot */

            iter->parent = exact_fit->parent;
            iter->link[0] = exact_fit->link[0];
            iter->link[1] = exact_fit->link[1];

            /* last, we make sure the back pointers of iter's new children are pointing it iter */

            if (exact_fit->link[0])
                exact_fit->link[0]->parent = iter;
            if (exact_fit->link[1])
                exact_fit->link[1]->parent = iter; 

            /* final: keep the coloring proper */
            iter->color = exact_fit->color;

            if (tree->max == chunk_words(exact_fit))
                tree->max = chunk_words(parent);
        }
        dirty_list_unlink(exact_fit);
    }

    tree->root = local_root.link[1];

    if (tree->root) {
        tree->root->color = BLACK;
        tree->root->parent = 0;
    }

    return exact_fit;
}
/*********************************************************************
 * Valgrind Support
 *********************************************************************/
/* This stuff is a bit complicated, and VERY fragile.  Unless you're 
 * running valgrind, and seeing errors from the allocator, this code 
 * IS NOT A PROBLEM.  Do not mess with it.
 */


/* One of the only sources of pre-processor goo in all the code.
 * If we have valgrind, we provide some hook functions, if not,
 * we compile those functions out.
 */

#ifdef HAVE_VALGRIND

/**
 * @brief tell valgrind about an allocation.
 * @param ptr the address given to client code.
 * @param size the size of the allocation.
 */
#define valgrind_malloc(ptr, size)\
    if (RUNNING_ON_VALGRIND && ptr)\
        VALGRIND_MALLOCLIKE_BLOCK(ptr, size, 0, 0)
/**
 * @brief tell valgrind an allocation has been freed.
 * @param ptr the address freed
 */
#define valgrind_free(ptr)\
    if (RUNNING_ON_VALGRIND && ptr)\
        VALGRIND_FREELIKE_BLOCK(ptr, 0)


/**
 * @brief make an internal structure defined.
 * @param block the chunk to make define.
 *
 * Given a chunk, make the head field defined, and read it.
 * Based on the bits/size of the chunk, we define a slab, binchunk,
 * or treechunk as needed.
 *
 * Note that only free chunks are processed.
 *
 * There is some rather interesting pointer math going on here,
 * as the field we care about (the real start of every chunk)is
 * 1 word into the structure.
 */
static inline void
valgrind_make_chunk_defined(chunk_t * block)
{
    if (!RUNNING_ON_VALGRIND || !block)return;


    VALGRIND_MAKE_MEM_DEFINED(((uintptr_t)block) + offsetof(chunk_t, head),
                              sizeof (uint32_t)); 
    if (!chunk_in_use(block)) {
        if (chunk_words(block) < bytes_to_words(SMALLBLOCK_MAX)) {
            VALGRIND_MAKE_MEM_DEFINED(((uintptr_t)block) + 
                                      offsetof(binchunk_t, head), 
                                      sizeof (binchunk_t)- sizeof (uint32_t));

        }else if (chunk_is_slab(block)) {
            slabpool_t * pool;
            VALGRIND_MAKE_MEM_DEFINED(((uintptr_t)block) + 
                                      offsetof(slab_t, head),
                                      sizeof (slab_t)- 
                                      offsetof(slab_t, head));
            pool = slab_cast(block)->pool;
            VALGRIND_MAKE_MEM_DEFINED(pool, sizeof (slabpool_t));
            VALGRIND_MAKE_MEM_DEFINED(((uintptr_t)block) + 
                                      offsetof(slab_t, head),
                                      sizeof (slab_t) + 
                                      words_to_bytes(pool->n_bitmap_words)-
                                      offsetof(slab_t, head));


        } else {
            VALGRIND_MAKE_MEM_DEFINED(((uintptr_t)block) + 
                                      offsetof(treechunk_t, head), 
                                      sizeof (treechunk_t)- sizeof (uint32_t));

        }
        VALGRIND_MAKE_MEM_DEFINED(next_chunk(block), sizeof (uint32_t));
    }

    
}
/**
 * @brief mark a chunk no access.
 * @param block the block to protect.
 * Here, we undo what valgrind_make_chunk_defined does.
 * 
 * No access means that valgrind will throw errors if 
 * such an address is read or written.
 *
 */
static inline void
valgrind_make_chunk_noaccess(chunk_t * block)
{
    if (!RUNNING_ON_VALGRIND || !block) {
        return;
    }
    if (!chunk_in_use(block)) {
        VALGRIND_MAKE_MEM_NOACCESS(next_chunk(block), sizeof (uint32_t));
        if (chunk_words(block) < bytes_to_words(SMALLBLOCK_MAX)) {
            VALGRIND_MAKE_MEM_NOACCESS(((uintptr_t)block) + 
                                       offsetof(binchunk_t, head), 
                                       sizeof (binchunk_t)- 
                                       offsetof(binchunk_t, head));

        } else if (chunk_is_slab(block)) {

            slabpool_t * pool;
            pool = slab_cast(block)->pool;
            VALGRIND_MAKE_MEM_DEFINED(pool, sizeof (slabpool_t));
            VALGRIND_MAKE_MEM_NOACCESS(((uintptr_t)block) + 
                                       offsetof(slab_t, head), 
                                       sizeof (slab_t) + 
                                      words_to_bytes(pool->n_bitmap_words)-
                                      offsetof(slab_t, head));
            VALGRIND_MAKE_MEM_NOACCESS(pool, sizeof (slabpool_t));



        } else {
            VALGRIND_MAKE_MEM_NOACCESS(((uintptr_t)block) + 
                                       offsetof(treechunk_t, head), 
                                       sizeof (treechunk_t)- 
                                       offsetof(treechunk_t, head));

        }


    }
    VALGRIND_MAKE_MEM_NOACCESS(((uintptr_t)block) + offsetof(chunk_t, head),
                               sizeof (uint32_t)); 
}

/**
 * @brief make all chunks and internal data defined.
 * @param self the malloc state to make defined.
 * for each mapping, iterate over every chunk.
 *    for each chunk, make it defined.
 *    define mapping bitmaps.
 */
static void
valgrind_make_internals_defined(malloc_state_t * self)
{
    mapping_t * mapping;

    if (!RUNNING_ON_VALGRIND)return;

    CALLGRIND_TOGGLE_COLLECT;
    LOCK(&self->heap_lock);
    if (RARELY(self->def_count != 0)) {
        self->def_count++;
        UNLOCK(&self->heap_lock);
        return;
    }
    self->def_count++;

    for (mapping = self->mappings; mapping != NULL; mapping = mapping->next) {
        chunk_t * block = mapping->fence;

        VALGRIND_MAKE_MEM_DEFINED(mapping->dirtymap, words_to_bytes(mapping->mapwords));
        VALGRIND_MAKE_MEM_DEFINED(mapping->slabmap, words_to_bytes(mapping->mapwords));
        valgrind_make_chunk_defined(block);

        while (chunk_words(block)!=0) {
            block = next_chunk(block);
            valgrind_make_chunk_defined(block);
        }

    }

    UNLOCK(&self->heap_lock);

    CALLGRIND_TOGGLE_COLLECT;
    return;

}
/**
 * @brief mark internals noaccess.
 * @param self the malloc state to make noaccess.
 * undoes what valgring_make_internals_defined()does.
 */
 
static void
valgrind_make_internals_noaccess(malloc_state_t *self)
{
    mapping_t * mapping;

    if (!RUNNING_ON_VALGRIND) {
        return;
    }
    CALLGRIND_TOGGLE_COLLECT;
    LOCK(&self->heap_lock);
    self->def_count--;

    if (RARELY (self->def_count != 0)) {
        UNLOCK(&self->heap_lock);
        return;
    }

    for (mapping = self->mappings; mapping != NULL; mapping = mapping->next) {
        chunk_t * block = mapping->fence;

        while (chunk_words (block)!=0) {
            chunk_t * next = next_chunk (block);
            valgrind_make_chunk_noaccess (block);
            block = next;
        }
        VALGRIND_MAKE_MEM_NOACCESS (mapping->dirtymap, words_to_bytes (mapping->mapwords));
        VALGRIND_MAKE_MEM_NOACCESS (mapping->slabmap, words_to_bytes (mapping->mapwords));
    }

    UNLOCK(&self->heap_lock);
    CALLGRIND_TOGGLE_COLLECT;
    return;

}
/**
 * @brief get the validity bits (vbits)for an allocation.
 * @param ptr the allocation to fetch vbits for.
 *
 * This is a helper for realloc.  We allocate some bits out of band, so that
 * we can set them correctly in the realloc'd pointer.
 *
 * @return a bitmap with the vbits.
 */

static inline bitmap_t *
valgrind_get_vbits(void * ptr)
{
    bitmap_t * bitmap;
    uint32_t oldsize;

    if (!RUNNING_ON_VALGRIND)return NULL;
    
    valgrind_make_chunk_defined(mem_to_chunk(ptr));
    oldsize = chunk_bytes(mem_to_chunk(ptr));

    bitmap = mmap(0, oldsize + sizeof (uint32_t), PROT_READ|PROT_WRITE,
                  MAP_PRIVATE|MAP_ANONYMOUS, -1, 0);

    *bitmap = oldsize + sizeof (uint32_t);

    bitmap++;

    VALGRIND_GET_VBITS(ptr, bitmap, oldsize);

    return bitmap;

}
/**
 * @brief free vbits.
 * @param vbits the validity bits to free.
 * just munmap the data.
 */
static inline void
valgrind_destroy_vbits(bitmap_t * vbits)
{
    if (!RUNNING_ON_VALGRIND)return;
    vbits--;
    munmap(vbits, *vbits);
}
/**
 * @brief mark a chunk as free.
 * @param block the block to mark free.
 * This really works in concert with free and make chunk defined.
 * Since we need a chunk to be available to have it be properly defined under valgrind,
 * but we don't necissarily want to free the chunk...
 */
#define valgrind_make_chunk_available(block)\
    block->head &= ~IN_USE_BIT

#define valgrind_printf VALGRIND_PRINTF
#define printf (...)malloc_printf (fileno(stdout), __VA_ARGS__)
#else

#define valgrind_make_chunk_available(block)
#define valgrind_make_chunk_defined(block)
#define valgrind_make_chunk_noaccess(block)
#define valgrind_make_internals_defined(self)
#define valgrind_make_internals_noaccess(self)
#define VALGRIND_MAKE_MEM_NOACCESS(ptr, size)
#define VALGRIND_MAKE_MEM_DEFINED(ptr, size)
#define valgrind_malloc(ptr, size)
#define valgrind_free(ptr)
#define valgrind_destroy_vbits(vbits)
#define VALGRIND_DO_LEAK_CHECK
#define VALGRIND_SET_VBITS(ptr, bitmap, bytes)
#define RUNNING_ON_VALGRIND 0
#define valgrind_printf(...)malloc_printf (fileno (stdout), __VA_ARGS__)
#define printf(...)malloc_printf (fileno (stdout), __VA_ARGS__)
#define valgrind_prepare_slab(slab)

#endif

/********************************************************************
 * Debug Assistance
 *******************************************************************/

static int internal_verify(malloc_state_t * self);
/**
 * default message printing.
 * @param p1 part 1 of the message.
 * @param p2 part 2 of the message.
 * @param p3 part 3 of the message.
 * @param p4 part 4 of the message.
 *
 * For development purposes, a printf like function was needed that didn't
 * allocate any memory, so that errors could be reported.
 */
static inline void
write_message(int fd, const char *p1, const char *p2, const char * p3, const char *p4)
{
    int ret = 0;
    if (p1)
        ret += write(fd, p1, strlen(p1));
    if (p2)
        ret += write(fd, p2, strlen(p2));
    if (p3)
        ret += write(fd, p3, strlen(p3));
    if (p4)
        ret += write(fd, p4, strlen(p4));

}

void (*_malloc_message)(int fd, const char *, const char *, 
                        const char *, const char *) = write_message;

/**
 * printf function that shouldn't allocate any memory.
 */
int
malloc_printf (int fd, const char * format, ...)
{
    char buf[1024];
    va_list ap;

    va_start(ap, format);
    vsnprintf (buf, sizeof (buf), format, ap);
    va_end(ap);

    if (RARELY(RUNNING_ON_VALGRIND)) {
        valgrind_printf ("%s", buf);
    } else {
        _malloc_message(fd, buf, 0,0,0);
    }
    return 0;
}

/*!
 * Add a history event to the history ring buffer.
 */
static inline void add_allocation_event(malloc_state_t * self, void * ptr, size_t size,
                                     void * func)ALWAYS_INLINE;

#define UNUSED(var)do{ if (0)((void)(var)); }while (0)
static inline void 
add_allocation_event(malloc_state_t * self, void * ptr, size_t size, void * func)
{
    self->history_tail->next = self->history_head;
    self->history_head = self->history_head->next;
    self->history_tail = self->history_tail->next;

    self->history_tail->return_function = func;
    self->history_tail->address = ptr;
    self->history_tail->count = self->malloccount;
    self->history_tail->size = size;
}

static inline void add_free_event(malloc_state_t * self, void * ptr, size_t size,
                                  void * func)ALWAYS_INLINE;
static inline void 
add_free_event(malloc_state_t * self, void * ptr, size_t size, void * func)
{
    self->history_tail->next = self->history_head;
    self->history_head = self->history_head->next;
    self->history_tail = self->history_tail->next;

    self->history_tail->return_function = func;
    self->history_tail->address = ptr;
    self->history_tail->count = self->freecount;
    self->history_tail->size = size;
}
void
breakonmalloc()
{
    printf ("%s\n", __func__);
}

/*!
 * Fill a chunk with trash.
 */
static inline void write_trash(malloc_state_t * self, void * ptr)ALWAYS_INLINE;

static inline void
write_trash(malloc_state_t * self, void * ptr)
{
    word_t * alias = (word_t *)ptr;
    int i;
    int words;

    if (RARELY (NULL == find_mapping (self, ptr))) {
        return;
    }

    if (alloc_is_slice (self, ptr)) {
        /* slabs have no overhead, unless we're checking bounds or doing a memverify */
        words = ptr_to_slab(ptr)->pool->n_slice_words;
        if (RARELY(check_bounds(self) || verify(self)))
            words -= WOVERHEAD;
    } else {
        words = chunk_words(mem_to_chunk(ptr))- bytes_to_words(self->overhead);
    }


    for (i = 0; i< words; i++) {
#if defined(DOUBLEALIGNED)
        alias[i] = 0xdeadbeefdeadbeefULL;
#else
        alias[i] = 0xdeadbeef; 
#endif
    }


}

/********************************************************************
 * Page Tracking
 *******************************************************************/

#define chunk_overhead(block)\
    (chunk_words(block) > bytes_to_words(SMALLBLOCK_MAX) ?\
      sizeof (treechunk_t):sizeof (binchunk_t))

#define chunk_to_unused(block)\
    (((uintptr_t)block) + chunk_overhead(block))

#define first_contained_page(block)\
    ((((uintptr_t)block) + PAGE_MASK) & -PAGESIZE_BYTES)

static inline unsigned int contained_pages(chunk_t * block)PURE_INLINE;
static inline unsigned int
contained_pages(chunk_t * block)
{
    uintptr_t unused = chunk_to_unused(block);
    uintptr_t end = ((uintptr_t)next_chunk(block));

    unused = first_contained_page(unused);
    end = (uintptr_t)ptr_to_page(end);
    
    if (end > unused && end - unused >= PAGESIZE_BYTES) {
        return (end - unused) / PAGESIZE_BYTES;
    }

    return 0;
}
static inline void unhook_pages(malloc_state_t * self, 
                                chunk_t * block)ALWAYS_INLINE;
static inline void
unhook_pages(malloc_state_t * self, chunk_t * block)
{
    self->available_pages -= contained_pages(block); 
}

static inline void hook_pages(malloc_state_t * self, 
                              chunk_t * block)ALWAYS_INLINE;
static inline void
hook_pages(malloc_state_t * self, chunk_t * block)
{
    self->available_pages+= contained_pages(block);
}
static inline void
free_pages(malloc_state_t * self,
           chunk_t * block)
{
    self->available_pages+= contained_pages(block);
    self->reclaimable_pages+= contained_pages(block);
}

static inline unsigned int
count_contained_dirty_pages(malloc_state_t * self, chunk_t * block)ALWAYS_INLINE;
static inline unsigned int
count_contained_dirty_pages(malloc_state_t * self, chunk_t * block)
{
    mapping_t * mapping; 
    bitmap_t * dirtymap;
    uintptr_t start;
    uintptr_t end;
    unsigned int i;
    uint32_t dirty_pages = 0;
    mapping = find_mapping(self,block);
    dirtymap = mapping->dirtymap;
    
    start = first_contained_page(chunk_to_unused(block));
    end = (uintptr_t)ptr_to_page(next_chunk(block));

    if (RARELY(end < start)) {
        return 0;
    }

    start -= (uintptr_t)mapping->top;
    end -= (uintptr_t)mapping->top;

    start/= PAGESIZE_BYTES;
    end/= PAGESIZE_BYTES;

    if (index2bit(start)) {
        dirty_pages = population_count (
                              mask_right (dirtymap[index2word (start)],index2bit (start)));
    }

    start = (start + BITS_PER_WORD -1) & -BITS_PER_WORD;

    for (i = start; i < end; i+= BITS_PER_WORD) {
        dirty_pages += population_count(dirtymap[ index2word(i)]);
    }

    if (index2bit(end)) {
        dirty_pages -= population_count (mask_right (dirtymap[index2word (end)], index2bit(end)));
    }


    return dirty_pages;
}
static inline unsigned int
mark_pages_dirty(malloc_state_t * self, chunk_t *block)
{
    unsigned int i;
    mapping_t * mapping; 
    bitmap_t * dirtymap;
    bitmap_t map;
    uintptr_t start;
    uintptr_t end = (uintptr_t)next_chunk(block) + 1;
    uint32_t dirty_pages = 0;
    bitmap_t val = 0xFFFFFFFF;

    start = first_contained_page(chunk_to_unused(block));
    end = (uintptr_t)ptr_to_page(end);

    if (!(start < end && end - start >= PAGESIZE_BYTES))
        return 0;
    
    mapping = find_mapping(self,block);
    dirtymap = mapping->dirtymap;

    start -= (uintptr_t)mapping->top;
    end -= (uintptr_t)mapping->top;
    end /= PAGESIZE_BYTES;
    start/= PAGESIZE_BYTES;

    /* we grab the end, because we may over-write it while toggeling bits.. */
    map = dirtymap[index2word(end)];
    /* count already dirty pages in the map */
    /* note that if start & end index into the same page, we end up flipping
     * way too many bits.  we turn those off later.
     */
    if (index2bit(start)) {
        bitmap_t mask = mask_right (val, index2bit(start));
        dirty_pages = population_count(
                                mask_right(dirtymap[ index2word(start)], 
                                                  index2bit(start)));
        dirtymap[index2word(start)] |= mask;
    }

    start = (start + BITS_PER_WORD -1) & -BITS_PER_WORD;

    for (i = start; i < end; i+= BITS_PER_WORD) {
        dirty_pages += population_count (dirtymap[ index2word(i)]);
        dirtymap[ index2word(i)] = val;
    }

    if (index2bit(end)) {
        dirty_pages -= population_count (mask_right (map, index2bit (end)));
        dirtymap[ index2word (end)] &= ~mask_right (val, index2bit (end));
        dirtymap[ index2word (end)] |= map;
    }

    return dirty_pages;
}
static inline unsigned int
mark_pages_clean(malloc_state_t * self, chunk_t * block)
{

    unsigned int i;
    mapping_t * mapping; 
    bitmap_t * dirtymap;
    bitmap_t map;
    uintptr_t start;
    uintptr_t end = (uintptr_t)next_chunk(block) + 1;
    uint32_t dirty_pages = 0;
    bitmap_t val = 0;
    mapping = find_mapping(self,block);
    dirtymap = mapping->dirtymap;

    start = first_contained_page(chunk_to_unused(block));
    start -= (uintptr_t)mapping->top;
    end -= (uintptr_t)mapping->top;
    start/= PAGESIZE_BYTES;
    end = (uintptr_t)ptr_to_page(end);
    end /= PAGESIZE_BYTES;

    /* we grab the end, because we may over-write it while toggeling bits.. */
    map = dirtymap[index2word(end)];

    if (index2bit(start)) {
        bitmap_t mask = mask_right(0xffffffff, index2bit(start));
        dirty_pages = population_count(
                                mask_right(dirtymap[ index2word(start)], 
                                                  index2bit(start)));
        dirtymap[index2word(start)] &= ~mask;
    }

    start = (start + BITS_PER_WORD -1) & -BITS_PER_WORD;
    for (i = start; i < end; i+= BITS_PER_WORD) {
        dirty_pages += population_count (dirtymap[ index2word (i)]);
        dirtymap[ index2word (i)] = val;
    }

    if (index2bit(end)) {
        dirty_pages -= population_count (mask_right (map, index2bit (end)));
        dirtymap[index2word(end)] |=  (map & mask_right (0xffffffff, index2bit(end)));
    }

    return dirty_pages;
}

static inline unsigned int
check_process_dirty_pages(mapping_t * mapping)
{
    int fd;

    char buf[1024];
    int pages;
    int mode = 0;
    void * start;
    int bytes;

    snprintf (buf, sizeof (buf), "/proc/%d/smaps", getpid ());
    fd = open (buf, O_RDONLY); 

    if (fd < 0)return 0;

    while ((bytes = read (fd, buf, sizeof (buf)- 1)) > 0) {
        char * s = buf;
        buf[bytes] = '\0';
        while (s && s < buf + strlen(buf)) {
            if (mode == 0) {
                if (sscanf (s, "%p-%*p", &start)) {
                    if (start == mapping->top) {
                        mode = 1;
                    }
                }
            } else {
                if (sscanf (s, "Rss: %d kB", &pages)) {
                    pages = pages * 1024 / PAGESIZE_BYTES;
                    close (fd);
                    return pages;
                }
            } 
            s = strchr (s, '\n');
            if (s) {
                s++;
            }
        }
    }
    close (fd);

    return 0;

}


static inline void
clean_pages(malloc_state_t * self, uint32_t needed)
{
    uint32_t returned_pages = 0;
    treechunk_t * dirty = self->dirty.dirty.prev;
    treechunk_t * next;

    while (returned_pages < needed && dirty != &self->dirty) {
        uint32_t pages;
        uintptr_t unused = first_contained_page(chunk_to_unused(dirty));
        pages = contained_pages(chunk_cast(dirty)); 
        if (pages) {
            uint32_t dirty_pages = 0;
            if (RARELY(0 != madvise ((void *)unused, pages * PAGESIZE_BYTES , MADV_DONTNEED)))
                crash ("Attempt to Return Pages to the system failed!"); 
            dirty_pages = mark_pages_clean(self, chunk_cast(dirty)); 
            returned_pages += dirty_pages;
            self->reclaimable_pages -= dirty_pages;
        }
        next = dirty->dirty.prev;
        dirty_list_unlink(dirty);
        dirty = next;
    }

    debug_assert(needed <= returned_pages);
}
static inline void
allocate_pages(malloc_state_t * self, chunk_t * block)
{
    uint32_t pages;
    uint32_t dirty_pages = 0;
    pages = contained_pages(block);

    if (!pages)
        return;

    dirty_pages = mark_pages_dirty(self, block);
    self->reclaimable_pages -= dirty_pages;
    self->available_pages -= dirty_pages;

    if (RARELY(((int)(self->available_pages - (pages - dirty_pages))) < self->reclaimable_pages))
        clean_pages(self, self->reclaimable_pages);
    
    if (dirty_pages < pages) {
        if (pages - dirty_pages > (uint32_t)self->available_pages)
           clean_pages(self, pages - dirty_pages - self->available_pages); 
        self->available_pages -= (pages - dirty_pages);
    }
}

static inline unsigned int crossed_pages(chunk_t * block)ALWAYS_INLINE;
static inline unsigned int
crossed_pages(chunk_t * block)
{
    uint32_t pages = 0;
    uintptr_t start = (uintptr_t) &block->head;
    uintptr_t end = (uintptr_t) &next_chunk(block)->head;
    start = first_contained_page(start);
    end = first_contained_page(end);
    if (end-start >= PAGESIZE_BYTES)
        pages = (end - start) / PAGESIZE_BYTES;

    return pages;
}

static inline bool
new_dirty_pages(malloc_state_t * self,
                chunk_t * block,
                uint32_t words)
{
    mapping_t * mapping; 
    uintptr_t fs;
    uintptr_t fe;
    uintptr_t ss;
    uintptr_t se;
    uintptr_t top;
    uintptr_t offset;
    chunk_t * next = next_chunk(block);
    uintptr_t next_start = ptr_to_page(next);
    
    fs = ptr_to_page(block);
    fe = ptr_to_page(chunk_to_unused(block)-1);
    ss = ((uintptr_t)block) + words_to_bytes(words);
    se = ss + new_chunk_overhead(words_to_bytes(chunk_words(block)- words))-1;
    ss = ptr_to_page(ss);
    se = ptr_to_page(se);
    if (se > fs && next_start > ss 
        && se - fs >= PAGESIZE_BYTES 
        && next_start - ss >= PAGESIZE_BYTES) {
        mapping = find_mapping(self, block);
        top = (uintptr_t)mapping->top;

        if (ss > fe && ss - fe >= PAGESIZE_BYTES) {
            offset = (ss - top) /PAGESIZE_BYTES;

            if (!(mapping->dirtymap[index2word(offset)] & 1 << index2bit(offset))) {
                return true;
            }

        }

        if (RARELY(fe < se && ss < se 
                    && se - ss >= PAGESIZE_BYTES 
                    && se - fe >= PAGESIZE_BYTES)) {
            if (next_start > se && next_start - se >= PAGESIZE_BYTES) {
                offset = (se - top) /PAGESIZE_BYTES;
                if (!(mapping->dirtymap[index2word(offset)] & 1<< index2bit(offset))) {
                    return true;
                }
            }

        }
    }

    return false;
}
/*******************************************************************
 * Chunk Management 
 *******************************************************************/

static inline void
initialize_chunk(malloc_state_t * self,  chunk_t * old, chunk_t *new, uint32_t new_words)
{

    mapping_t * mapping; 
    /* figure out if newblock header data is on a clean or dirty page */
    /* also, check if newblock crosses onto a new page*/
    uintptr_t old_start;
    uintptr_t old_end;
    uintptr_t new_start;
    uintptr_t new_end;
    uintptr_t offset;
    uintptr_t top;
    chunk_t * next = next_chunk(old);
    uintptr_t next_start = (uintptr_t)ptr_to_page(next);

    old_start = (uintptr_t)ptr_to_page(old);
    old_end = (uintptr_t)ptr_to_page(((uintptr_t)old) + chunk_overhead(old)-1);

    new_start = (uintptr_t)ptr_to_page(new);
    new_end = (uintptr_t)ptr_to_page(((uintptr_t)new) + new_chunk_overhead(words_to_bytes(new_words))-1);

    /*
    printf ("old: %p (%p) => %p\n", old_start, old_end, next_start);
    printf ("new: %p (%p)\n", new_start, new_end);
    */
    if (new_end > old_start 
        && next_start > new_start 
        && new_end - old_start >= PAGESIZE_BYTES 
        && next_start - new_start >= PAGESIZE_BYTES) {
        mapping = find_mapping(self, new);
        top = (uintptr_t)mapping->top;

        if (new_start > old_end && new_start - old_end >= PAGESIZE_BYTES) {
            offset = (new_start - top) /PAGESIZE_BYTES;

            if (mapping->dirtymap[index2word(offset)] & 1 << index2bit(offset)) {
                /*
                printf ("rp-- #1 -- page %d in use\n", offset);
                */
                self->reclaimable_pages--;
            }else
                mapping->dirtymap[index2word(offset)] |= 1 << index2bit(offset);
        }

        if (RARELY(new_end> old_end 
                    && new_start < new_end 
                    && new_end - new_start >= PAGESIZE_BYTES 
                    && old_end - new_end >= PAGESIZE_BYTES)) {
            if (next_start > new_end && next_start - new_end >= PAGESIZE_BYTES) {
                offset = (new_end - top) /PAGESIZE_BYTES;
                if (mapping->dirtymap[index2word(offset)] & 1 << index2bit(offset)) {
                    /*
                    printf ("rp-- #2 -- page %d in use\n", offset);
                    */
                    self->reclaimable_pages--;
                }else
                    mapping->dirtymap[index2word(offset)] |= 1 << index2bit(offset);
            }
        } 

    }

}
static inline void 
deinit_chunk(malloc_state_t * self, chunk_t * first, chunk_t * second)
{
    mapping_t * mapping; 
    uintptr_t fs;
    uintptr_t fe;
    uintptr_t ss;
    uintptr_t se;
    uintptr_t top;
    uintptr_t offset;
    chunk_t * next = next_chunk(second);
    uintptr_t next_start = (uintptr_t)ptr_to_page(next);
    
    fs = ptr_to_page(first);
    fe = ptr_to_page(((uintptr_t)first) + new_chunk_overhead(chunk_bytes(first) + 
                                                             chunk_bytes(second))-1);
    ss = ptr_to_page(second);
    se = ptr_to_page(((uintptr_t)second) + chunk_overhead(second)-1);
/*
    printf ("combined: %p (%p) => %p\n", fs, fe, next_start);
    printf ("internal: %p (%p)\n", ss, se);
*/
    if (se > fs && next_start > ss 
        && se -fs >= PAGESIZE_BYTES 
        && next_start - ss >= PAGESIZE_BYTES) {
        mapping = find_mapping(self, first);
        top = (uintptr_t)mapping->top;

        if (ss > fe && ss - fe >= PAGESIZE_BYTES) {
            offset = (ss - top) /PAGESIZE_BYTES;

            if (mapping->dirtymap[index2word(offset)] & 1 << index2bit(offset)) {
                /*
                printf ("rp++ #1 -- page %d reclaimable\n", offset);
                */
                self->reclaimable_pages++;
            }

        }

        if (RARELY(fe < se 
                    && ss < se 
                    && se - ss >= PAGESIZE_BYTES 
                    && se - fe >= PAGESIZE_BYTES)) {

            if (next_start > se && next_start - se >= PAGESIZE_BYTES) {
                offset = (se - top) /PAGESIZE_BYTES;
                if (mapping->dirtymap[index2word(offset)] & 1<< index2bit(offset)) {
                    /*
                    printf ("rp++ #2 -- page %d reclaimable\n", offset);
                    */
                    self->reclaimable_pages++;
                }
            }

        }
    }
}

/**
 * join 2 chunks together.
 * @param first the chunk lower in the address space.
 * @param second the chunk higher in the address space.
 * @precondition first < second.
 * @return the combined chunk.
 */
static inline chunk_t *
join_chunk(malloc_state_t * self, chunk_t * first, chunk_t * second)
{
    uint32_t combined_size; 
    chunk_t * end;     


    deinit_chunk(self, first, second);
    combined_size = chunk_words(first) + chunk_words(second);
    end = next_chunk(second);
    end->prev_size = combined_size;
    first->head = (first->head & INTERNAL_BITS) | combined_size;

    valgrind_make_chunk_defined(first); /* first just grew */
    VALGRIND_MAKE_MEM_NOACCESS(second , sizeof (chunk_t));

    return first;
}

/**
 * split the top of a chunk off.
 * @param block the chunk to split.
 * @param words the size of the new chunk.
 * @return the new chunk
 */

static inline chunk_t *
split_chunk_top(malloc_state_t * self, chunk_t * block, size_t words)
{
    chunk_t * newblock;
    chunk_t * next;
   
    unhook_pages(self, block);
    next = next_chunk(block);
    next->prev_size = chunk_words(block)- words;

    newblock = new_block_from_offset(block, words_to_bytes(words));

    initialize_chunk(self, block, newblock, chunk_words(block)- words);

    VALGRIND_MAKE_MEM_DEFINED(newblock, sizeof (chunk_t));

    newblock->head = chunk_words(block)- words;
    newblock->prev_size = words;
    /* Have to adjust the block we're splitting -- may have shrunk */
    valgrind_make_chunk_noaccess(block);
    VALGRIND_MAKE_MEM_DEFINED(((uintptr_t)block) + offsetof(chunk_t, head), 
                               sizeof (uint32_t));

    block->head = (block->head & INTERNAL_BITS) | words;
    
    valgrind_make_chunk_defined(newblock);
    valgrind_make_chunk_defined(block);

    hook_pages(self, block);
    hook_pages(self, newblock);

    return newblock;
} 
/**
 * split the bottom of a chunk off.
 * @param block the chunk to split.
 * @param words the size of the new chunk.
 * @return the new chunk.
 */
#if 0
currently, this is unused.  need to see if there could be a use, or if I can
nuke it.
static inline chunk_t * 
split_chunk_bottom(chunk_t * block, size_t words)
{
    chunk_t * newblock;
    chunk_t * next;
   
   
    next = next_chunk(block);
    
    newblock = new_block_from_offset(block, 
                                     words_to_bytes(chunk_words(block)- words));
    newblock->head = chunk_words(block)- words;


    next->prev_size -= words;
    block->head = (block->head & INTERNAL_BITS) | words;
    newblock->prev_size = words;

    return newblock;
}
#endif
/**
 * copy data from one place to another.  realloc helper.
 * @param dest the destination.
 * @param source the source.
 * @param total_words the amount of data to copy.
 *
 * This should prolly be asm code.
 */
static inline void
copy_chunk(word_t * dest_word, word_t * source_word, unsigned int total_words)
{
    uint32_t word =0;

    VALGRIND_MAKE_MEM_DEFINED(source_word, words_to_bytes(total_words));
    VALGRIND_MAKE_MEM_DEFINED(dest_word, words_to_bytes(total_words));

    for (; word < total_words; word++) {
        dest_word[word] = source_word[word];
    }
}

/*****************************************************************************
 *  Free list management 
 ****************************************************************************/
/*----------------- Fun with segrigated free lists -------------------*/

/*
 * Instruction counts:
 *
 * index2word: 1
 * index_mask: 1
 * index2bit: 3
 * mark_smallmap: 4
 * clear_smallmap: 5 (4 of we have and not)
 * smallbin_populated: 4
 *
 */
/* small maps and bins */
#define mark_smallmap(mstate, index)\
    ((mstate)->smallmap[ index2word(index)] |= (1U<< index2bit(index)))
#define clear_smallmap(mstate, index)\
    ((mstate)->smallmap[ index2word(index)] &=~ (1U<< index2bit(index)))
#define smallbin_populated(mstate, index)\
    ((mstate)->smallmap[ index2word(index)] & (1U<<index2bit(index)))
#define smallbin_at(mstate, index)\
    ((binchunk_t *)((uintptr_t) &(mstate)->smallbins[index]))
#if MALLOC_DEBUG > 0
#define clear_smallbin(mstate, index)\
    (mstate->smallbins[index].link.prev = mstate->smallbins[index].link.next = &mstate->smallbins[index])
#else
#define clear_smallbin(mstate, index)
#endif


#define SMALL_INDEX(words)((words)- bytes_to_words(MIN_BLOCK_SIZE))

/* large maps and bins */

#define mark_largemap(mstate, index)\
    ((mstate)->largemap[ index2word(index)] |= (1U << (index2bit(index))))
#define clear_largemap(mstate, index)\
    ((mstate)->largemap[ index2word(index)] &=~(1U<< (index2bit(index))))
#define largebin_populated(mstate, index)\
    ((mstate)->largemap[ index2word(index)] & 1U << (index2bit(index)))
#define largebin_at(mstate, index)\
    (&(mstate)->largebins[index])

/**
 * @param self the malloc state
 * @param block the chunk to remove from its list.
 */

static inline void unlink_chunk (malloc_state_t * self, chunk_t * block)ALWAYS_INLINE;
static inline void
unlink_chunk (malloc_state_t * self, chunk_t * block)
{
    chunk_t * next; 
    chunk_t * prev; 
    unsigned int index; 
    unsigned int blockwords; 
   
    blockwords = chunk_words(block);

    if (chunk_is_slab(block) && !chunk_in_use(block)) {
        blockwords = ((slab_t *)block)->pool->n_slice_words+WOVERHEAD;
    }

    if (blockwords < bytes_to_words(SMALLBLOCK_MAX)) {

        index = SMALL_INDEX(blockwords);

        debug_assert(index < SMALLBINS);
        next = chunk_cast(bin_cast(block)->link.next);
        prev = chunk_cast(bin_cast(block)->link.prev);

        if (next == prev) {
            clear_smallmap(self, index);
            clear_smallbin(self, index);
        } else if ((bin_cast(next) == smallbin_at(self, index)
                    || address_ok(self, next))
                   && (bin_cast(prev) == smallbin_at(self, index)
                       || address_ok(self, prev))) { 
            debug_assert(smallbin_populated(self, index));
            bin_cast(next)->link.prev = bin_cast(prev);
            bin_cast(prev)->link.next = bin_cast(next);
        } else {
            /* corrupt heap */
            crash("corrupt heap");
        }
    } else {
        treechunk_t * rem = (treechunk_t *)block;
        index = large_index(blockwords);

        debug_assert(largebin_populated(self, index));

        if (rem->list.prev != rem) {  /* there is a list of nodes of this size.  no tree rebalance ops needed */
            if (rem == largebin_at(self, index)->root)
                largebin_at(self, index)->root = rem->list.prev;

            dirty_list_unlink(rem);
            unlink_treechunk(rem);
        } else {  /* we're the only node of this size in the tree, so we have
                     to do hard removal */
            treechunk_t * check;
            debug_assert(rem->color!= NON_TREE_NODE);
            check = rb_remove_size(largebin_at(self, index), blockwords);

            debug_assert(check == rem);

            dirty_list_unlink(rem);
            if (largebin_at(self, index)-> root == NULL)
                clear_largemap(self, index);
        }

    }
}
/**
 * @param self the malloc state
 * @param bin the bin containing the block.
 * @param block - the block to unlink.
 * @param index  the index of the bin.
 */
static inline void
unlink_first_freeblock(malloc_state_t * self, 
                        binchunk_t * bin, 
                        binchunk_t * block,
                        unsigned int index)
{
    binchunk_t * next; 
    
    next = block->link.next;

    if (bin == next) { 
        debug_assert(index < SMALLBINS);
        clear_smallmap(self, index);
        clear_smallbin(self, index);
    } else if (address_ok(self, next)) {
        bin->link.next = next;
        next->link.prev = bin;
    } else {
        crash("corrupt heap");
    }
}
/**
 * Insert a chunk at the tail of the list.
 * @param self the malloc state
 * @param bin the bin to insert into
 * @param block the block to insert
 * @param index the index of the bin
 */

static inline void insert_last_binchunk(malloc_state_t *, binchunk_t *, 
                                        binchunk_t *, unsigned int)
    ALWAYS_INLINE;
static inline void
insert_last_binchunk(malloc_state_t * self,
                      binchunk_t * bin,
                      binchunk_t * block,
                      unsigned int index)
{
    
    binchunk_t * prev;

    prev = bin;

    if (RARELY(!block))return;

    if (!smallbin_populated(self, index))
        mark_smallmap(self, index);
    else if (address_ok(self, prev->link.prev)) {
        prev = prev_binchunk(bin);
    }else {
        crash("corrupt heap");
    }

    prev->link.next = block;
    bin->link.prev = block;
    block->link.next = bin;
    block->link.prev = prev;
}
/**
 * Insert a chunk at the head of the list.
 * @param self the malloc state
 * @param bin the bin to insert into
 * @param block the block to insert
 * @param index the index of the bin
 */

static inline void insert_first_binchunk(malloc_state_t *, binchunk_t *, 
                                        binchunk_t *, unsigned int)
    ALWAYS_INLINE;
static inline void
insert_first_binchunk(malloc_state_t * self,
                      binchunk_t * bin,
                      binchunk_t * block,
                      unsigned int index)
{
    
    binchunk_t * next;

    next = bin;

    if (RARELY(!block))return;

    if (!smallbin_populated(self, index))
        mark_smallmap(self, index);
    else if (address_ok(self, next->link.prev)) {
        next = next_binchunk(bin);
    }else {
        crash("corrupt heap");
    }

    next->link.prev = block;
    bin->link.next = block;
    block->link.next = next;
    block->link.prev = bin;
}
/**********************************************************************/
/*------------------------- Slabs ------------------------------------*/
/**********************************************************************/

static void slab_init (slab_t * slab, unsigned n_bitmap_words);
static inline void slab_fix_up(malloc_state_t * self, slabpool_t *pool, slab_t * slab);

static inline slab_t *
internal_allocate_slab(malloc_state_t * self,  slabpool_t * pool)
{
    uint32_t bytes = pool->slab_size;
    chunk_t * al_block = NULL;
    mapping_t * mapping;
    uint32_t offset;

    if (RARELY(self->available_pages < 4))return NULL;
    

    al_block = find_treechunk(self, 
                          bytes_to_words(bytes + 
                                         PAGESIZE_BYTES + 
                                         MIN_BLOCK_SIZE + 
                                         self->overhead));


    if (RARELY(!al_block))
        return NULL;

    if ((((uintptr_t)al_block) & ~(PAGE_MASK))!= (uintptr_t)al_block) {
        uintptr_t bottom;
        uintptr_t topsize;
        chunk_t * block = al_block;
        void * temp = (void *)((((uintptr_t)block) + 
                       self->overhead + PAGESIZE_BYTES) & -PAGESIZE_BYTES);

        if (((intptr_t)temp)- ((intptr_t)al_block) > (intptr_t)MIN_BLOCK_SIZE)
            bottom = (uintptr_t)temp; 
        else
            bottom = ((uintptr_t)temp) +PAGESIZE_BYTES;

        topsize = bottom - ((uintptr_t)block);

        al_block = split_chunk_top(self, block , bytes_to_words(topsize));
        valgrind_make_chunk_defined(block);
        set_available(block);

        if (!prev_chunk_in_use(block)) {
            unlink_chunk(self, prev_chunk(block));
            block = join_chunk(self, prev_chunk(block), block);
        }
        insert_chunk(self, block);
    }

    if (chunk_bytes(al_block) >= bytes + MIN_BLOCK_SIZE 
         || (!next_chunk_in_use(al_block)
             && !chunk_is_slab(next_chunk(al_block)))) {
        chunk_t * remainder = al_block;

        if (!next_chunk_in_use(remainder)
            && !chunk_is_slab(next_chunk(remainder))) {
            unlink_chunk(self, next_chunk(remainder));
            remainder = join_chunk(self, remainder, next_chunk(remainder));
        }

        remainder = split_chunk_top(self,al_block, 
                                    bytes_to_words(bytes));

        set_available(remainder);
        set_prev_in_use(remainder);
        insert_chunk(self, remainder);
    }

    debug_assert((((uintptr_t)al_block) & (PAGE_MASK)) == 0); 
    debug_assert(chunk_bytes(al_block) >= bytes);

    valgrind_make_chunk_defined(al_block);
    set_in_use(al_block);
    al_block->head &= ~IN_USE_BIT;

    ((slab_t*)al_block)->pool = pool;
    set_slab(al_block);
    valgrind_make_chunk_defined(al_block);
    slab_init(slab_cast(al_block), pool->n_bitmap_words);
    slab_fix_up(self, pool, slab_cast(al_block));
    ((slab_t*)al_block)->n_free = pool->n_slices_per_slab;
    pool->n_available += pool->n_slices_per_slab;
    mapping = find_mapping(self, al_block);

    offset = (((uintptr_t)al_block)- 
              ((uintptr_t)mapping->top)) /PAGESIZE_BYTES;

    mapping->slabmap[ index2word(offset)] |= 1 << index2bit(offset);
    debug_assert(mapping->dirtymap[index2word(offset)] & 1 << index2bit(offset));

    self->available_words -= bytes_to_words(sizeof (slab_t));
    

    return (slab_t *)al_block;
}

#define SLAB_HEADER_WORDS  NEAREST_N_WORDS(sizeof (slab_t))

/*
   slab_t layout:

    +------------------------------------------------------------------------+
    | header   NEAREST_N_WORDS(sizeof (slab_t))                             |
    +------------------------------------------------------------------------+
    | bitmap   sizeof (bitmap_t) * pool->n_bitmap_words                      |
    +------------------------------------------------------------------------+
    | slices   sizeof (word_t) * pool->n_slices_per_slab * pool->slice_words |
    +------------------------------------------------------------------------+
    | wastage  remainder                                                     |
    +------------------------------------------------------------------------+

   Unfortunately, because both the bitmap and slices fields vary in size,
   we have to do all the offset math by hand.
 */

static inline bitmap_t * slab_get_bitmap (slab_t * slab)ALWAYS_INLINE;
static inline bitmap_t *
slab_get_bitmap (slab_t * slab)
{
    return (bitmap_t *)(((word_t*)slab) + SLAB_HEADER_WORDS);
}

static inline word_t * slab_get_base (slab_t * slab, 
                                      unsigned n_bitmap_words)ALWAYS_INLINE;
static inline word_t *
slab_get_base (slab_t * slab,
                unsigned n_bitmap_words)
{
    return (word_t *)(slab_get_bitmap (slab) + n_bitmap_words);
}

static inline word_t * slab_get_slice (slab_t * slab, unsigned n_bitmap_words, 
                                       unsigned slice_words, int index)ALWAYS_INLINE;
static inline word_t *
slab_get_slice (slab_t * slab,
                 unsigned n_bitmap_words,
                 unsigned slice_words,
                 int index)
{
    return slab_get_base (slab, n_bitmap_words) + index * slice_words;
}

/**
 * Reverse-map an address to a slice index.
 * @param pool   A single chunk.
 * @param mem    The address to map.  Assume that any DebugInfo has already
 *               been backed off.
 * @param bitmap Address at which to store the pointer to the bitmap.
 * @param index  Address at which to store the index.
 * @return true if @p mem is exactly one of the slices; false otherwise.
 *         If false, and the variable at @p bitmap is not NULL, then the
 *         pointer did not point to one of the allocations we gave out,
 *         but at something inside our heap.  In this case, @p index will
 *         point to the slice containing the pointer.
 */

static inline bool look_up_address (slabpool_t * pool, slab_t * chunk, 
                                    void * mem, bitmap_t ** bitmap, 
                                    int * index)ALWAYS_INLINE;
static inline bool
look_up_address (slabpool_t * pool,
                 slab_t * chunk, /* haystack */
                 void * mem,
                 bitmap_t ** bitmap,
                 int * index)
{
    word_t * base = mem;
    int j = 0;
    int i = 0;

    base = slab_get_base (chunk, pool->n_bitmap_words);

    i = (int)(((((word_t *)mem)- base) * pool->magic) >> pool->shift);
        
    j = (int)pool->n_slices_per_slab;

    if (USUALLY((i >= 0) && (i < j))) {
        /* It's in our address range.... it is right on? */
        *bitmap = slab_get_bitmap (chunk);
        *index = i;
        return (mem == (base + i * pool->n_slice_words));
    }

    *bitmap = NULL;
    *index = -1;
    return false;
}


#define is_allocated(bitmap, index)\
    (bitmap[ index2word(index)] & (1 << (31 - index2bit(index))))

static inline void mark_slice (slab_t * slab, void * mem);
static inline void
mark_slice (slab_t * slab, void * mem)
{
    if (mem) {
        bitmap_t * bitmap = NULL;
        int index;
        word_t * seek = mem;

        if (look_up_address (slab->pool, slab, seek, &bitmap, &index)) {
            /* Found it. */
            if (is_allocated (bitmap, index)) {
                int word = index2word(index);
                int bit = index2bit(index);
                /* Move to the second half of the bitmap, where we store
                 * marking info. */
                bitmap += (slab->pool->n_bitmap_words >> 1);
                bitmap[word] |= 1 << (31 - bit);
            } else {
                /* Attempting to mark a freed pointer; error? */
            }
            return;

        } else if (bitmap != NULL) {
            /* Pointer was in this block, but is invalid.
             * Stop searching. */
            return;
        }
    }
}

static inline bool slice_is_allocated(void * ptr)ALWAYS_INLINE;
static inline bool
slice_is_allocated(void * ptr)
{
    if (USUALLY(ptr != NULL)) {
        bitmap_t * bitmap;
        slab_t * slab = ptr_to_slab(ptr);
        int index;
        if (look_up_address(slab->pool, slab, (word_t *)ptr, &bitmap, &index))
            return is_allocated(bitmap, index);
    }
    return false;
}
#if HAVE_VALGRIND
static inline void
valgrind_prepare_slab(slab_t * slab)
{
    int i = 0;
    slabpool_t * pool = slab->pool;

    bitmap_t * allocated;
    bitmap_t * marked;
    bitmap_t mask;

    allocated = slab_get_bitmap (slab);
    marked = allocated + (pool->n_bitmap_words >>1);

    mask = 1<<31;

    for (; (unsigned)i < pool->n_slices_per_slab ; i++) {
        if ((mask & *allocated) && (mask & *marked)) {
            word_t * slice = slab_get_slice(slab, pool->n_bitmap_words,
                                            pool->n_slice_words, i);

            valgrind_free(slice);
        }


        mask >>= 1;
        if (!mask) {
            mask = 1<<31;
            allocated ++;
            marked ++;
        }
    }

}
#endif

static inline unsigned int  
slab_reclaim(malloc_state_t * self, slab_t * slab, ReportFunction func)
{
    unsigned int reaped = 0;
    int i = 0;
    slabpool_t * pool = slab->pool;

    bitmap_t * allocated;
    bitmap_t * marked;
    bitmap_t mask;

    allocated = slab_get_bitmap (slab);
    marked = allocated + (pool->n_bitmap_words >>1);


    /* quickly check all slices.  If they are all marked, we can save
     * some time on the reclaim.
     */

    while (i < pool->n_slices_per_slab 
           && *allocated == *marked) {
        i += BITS_PER_WORD;
        *marked = 0;
        allocated ++;
        marked ++;
    }

    mask = 1<<31;

    for (; (unsigned)i < pool->n_slices_per_slab ; i++) {
        if ((mask & *allocated) && !(mask & *marked)) {
            word_t * slice = slab_get_slice(slab, pool->n_bitmap_words,
                                            pool->n_slice_words, i);

            debug ("*** leaking garbage at %p (%d bytes)from slab %p\n", 
                             slice, 
                             pool->slice_size,
                             slab);
            if (func) {
                func (fileno(stdout), slice, pool->slice_size);
            }
            valgrind_free(slice);

            *allocated &= ~mask;
            
            if (RARELY(fill_with_trash(self)))
                write_trash(self, slice);

            if (RARELY(slab->n_free == 0)) {
                insert_chunk(self, chunk_cast(slab));
            }
            if (RARELY(pool->n_available == 0)) {
                unlink_chunk(self, chunk_cast(pool));
            }
            slab->n_free++;
            pool->n_available++;

            self->available_words+= pool->n_slice_words;

            reaped+= pool->slice_size;
        } 
        else if (RARELY(RUNNING_ON_VALGRIND 
                        && (mask & *allocated)
                        && (mask & *marked))) {
            valgrind_malloc(slab_get_slice(slab, pool->n_bitmap_words,
                                            pool->n_slice_words, i), 
                            pool->slice_size);
            VALGRIND_MAKE_MEM_DEFINED(slab_get_slice(slab, pool->n_bitmap_words, pool->n_slice_words, i),
                                      pool->slice_size);
        }

        *marked &= ~mask;

        mask >>= 1;
        if (!mask) {
            mask = 1<<31;
            allocated ++;
            marked ++;
        }
    }

    if (reaped)
        slab->last_bitmap_word = 0;
    return reaped;
}

static inline unsigned int
slab_dump (slab_t * slab, ReportFunction func, int fd)
{
    unsigned int reaped = 0;
    int i = 0;
    slabpool_t * pool = slab->pool;

    bitmap_t * allocated;
    bitmap_t * marked;
    bitmap_t mask;

    allocated = slab_get_bitmap (slab);
    marked = allocated + (pool->n_bitmap_words >>1);


    malloc_printf (fd, "Dumping slab\n");

    /* quickly check all slices.  If they are all marked, we can save
     * some time on the reclaim.
     */

    while (i < pool->n_slices_per_slab 
           && *allocated == *marked) {
        i += BITS_PER_WORD;
        *marked = 0;
        allocated ++;
        marked ++;
    }

    mask = 1<<31;

    for (; (unsigned)i < pool->n_slices_per_slab ; i++) {
        if ((mask & *allocated) && !(mask & *marked)) {
            word_t * slice = slab_get_slice(slab, pool->n_bitmap_words,
                                            pool->n_slice_words, i);

            debug ("*** leaking garbage at %p (%d bytes)from slab %p\n", 
                             slice, 
                             pool->slice_size,
                             slab);
            if (func) {
                func (fd, slice, pool->slice_size);
            }

            
        } 

        mask >>= 1;
        if (!mask) {
            mask = 1<<31;
            allocated ++;
            marked ++;
        }
    }

    return reaped;
}


static void
slab_init (slab_t * slab, unsigned n_bitmap_words)
{
    word_t * slab_start = (word_t *)slab_get_bitmap(slab);
    word_t * slab_end = slab_start + n_bitmap_words/2;
    slab->last_bitmap_word = 0;

    /* The below loop is unrolled for speed.
     *
     * The "easy" version is this:
     *
     * for (word_t * start = slab_get_bitmap(slab); 
     *     start < slab_get_bitmap(slab) + n_bitmap_words;
     *      start++) {
     *      *start = 0;
     * }
     */
    while (slab_end - slab_start  > 0) {
        switch((slab_end - slab_start)% 8) {
        case 0:
            *slab_start = 0;
            *(slab_start + n_bitmap_words/2) = 0;
            slab_start++;
        case 7:
            *slab_start = 0;
            *(slab_start + n_bitmap_words/2) = 0;
            slab_start++;
        case 6:
            *slab_start = 0;
            *(slab_start + n_bitmap_words/2) = 0;
            slab_start++;
        case 5:
            *slab_start = 0;
            *(slab_start + n_bitmap_words/2) = 0;
            slab_start++;
        case 4:
            *slab_start = 0;
            *(slab_start + n_bitmap_words/2) = 0;
            slab_start++;
        case 3:
            *slab_start = 0;
            *(slab_start + n_bitmap_words/2) = 0;
            slab_start++;
        case 2:
            *slab_start = 0;
            *(slab_start + n_bitmap_words/2) = 0;
            slab_start++;
        case 1:
            *slab_start = 0;
            *(slab_start + n_bitmap_words/2) = 0;
            slab_start++;
        }
    }

}

void
slab_fix_up(malloc_state_t * self,
            slabpool_t *pool,
             slab_t * slab)
{

    unsigned int i;
    unsigned int last_word = pool->n_slices_per_slab /BITS_PER_WORD;
    unsigned int last_word_bits = BITS_PER_WORD * pool->n_bitmap_words/2 - pool->n_slices_per_slab; 

    bitmap_t * bitmap = slab_get_bitmap(slab);
    bitmap_t * mark = bitmap + (pool->n_bitmap_words >> 1);
    bitmap_t mask = mask_right(0xffffffff, last_word_bits); 

    mask = ~mask;
    bitmap[last_word] = mask;
    mark[last_word] = mask;

    debug_assert(population_count(bitmap[last_word]) == last_word_bits);

    if (RARELY(check_bounds(self) || verify(self)))
        for (i = 0; i< pool->n_slices_per_slab; i++) {
            word_t * mem = slab_get_slice(slab, pool->n_bitmap_words, pool->n_slice_words,i);
            mem[ pool->n_slice_words -1] = pool->slice_size;
        }

}
static inline unsigned int get_bitmap_words (unsigned int n_slices)PURE;
static inline unsigned int
get_bitmap_words (unsigned int n_slices)
{
    /* Round up to nearest word. */
    return 2 * ((n_slices + BITS_PER_WORD - 1) /BITS_PER_WORD);
}

static inline int num_slices_per_slab (unsigned int slab_size, 
                                       unsigned int slice_size)
    PURE;
static inline int
num_slices_per_slab (unsigned int slab_size,
                      unsigned int slice_size)
{
    /* Let's do some algebra to derive a formula that doesn't involve 
     * retrying after calculating the number of words required to store the
     * bitmap.
     *
     *   n := number of slices
     *   u := number of usable words
     *   s := number of words per slice
     *   b := number of bitmap words
     *   c := number of slab words (already less header words)
     *   a := number of bits in a word
     *
     *   n = u / s
     *
     * and
     *
     *   u = c - b
     *
     * But the number of bitmap words is a function of the number of slices.
     *
     *   b = (n + a - 1) / a
     *
     * So
     *
     *   n = u / s
     *   n = (c - b) / s
     *   n = c / s - b / s
     *   n = c / s - (n + a - 1) / (s * a)
     *   n * s * a = c * a - n - a + 1
     *   n * s * a + n = c * a - a + 1
     *   n * (s * a + 1) = c * a - a + 1
     *   n = (c * a - a + 1) / (s * a + 1)
     *
     * Whee!
     *
     * And then, when we need two bitmaps, so that we can store the mark status
     * as well as the allocation status, we need to double the number of bitmap
     * words.  This gets messy.  To make the normal case, allocation, faster,
     * we won't store the bits together.
     *
     *   b = 2 * (n + a - 1) / a
     *
     * So
     *
     *   n = u/s
     *   n = (c - b) /s
     *   n = c/s - b/s
     *   n = c/s - 2*(n + a - 1) / (s*a)
     *   n*s*a = c*a - 2*n - 2*a + 2
     *   n*s*a + 2*n = c*a - 2*a + 2
     *   n*(s*a + 2) = c*a - 2*a + 2
     *   n = (c*a - 2*a + 2) /(s*a + 2)
     *
     * Whee!
     */

    unsigned int s;
    unsigned int c;
    const int a = BITS_PER_WORD;
    s = slice_size;
    c = slab_size / BYTES_PER_WORD;
    c -= SLAB_HEADER_WORDS;

    return (c * a - 2 * a + 2) / (s * a + 2);
}

/* 
 * here, we avoid division in look_up_index by pre computing a kind of
 * recriprical for our divisor.
 *
 */
static inline void
initialize_slab_magic(slabpool_t * pool)
{
    // Must have 1 <= pool->n_slice_words <= 2^16-1.
    int p;
    unsigned ad, anc, delta, q1, r1, q2, r2, t;
    const unsigned two16 = 0x8000;     // 2^16.
    short magic;
    short shift;

    if ((pool->slice_size & (pool->slice_size -1)) == 0) {
        pool->magic = BYTES_PER_WORD;
        pool->shift = (uint16_t)BITS_PER_WORD -1 - clz(pool->slice_size);
        return;
    }

    ad = pool->n_slice_words;
    t = two16 + (ad >> 16);
    anc = t - 1 - t%ad;     // Absolute value of nc.
    p = 16;                 // Init. p.
    q1 = two16/anc;         // Init. q1 = 2**p/|nc|.
    r1 = two16 - q1*anc;    // Init. r1 = rem(2**p, |nc|).
    q2 = two16/ad;          // Init. q2 = 2**p/|d|.
    r2 = two16 - q2*ad;     // Init. r2 = rem(2**p, |d|).
    do {
        p = p + 1;
        q1 = 2*q1;           // Update q1 = 2**p/|nc|.
        r1 = 2*r1;           // Update r1 = rem(2**p, |nc|).
        if (r1 >= anc) {     // (Must be an unsigned
            q1 = q1 + 1;      // comparison here).
            r1 = r1 - anc;
        }
        q2 = 2*q2;           // Update q2 = 2**p/|d|.
        r2 = 2*r2;           // Update r2 = rem(2**p, |d|).
        if (r2 >= ad) {      // (Must be an unsigned
            q2 = q2 + 1;      // comparison here).
            r2 = r2 - ad;
        }
        delta = ad - r2;
    } while (q1 < delta || (q1 == delta && r1 == 0));

    magic = q2 + 1;
    shift  = p -1;            // shift amount to return.

    pool->magic = (unsigned short)magic;
    pool->shift = (unsigned short)shift; 
}


/*! Allocate a slice from a slab
 */

static inline void * allocate_slice (malloc_state_t * self, 
                                     slab_t * slab)ALWAYS_INLINE;
static inline void *
allocate_slice (malloc_state_t * self, slab_t * slab)
{
    word_t * mem = NULL;
    unsigned int i;
    slabpool_t * pool = slab->pool;

    /*
     * The first thing in the slab list either has a free slot, or there isn't
     * anything in the list.
     *
     * We take full slabs out of the list, and only put them back in when they
     * have a certain number of free slots.
     */
    bitmap_t * bitmap;
    bitmap_t mask;
    int index;
    unsigned short start;
    unsigned short word;
    unsigned short too_far;

    debug_assert (slab->last_bitmap_word < pool->n_bitmap_words/2);

    /* start at the bitmap word from which we most recently allocated. */
    start = slab->last_bitmap_word;
    word = start;
    too_far = pool->n_bitmap_words / 2;

    bitmap = slab_get_bitmap (slab) + start;

    while (1) {
        if (RARELY(word >= too_far)) {
            /* wrap around */
            word = 0;
            bitmap = slab_get_bitmap (slab);
        }
        if (~*bitmap)
            break;
        word ++;
        bitmap ++;

        if (RARELY (word == start))
            crash ("slab is full in allocate_slice() ?!?");
    }

    slab->last_bitmap_word = word;
    index = BITS_PER_WORD * slab->last_bitmap_word;
    i = clz (~*bitmap);
    index += i;

    mask = (1 << (31 - i));

    debug_assert (slab->last_bitmap_word < pool->n_bitmap_words/2);

    *bitmap |= mask;
    slab->n_free--;
    slab->pool->n_available--;

    mem = slab_get_slice (slab, pool->n_bitmap_words,
                          pool->n_slice_words, index);

    if (RARELY(slab->n_free == 0)) {
        unlink_chunk(self, chunk_cast(slab));

        if (RARELY(slab->pool->n_available == 0)) {
            insert_chunk(self, chunk_cast(slab->pool));
        }
    }

    debug ("  [%d/#%d] allocated slab %p index %d -> %p\n",
           pool->slice_size, self->malloccount,
           slab, index, mem);
    self->available_words -= pool->n_slice_words;

    return mem;
}

static inline void deallocate_slice (malloc_state_t * self, slab_t * slab, 
                                     void * mem)ALWAYS_INLINE;
static inline void
deallocate_slice (malloc_state_t * self, slab_t * slab, void * mem)
{
    word_t * data = mem;
    int index;
    bitmap_t * bitmap = NULL;
    slabpool_t * pool = slab->pool;

    if (USUALLY(look_up_address (pool, slab, data, &bitmap, &index))) {
        /* Found the pointer */
        int bitmap_word = index2word(index);
        int bitmap_bit = index2bit(index); 
        bitmap_t mask = 1 << (31 - bitmap_bit);
        if (bitmap[bitmap_word] & mask) {
            bitmap[bitmap_word] &= ~mask;
            bitmap += pool->n_bitmap_words/2;
            bitmap[bitmap_word] &= ~mask;

            slab->n_free++;

            self->available_words+= pool->n_slice_words;

            pool->n_available++;

            if (RARELY(check_bounds(self)
                       && ((word_t *)mem)[pool->n_slice_words -1] != pool->slice_size))
                crash("buffer overrun");

            if (slab->last_bitmap_word > bitmap_word) {
                slab->last_bitmap_word = bitmap_word;
                debug_assert (slab->last_bitmap_word < pool->n_bitmap_words/2);
            }
            if (RARELY (slab->n_free == 1)) {
                uint32_t index = SMALL_INDEX(pool->n_slice_words + WOVERHEAD);
                insert_first_binchunk(self, smallbin_at(self,index), bin_cast(slab), index);
                
                if (RARELY (pool->n_available == 1 && pool->link.next)) {
                    unlink_chunk(self, chunk_cast(pool));
                    pool->link.next = pool->link.prev = NULL;
                }

            } else if (RARELY(slab->n_free == pool->n_slices_per_slab 
                              && pool->n_available > pool->n_slices_per_slab)) {
                /* free the slab */

                mapping_t * mapping = find_mapping(self, slab);
                uintptr_t page_addr = ((uintptr_t)slab);
                uint32_t offset = page_addr - ((uintptr_t)mapping->top);
                chunk_t * chunk = chunk_cast(slab);
                offset /= PAGESIZE_BYTES;
                mapping->slabmap[index2word(offset)] &= ~(1 << index2bit(offset));
                unlink_chunk(self, chunk_cast(slab));
                pool->n_available-= pool->n_slices_per_slab;
                set_available(slab);
                slab->head &= ~SLAB_BIT;

                if (!prev_chunk_in_use(slab)
                    && !chunk_is_slab(prev_chunk(chunk))) {
                    unhook_pages(self, prev_chunk(chunk));
                    unlink_chunk(self, prev_chunk(chunk));
                    chunk = join_chunk(self, prev_chunk(chunk), chunk);
                }
                if (!next_chunk_in_use(chunk)
                    && !chunk_is_slab(next_chunk(chunk))) {
                    unhook_pages(self, next_chunk(chunk));
                    unlink_chunk(self, next_chunk(chunk));
                    chunk = join_chunk(self, chunk, next_chunk(chunk));
                }

                valgrind_make_chunk_defined(chunk);
                insert_chunk(self, chunk);
                hook_pages(self, chunk);
                self->available_words += bytes_to_words(sizeof (slab_t));

            } else if (RARELY(slab->n_free == pool->n_slices_per_slab/2)) {

                /* here, we move the slab to the end of the list, if there is
                 * one, because it looks like it is freeing up */

                uint32_t index = SMALL_INDEX(pool->n_slice_words + WOVERHEAD);

                if (next_binchunk(slab)!= prev_binchunk(slab)
                    && prev_binchunk(slab)!= smallbin_at(self, index)) {
                    unlink_chunk(self, chunk_cast(slab)); 
                    insert_last_binchunk(self, smallbin_at(self, index), bin_cast(slab), index);
                } 
            }
        } else {
            crash("double free of addr %p", mem);
        }
        return;
    } else if (bitmap != NULL) {
        /* The pointer was in this slab, but is invalid */
        word_t * almost;
        word_t * base;
        debug ("*** attempt to free bad address %p\n", mem);
        base = slab_get_base (slab, pool->n_bitmap_words);
        almost = base + index * pool->n_slice_words;
        debug ("*** %p is %d bytes away from valid address %p\n",
                mem, ((unsigned char *)mem)- ((unsigned char *)almost),
                almost);
        crash("Wild pointer to free: %p", mem);
        return;
    }

    debug ("*** attempt to free unallocated pointer %p\n", mem);
}
/*
 * This is a bit ... interesting.
 * We have to make a slab for slabpool_t structures.
 * So, we do it the hard way:
 * Get a page aligned block big enough to hold an integral number of slabpool_t's.
 * We then fill in the first such structure with the relavent data for this
 * pool, and insert it into the proper smallbin.
 */
static inline void
create_slabpool_slab(malloc_state_t * self)
{
    slabpool_t pool;
    slabpool_t * real_pool;
    unsigned int slices_per_slab;
    unsigned int slice_bytes;
    unsigned int slab_size;
    slab_t * slab; 

    slice_bytes = words_to_bytes(NEAREST_N_WORDS(sizeof (slabpool_t)));
    slice_bytes += self->overhead - OVERHEAD;
    slices_per_slab = num_slices_per_slab(PAGESIZE_BYTES, 
                                          bytes_to_words(slice_bytes));

    slab_size = slice_bytes * slices_per_slab + 
        words_to_bytes(get_bitmap_words(slices_per_slab)) + sizeof (slab_t);
    
    pool.slice_size = slice_bytes;
    pool.slab_size = slab_size;
    pool.n_slice_words = bytes_to_words(slice_bytes);
    pool.n_slices_per_slab = slices_per_slab;
    pool.n_bitmap_words = get_bitmap_words(slices_per_slab);

    debug_assert (pool.n_bitmap_words % 2 == 0);
    
    slab = internal_allocate_slab(self, &pool);

    valgrind_make_internals_defined(self);

    slab->n_free = pool.n_slices_per_slab;
    slab->last_bitmap_word =0;
    slab->pool = &pool;

    real_pool = allocate_slice(self, slab);
    self->slab_pool = (slab_t *)real_pool;
    VALGRIND_MAKE_MEM_DEFINED(real_pool, sizeof (slabpool_t));
    memcpy(real_pool, &pool, sizeof (slabpool_t));

    real_pool->head = SLAB_BIT | IN_USE_BIT | 
        (real_pool->n_slice_words +WOVERHEAD);
    real_pool->n_available = slab->n_free;

    slab->pool = real_pool;
    self->slab_pool = slab;
    insert_chunk(self, chunk_cast(slab));

    initialize_slab_magic(real_pool);
    
    debug("Created pool %p header: 0x%08x slice: %d count: %d\n", 
          real_pool, 
          real_pool->head, 
          real_pool->slice_size, 
          real_pool->n_slices_per_slab);
}
static inline void 
slab_pool_create (malloc_state_t * self, 
                  unsigned int slice_size,
                  unsigned int slab_size)
{
    slabpool_t * pool;
    slab_t * slab;
    unsigned int slices_per_slab;
    unsigned int slice_bytes;

    slice_bytes = words_to_bytes(bytes_to_words(slice_size));
    slices_per_slab = num_slices_per_slab(slab_size, 
                                          bytes_to_words(slice_bytes));

    slab_size = pad_size(self, slice_bytes * slices_per_slab + 
        words_to_bytes(get_bitmap_words(slices_per_slab)) + sizeof (slab_t)- self->overhead);

    /* Have to fix this -- self->slab_pool isn't updated correctly */

    pool = allocate_slice(self, self->slab_pool);

    VALGRIND_MAKE_MEM_DEFINED(pool, sizeof (slabpool_t));

    pool->n_slice_words = bytes_to_words(slice_bytes);
    pool->n_available = 0;
    pool->n_slices_per_slab = slices_per_slab;
    pool->slab_size = slab_size;
    pool->slice_size = slice_bytes;
    pool->n_bitmap_words = get_bitmap_words(slices_per_slab);
    pool->head = SLAB_BIT | IN_USE_BIT | (pool->n_slice_words +WOVERHEAD);

    initialize_slab_magic(pool);

    slab = internal_allocate_slab(self , pool);
    valgrind_make_chunk_defined(chunk_cast(slab));
    insert_chunk(self, chunk_cast(slab));

    debug("Created pool %p header: 0x%08x slice: %d count: %d\n", 
          pool, 
          pool->head, 
          pool->slice_size, 
          pool->n_slices_per_slab);

    return;
}
/* End Slab code */

/**
 * Insert a chunk into the appropriate bin or tree.
 * @param self the malloc state
 * @param block the chunk to insert
 */
static void
insert_chunk(malloc_state_t * self, chunk_t * block)
{
    size_t size; 
    unsigned int index;
    
    size = chunk_words(block);

    /* Slabs always have 1 less word of overhead than the rest of the allocator.
     * In bounds check mode, they have 1 word, while normal allocs have 2 words.
     * In normal mode, they have 0 words, while normal allocs have 1 word.
     *
     * So, no matter the mode, it is always correct to bump the number of words
     * by 1 for binning purposes.
     */

    if (chunk_is_slab(block) && !chunk_in_use(block)) {
        size = ((slab_t *)block)->pool->n_slice_words + WOVERHEAD;
    }
    if (size < bytes_to_words(SMALLBLOCK_MAX)) {

        binchunk_t * list;    
        binchunk_t * chunk = bin_cast(block);
        index = SMALL_INDEX(size);
        debug_assert(index < SMALLBINS);
        list = smallbin_at(self, index);
        insert_last_binchunk(self, list, chunk, index);

    } else {
        struct lbin * list; 
        index = large_index(size);
        list = largebin_at(self, index);

        rb_insert(list, tree_cast(block));

        if (!largebin_populated(self, index)) {
            mark_largemap(self, index);
        }

        dirty_list_link(tree_cast(block), self->dirty.dirty.next);
    }
}

static inline chunk_t *
manage_slab(malloc_state_t * self,  chunk_t * block)
{
    /* we're either a slab or a slab pool */

    slabpool_t * pool;

    if (USUALLY(!chunk_in_use (block)))
        return block;

    pool = (slabpool_t *)block;

    if (USUALLY(pool->n_available == 0)) {
        unsigned int index;
        slab_t * slab;

        unlink_chunk(self, chunk_cast(pool));
        pool->link.next = pool->link.prev = NULL;

        index = SMALL_INDEX(pool->n_slice_words + WOVERHEAD);
        slab = internal_allocate_slab(self, pool);

        debug ("allocating new slab for pool: %p [%d]\n", pool, pool->slice_size);

        if (USUALLY(slab != NULL)) {
            insert_first_binchunk (self, smallbin_at(self, index), bin_cast(slab), index); 
            return chunk_cast(slab);
        } else if (smallbin_populated(self, index)) {
            block = chunk_cast(smallbin_at(self,index)->link.next);
            unlink_first_freeblock(self, 
                                   smallbin_at(self, index), 
                                   bin_cast(block), 
                                   index);
            debug ("slab alloc failed, returning next block\n");
            return block;
        }     
    
        return NULL;
    } else {


        /* slab pool with stuff available -- should never happen */
        crash("Slab pool in bins with slabs available");
    } 
    return block;

}

/**
 * find a binchunk.
 * @param self the malloc state.
 * @param words the chunk size requested, in words.
 * @return a chunk large enough to hold the requested block or NULL
 */
static inline chunk_t *
find_binchunk(malloc_state_t * self, size_t words)
{
    unsigned int index;
    bitmap_t * bitmap;
    binchunk_t * bin;
    chunk_t * block;
    index = SMALL_INDEX(words);

    bitmap = &self->smallmap[ index2word(index)];

    if (mask_right((*bitmap), index2bit(index))) {
        index += ctz((*bitmap) >> index2bit(index));
        bin = bin_cast(smallbin_at(self, index));
        block = chunk_cast(bin->link.next);
        debug_assert(smallbin_populated(self, index));
        if (chunk_is_slab(block)) { 
            /* we're either a slab or a slab pool */
            block = manage_slab(self, block);
            if (USUALLY(block != NULL))return block;
            
            index++;
            bitmap = &self->smallmap[index2word(index)];
            goto EXHAUSTIVE_SEARCH;

        } else {
            unlink_first_freeblock(self, bin, bin_cast(block), index);
        }
    } else {
        /* try to find anything in the small bins, then go to the large bins */
EXHAUSTIVE_SEARCH:
        while (!mask_right(*bitmap, index2bit(index))
               &&  index < SMALLBINS) {
            bitmap++;
            index = (index2word(index) +1) << 5U;
        }
        if (*bitmap && index < SMALLBINS) {
            index += ctz (*bitmap >> index2bit(index));
            bin = bin_cast(smallbin_at(self, index));
            block = chunk_cast(bin->link.next);
            debug_assert(smallbin_populated(self, index));
            if (chunk_is_slab(block)) {
                block = manage_slab(self, block);
                if (USUALLY(block!=NULL))return block;
                index++;
                bitmap = &self->smallmap[index2word(index)];
                goto EXHAUSTIVE_SEARCH;   
            } else
                unlink_first_freeblock(self, bin, bin_cast(block), index);
        } else {
            block = find_treechunk(self,  bytes_to_words(MIN_LARGEBLOCK_SIZE));

            if (!block)return NULL;
        }
    }
    return block;
}

/**
 * find a large chunk
 * @param self the malloc state.
 * @param words the requested chunk size, in words.
 * @return a chunk large enough to hold the requested size, or NULL.
 */
static chunk_t * 
find_treechunk(malloc_state_t * self, size_t words)
{
    unsigned int index; 
    treechunk_t * block = NULL;
    bitmap_t * bitmap;

    index = large_index(words);

    bitmap = &self->largemap[ index2word(index)];

    while (!block && index < LARGEBINS) {
        if (largebin_populated(self, index)
            && largebin_at(self,index)->max >= words) {
            block = rb_remove_size(largebin_at(self, index), words);
        } 
        if (!block) {
            if (mask_right(*bitmap, index2bit(index+1)))
                index+= ctz(mask_right(*bitmap, index2bit(index +1)) >> index2bit(index));
            else{
                bitmap++;
                index = (index2word(index) +1) << 5U;
            }
        }
    }

    if (block) {
        dirty_list_unlink(block);
        if (largebin_at(self, index)-> root == NULL) {
            largebin_at(self, index)->max = 0;
            clear_largemap(self, index);
        }
    }

    return chunk_cast(block);
}
/**
 * find a chunk.
 * @param self the malloc state.
 * @param words the request size, in words.
 * @return a chunk at least the requested number of words or NULL
 *
 * Glorified wrapper function.  Based on size, calls find_binchunk or
 * find_treechunk.
 *
 * Also does the common block splitting.
 */
static inline chunk_t *
find_chunk(malloc_state_t * self, size_t words)
{
    chunk_t * block;
    chunk_t * remainder = NULL;

    if (RARELY(words > self->available_words))
        return NULL;

    if (RARELY (words > bytes_to_words(PAGESIZE_BYTES)
                && words > 
                (unsigned int)bytes_to_words((2 + self->available_pages)
                                             * PAGESIZE_BYTES))) {
        /*
        printf ("not enough pages:need %d, have %d\n",
               words/1024, self->available_pages
              );
               */
       return NULL;
    }

    if (RARELY(self->available_pages < 2 
               && words >= bytes_to_words(PAGESIZE_BYTES))) {
        /*
        printf ("not enough pages, not small alloc\n ");
        */
        return NULL;
    }

    if (RARELY(self->available_pages > 1 
               && self->available_pages == self->reclaimable_pages)) {
        clean_pages(self, 1);
    }


    if (USUALLY(words < bytes_to_words(SMALLBLOCK_MAX))) {
        block = find_binchunk(self, words);
        if (chunk_is_slab(block)) {
            return block;
        }
    } else {
        block = find_treechunk(self, words);
    }

    if (RARELY(!block)) {
        return NULL;
    }

    if (RARELY(words > (unsigned int)bytes_to_words(self->available_pages * PAGESIZE_BYTES)
               && ptr_plus_offset_crosses_page(&block->head, words_to_bytes(words)
                               + new_chunk_overhead(words_to_bytes(chunk_words(block)
                                                                   - words)))
               && new_dirty_pages(self, block, words))) {
        insert_chunk(self, block);
        /*
        printf ("would dirty a page I don't have\n");
        */
        return NULL;
    }

    if (chunk_words(block) >= words + bytes_to_words(MIN_BLOCK_SIZE)) {
        remainder = split_chunk_top(self, block, words);
    } 


    if (RARELY(self->available_pages < (int)contained_pages(block))) {
        if (!next_chunk_in_use(block)
            && !chunk_is_slab(next_chunk(block))
            && remainder) {
            unhook_pages(self, block);
            unhook_pages(self, remainder);
            block = join_chunk(self, block, remainder);
            hook_pages(self, block);
        }
        insert_chunk(self, block);
        block = NULL;
        /*
        printf ("not enough available pages\n");
        */
    } else if (USUALLY(remainder != NULL)) {
        insert_chunk (self, remainder);
    }
    return block;
}
/**
 * perform bounds checking on chunks
 * @param self the malloc state.
 * @return 0 if everything checkous out, non 0 on error.
 */    
static inline int 
check_chunks (malloc_state_t * self)
{

    mapping_t * mapping; 
    int ret = 0;

    for (mapping = self->mappings; mapping; mapping = mapping->next) {
        chunk_t * chunk = mapping->fence;

        while (chunk_words(chunk) && ret == 0) {
            if (chunk_is_slab(chunk)) {
                slab_t * slab = slab_cast(chunk);
                slabpool_t * pool = slab->pool;
                int i;

                for (i = 0; i< pool->n_slices_per_slab; i++) {
                    word_t * mem = slab_get_slice(slab, pool->n_bitmap_words, pool->n_slice_words,i);
                    ret += mem[ pool->n_slice_words -1] != pool->slice_size;
                }
            } else {
                ret += chunk_words(chunk)!= prev_chunk_words(next_chunk(chunk));
            }

            chunk = next_chunk(chunk);
        }

    }

    return ret;
}
/**
 * Return a chunk of memory.
 * @param self the malloc state.
 * @param ptr address of the block to reclaim
 *
 */

static inline void internal_deallocate(malloc_state_t *, void *)ALWAYS_INLINE;
static inline void
internal_deallocate(malloc_state_t * self, void * ptr)
{
    chunk_t * block = mem_to_chunk(ptr);

    valgrind_make_internals_defined(self);
    memory_debug_assert(address_ok(self, ptr));

    if (RARELY(misaligned_chunk(ptr)))
        crash ("misaligned pointer to free: %p", ptr);

    valgrind_free(ptr);

    if (RARELY(fill_with_trash(self)))
       write_trash(self, ptr);

    self->freecount++;
    if (RARELY(self->freecount == self->malloccompare))
        breakonmalloc();

    if (RARELY (verify (self)))
        check_chunks (self);
    
    if (alloc_is_slice(self, ptr)) {
        deallocate_slice(self, ptr_to_slab(ptr), ptr);
        memory_debug_assert(0 == internal_verify (self));
        valgrind_make_internals_noaccess(self);
        return;
    }

    if (RARELY(check_bounds(self))) {
        if (prev_chunk_words(block)!= chunk_words(prev_chunk(block)))
            crash("buffer overrun - previous chunk");
        if (chunk_words(block)!= prev_chunk_words(next_chunk(block)))
            crash("buffer overrun"); 
    }

    valgrind_make_chunk_defined(block);
    if (RARELY(!chunk_in_use(mem_to_chunk(ptr)))) {
        crash("double free of addr %p", ptr);
        return;
    }

    if (RARELY(chunk_is_slab(block))) {
        crash("buffer overrun - previous chunk");
    }


    valgrind_make_chunk_available(block);
    valgrind_make_chunk_defined(block);
    valgrind_make_chunk_defined(next_chunk(block));

    set_available(block);
    self->available_words += chunk_words(block);
    free_pages(self, block);
    unhook_pages(self, block);

    if (!prev_chunk_in_use(block) || !next_chunk_in_use(block)) {
        chunk_t * next = NULL;
        chunk_t * prev = NULL;
        
        if (!prev_chunk_in_use(block)) {
            prev = prev_chunk(block);
            if (!chunk_is_slab(prev)) {
                unlink_chunk (self, prev);
                unhook_pages(self, prev);
                block = join_chunk(self, prev , block);
            }
        }

        next = next_chunk (block);

        if (next 
            && !chunk_in_use(next)
            && !chunk_is_slab(next)) {
            unlink_chunk(self, next);
            unhook_pages(self, next);
            block = join_chunk (self, block, next);
            
        }

    }     
    hook_pages(self, block);
    valgrind_make_chunk_defined(block);
    insert_chunk(self, block);
    if (RARELY(self->reclaimable_pages >= self->dirty_page_ceiling))
        clean_pages(self, 
                    self->reclaimable_pages> 1 ? self->reclaimable_pages/2: 
                                                 self->reclaimable_pages);
        
    memory_debug_assert(0 == internal_verify (self));
    valgrind_make_internals_noaccess(self);
}

/**
 * Flush the freelist.
 * @param self The malloc state
 */
static inline void
unlocked_flush_freelist(malloc_state_t * self)
{
    freechunk_t * node = self->free_list;
    freechunk_t * next;

    while (node) {
        next = node->next;
        if (USUALLY (NULL != find_mapping (self, node))) {
            internal_deallocate(self, node);
            add_free_event(self, FREE_EVENT((void *)node), 0, NULL);
        } else 
            crash("free of wild pointer");
        node = next;
    }
    self->free_list = NULL;
} 

/******************************************************************
 * mapped space managment.
 * We may have multiple mappings avaialble.  
 *****************************************************************/
/**
 * map some address space.
 * @param available bytes to service from this mapping.
 * @param bytes size of the mapping to make.
 * @return the created mapping, or NULL.
 */
static inline mapping_t * 
map_address_space(unsigned int available, unsigned int bytes)
{
    uint32_t usable_size;
    uint32_t mapwords;
    mapping_t * area;
    chunk_t * block;
    chunk_t * fencepost;
    void * mapping;

    bytes = (bytes + PAGE_MASK) & ~(PAGE_MASK);

    mapwords = bytes/PAGESIZE_BYTES;
    mapwords = (mapwords + BITS_PER_WORD-1) &~(BITS_PER_WORD-1);
    mapwords /= BITS_PER_WORD;
    mapwords = max(1, mapwords);
    mapwords = (mapwords + INTALIGN_MASK) & ~(INTALIGN_MASK);

    mapping =  mmap(0, bytes, PROT_READ|PROT_WRITE,MAP_PRIVATE|MAP_ANONYMOUS|MAP_NORESERVE, -1, 0);

    if (mapping == MAP_FAILED)return NULL;

    area = int_aligned_adder_from_offset(mapping,words_to_bytes(mapwords*2));

    area->next = NULL;
    area->size = bytes_to_words(bytes);
    area->mapwords = mapwords;
    area->slabmap = (uint32_t *)mapping;
    area->dirtymap = int_aligned_adder_from_offset(mapping, words_to_bytes(mapwords));
    area->fake =1;

    block = word_aligned_adder_from_offset(area, sizeof (*area));

    usable_size = bytes - words_to_bytes(mapwords) *2;
    usable_size -= sizeof (*area);
    usable_size -= offsetof(chunk_t, head);
    usable_size -= offsetof(chunk_t, head);

    usable_size = bytes_to_words(usable_size);

    block->head = usable_size;
    block->prev_size = 1;
    set_prev_in_use(block);

    area->top = mapping;
    area->fence = block;
    
    fencepost = new_block_from_offset(block, words_to_bytes(usable_size));

    fencepost->head = PREV_IN_USE_BIT|IN_USE_BIT;
    fencepost->prev_size = usable_size;

    area->bottom = &fencepost->head;

    area->total_words = bytes_to_words(available)- 
        (bytes_to_words(bytes)- usable_size);

    VALGRIND_MAKE_MEM_NOACCESS(((uintptr_t)area->fence) + 
                               offsetof(chunk_t, head), 
                               words_to_bytes(area->total_words));
    valgrind_make_chunk_defined(area->fence);

    area->map_pages = max(crossed_pages(area->fence),1);

    
    return area;
}

static const unsigned int HALF_GB = 512 * 1024 * 1024;
static const unsigned int BACKOFF = 1024 * 1024 * 128; 

void
_anr_core_add_mapping(malloc_state_t * self, unsigned int available, unsigned int bytes)
{
    uintptr_t offset;
    uintptr_t end;
    uintptr_t start;
    mapping_t * mapping = map_address_space(available, bytes);

    LOCK(&self->heap_lock);

    /* yes, this looks a bit odd.
     * Here's the deal:
     * If we try to make a mapping less than 512MB, and it is our first mapping,
     * something is seriously wrong, and needs to be looked at.  
     * So we puke.
     *
     * If we're trying to make a larger mapping, or we have a mapping already,
     * we try to back off in 128MB increments.
     *
     * Lastly, we verify that we have some sort of mapping avaialable and 
     * crash or bail out as needed.
     *
     */

    if (RARELY(mapping == NULL 
               && bytes < HALF_GB 
               && self->mappings == NULL)) {
        crash("could not map memory");
    } else {
       
        while (mapping == NULL && bytes >= available) {
            bytes -= BACKOFF;
            mapping = map_address_space (available, bytes);
        }
    }

    if (RARELY(mapping == NULL)) {
        if (self->mappings == NULL)
            crash("could not map memory");
        else {
            UNLOCK(&self->heap_lock);
            return;
        }
    }


    self->total_words += mapping->total_words;
    self->available_words += mapping->total_words;
    self->available_pages += mapping->total_words/bytes_to_words(PAGESIZE_BYTES); 
    self->total_pages += mapping->total_words/bytes_to_words(PAGESIZE_BYTES);

    self->available_pages -= 1; /* fencepost at bottom of mapping */

    mapping->next = self->mappings;
    self->mappings = mapping;

    valgrind_make_internals_defined(self);
    insert_chunk(self, mapping->fence);
    dirty_list_unlink(tree_cast(mapping->fence));
    tree_cast(mapping->fence)->dirty.prev = tree_cast(mapping->fence);
    tree_cast(mapping->fence)->dirty.next = tree_cast(mapping->fence);

    offset = (uintptr_t) &next_chunk(mapping->fence)->head;
    offset -= (uintptr_t)mapping->top;
    offset /= PAGESIZE_BYTES;

    debug_assert(offset/BITS_PER_WORD  < mapping->mapwords);
    mapping->dirtymap[ index2word(offset)] = 1 << index2bit(offset);

    /* mark the dirtymap bits for area->top -> area->fence */
    end = (uintptr_t)mapping->fence - (uintptr_t)mapping->top;
    end += (uintptr_t)new_chunk_overhead (chunk_bytes(mapping->fence));
    start = (uintptr_t)0;

    end /= PAGESIZE_BYTES;

    for (; start <= end; start++) {
        mapping->dirtymap[ index2word(start)] |= 1 << index2bit(start);
    }

    valgrind_make_internals_noaccess(self);
    UNLOCK(&self->heap_lock);
}
unsigned int
_anr_core_mapping_overhead(unsigned int bytes)
{
    unsigned int mapwords; 
    unsigned int overhead;
    mapwords = bytes/PAGESIZE_BYTES/BITS_PER_WORD *2;

    overhead = offsetof(mapping_t, size);
    overhead += offsetof(chunk_t, head);
    overhead += offsetof(chunk_t, head);

    return overhead + words_to_bytes(mapwords);
}


/**
 * Default giveback function.  Should not be used.
 * @param size the bytes requested.
 * @param attempt anxiety of the request.
 */
static unsigned int 
default_more_memory(unsigned int attempt, unsigned int size)
{
    UNUSED(size);
    UNUSED(attempt);
    return 0;
    
}
static inline void
default_memfull(uint32_t a, uint32_t b, uint32_t c, uint32_t d)
{
    UNUSED(a);
    UNUSED(b);
    UNUSED(c);
    UNUSED(d);
}
/*****************************************************************************
 * Client API
 ****************************************************************************/
void 
_anr_core_block_until_free(malloc_state_t * self)
{
    LOCK(&self->freelist_lock);

    while (!self->free_list) {
        WAIT(&self->list_populated, &self->freelist_lock);
    }

    unlocked_flush_freelist(self);

    UNLOCK(&self->freelist_lock);

    return;
}

int
_anr_core_init(malloc_state_t ** out_self,  
               int flags, 
               unsigned int pool_size, 
               unsigned int mapsize, 
               int dirty_page_ceiling,
               unsigned int slab_count,
               unsigned int * slab_sizes,
               MoreMemoryFunction gb_func, 
               void * cb_context,
               AbortFunction abort_func)
{
    uintptr_t offset; 
    event_t * event;
    malloc_state_t * self;
    unsigned int i;
    int mapcount = 1;
    pthread_mutexattr_t attr;

    
    if (RARELY(pool_size > mapsize))
        crash("invalid map size %u", mapsize);

    self = mmap(0, 2 * PAGESIZE_BYTES, 
                PROT_READ|PROT_WRITE,MAP_PRIVATE|MAP_ANONYMOUS|MAP_NORESERVE, 
                -1, 
                0);
    if (self == MAP_FAILED)
        return 1;


    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE_NP);
    pthread_mutex_init (&self->heap_lock, &attr);
    pthread_mutex_init (&self->freelist_lock, &attr);
    pthread_mutexattr_destroy(&attr);

    pthread_cond_init(&self->list_populated, NULL);

    LOCK(&self->heap_lock);

    self->overhead = OVERHEAD;

    for (i = 0; i < SMALLBINS; i++) {
        clear_smallbin(self, i);
        smallbin_at (self, i)->head = IN_USE_BIT | (i + sizeof (binchunk_t));
    }


    if (flags & FILL_WITH_TRASH)
        enable_trash(self);

    if (flags & BOUNDS_CHECK)
        enable_bounds_check(self);

    if (flags & MALLOC_VERIFY)
        enable_verify(self);

    if (RARELY(verify(self) || check_bounds(self))) {
        self->overhead += OVERHEAD;
        for (i = 0; i < slab_count; i++) {
            if (slab_sizes[i] == words_to_bytes(NEAREST_N_WORDS(sizeof (slabpool_t))))
                slab_sizes[i] = 0;
            else {
                slab_sizes[i] = words_to_bytes(NEAREST_N_WORDS(slab_sizes[i] + OVERHEAD));
            }
        }
    } else {
        for (i = 0; i< slab_count; i++) {
            if (slab_sizes[i] == words_to_bytes(NEAREST_N_WORDS(sizeof (slabpool_t))))
                slab_sizes[i] = 0;
            else
                slab_sizes[i] = words_to_bytes(NEAREST_N_WORDS(slab_sizes[i]));

        }
    }

    for (i = 0; i < slab_count; i++) {
        uint32_t idx;
        for (idx = 0; idx < slab_count; idx++) {
            if (i != idx 
                && slab_sizes[i] != 0 
                && slab_sizes[i] == slab_sizes[idx]) {
                debug("removing duplicate slab size %d\n", slab_sizes[i]);
                slab_sizes[i] = 0;
            }
        }

    }


    self->malloccompare = getenv("MALLOC_COMPARE")!= 0 ? atoi(getenv("MALLOC_COMPARE")):0;
    self->dirty_page_ceiling = dirty_page_ceiling;

    self->history_head = (event_t *)(((uintptr_t)self) + sizeof (malloc_state_t));
    event = self->history_head;

    offset = (uintptr_t)event;

    for (i =0; 
         i < (((uintptr_t)self + 2 * PAGESIZE_BYTES)- offset) /sizeof (event_t);
         i++) {
        event = &self->history_head[i];
        event->next = &self->history_head[i+1];
    }

    self->history_tail = event;
    self->history_tail->next = NULL;

    if (abort_func)
        _abort = abort_func;

    self->free_list = NULL;
    self->dirty.dirty.next = self->dirty.dirty.prev = &self->dirty;
    /* we map the state struct in 2 pages.  gotta account for that */
    mapsize -= 2 * PAGESIZE_BYTES;
    pool_size -= 2 * PAGESIZE_BYTES;
    switch (bytes_to_words(mapsize) & INTERNAL_BITS) {
        case 0xF0000000:  /* all of these cases are not possible in 32 bit. */
        case 0xE0000000:  /* as we just knocked out the top 2 bits of mapsize in the switch */
        case 0xD0000000:  /* so, any case that has either of the top two bits set is invalid */
        case 0xC0000000:
        case 0xB0000000:
        case 0xA0000000:
        case 0x90000000:
        case 0x80000000:
            mapcount = 1 << 4;
            mapsize >>= 4; 
            pool_size >>=4;
            break;
        case 0x70000000:
        case 0x60000000:
        case 0x50000000:
        case 0x40000000:
            mapcount = 1 <<3;
            mapsize >>= 3;
            pool_size >>=3;
            break;
        case 0x30000000:  /* these cases are valid in 32 bit.  0x03 is 0011b. */
        case 0x20000000:
            mapcount = 1 << 2;
            mapsize >>= 2;
            pool_size >>=2;
            break;
        case 0x10000000:
            mapcount = 1 << 1;
            mapsize >>=1;
            pool_size >>=1;
            break;
        case 0x00000000:
            mapcount = 1;
            break;
    }


    for (; mapcount > 0; mapcount--) {
        _anr_core_add_mapping (self, pool_size, mapsize);
        self->dirty.dirty.next = self->dirty.dirty.prev = &self->dirty;
    }

    valgrind_make_internals_defined(self);

    if (gb_func)
        self->_more_memory = gb_func;
    else
        self->_more_memory = (MoreMemoryFunction)default_more_memory;
    self->cb_context = cb_context;
    self->reclaimable_pages = 0;

    if (slab_count > 0) {
        uint32_t idx = 0;
        create_slabpool_slab(self);
        for (idx = 0; idx < slab_count; idx++) {
            if (slab_sizes[idx] > 0 
                && slab_sizes[idx] < PAGESIZE_BYTES/4 
                && slab_sizes[idx] != words_to_bytes(NEAREST_N_WORDS(sizeof (slabpool_t))))

                slab_pool_create(self, slab_sizes[idx], PAGESIZE_BYTES);

        }
    }

    *out_self = self;
    UNLOCK(&self->heap_lock);
    
    return 0;
}
static void 
local_abort(const char *file, unsigned int line, const char * msg)
{
    printf ("%s:%d %s\n", file, line, msg);
    abort();
}

void
_anr_core_abort(const char * file, unsigned int line, const char * format, ...)
{
    va_list ap;
    char msg[256];
    va_start(ap, format);
    vsnprintf (msg, sizeof (msg), format, ap);
    va_end(ap);
    _abort(file, line, msg);
    abort(); /* should not be reached, _abort()should not return */
}

int 
_anr_core_default_init(malloc_state_t ** self)
{
    /* TODO 
     * Call getrlimit, and use that as the map size
     */

#define DEFAULT_MAP_FACTOR 2
#define DEFAULT_POOL_SIZE_BYTES 64 * 1024 * 1024
#define DEFAULT_MAP_SIZE_BYTES DEFAULT_POOL_SIZE_BYTES * DEFAULT_MAP_FACTOR
#define PAGE_CEILING 1024
    return _anr_core_init(self, 
                          0,
                          DEFAULT_POOL_SIZE_BYTES, 
                          DEFAULT_MAP_SIZE_BYTES, PAGE_CEILING, 
                          0, NULL,
                          (MoreMemoryFunction)default_more_memory, 
                          (MemoryFullFunction)default_memfull,
                          (AbortFunction)local_abort
                        );
        
#undef DEFAULT_MAP_SIZE_BYTES
#undef DEFAULT_MAP_FACTOR
#undef DEFAULT_POOL_SIZE_BYTES
#undef PAGE_CEILING
} 

void
_anr_core_teardown(malloc_state_t * self)
{
    mapping_t * mapping;

    _anr_core_reclaim(self, (ReportFunction)NULL);

    for (mapping = self->mappings; mapping != NULL;) {
        mapping_t * next = mapping->next;
        VALGRIND_MAKE_MEM_DEFINED(mapping->fence, sizeof (uint32_t));
        munmap(mapping->top, words_to_bytes(mapping->size));
        mapping = next;
    }
    munmap(self, 2 * PAGESIZE_BYTES);

}

void
_anr_core_mark(malloc_state_t * self,  void * ptr)
{
    mapping_t * mapping;
    LOCK(&self->heap_lock);

    valgrind_make_internals_defined(self);

    mapping = find_mapping(self ,ptr);

    if (USUALLY(mapping!= NULL)) {
        if (alloc_is_slice(self, ptr)) {
            mark_slice(ptr_to_slab(ptr),ptr);
        } else if (chunk_in_use(mem_to_chunk(ptr)))
            mark_block(mem_to_chunk(ptr));
    } 
    valgrind_make_internals_noaccess(self);

    UNLOCK(&self->heap_lock);
    return;

}

unsigned int
_anr_core_reclaim(malloc_state_t * self, ReportFunction func)
{
    mapping_t * mapping;
    unsigned int reclaimed = 0;

    freechunk_t * list = NULL;

    LOCK(&self->heap_lock);

    valgrind_make_internals_defined(self);

    LOCK(&self->freelist_lock);
    unlocked_flush_freelist(self);
    UNLOCK(&self->freelist_lock);

    /* annoyingly, we need to tell valgrind that marked allocations aren't really memory leaks.
     */
#if HAVE_VALGRIND
    if (RUNNING_ON_VALGRIND) {
        for (mapping = self->mappings; mapping != NULL; mapping = mapping->next) {
            chunk_t * block = mapping->fence;

            while (chunk_words(block)!= 0) {
                if (!chunk_is_slab(block) && chunk_marked(block)) {
                    valgrind_free(chunk_to_mem(block));
                } else if (chunk_is_slab(block))
                    valgrind_prepare_slab((slab_t *)block);

                block = next_chunk(block);
            }

        }
    }
#endif
    /* We must mark slab pool structures.  They get reclaimed if we don't.
     */
    for (mapping = self->mappings; mapping != NULL; mapping = mapping->next) {
        chunk_t * block = mapping->fence;

        while (chunk_words(block)!= 0) {
            if (chunk_is_slab(block)) {
                mark_slice(ptr_to_slab(slab_cast(block)->pool),
                            slab_cast(block)->pool);
            }

            block = next_chunk(block);
        }

    }
    VALGRIND_DO_LEAK_CHECK;  /* this makes valgrind spit out leaks */
    /* This is anrmalloc's leak check.
     */
    for (mapping = self->mappings; mapping != NULL; mapping = mapping->next) {
        chunk_t * block = mapping->fence;

        while (chunk_words(block)!=0) {
            if (chunk_in_use(block) && !chunk_is_slab(block)) {
                if (chunk_marked(block)) {
                    clear_mark(block);
                    valgrind_malloc(chunk_to_mem(block), 
                                    user_bytes(self, block));
                    VALGRIND_MAKE_MEM_DEFINED(chunk_to_mem(block), user_bytes(self,block));
                } else {
                    freechunk_t * node  = chunk_to_mem(block);
                    debug ("Leaking Object: %p size: %d\n", 
                                    chunk_to_mem(block), chunk_bytes(block));
                    if (func)
                        func (fileno (stdout), node, user_bytes(self,block));
                    reclaimed+= chunk_bytes(block);
                    node->next = list;
                    list = node;
                }
            } else if (chunk_is_slab(block) && !chunk_in_use(block)) {
                reclaimed += slab_reclaim(self, (slab_t *)block, func);
            }           
            block = next_chunk(block);
        }

    }

    valgrind_make_internals_noaccess(self);
    if (list) {
        while (list) {
            freechunk_t * node = list->next;
            add_free_event(self, FREE_EVENT(list), 0, __builtin_return_address(0));
            if (RARELY(fill_with_trash(self)))
                write_trash(self, list);
            /* deliberate non check for a valid mapping -- reclaim populated
             * this list, so if we get a wild pointer, very bad things are
             * happening
             */
            internal_deallocate(self, list);        
            list = node;
        }
    }

    UNLOCK(&self->heap_lock);

    if (reclaimed)
        printf ("*** Reclaimed %d bytes\n", reclaimed);


    return reclaimed;
}
bool
_anr_core_get_error(malloc_state_t * self)
{
    int error;
    LOCK(&self->heap_lock);
    error = error_state(self);
    UNLOCK(&self->heap_lock);

    return (error != 0);

}

void
_anr_core_set_error(malloc_state_t * self, bool error)
{
    LOCK(&self->heap_lock);
    if (error)
        set_malloc_error(self);
    else 
        clear_malloc_error(self);
    UNLOCK(&self->heap_lock);
}


static inline void dump_block(chunk_t * block);
void 
_anr_core_pointer_info(malloc_state_t * self,  void * ptr)
{
    chunk_t * block = mem_to_chunk(ptr);

    LOCK(&self->heap_lock);

    if (! find_mapping(self, block))
       printf ("Nothing known about %p\n", ptr); 
    else {
        char * description;
        if (alloc_is_slice(self, ptr)) {
            if (slice_is_allocated(ptr)) {
                description = "allocated";
            } else
                description = "free";
            printf ("%p - %s slice\n", ptr, description);
            dump_block((chunk_t *)ptr_to_slab(ptr));
        } else {
            chunk_t * next = next_chunk (block);

            if ((!chunk_in_use(block)
                 || check_bounds(self))
                && next->prev_size != chunk_words(block))
                printf ("Crap.  Corrupt heap\n");

            if (!prev_chunk_in_use(block)) {
                next = prev_chunk(block);
                if (chunk_in_use(next) || chunk_words(next)!= block->prev_size)
                    printf ("Crap.  prev block - currupt heap\n");
            }

            printf ("Allocation: %p - %d bytes\n", ptr, chunk_bytes(block));

        }

    }

    UNLOCK(&self->heap_lock);
    return;
}
unsigned int
_anr_core_free_bytes(malloc_state_t * self)
{
    unsigned int bytes = 0;
    if (0 == TRYLOCK(&self->heap_lock)) {
        bytes = words_to_bytes(self->available_words);
        UNLOCK(&self->heap_lock);
    }
    return bytes;
}
unsigned int
_anr_core_total_bytes(malloc_state_t * self)
{
    unsigned int bytes = 0;
    if (0 == TRYLOCK(&self->heap_lock)) {
        bytes = words_to_bytes(self->total_words);
        UNLOCK(&self->heap_lock);
    }
    return bytes;
}
size_t
_anr_core_usable_size(malloc_state_t * self,  void * ptr) {
    size_t size;

    LOCK(&self->heap_lock);
    
    if (RARELY(NULL == find_mapping(self, ptr))) {
        UNLOCK (&self->heap_lock);
        return 0;
    }

    if (alloc_is_slice(self, ptr)) {
        valgrind_make_chunk_defined(chunk_cast(ptr_to_slab(ptr)));
        size = (size_t)(ptr_to_slab(ptr)->pool->slice_size);
        valgrind_make_chunk_noaccess(chunk_cast(ptr_to_slab(ptr)));
    }else{
        valgrind_make_chunk_defined(mem_to_chunk(ptr));
        size = (user_bytes(self, mem_to_chunk(ptr)));
        valgrind_make_chunk_noaccess(mem_to_chunk(ptr));
    }
    UNLOCK(&self->heap_lock);
    return size;
}
unsigned int
_anr_core_event_count(malloc_state_t * self)
{
    unsigned int count;
    LOCK(&self->heap_lock);
    count = self->malloccount;
    UNLOCK(&self->heap_lock);

    return count;
}

void
_anr_core_prefork_prepare(malloc_state_t * self)
{
    LOCK(&self->heap_lock);
    LOCK(&self->freelist_lock);
}
int
_anr_core_postfork(malloc_state_t * self)
{
    int ret;
    ret = UNLOCK(&self->freelist_lock);

    if (ret)return ret;
    
    return UNLOCK(&self->heap_lock);
}
void
_anr_core_postfork_child(malloc_state_t * self)
{
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE_NP);
    pthread_mutex_init (&self->heap_lock, &attr);
    pthread_mutex_init (&self->freelist_lock, &attr);
    pthread_mutexattr_destroy(&attr);
    pthread_cond_init(&self->list_populated, NULL);

    _abort = (AbortFunction)local_abort;

}
void
_anr_core_lock_heap(malloc_state_t * self)
{
    LOCK(&self->heap_lock);
}
int
_anr_core_unlock_heap(malloc_state_t * self)
{
    return UNLOCK(&self->heap_lock);
}
bool 
_anr_core_trylock_heap(malloc_state_t * self)
{
    return 0 == TRYLOCK(&self->heap_lock);
}
bool
_anr_core_valid_pointer(malloc_state_t * self, void * ptr)
{
    bool ret;
    LOCK(&self->heap_lock);
    ret = (find_mapping(self, ptr)!= NULL);
    UNLOCK(&self->heap_lock);
    return ret;
}
unsigned int
_anr_core_expand(malloc_state_t * self, unsigned int pages)
{
    mapping_t * mapping;
    unsigned int total_possible_size = 0;
    LOCK(&self->heap_lock);
    for (mapping = self->mappings; mapping; mapping = mapping->next) {
        total_possible_size += mapping->total_words;
    }
    total_possible_size = words_to_bytes(total_possible_size) /PAGESIZE_BYTES;
    total_possible_size -= self->total_pages;
    pages = min(pages, total_possible_size);
    self->total_pages += pages;
    self->available_pages += pages;
    self->available_words += bytes_to_words(pages * PAGESIZE_BYTES);
    self->total_words += bytes_to_words(pages * PAGESIZE_BYTES);
    UNLOCK(&self->heap_lock);
    return pages;
}
unsigned int
_anr_core_shrink(malloc_state_t * self, unsigned int pages)
{
    LOCK(&self->heap_lock);
    LOCK(&self->freelist_lock);
    unlocked_flush_freelist(self);
    UNLOCK(&self->freelist_lock);

    pages = min(pages, (unsigned int)self->available_pages);
    self->total_pages -= pages;
    self->available_pages -= pages;
    self->available_words -= bytes_to_words(pages * PAGESIZE_BYTES);
    self->total_words -= bytes_to_words(pages * PAGESIZE_BYTES);

    if (self->available_pages < self->reclaimable_pages)
        clean_pages(self, self->reclaimable_pages);

    UNLOCK(&self->heap_lock);

    return pages;
}
void
_anr_core_set_size(malloc_state_t *self, int pages)
{
    LOCK(&self->heap_lock);
    LOCK(&self->freelist_lock);
    unlocked_flush_freelist(self);
    UNLOCK(&self->freelist_lock);

    self->available_pages = pages - (self->total_pages - self->available_pages);
    self->total_pages = pages;
    self->available_words = bytes_to_words(pages * PAGESIZE_BYTES)- (self->total_words - self->available_words);
    self->total_words = bytes_to_words(pages * PAGESIZE_BYTES);

    if (self->available_pages < self->reclaimable_pages)
        clean_pages(self, self->reclaimable_pages);

    UNLOCK(&self->heap_lock);
}


int
_anr_core_available_pages(malloc_state_t * self)
{
    int pages;
    bool locked = (0 == TRYLOCK(&self->heap_lock));

    if (locked) {
        pages = self->available_pages;
        UNLOCK(&self->heap_lock);
    } else 
        pages = -1;

    return pages;
}

void
_anr_core_free(malloc_state_t * self, void * ptr, void * func)
{
    if (RARELY(ptr == NULL))return;


    if (EBUSY == TRYLOCK(&self->heap_lock)) {
        freechunk_t * node = (freechunk_t *)ptr;
        node->next = NULL;
        LOCK(&self->freelist_lock);
        node->next = self->free_list;
        self->free_list = node;
        SIGNAL(&self->list_populated);
        UNLOCK(&self->freelist_lock);
    } else {
        if (RARELY (NULL == find_mapping(self, ptr)))
            crash ("free of wild pointer %p", ptr);

        add_free_event (self, FREE_EVENT(ptr), 0, func);
        internal_deallocate (self, ptr);
        
        /* So, we're cheating here.  The freelist may be empty, and 
         * we don't really want to have the overhead of grabbing the
         * lock on an empty list
         */
        if (self->free_list) {
            LOCK(&self->freelist_lock);
            unlocked_flush_freelist(self);
            UNLOCK(&self->freelist_lock);
        }
        UNLOCK(&self->heap_lock);
    }
}
/**
 * Allocate a chunk of memory
 * @param self the malloc state
 * @param size the requested size in bytes.
 * @return address of a block large enough to write size bytes into.
 */
void *
_anr_core_allocate(malloc_state_t * self, size_t size, void * func)
{
    size_t allocation_size = max(pad_size(self,size), MIN_BLOCK_SIZE);
    size_t allocation_words = bytes_to_words (allocation_size);
    chunk_t * block; 
    unsigned int attempt = 0;
    void * ret = NULL;

    LOCK(&self->heap_lock);

    if (RARELY (verify (self)))
        check_chunks (self);

    if (RARELY(error_state(self)))
        goto POSTACTION;

    valgrind_make_internals_defined(self);

    self->malloccount++;

    if (RARELY(self->malloccount == self->malloccompare))
        breakonmalloc();

    block = find_chunk (self, allocation_words); 

    while (!block) {
        if (USUALLY (self->_more_memory (self->cb_context, 
                                         attempt++,
                                         size)!= 0)) {
            LOCK (&self->freelist_lock);
            unlocked_flush_freelist (self);
            UNLOCK (&self->freelist_lock);
            block = find_chunk (self, allocation_words);
        } else if (attempt > MORE_MEM_RETRIES) {
            break;
        }
    }

    if (RARELY (!block)) { goto POSTACTION;}

    debug_assert (!chunk_in_use (block) && find_mapping (self, block));

    if (!chunk_is_slab (block)) {
        set_in_use (block);
        self->available_words -= chunk_words (block);
        ret = chunk_to_mem (block);
        allocate_pages (self, block);
    } else {
        ret = allocate_slice (self, slab_cast (block));
    } 

    valgrind_malloc (ret, size);
    memory_debug_assert (address_ok (self, ret));
    add_allocation_event (self, ALLOC_EVENT (ret), size, func);

POSTACTION:
    memory_debug_assert (0 == internal_verify (self));
    valgrind_make_internals_noaccess (self);
    UNLOCK (&self->heap_lock);
    return ret;
}

#define simple_join(top, bottom, state)\
    do{\
        top->head = (state->head & INTERNAL_BITS) | \
        (chunk_words(top) + chunk_words(bottom));\
        valgrind_make_chunk_noaccess(bottom);\
        valgrind_make_chunk_defined(top);\
    }while (0)
/**
 * perform a grow or shrink of a block
 * @param self the malloc state.
 * @param ptr the address of the block to modify.
 * @param size the desired new size for the block
 * @return The address of a block of the requested size.
 * 
 * Realloc is difficult.  
 * We try our hardest to avoid copying data by default.
 * ALTERNATE POLICY: 
 * when doing a giveback, we always copy.
 *
 * If we're shrinking, we carve off the bottom of the block and insert it into
 * internal structures.
 *
 * If we're growing, we look at the blocks above and below, and try to join
 * with them if possible.  If not, we allocate a new block and copy.
 *
 * If we're growing into the block above, well, we have to copy then as well.
 */
void * 
_anr_core_realloc(malloc_state_t * self, 
                  void * ptr, size_t size,
                  void * func)
{
    chunk_t * block; 
    chunk_t * below;
    chunk_t * above;
    void * ret_ptr = NULL;
    uint32_t alloc_words;  

    size_t realloc_bytes = max(MIN_BLOCK_SIZE, pad_size(self, size));
    uint32_t realloc_words = bytes_to_words(realloc_bytes);


#ifdef HAVE_VALGRIND
    bitmap_t * vbits;
#endif

    if (RARELY(misaligned_chunk(ptr)))
        crash ("misalignemd pointer to realloc :%p", ptr);

    LOCK(&self->heap_lock);

    if (RARELY (find_mapping(self, mem_to_chunk(ptr)) == NULL))
        crash ("realloc of wild pointer %p", ptr);

    memory_debug_assert(address_ok(self, ptr));
    self->malloccount++;

    if (RARELY(self->malloccount == self->malloccompare))
        breakonmalloc();
    
    valgrind_make_internals_defined(self);
#ifdef HAVE_VALGRIND
    vbits = valgrind_get_vbits(ptr);
#endif
    /* this is annoying, but necessary.  vagrind client API has no way to
     * shrink a block.  We have to free it first.  The act of freeing a buffer
     * makes the whole thing undefined.
     */
    valgrind_free(ptr);

    if (RARELY (verify (self)))
         check_chunks (self);

    if (alloc_is_slice(self, ptr)) {
        if (RARELY(!slice_is_allocated(ptr)))
            crash("realloc of free pointer %p", ptr);
        if (size <= slice_to_slab(ptr)->pool->slice_size - self->overhead + BYTES_PER_WORD) {
            ret_ptr = ptr;
            alloc_words = bytes_to_words(slice_to_slab(ptr)->pool->slice_size);
            goto POSTACTION;
        } else {
            ret_ptr = _anr_core_allocate(self, size, func);
            VALGRIND_MAKE_MEM_DEFINED(ret_ptr, size);
            /* although the slice is allocated, we don't know what the client 
             * request size is, so we make the whole slice defined */
            VALGRIND_MAKE_MEM_DEFINED(ptr, slice_to_slab(ptr)->pool->slice_size);
            memcpy (ret_ptr, ptr, min(size, slice_to_slab(ptr)->pool->slice_size));
            VALGRIND_MAKE_MEM_NOACCESS(ret_ptr, size);

            if (RARELY(fill_with_trash(self)))
                write_trash (self, ptr);

            alloc_words = bytes_to_words(slice_to_slab(ptr)->pool->slice_size); 
            deallocate_slice(self, ptr_to_slab(ptr), ptr);
            goto POSTACTION;
        }
    }

    block = mem_to_chunk(ptr);

    if (RARELY(check_bounds(self)
               && chunk_words(block)!= prev_chunk_words(next_chunk(block)))) {
        crash("buffer overrun");
    }

    if (RARELY(!chunk_in_use(block))) {
        crash("realloc of free pointer %p", ptr);
    }

    alloc_words = chunk_words(block);

    if (RARELY(alloc_words == realloc_words)) {
        ret_ptr = ptr;
        goto POSTACTION;
    }

    /* can we even service this request? get more memory if we can't */
    if (realloc_words > alloc_words 
        && (realloc_words - alloc_words > self->available_words 
            || (realloc_words - alloc_words) > 
                bytes_to_words(PAGESIZE_BYTES * self->available_pages))) {
        int giveback_attempt = 0;

        do{
            if (RARELY (giveback_attempt > MORE_MEM_RETRIES 
                        || 0 == self->_more_memory (self->cb_context, 
                                                    giveback_attempt++,
                                               words_to_bytes(realloc_words)))) {
                goto FAILED;
            }    
            LOCK(&self->freelist_lock);
            unlocked_flush_freelist(self);
            UNLOCK(&self->freelist_lock);
        } while (realloc_words - alloc_words > self->available_words);
    }
    /* catch double frees */
    block->head &= ~IN_USE_BIT;

    /* alternate policy.  if the alloc is shrinking and destination is small,
     * we do a hard copy 
     *
     * This is referred to as a defragmenting realloc.
     */
    if (RARELY (alloc_words > realloc_words 
                && realloc_words < bytes_to_words (SMALLBLOCK_MAX))) {
        chunk_t * new = find_chunk(self, realloc_words);

        if (USUALLY(new!= NULL)) {
            if (!chunk_is_slab(new)) {
                copy_chunk (chunk_to_mem (new), chunk_to_mem (block),
                            user_words (self, new));
                ret_ptr= chunk_to_mem(new);
                set_in_use(new);
                allocate_pages(self, new);
                self->available_words -= chunk_words(new);
                if (RARELY(verify(self) || check_bounds(self)))
                    next_chunk(new)->prev_size = chunk_words(new);
                    
            }else{
                ret_ptr = allocate_slice(self, slab_cast(new));
                VALGRIND_MAKE_MEM_DEFINED(ret_ptr, size);
                VALGRIND_MAKE_MEM_DEFINED(ptr, size);
                mempcpy (ret_ptr, ptr, size);
            }
            /* this isn't a double free... really */
            block->head |= IN_USE_BIT;
            valgrind_malloc(ptr, words_to_bytes(alloc_words));
            if (RARELY(fill_with_trash(self)))
               write_trash(self, ptr);
            internal_deallocate(self, ptr);
            goto POSTACTION;
        }
    }

    /* several cases for realloc */

    /* if we're here, we're either shrinking by a good amount, or we're
     * growing.
     */
    /* ----------------- shrinking blocks ----------------------------*/

    if (alloc_words >= realloc_words) {
        ret_ptr = ptr;

        if (RARELY(fill_with_trash(self)
                   && chunk_words(mem_to_chunk(ptr)) > realloc_words 
                   + bytes_to_words(MIN_BLOCK_SIZE))) {
            below = new_block_from_offset(mem_to_chunk(ptr), realloc_bytes);

            VALGRIND_MAKE_MEM_DEFINED(((uintptr_t)below) + offsetof(chunk_t, head), 
                                      sizeof (uint32_t));

            below->head = PREV_IN_USE_BIT | (chunk_words(mem_to_chunk(ptr))- realloc_words);
            valgrind_make_chunk_defined(below);

            /* while annoying, the best way to fill the remainder with trash is to do
             * so here.  This will avoid filling remainder chunks that are 
             * otherwise unallocated with data.
             */

            write_trash (self, chunk_to_mem(below));

        }
    } else {
        /*--------------- Growing Blocks --------------------------------*/

        /* Case 2: block below is available, and the combined size is big enough to
         * service the allocation.
         * We want to do this next because then we don't have to move anything.
         */
        below = next_chunk(block);
        above = prev_chunk(block);

        if (! chunk_in_use(below)
            && ! chunk_is_slab(below)
            && (chunk_words (below) + alloc_words) >= realloc_words) {
            unlink_chunk(self, below);
            unhook_pages(self, below);
            self->available_words -= chunk_words(below);
            self->reclaimable_pages -= count_contained_dirty_pages(self, below);

            next_chunk(below)->prev_size = chunk_words(block) + chunk_words(below);
            simple_join(block, below, block);
            
            ret_ptr = ptr;
        }
        /* Case 3:
         * Block above is available, and the combined size is >= what we need.
         */ 
        else if (! prev_chunk_in_use (block)
                 && ! chunk_is_slab (above)
                 && (chunk_words (above) + alloc_words) >= realloc_words) {
            chunk_t * new_block = above;
            unlink_chunk(self, above); 
            unhook_pages(self, above);
            self->reclaimable_pages -= count_contained_dirty_pages(self, above);
            self->available_words -= chunk_words(above);
            simple_join(above, block, above);
            copy_chunk (chunk_to_mem (new_block), chunk_to_mem (block),
                        bytes_to_words (user_bytes (self, block)));
            ret_ptr = chunk_to_mem(new_block);
        }
        /* Case 4:
         * Have to combine both above and below blocks to get a large enough size
         */
        else if (! prev_chunk_in_use (block)
                 && ! (chunk_in_use (below))
                 && ! chunk_is_slab (below)
                 && ! chunk_is_slab (above)
                 && chunk_words (above) + chunk_words (below) + alloc_words >= realloc_words) {
            unlink_chunk(self,above);
            unhook_pages(self,above);
            unlink_chunk(self,below);
            unhook_pages(self,below);
            self->available_words -= chunk_words(below);
            self->available_words -= chunk_words(above);
            self->reclaimable_pages -= count_contained_dirty_pages(self, below);
            self->reclaimable_pages -= count_contained_dirty_pages(self, above);
            simple_join(above, block, above);

            simple_join(above, below, above);
            copy_chunk (chunk_to_mem (above), chunk_to_mem (block),
                        bytes_to_words (user_bytes (self, block)));

            next_chunk(below)->prev_size = chunk_words(above);

            ret_ptr = chunk_to_mem(above);

        } 
        /* Case 5:
         * Have to get a new block large enough
         */
        else {
            chunk_cast(mem_to_chunk(ptr))->head |= IN_USE_BIT;
            ret_ptr = _anr_core_allocate(self, size, func);

            if (ret_ptr == NULL) {
                goto FAILED;
            }
            
            copy_chunk (ret_ptr, ptr, bytes_to_words (user_bytes (self, block)));
            if (RARELY(fill_with_trash(self)))
                write_trash(self, ptr);
            /* so, we've freed the pointer above.  we need to re-alloc it for valgrind.
             */
            valgrind_malloc(ptr, bytes_to_words(alloc_words));
            internal_deallocate(self, ptr);
            goto POSTACTION;
        }
    }

    block = mem_to_chunk(ret_ptr);

    block->head |= IN_USE_BIT;

    if (chunk_words(block) > realloc_words + bytes_to_words(MIN_BLOCK_SIZE)) {
        chunk_t * next = next_chunk(block);
        uintptr_t offset;
        mapping_t * mapping = find_mapping(self, block);
        below = new_block_from_offset(block, realloc_bytes);

        VALGRIND_MAKE_MEM_DEFINED(((uintptr_t)below) + offsetof(chunk_t, head), 
                                  sizeof (uint32_t));

        below->head = PREV_IN_USE_BIT | (chunk_words(block)- realloc_words);

        if (RARELY(verify(self) || check_bounds(self)))
            below->prev_size = realloc_words;

        valgrind_make_chunk_defined(below);

        offset = (ptr_to_page(below)- (uintptr_t)mapping->top) /PAGESIZE_BYTES;
        mapping->dirtymap[index2word(offset)] |= 1 << index2bit(offset);
        offset = (ptr_to_page(((uintptr_t)below) + 
                 new_chunk_overhead(words_to_bytes(chunk_words(block)- realloc_words)))
                  - (uintptr_t)mapping->top) /PAGESIZE_BYTES;
        mapping->dirtymap[index2word(offset)] |= 1 << index2bit(offset);

        block->head = (block->head & INTERNAL_BITS) | realloc_words;
        if (!chunk_in_use(next) && !chunk_is_slab(next)) {
            unlink_chunk(self, next);
            unhook_pages(self, next);
            self->available_words -= chunk_words(next);
            self->reclaimable_pages -= count_contained_dirty_pages(self, next);
            simple_join(below, next, below);
            deinit_chunk(self, next, below);
        } 
        next_chunk(below)->prev_size = chunk_words(below);
        set_available(below);

        insert_chunk(self, below);
        hook_pages(self, below);
        self->reclaimable_pages += count_contained_dirty_pages(self, below);
        self->available_words += chunk_words(below);
    }     

    mark_pages_dirty(self, block);
    set_in_use(block);

POSTACTION:
    valgrind_malloc(ret_ptr, size);
    VALGRIND_SET_VBITS(ret_ptr, vbits, min(size,words_to_bytes(alloc_words)));
    valgrind_destroy_vbits(vbits);

    add_allocation_event(self, ptr, words_to_bytes(alloc_words), func);
    add_allocation_event(self, ret_ptr, size, func);

    memory_debug_assert(address_ok(self, ret_ptr));
FAILED:
    valgrind_make_internals_noaccess(self);
    memory_debug_assert(0 == internal_verify (self));
    UNLOCK(&self->heap_lock);
    return ret_ptr;
}
/**
 * Allocate a block of aligned memory
 * @param self the malloc state.
 * @param alignment the desired alignment.  Will be pushed to the next power of
 * 2 if it isn't one.
 * @param size The desired size of the chunk.
 * @return the address of a block that is size bytes that is aligned accordingly.
 */
inline void * 
_anr_core_memalign(malloc_state_t * self, size_t alignment, size_t bytes)
{
    void * ptr = NULL;
    chunk_t * al_block = NULL;
    uint32_t alloc_size;
    unsigned int attempt = 0;

    if (alignment < MIN_BLOCK_SIZE)
        alignment = MIN_BLOCK_SIZE;

    if ((alignment & (alignment -1))!=0) {
        alignment = pow2ceil(alignment);
    }
    
    if (RARELY(bytes >= (words_to_bytes(MAX_ALLOC)- alignment))) {
        crash("Allocation request too big");
    }

    LOCK(&self->heap_lock);

    self->malloccount++;
    if (self->malloccount == self->malloccompare)
        breakonmalloc();

    alloc_size = bytes + alignment + MIN_BLOCK_SIZE + sizeof (chunk_t);
    alloc_size = bytes_to_words(alloc_size);

    valgrind_make_internals_defined(self);

    ptr = find_chunk (self, alloc_size);

    while (!ptr) {
        if (USUALLY(self->_more_memory(self->cb_context, 
                                      attempt++,
                                      alloc_size)!= 0)) {
            LOCK(&self->freelist_lock);
            unlocked_flush_freelist(self);
            UNLOCK(&self->freelist_lock);
            ptr = find_chunk(self, alloc_size);
        } else if (attempt > MORE_MEM_RETRIES) {
            break;
        }
    }

    if (RARELY(ptr == NULL)) {
        UNLOCK(&self->heap_lock);
        return NULL;
    }


    al_block =  ptr;

    if ((((uintptr_t)chunk_to_mem(ptr)) & ~(alignment-1))!= (uintptr_t)chunk_to_mem(ptr)) {
        uintptr_t bottom;
        uintptr_t topsize;
        chunk_t * block =  ptr;
        void * temp = (void *)(((uintptr_t)chunk_to_mem(block) + alignment) & -alignment);

        temp = mem_to_chunk(temp);

        /* at this point, temp is 1 chunk_t above an aligned address -- that
         * is, temp should be the address of the chunk servicing the aligned
         * request.
         */

        /* before we hand it off, we make sure that we can carve something off of the top.
         */

        if (((intptr_t)temp)- ((intptr_t)ptr) > (intptr_t)MIN_BLOCK_SIZE)
            bottom = (uintptr_t)temp; 
        else
            bottom = ((uintptr_t)temp) +alignment;

        topsize = bottom - ((uintptr_t)block);


        al_block = split_chunk_top(self, block , bytes_to_words(topsize));
        valgrind_make_chunk_defined(block);
        set_available(block);

        if (!prev_chunk_in_use(block)) {
            unlink_chunk(self, prev_chunk(block));
            unhook_pages(self, prev_chunk(block));
            unhook_pages(self, block);
            block = join_chunk(self, prev_chunk(block), block);
            hook_pages(self, block);
        }
        insert_chunk(self, block);
    }

    if (chunk_bytes(al_block) >= pad_size(self, bytes) + MIN_BLOCK_SIZE) {
        chunk_t * remainder;

        if (!next_chunk_in_use(al_block) 
            && !chunk_is_slab(next_chunk(al_block))) {
            unlink_chunk(self, next_chunk(al_block));
            unhook_pages(self, next_chunk(al_block));
            unhook_pages(self, al_block);
            al_block = join_chunk(self, al_block, next_chunk(al_block));
            hook_pages(self, al_block);
        }

        remainder = split_chunk_top(self,al_block, 
                                    bytes_to_words(pad_size(self, bytes)));

        set_available(remainder);
        set_prev_in_use(remainder);
        insert_chunk(self, remainder);
    }

    debug_assert((((uintptr_t)chunk_to_mem(al_block)) & (alignment-1)) ==0); 
    debug_assert(chunk_bytes(al_block)- self->overhead >= bytes);
    set_in_use(al_block);
    allocate_pages(self, al_block);
    self->available_words -= chunk_words(al_block);
    add_allocation_event(self, ALLOC_EVENT(chunk_to_mem(al_block)), bytes, 0);

    valgrind_make_internals_noaccess(self);

    valgrind_malloc(chunk_to_mem(al_block), 
                    _anr_core_usable_size(self, 
                                          chunk_to_mem(al_block)));
    UNLOCK(&self->heap_lock);

    return chunk_to_mem(al_block);
}




/********************************************************************
 * Internal State Verification
 ********************************************************************/
static inline int
check_smallbin_list(malloc_state_t * self,  binchunk_t * bin)
{
    binchunk_t * forward = bin->link.next;
    binchunk_t * back = bin->link.prev;
    binchunk_t * above;
    binchunk_t * below;

    while (forward != bin) {

        if (forward == NULL || back == NULL) {
            printf ("%s corrupt heap - NULL bin links\n");
            return 1;
        }

        if (forward && ! find_mapping (self, forward)) {
            printf ("%s: corrupt heap - forward address not mapped\n", __func__);
            return 1;
        }
        if (back && ! find_mapping(self, back)) {
            printf ("%s: corrupt heap - back address not mapped\n", __func__);
            return 1;
        }

        if ((forward && !back) || (back && !forward)) {
            printf (" forward and back links unhappy (%p, %p)\n", 
                   forward, back);
            return 1;
        }

        if (!chunk_is_slab(forward)) {
            above = bin_cast(prev_chunk(forward));
            below = bin_cast(next_chunk(forward));

            if ((!prev_chunk_in_use(forward) || check_bounds(self))
                && chunk_words(above)!= forward->prev_size) {
                printf ("%s: corrupt heap - tag word mismatch above (%d (%x), %d (%x))\n", 
                       __func__, chunk_words(above),chunk_words(above), forward->prev_size,
                       forward->prev_size);
                return 1;
            }

            if ((!chunk_in_use(forward) || check_bounds(self))
                && chunk_words(forward)!= below->prev_size) {
                printf ("%s: corrupt heap - tag word below (%d (%x), %d (%x))\n", 
                       __func__, chunk_words(below), chunk_words(below),
                       below->prev_size, below->prev_size);
                return 1;
            }

            if (smallbin_at (self, SMALL_INDEX (chunk_words (forward)))!= bin) {
                printf ("buffer over run detected\n");
                return 1;
            }
        }
        forward = forward->link.next;
        back = back->link.prev;
    }

    if (back != forward) {
        printf (" list forward links != list back links\n");
        return 1;
    }

    return 0;

}
static inline int
check_smallbins(malloc_state_t * self)
{
    int i;

    for (i =0 ; i< SMALLBINS; i++) {
       if (smallbin_populated(self, i)) {
          if (check_smallbin_list(self, &self->smallbins[i])) {
              printf ("smallbin hosed\n");
              return 1;
          }

          if (MALLOC_DEBUG 
              && self->smallbins[i].link.next == self->smallbins[i].link.prev 
              && self->smallbins[i].link.next == &self->smallbins[i]) {
              printf ("map indicated populated bin -- bin %d is empty\n", i); 
              return 1;
          }

       } else if (MALLOC_DEBUG 
                  && self->smallbins[i].link.next == self->smallbins[i].link.prev 
                  && self->smallbins[i].link.next != &self->smallbins[i]) {
            printf ("map indicates non populated bin %d -- bin contains entries\n", i);
            return 1;
       }
    }
    return 0;

}

static inline void
dump_largeblock(treechunk_t * block)
{
    if (!block || chunk_in_use(block))return;
    printf ("block->parent = %p\n", block->parent);
    if (block->list.next != block)
        printf ("block->list = { %p, %p} \n", block->list.prev, block->list.next);
    printf ("block->link = { %p, %p} \n", block->link[0], block->link[1]);
    printf ("block->color = %s\n", block->color==RED?"red":(block->color==BLACK?"black":"broken!"));
    printf ("block->prev_size: %u\n", block->prev_size);

}

#define PRINT_MEMBER(ptr , member)\
    printf ("%s->%s = %u\n", #ptr, #member, ptr->member)


static inline void
dump_slabpool(slabpool_t * pool)
{
    PRINT_MEMBER(pool , n_available);
    PRINT_MEMBER(pool , head);
    PRINT_MEMBER(pool , slice_size);
    PRINT_MEMBER(pool , slab_size);
    PRINT_MEMBER(pool , n_slices_per_slab);
    PRINT_MEMBER(pool , n_slice_words);
    PRINT_MEMBER(pool , n_bitmap_words);
    PRINT_MEMBER(pool , magic);
    PRINT_MEMBER(pool , shift);

}

static inline void
dump_slab(slab_t * slab)
{
    int i;
    int j;
    bitmap_t * bitmap = slab_get_bitmap (slab);
    PRINT_MEMBER(slab , n_free);
    PRINT_MEMBER(slab , last_bitmap_word);

    for (i = 0; i< slab->pool->n_bitmap_words/2; i++) {
        printf ("allocmap[%d] = %08x\n", i, bitmap[i]);
    } 
    for (j = 0 ; i< slab->pool->n_bitmap_words; j++, i++) {
        printf (" markmap[%d] = %08x\n", j, bitmap[i]); 
    }

    dump_slabpool (slab->pool);
}


static inline void
dump_block(chunk_t * block)
{
    if (!block)return;
    printf ("block: %p\n", block);
    printf ("block->size: %u\n", chunk_words(block));
    printf ("block->state: ");
    if (chunk_is_slab(block) && chunk_in_use(block)) {
        printf (" slab pool, ");
    } else if (chunk_is_slab(block)) {
        printf (" slab, ");
    } else if (chunk_in_use(block)) {
        printf (" in use, ");
    } else {
        printf (" available, ");
    }
    if (prev_chunk_in_use(block)) {
        printf (" prev in use ");
    } else {
        printf ("prev available");
    }
    
    if (chunk_marked(block)) {
        printf (", marked ");
    }
    printf ("\n");

    if (!chunk_in_use(block) && chunk_words(block) < 
       bytes_to_words(SMALLBLOCK_MAX)) {
        printf ("link= {%p,%p}\n", 
               bin_cast(block)->link.next, 
               bin_cast(block)->link.prev);
    } else if (chunk_in_use(block) && chunk_is_slab(block)) {
        dump_slabpool((slabpool_t *)block);
    } else if (chunk_is_slab(block)) {
        dump_slab((slab_t *)block);
    } else {
        dump_largeblock ((treechunk_t *)block);
    }

}

void 
dump_smallbin(binchunk_t * bin)
{
    binchunk_t * iter = bin->link.next;

    while (iter && iter != bin) {
        dump_block(chunk_cast(iter));
        iter = iter->link.next;
    }


}

static inline int
check_treechunk_list(malloc_state_t * self, treechunk_t * block)
{
    treechunk_t * forward;
    treechunk_t * back;
    chunk_t * above;
    chunk_t * below;

    forward = block->list.next;
    back = block->list.prev;

    above = prev_chunk(block);
    below = next_chunk(block);

    if ((!prev_chunk_in_use(block) || check_bounds(self))
        && chunk_words(above)!= block->prev_size) {
        printf ("%s: tag word mismatch above (%d (%x), %d (%x))\n", 
               __func__, block->prev_size, block->prev_size, 
               chunk_words(above), chunk_words(above));
        printf ("above=>\n");
        dump_block(above);
        printf ("block=>\n");
        dump_block(chunk_cast(block));
        return 1;
    }

    if ((!chunk_in_use(block) || check_bounds(self))
        && chunk_words(block)!= below->prev_size) {
        printf ("%s: tag word mismatch below (%d (%x), %d (%x))\n", 
               __func__, chunk_words(block), chunk_words(below), 
               below->prev_size,below->prev_size);
        printf ("block=>\n");
        dump_block(chunk_cast(block));
        printf ("below=>\n");
        dump_block(below);
        return 1;
    }


    while (forward 
           && block != forward 
           && block != back) {
        above = prev_chunk(forward);
        below = next_chunk(forward);

        if ((!prev_chunk_in_use(forward) || check_bounds(self))
            && chunk_words(above)!= forward->prev_size) {
            printf ("%s: corrupt heap - tag word mismatch (%d, %d)\n", 
                   __func__, chunk_words(above), forward->prev_size);
            printf ("above=>\n");
            dump_block(above);
            printf ("forward=>\n");
            dump_block(chunk_cast(forward));
            return 1;
        }

        if ((!chunk_in_use(below) || check_bounds(self))
            && chunk_words(forward)!= below->prev_size) {
            printf ("%s: corrupt heap - tag word mismatch (%d, %d)\n", 
                   __func__, chunk_words(above), forward->prev_size);
            printf ("block=>\n");
            dump_block(chunk_cast(forward));
            printf ("below=>\n");
            dump_block(below);
            return 1;
        }

        forward = forward->list.next;
        back = back->list.prev;
    }

    if (back != forward) {
        printf (" list forward (%p)links != list back links (%p)\n", 
               forward, back);
        return 1;
    }

    return 0;

}

static inline void
print_tree (treechunk_t * root)
{
    if (!root)return;

    print_tree(root->link[0]);
    print_tree(root->link[1]);

    dump_block(chunk_cast(root));

}


static inline int
check_largebin_tree(malloc_state_t *self, treechunk_t * root)
{
    int left_state;
    int right_state;
    treechunk_t * left;
    treechunk_t * right;
    if (!root)return 1;


    if (root->head == 0xffffffff) {
        printf ("combined block in tree!!?\n");
        return -1;
    }


    if (root->parent 
        && root->parent->link[0] != root 
        && root->parent->link[1] != root) {
        printf ("wrong parent: root => ");
        dump_block((chunk_t *)root);
        printf ("root->parent=> ");
        dump_block((chunk_t *)root->parent);
        return -1;
    }



    if (root->color == NON_TREE_NODE) {
        printf ("non node in the tree!\n");
        return -1;
    }


    if (check_treechunk_list(self, root)) {
        printf ("list verification failed\n");
        return -1;
    }

    left = root->link[0];
    right = root->link[1];

    if ((left 
         && left->parent != root)
        || (right 
            && right->parent!= root)) {
        printf ("node not pointing at proper parent\n");
        printf ("root=>\n");
        dump_block((chunk_t *)root);
        printf ("left=>\n");
        dump_block((chunk_t *)left);
        printf ("right=>\n");
        dump_block((chunk_t *)right);
        return -1;
    }


    if (red_node(root)
         && (red_node (left)
              || red_node (right))) {
        printf ("red violation in treebin\n");
        printf ("root=>\n");
        dump_largeblock(root);
        printf ("left=>\n");
        dump_largeblock(left);
        printf ("right=>\n");
        dump_largeblock(right);
        return -1;
    }

    left_state = check_largebin_tree (self, left);

    if (left_state < 0) {
        printf ("left state foobared.  root=>");
        dump_block((chunk_t *)root);
        return -1;
    }


    right_state = check_largebin_tree (self, right); 

    if (right_state < 0) {
        printf ("right state foobared.  root=>");
        dump_block((chunk_t *)root);
        return -1;
    }

    if ((left 
          && chunk_words(left) >= chunk_words(root))
        || (right 
             && chunk_words(right) <= chunk_words(root))) {
        printf ("Order violation in treebin\n");
        return -1;
    }

    if (left_state != 0 
         && right_state != 0 
         && left_state != right_state) {
        printf ("black violation in treebin(%d vs %d)\n", 
               left_state, right_state);
        printf ("BEGIN TREE DUMP\n");
        print_tree(root);
        printf ("END TREE DUMP\n");
        crash("balance");
        return -1;
    }

    if (left_state != 0 
         && right_state != 0) {
        return red_node(root) ? left_state: left_state+1;
    }
    return 0;
}
static inline int
check_largebins(malloc_state_t * self)
{
    int i;
    for (i =0; i< LARGEBINS; i++) {
        if (largebin_populated(self, i)) {
            if (!self->largebins[i].root) {
                printf ("large mask claims tree, but tree root is NULL\n");
                return 1;
            }
            if (0 > check_largebin_tree(self, self->largebins[i].root)) {
                printf ("largebin tree @ %d hosed\n", i);
                return 1;
            }
        } else if (self->largebins[i].root != NULL) {
            printf ("large mask claims no tree, but treeroot is non null\n");
            return 1;
        }
    }
    return 0;
}
static inline int
check_dirtymap(malloc_state_t * self, bool print)
{
    mapping_t * mapping;
    uint32_t dirty_page_check = 0;
    uint32_t i;
    for (mapping = self->mappings; mapping != NULL; mapping= mapping->next) {
        for (i= 0; i< mapping->mapwords; i++) {
            dirty_page_check += population_count(mapping->dirtymap[i]);
            if (print 
                && mapping->dirtymap[i])
                printf ("%p -- %08x\n",
                       mapping->top + PAGESIZE_BYTES * i * BITS_PER_WORD,
                       mapping->dirtymap[i]);

        }

    }

    if (print)printf ("%d dirty pages\n", dirty_page_check);

    return 0;
}
static inline int
check_mappings (malloc_state_t * self)
{
    mapping_t * mapping;

    chunk_t * next;
    uint32_t in_use_word_check = 0;
    uint32_t dirty_page_check = 0;
    int32_t reclaimable_page_check = 0;

    for (mapping = self->mappings; mapping != NULL; mapping= mapping->next) {
        chunk_t * block = mapping->fence;

        while (chunk_words(block)!=0) {

            dirty_page_check += count_contained_dirty_pages(self, block);
            
            if (!chunk_in_use(block)
                && !chunk_is_slab(block))
                reclaimable_page_check += count_contained_dirty_pages(self,block);

            next = next_chunk(block);
            if (chunk_in_use(block)
                && !chunk_is_slab(block)) {
                in_use_word_check += chunk_words(block);
                if (!prev_chunk_in_use(next)) {
                    printf ("chunk (%p)in use, next block (%p)thinks free\n",
                           block, next);
                    return 1;
                }
            }                
            else if (chunk_is_slab(block) && !chunk_in_use(block)) {
                int i;
                int total = 0;
                slab_t * slab = (slab_t *)block;
                bitmap_t * bitmap = slab_get_bitmap(slab);
                in_use_word_check += bytes_to_words(sizeof (slab_t));
                if (slab->last_bitmap_word >= slab->pool->n_bitmap_words/2) {
                    printf ("slab bitmap hosed\n");
                    return 1;
                }
                for (i = 0; i< slab->pool->n_bitmap_words/2; i++) {
                   total += population_count(~(bitmap[i]));
                }
                if (slab->n_free != total) {
                    printf ("slab allocation bits (%d)!= n_free (%d)\n",
                           total, slab->n_free);
                    return 1;
                }
                in_use_word_check += slab->pool->n_slice_words * 
                    (slab->pool->n_slices_per_slab - slab->n_free);

                if (!prev_chunk_in_use(next)) {
                    printf ("next block thinks slab is available\n");
                    return 1;
                }

                if (slab->n_free > 0) {
                    if (!smallbin_populated(self,  SMALL_INDEX(slab->pool->n_slice_words + WOVERHEAD))) {   
                        printf ("lost slab!\n");
                        return 1;
                    }
                            
                }

            } else if (chunk_in_use(block) && chunk_is_slab(block)) {
                slabpool_t * pool = (slabpool_t *)block;

                if ((pool->head &~INTERNAL_BITS)!= pool->n_slice_words + WOVERHEAD) {
                    printf ("corrupt slab %p\n", pool);
                    return 1;
                }
            }

            if (find_mapping(self, block)!= mapping) {
                printf ("Corrupt heap -- couldn't find mapping for %p\n", block);
                printf ("mapping: %p -> %p\n", mapping->top, 
                       (void *)((uintptr_t)mapping->top) + 
                       words_to_bytes(mapping->size));
                return 1;
            } else if (!chunk_in_use(block)
                       && !chunk_is_slab(block)) {
                chunk_t * next = next_chunk(block);
                if (prev_chunk_in_use(next)
                    && chunk_words(next)!= 0) {
                    printf ("next thinks unallocated block is allocated\n");
                    dump_block(block);
                    dump_block(next);
                    return 1;
                }
                if (chunk_words(block) < bytes_to_words(SMALLBLOCK_MAX)) {
                    unsigned int index = SMALL_INDEX(chunk_words(block));
                    binchunk_t * bin = (binchunk_t *)smallbin_at(self, index);
                    binchunk_t * iter = bin;
                    
                    if (!smallbin_populated(self, index)) {
                        printf ("block %p claims bin %d, but it is marked empty\n", 
                               block, index);
                        _anr_core_pointer_info(self, chunk_to_mem(block));
                        return 1;
                    }
                    

                    while (iter 
                           && iter->link.next != bin 
                           && bin_cast(block)!= iter) {
                        iter = iter->link.next;
                    }
                    if (bin_cast(block)!= iter) {
                        printf ("Could not find block %p in smallbin %d\n", block, index);
                        return 1;
                    }
                } else {
                    unsigned int index = large_index(chunk_words(block));
                    struct lbin * bin = largebin_at(self, index);
                    treechunk_t * root = bin->root;
                    treechunk_t * query = (treechunk_t *)block;

                    if (!root) {
                        printf ("block %p claims tree at bin %d, but root is null\n", 
                               block, index);
                        return 1;
                    }

                    if (query != root && query->parent) {
                        while (query->parent) {
                            query = query->parent; 
                        }

                        if (root && query != root) {
                            printf ("Could not find block %p in tree at largebin %d\n", block, index);
                            return 1;
                        }
                    }
                    
                    if (!chunk_is_slab(block)
                        && count_contained_dirty_pages(self, block) > 1 
                        && contained_pages(block) >=1) {
                        treechunk_t * chunk = tree_cast(block);

                        if (chunk->dirty.next == chunk) {
                            printf ("investigate dirty list: %p\n", chunk);
                            return 1;
                        }else{
                            while (chunk->dirty.next != chunk 
                                   && chunk->dirty.next != &self->dirty) {
                                chunk = chunk->dirty.next;
                            }
                            if (chunk->dirty.next != &self->dirty) {
                                printf ("dirty chunk not in dirty list!\n");
                                return 1;
                            }
                        }
                    }
                }

            }
            block = next_chunk(block);
        }

        if (((uintptr_t)block) + 8 > 
            ((uintptr_t)mapping->top) + words_to_bytes(mapping->size)) {
            return 1;
        }

    }

    if (self->total_words - self->available_words != in_use_word_check) {
        printf ("available word check failed!  computed: %d tracked %d\n", 
               in_use_word_check, self->total_words - self->available_words);
        return 1;
    }

    if (self->reclaimable_pages != reclaimable_page_check) {
        printf ("reclaimable page check failed.  tracked:%d checked:%d\n",
               self->reclaimable_pages, reclaimable_page_check);
        return 1;
    }

    return 0;
}
int
check_dirty_list(malloc_state_t * self, bool print)
{
    treechunk_t * dirty = self->dirty.dirty.prev;
    int count = 0;

    while (dirty != &self->dirty && count <= self->total_pages) {
        if (print) { 
            printf ("%p - %d pages\n", 
                   dirty, count_contained_dirty_pages(self, chunk_cast(dirty)));
        }
        if (chunk_words(dirty) < bytes_to_words(SMALLBLOCK_MAX)) {
            printf ("small chunk in dirty list!\n");
            return 1;
        }
        if (dirty == dirty->dirty.prev || dirty == dirty->dirty.next) {
            printf ("dirty list linkage hosed\n");
            return 1;
        }

        count+= count_contained_dirty_pages(self, chunk_cast(dirty));
        dirty = dirty->dirty.prev;
    }

    if (dirty != &self->dirty) {
        printf ("dirty list hosed\n");
    }
        
    if (count < self->reclaimable_pages) {
        printf ("less pages in list than reclaimable\n");
        return 1;
    }
    return dirty != &self->dirty;
}


int 
_anr_core_check_chunks(malloc_state_t * self)
{
    int ret = 0;

    LOCK(&self->heap_lock);
    ret = check_chunks (self);
    UNLOCK(&self->heap_lock);

    if (ret)
        crash("buffer overrun detected");
    return ret;
}

static int
internal_verify (malloc_state_t * self)
{
    int status = 0;

    if (RUNNING_ON_VALGRIND) { 
        return 0;
    }

    if (self->available_words > self->total_words) {
        printf (" available > total!\n");
    }
    status+= self->available_words > self->total_words;
    if (self->available_pages  > self->total_pages) {
        printf (" bad available pages avail:%d total: %d\n",
               self->available_pages,  self->total_pages);
        status++;
    }
    if (!self->dirty.dirty.next && !self->dirty.dirty.prev) {
        printf ("dirty list hosed\n");
        status+= 1;
    }

    if (self->reclaimable_pages < 0 || self->available_pages < 0) {
        status++;
        printf ("bad page tracking: avail: %d  reclaimable:%d\n",
               self->available_pages, self->reclaimable_pages);
    }
    if (self->reclaimable_pages > self->available_pages) {
        status++;
        printf ("reclaimable pages %d avail: %d\n",
               self->reclaimable_pages, self->available_pages);
    }
    if (self->available_pages > (int)(words_to_bytes(self->available_words) /4096)) {
        status++;
        printf ("more pages than words?? %d vs %d\n", self->available_pages,
               (self->available_words/1024));
    }
    status += check_dirty_list (self, false);
    status += check_dirtymap (self, false);
    status += check_smallbins (self);
    status += check_largebins (self);
    status += check_mappings (self);

    if (verify(self)) {
        status += check_chunks (self);
    }
    if (status) {
        printf ("malloc count:%d\nfree count: %d\n", 
                self->malloccount, self->freecount);
    }

    return status;
}

int
_anr_core_verify(malloc_state_t * self)
{

    int status;
    LOCK(&self->heap_lock);
    status = internal_verify (self);
    UNLOCK(&self->heap_lock);

    return status;

}


int 
_anr_core_report(malloc_state_t * self, 
                       int fd, 
                       const char * params)
{

    uint32_t available_pages;
    uint32_t total_pages;
    uint32_t reclaimable_pages;
    uint32_t available_words;
    uint32_t total_words;
    uint32_t largest_free_block =0 ;
    uint32_t m_count;
    uint32_t f_count;
    uint32_t dirty_pages = 0;
    uint32_t flags;
    int i;
    bitmap_t smallmap[NEAREST_N_BITS(SMALLBINS)];
    bitmap_t largemap[NEAREST_N_BITS(LARGEBINS)];
    bool locked;
    bool report;
    mapping_t * mapping;

    report = (bool)(params && strstr("events", params)!= 0);

    locked = (0 == TRYLOCK(&self->heap_lock));

    available_pages = self->available_pages;
    total_pages = self->total_pages;
    reclaimable_pages = self->reclaimable_pages;

    available_words = self->available_words;
    total_words = self->total_words;
    dirty_pages = self->dirty_page_ceiling;
    flags = self->flags;

    for (i = 0; i< (int)(NEAREST_N_BITS(SMALLBINS)); i++)
        smallmap[i] = self->smallmap[i];

    for (i = 0; i< (int)(NEAREST_N_BITS(LARGEBINS)); i++)
        largemap[i] = self->largemap[i];

    for (i = LARGEBINS -1; i>= 0; i--) {
        if (largebin_populated(self , i)) {
            largest_free_block = self->largebins[i].max;
            break;
        }
    }
    if (largest_free_block == 0)
        for (i = SMALLBINS -1; i>=0; i--) {
            if (smallbin_populated(self, i)) {
                largest_free_block = i + bytes_to_words(MIN_BLOCK_SIZE);
                break;
            }
        }
    m_count = self->malloccount;
    f_count = self->freecount;

    if (locked && report) {
        event_t * event = self->history_head;
        self->history_tail->next = NULL;
        malloc_printf (fd, "Last State Modifying Events\n");
        while (event && event->count != 0) {
            malloc_printf (fd, "%s return address:(%p)pointer=%p size=%d count=%d\n", 
                           IS_ALLOC_EVENT(event->address) ?"alloc":IS_FREE_EVENT(event->address) ?"free":"realloc",
                           event->return_function, 
                           ADDRESS(event->address), 
                           event->size, event->count);
            event = event->next;
        }
        UNLOCK (&self->heap_lock);
    } else if (locked)
        UNLOCK(&self->heap_lock);

    malloc_printf (fd, "\n========================================");
    malloc_printf (fd, "========================================\n");
    malloc_printf (fd, "Heap Information\n");
    malloc_printf (fd, "State Mapped at %p\n", self);
    malloc_printf (fd, "Allocation alignment: 0x%08x\nAllocation Word Size: %d bytes\n",
                   BYTES_PER_WORD, BYTES_PER_WORD);
    malloc_printf (fd, "Verify %s\n", (flags & VERIFY_BIT) == VERIFY_BIT? "on":"off");
    malloc_printf (fd, "Bounds Check %s\n", (flags & BOUNDS_CHECK_BIT) == BOUNDS_CHECK_BIT? "on":"off");
    malloc_printf (fd, "Fill With Trash %s\n", (flags & TRASH_BIT) == TRASH_BIT? "on":"off");
    malloc_printf (fd, "Error: %s\n", (flags & MALLOC_ERROR_BIT) == MALLOC_ERROR_BIT? "on":"off");
    malloc_printf (fd, "Available Words: %d (%.fM)\n", 
                   available_words, 
                   ((double)words_to_bytes(available_words)) / (1024.0 * 1024.0));
    malloc_printf (fd, "Total Memory: %d bytes (%.fM)\n",
                   words_to_bytes(total_words), 
                   (words_to_bytes(total_words)) / (double)(1024*1024));
    malloc_printf (fd, "Largest Contiguous Address Range: %d (bytes)%.2f (MB)\n", 
                   (words_to_bytes(largest_free_block)), 
                   ((float)words_to_bytes(largest_free_block)) /1024.0f/1024.0f);

    malloc_printf (fd, "Smallbin Mapwords:\n");

    for (i = 0; i< (int)(NEAREST_N_BITS(SMALLBINS)); i++)
        malloc_printf (fd, "smallmap[ %d ] = 0x%08x\n", i, smallmap[i]);

    malloc_printf (fd, "Largebin Mapwords: \n");

    for (i =0; i< (int)(NEAREST_N_BITS(LARGEBINS)); i++)
        malloc_printf (fd, "largemap[ %d ] = 0x%08x\n", i, largemap[i]);

    malloc_printf (fd, "Available Pages: %d  Reclaimable Pages: %d ",
                   available_pages, reclaimable_pages); 
    malloc_printf (fd, "Dirty Page Ceiling: %d\n", dirty_pages);
    malloc_printf (fd, "Total Pages: %d\n", total_pages);
    malloc_printf (fd, "%d mallocs, %d frees\n", m_count, f_count);
    for (mapping = self->mappings; mapping; mapping = mapping->next) {
        malloc_printf (fd, "Mapping: %p - %p (%.2fMB)\n", 
                       mapping->top, 
                       (void *)(((uintptr_t)mapping->top) + 
                                words_to_bytes(mapping->size)), 
                       words_to_bytes(mapping->size) /(1024.0 * 1024.0));
        malloc_printf (fd, "Usable: %p - %p (%.2fMB)\n", mapping->fence, 
                       (void *)(((uintptr_t)mapping->fence) + words_to_bytes(mapping->total_words)),
                       words_to_bytes(mapping->total_words) /(1024.0 * 1024.0));
    }
    malloc_printf (fd, "System RSS (pages): %d\n", check_process_dirty_pages(self->mappings));
    
    malloc_printf (fd, "========================================");
    malloc_printf (fd, "========================================\n");
    return 0;
}
void
_anr_core_dump(malloc_state_t * self, int fd, ReportFunction func)
{
    mapping_t * mapping;

    bool locked = (0 == TRYLOCK(&self->heap_lock));

    if (!locked) {
        malloc_printf (fd, "Heap currently locked\n");
        return;
    }

    valgrind_make_internals_defined(self);

    for (mapping = self->mappings; mapping != NULL; mapping = mapping->next) {
        chunk_t * block = mapping->fence;

        while (chunk_words(block)!=0) {
            if (chunk_is_slab(block))
                mark_slice (ptr_to_slab(slab_cast(block)->pool),
                            slab_cast(block)->pool);

            if (func && !chunk_is_slab(block) && chunk_in_use(block))
                func(fd, chunk_to_mem(block), chunk_bytes(block));
            else if (func && chunk_is_slab(block))
                slab_dump (slab_cast(block), func, fd);

            block = next_chunk(block);
        }
    }

    valgrind_make_internals_noaccess(self);
    UNLOCK(&self->heap_lock);

}


/********************************************************************
 * Unit Testing (in file no less!)
 *******************************************************************/

#ifdef BUILD_TESTS

#define UNIT_TEST(name)\
    void __attribute__ ((section (".unit_tests")))\
     __unit_test_ ## name (void)

#define UNIT_TEST_HELPER __attribute__((section (".unit_tests")))
    
#define UNIT_TEST_HEADER printf ("BEGIN - %s\n", __func__)
#define UNIT_TEST_FOOTER printf ("END - %s\n", __func__)

static malloc_state_t * ut_state; /* state for unit tests */

#define ut_malloc(size)\
    _anr_core_allocate(ut_state, (size), __builtin_return_address(0))

#define ut_free(ptr)\
    _anr_core_free(ut_state, (ptr), __builtin_return_address(0))


static inline void * 
ut_realloc(void * ptr, size_t size) {

    if (ptr == NULL && size > 0)
        return ut_malloc(size);
    else if (ptr && size == 0) {
        ut_free(ptr);
        return NULL;
    } else if (ptr) {
       return  _anr_core_realloc(ut_state, ptr, size, __builtin_return_address(0));
    }

    return NULL;

}


UNIT_TEST(malloc_simple)
{
    void * ptr;
    chunk_t * block;
    uint32_t word;
    unsigned int * alias;
    UNIT_TEST_HEADER;
    _anr_core_default_init(&ut_state);
    ptr = ut_malloc(40);

    block = mem_to_chunk(ptr);

    assert(chunk_in_use(block));
    assert(chunk_bytes(block) <= 40 + ut_state->overhead + MIN_BLOCK_SIZE);

    ut_free(ptr);

    ptr = ut_realloc(0, 40);

    block = mem_to_chunk(ptr);

    assert(chunk_in_use(block));
    assert(chunk_bytes(block) <= 40 + ut_state->overhead + MIN_BLOCK_SIZE);

    ptr = ut_realloc(ptr, 0);
    assert(!chunk_in_use(block));
    assert(!ptr);

    alias =ut_malloc(100 * sizeof (int));
    memset(alias, 0, 100 * sizeof (int));

    block = mem_to_chunk(alias);


    assert(chunk_in_use(block));
    assert(chunk_bytes(block) <= 100 * sizeof (int) + ut_state->overhead + MIN_BLOCK_SIZE);

    for (word =0; word < 100; word++) {
        assert(alias[word] == 0);
    }

    ut_free(alias);

    _anr_core_teardown(ut_state);
    UNIT_TEST_FOOTER;
}



UNIT_TEST(chunk_split_and_join)
{
    void * ptrs[31];
    int i;
    chunk_t * block;

    UNIT_TEST_HEADER;

    _anr_core_default_init(&ut_state);

    for (i =0; i < 30; i++) {
        ptrs[i] = ut_malloc(128);
    }
    for (i =0; i< 30; i++) {
        block = mem_to_chunk(ptrs[i]);
        /* valgrind pukes about this.  it would be easy to fix, but it is
         * better to show the warning.
         */
        assert(chunk_in_use(block));
        assert(chunk_bytes(block) == pad_size(ut_state, 128));
    }

    for (i =0; i< 30; i+=2) {
        ut_free(ptrs[i]);
        block = mem_to_chunk(ptrs[i]);
        assert(!chunk_in_use(block) || block->head == 0xFFFFFFFF);
        assert(chunk_bytes(block) == pad_size(ut_state, 128) || block->head == 0xFFFFFFFF);
    }

    for (i = 1; i < 30 ; i+=2) {
        ut_free(ptrs[i]);
        block = mem_to_chunk(ptrs[0]);

        assert(!chunk_in_use(block) || block->head == 0xFFFFFFFF);
        assert(block->head == 0xFFFFFFFF || chunk_bytes(block) >= 128U * ((unsigned int)(i+1)));

    }


    _anr_core_teardown(ut_state);
    UNIT_TEST_FOOTER;
}
UNIT_TEST(mark_and_reclaim)
{
    void * ptr;

    int reclaimed = 0;
    int i;

    UNIT_TEST_HEADER;

    _anr_core_default_init(&ut_state);
    for (i =0; i< 10; i++) {
        ptr = ut_malloc(50);
    }
    
    _anr_core_mark(ut_state, ptr);

    printf ("Leaks are deliberate.\n");

    reclaimed = _anr_core_reclaim(ut_state, NULL);
    assert(50 * 9 < reclaimed);

    reclaimed = _anr_core_reclaim(ut_state, NULL);

    assert(50 < reclaimed);
    

    _anr_core_teardown(ut_state);
    UNIT_TEST_FOOTER;
}

struct gb_context{
    freechunk_t * giveback_list;
    int giveback_counter;
};


static unsigned int UNIT_TEST_HELPER 
giveback(void * ctx, unsigned int attempt, unsigned int size)
{
    unsigned int returned_size = 0;
    freechunk_t * node;
    struct gb_context * context = (struct gb_context *)ctx;

    node = context->giveback_list;

    context->giveback_counter++;

    if (context->giveback_counter < 10) {
        while (node && returned_size < size) {
            context->giveback_list = node->next;
            returned_size+= chunk_bytes(mem_to_chunk(node));
            ut_free(node);
            node = context->giveback_list;
        }
    } else if (attempt == MORE_MEM_RETRIES) {
        printf ("About to hit OOM\n");
        _anr_core_set_error(ut_state, true);
    }

    return returned_size;

}

UNIT_TEST(giveback)
{
    unsigned int pool_size = 10 * 1024 * 1024;
    unsigned int allocated_bytes =0;

    struct gb_context context;
    freechunk_t * node;

    context.giveback_list = 0;
    context.giveback_counter = 0;

    UNIT_TEST_HEADER;
    _anr_core_init(&ut_state, 
                    0, 
                    pool_size, 
                    pool_size * 2, 
                    1024, 
                    0, 
                    NULL,
                 (MoreMemoryFunction)giveback, 
                 &context,
                 (AbortFunction)local_abort);

    while (_anr_core_get_error(ut_state) == false) {
        node = ut_malloc(1000);
        if (node) {
            node->next = context.giveback_list;
            context.giveback_list = node;
            allocated_bytes += 1000;
        }
    }

    printf ("bytes allocated: %d\tbytes available in pool: %d\n", 
           allocated_bytes, 
           words_to_bytes(ut_state->available_words));

    node = context.giveback_list;

    while (node) {
        freechunk_t  * next = node->next;
        ut_free(node);
        node = next;
    }

    _anr_core_set_error(ut_state, false);

    _anr_core_reclaim(ut_state, NULL);

    _anr_core_teardown(ut_state);

    UNIT_TEST_FOOTER;
}

UNIT_TEST(full_buffers)
{
    unsigned int alloc_sizes[1024];
    unsigned int allocated_bytes=0;
    void * ptrs[1024];
    unsigned int i;

    UNIT_TEST_HEADER;
    _anr_core_default_init(&ut_state);

    for (i = 0; i< sizeof (alloc_sizes) /sizeof (alloc_sizes[0]); i++) {
        if ((i & (i-1)) ==0)
            alloc_sizes[i] = 16 * (i+1);
        else
            alloc_sizes[i] = 1024 * (1025 - i);

        ptrs[i] = ut_malloc(alloc_sizes[i] * sizeof (word_t));
    
        if (ptrs[i])
            allocated_bytes+= chunk_bytes(mem_to_chunk(ptrs[i]));
        else{
            assert(alloc_sizes[i] * sizeof (word_t) > min(_anr_core_free_bytes(ut_state), 
                                                       PAGESIZE_BYTES * ut_state->available_pages - ut_state->overhead));
            _anr_core_set_error(ut_state, false);
        }

    }

    printf ("bytes allocated: %d\tbytes available in pool: %d\n", 
           allocated_bytes, 
           _anr_core_free_bytes(ut_state));

    for (i = 0; i< sizeof (alloc_sizes) /sizeof (alloc_sizes[0]); i++) {
        unsigned int j;
        word_t * alias;
        alias = ptrs[i];
        if (alias) {
            for (j =0; j< bytes_to_words(_anr_core_usable_size (ut_state, ptrs[i])); j++) {
                alias[j] = alloc_sizes[i];
            }
        }
    }

    _anr_core_set_error(ut_state, false);
    assert(0 == _anr_core_verify(ut_state));
    for (i = 0; i < sizeof (alloc_sizes) /sizeof (alloc_sizes[0]); i++) {
        if (i%2 ==0 && ptrs[i]) {

            allocated_bytes -= chunk_bytes(mem_to_chunk(ptrs[i]));
            ut_free(ptrs[i]);
            assert(0 == _anr_core_verify(ut_state));
            ptrs[i] = ut_malloc(100);
            allocated_bytes += chunk_bytes(mem_to_chunk(ptrs[i]));
        } else if (ptrs[i]) {
            unsigned int j;
            word_t * alias;
            alias = ptrs[i];
            if (alias)
                for (j =0; j< alloc_sizes[i]; j++) {
                    assert(alias[j] == alloc_sizes[i]);
                }
        }
         
    }

    printf ("bytes allocated: %d\tbytes available in pool: %d\n", 
           allocated_bytes, 
           _anr_core_free_bytes(ut_state));

    for (i = 0; i< sizeof (alloc_sizes) /sizeof (alloc_sizes[0]); i++) {
        ut_free(ptrs[i]);
        assert(0 == _anr_core_verify(ut_state));
    }

    _anr_core_teardown(ut_state);
    UNIT_TEST_FOOTER;
}

UNIT_TEST(realloc_grow_and_shrink_easy)
{
    void * ptr;
    void * realloc_ptr;
    chunk_t * block;

    UNIT_TEST_HEADER;
    _anr_core_default_init(&ut_state);
    ptr = ut_malloc(4048);
    block = mem_to_chunk(ptr);
    assert(chunk_bytes(block) >= 4048);
    realloc_ptr = ut_realloc(ptr, 3632); 
    assert(ptr == realloc_ptr);
    block = mem_to_chunk(ptr);
    assert (chunk_bytes(block) >= 3632 && chunk_bytes(block) <= 3632+ MIN_BLOCK_SIZE);
    ptr = realloc_ptr;
    realloc_ptr = ut_realloc(ptr, 4048);
    assert(ptr == realloc_ptr);
    ut_free(ptr);
    _anr_core_teardown(ut_state);
    UNIT_TEST_FOOTER;
}

UNIT_TEST(realloc_grow_and_shrink_hard)
{
    void * ptr[4];

    chunk_t * block[4];
    word_t * alias;
    unsigned int i;

    UNIT_TEST_HEADER;
    _anr_core_init (&ut_state,
                    MALLOC_VERIFY, 
                    1024,
                    20 *1024 * 1024 , 40 * 1024 * 1024 , 0, NULL, NULL, NULL, NULL);

    for (i= 0; i< 4; i++) {
        ptr[i] = ut_malloc(4048);
        memset(ptr[i], 0, 4048);
        block[i] = mem_to_chunk(ptr[i]);
    }

    /* first, lets shrink ptr 0 a few times */

    ptr[0] = ut_realloc(ptr[0], 3800);
    ptr[0] = ut_realloc(ptr[0], 3600);
    ptr[0] = ut_realloc(ptr[0], 3500);
    ptr[0] = ut_realloc(ptr[0], 3400);
    ptr[0] = ut_realloc(ptr[0], 3300);

    assert(block[0] == mem_to_chunk(ptr[0]));

    /* should be able to grow ptr 1 up into ptr 0's old space now.
     */

    alias = ptr[1];
    /* write some stuff into it to verify that copy works */
    for (i = 0; i< bytes_to_words(4048); i++) {
        alias[i] = i;
    }

    ptr[1] = ut_realloc(ptr[1], 4048 + 500);
    assert (0 == _anr_core_verify(ut_state));

    alias = ptr[1];

    /* check the copy */
    for (i =0; i< bytes_to_words(4048); i++) {
        assert(alias[i] ==  i);
    }

    assert(block[1] != mem_to_chunk(ptr[1]));
    assert(next_chunk(block[0]) == mem_to_chunk(ptr[1]));

    /* shrink 1 back to origional size */

    ptr[1] = ut_realloc(ptr[1], 4048);
    alias = ptr[1];

    assert (0 == _anr_core_verify(ut_state));
    /* check the copy */
    for (i =0; i< bytes_to_words(4048); i++) {
        assert(alias[i] ==  i);
    }

    assert(block[1] != mem_to_chunk(ptr[1]));
    assert(next_chunk(block[0]) == mem_to_chunk(ptr[1]));

    /* grow downward */
    ut_free(ptr[2]);

    ptr[1] = ut_realloc(ptr[1], 4048 + 1024);

    assert (0 == _anr_core_verify(ut_state));
    /* check the copy */
    alias = ptr[1];
    for (i =0; i< bytes_to_words(4048); i++) {
        assert(alias[i] ==  i);
    }
    assert(block[1] != mem_to_chunk(ptr[1]));
    assert(next_chunk(block[0]) == mem_to_chunk(ptr[1]));

    /* shrink back to orig */
    ptr[1] = ut_realloc(ptr[1], 4048);

    assert (0 == _anr_core_verify(ut_state));
    /* check the copy */
    alias = ptr[1];
    for (i =0; i< bytes_to_words(4048); i++) {
        assert(alias[i] ==  i);
    }

    assert(block[1] != mem_to_chunk(ptr[1]));
    assert(next_chunk(block[0]) == mem_to_chunk(ptr[1]));

    ptr[2] = ut_malloc(4048);

    /* realloc forcing a move and copy */
    ptr[1] = ut_realloc(ptr[1], 8096);

    assert (0 == _anr_core_verify(ut_state));
    /* check the copy */
    alias = ptr[1];
    for (i =0; i< bytes_to_words(4048); i++) {
        assert(alias[i] ==  i);
    }
    
    assert(block[1] != mem_to_chunk(ptr[1]));


    for (i= 0; i< 4; i++) {
        ut_free(ptr[i]);
    }
    assert (0 == _anr_core_verify(ut_state));

    /* trigger an actual bug.  we're going to create 2 blocks, and force the
     * first to realloc into the second. */

    block[0] = find_chunk (ut_state, 30);
    set_in_use (block[0]);
    ut_state->available_words -= chunk_words (block[0]);
    allocate_pages (ut_state, block[0]);

    block[1] = split_chunk_top (ut_state, block[0], 5);

    set_prev_in_use(block[1]);
    set_in_use (block[1]);

    block[2] = split_chunk_top (ut_state, block[1], 6);
    set_prev_in_use (block[2]);
    set_in_use (block[2]);

    block[3] = split_chunk_top (ut_state, block[2], 6);

    set_prev_in_use (block[3]);
    set_in_use (block[3]);

    block[0] = split_chunk_top (ut_state, block[3], 7);
    set_prev_in_use (block[0]);
    set_in_use (block[0]);

    ut_free (chunk_to_mem (block[3]));
    
    write_trash (ut_state, chunk_to_mem (block[1]));

    ptr[0] = ut_realloc (chunk_to_mem(block[1]), user_bytes(ut_state, block[1]) + words_to_bytes (1));

    assert (0 == _anr_core_verify(ut_state));

    _anr_core_teardown (ut_state);
    UNIT_TEST_FOOTER;
}

UNIT_TEST(valgrind)
{
    int * array;
    uint32_t oldhead;

    if (!RUNNING_ON_VALGRIND)return;

    UNIT_TEST_HEADER;

    _anr_core_default_init(&ut_state);

    printf ("valgrind should puke about buffer over/under runs\n");

    array = ut_malloc(sizeof (int) * 4);

    array[0] = 0;
    array[1] = 1;
    array[2] = 2;
    array[3] = 3;
    oldhead = array[4];       /* invalid read */
    array[4] = 10 + oldhead;  /* invalid write (past end) */
    array[4] -= oldhead;

    oldhead = array[-2];
    array[-2] = 100;
    array[-2] = oldhead; /* invalid write (past beginning) */
    

    _anr_core_teardown(ut_state);
    UNIT_TEST_FOOTER;
}
UNIT_TEST(aligned_malloc_easy)
{
    void * ptr;
    UNIT_TEST_HEADER;

    _anr_core_default_init(&ut_state);

    assert(NULL != (ptr = _anr_core_memalign(ut_state , PAGESIZE_BYTES, PAGESIZE_BYTES)));
    assert((((uintptr_t)ptr) & ~(PAGE_MASK)) == (uintptr_t)ptr);

    ut_free(ptr);
    _anr_core_reclaim(ut_state, NULL);

    _anr_core_teardown(ut_state);

    UNIT_TEST_FOOTER;
}

UNIT_TEST(aligned_malloc_pow2_sizes)
{
    int alignments[32];
    int i;
    unsigned int j;
    void ** ptrs;
    UNIT_TEST_HEADER;

    _anr_core_default_init(&ut_state);

    memset(alignments, 0, sizeof (alignments));

    for (i=0 , j = 16; j < words_to_bytes(MAX_ALLOC) /2; i++,j <<=1) {
        alignments[i] = j;
    }
    ptrs = malloc(sizeof (void *) * 1000);
    memset(ptrs, 0, sizeof (void *) * 1000);

    for (j = 0; j < sizeof (alignments) /sizeof (alignments[0]) && alignments[j] != 0; j++) {
        for (i = 0; i < 1000; i++) {
            ptrs[i] = _anr_core_memalign(ut_state, alignments[j], 32);
            if (ptrs[i] == NULL)
                break;
            assert((((uintptr_t)ptrs[i]) & ~(alignments[j] -1)) == (uintptr_t)ptrs[i]);
        }
        for (i= i-1 ; i >=0; i--) {
            ut_free(ptrs[i]);
            ptrs[i] = NULL;
        }
    }

    free(ptrs);
    _anr_core_reclaim(ut_state, NULL);

    _anr_core_teardown(ut_state);
    UNIT_TEST_FOOTER;

}
UNIT_TEST(aligned_malloc_non_pow2_sizes)
{

    int alignments[32];
    int i;
    unsigned int j;
    void ** ptrs;
    UNIT_TEST_HEADER;

    _anr_core_default_init(&ut_state);

    memset(alignments, 0, sizeof (alignments));

    for (i=0 , j = 16; j < words_to_bytes(MAX_ALLOC) /2; i++,j <<=1) {
        alignments[i] = j - 1;
    }
    ptrs = malloc(sizeof (void *) * 1000);
    memset(ptrs, 0, sizeof (void *) * 1000);

    for (j = 0; j < sizeof (alignments) /sizeof (alignments[0]) && alignments[j] != 0; j++) {
        for (i = 0; i < 1000; i++) {
            ptrs[i] = _anr_core_memalign(ut_state, alignments[j], 32);
            if (ptrs[i] == NULL)
                break;
            assert((((uintptr_t)ptrs[i]) & ~(alignments[j] -1)) == (uintptr_t)ptrs[i]);
        }
        for (i= i-1 ; i >=0; i--) {
            ut_free(ptrs[i]);
            ptrs[i] = NULL;
        }
    }

    free(ptrs);
    _anr_core_reclaim(ut_state, NULL);

    _anr_core_teardown(ut_state);
    UNIT_TEST_FOOTER;

}


UNIT_TEST(large_allocs)
{
    void * ptrs[1024];
    void * blockers[1024];
    int i;

    UNIT_TEST_HEADER;

    _anr_core_default_init(&ut_state);

    /* allocate a bunch of blocks larger than 1040 bytes that can't be coalesced. */


    for (i =0; i< 1024; i++) {
        ptrs[i] = ut_malloc(1500);
        blockers[i] = ut_malloc(20);
    }

    for (i=0; i< 10; i++) {
        assert(_anr_core_verify(ut_state) == 0);

        ut_free(ptrs[i]);

    }

    for (i = 0 ; i< 10; i++) {
        assert(_anr_core_verify(ut_state) == 0);

        ptrs[i] = ut_malloc(1500);
    }

    for (i = 0; i< 1024; i++) {
        ut_free(ptrs[i]);
        ut_free(blockers[i]);
    }

    _anr_core_reclaim(ut_state, NULL);

    _anr_core_teardown(ut_state);

    UNIT_TEST_FOOTER;

}

UNIT_TEST(bins)
{
    
    malloc_state_t state;
    void * ptr;
    unsigned int i;
    binchunk_t * block;
    treechunk_t * tree;

    UNIT_TEST_HEADER;

    memset(&state, 0, sizeof (malloc_state_t));

    ptr = mmap(0, 4096, PROT_READ|PROT_WRITE,MAP_PRIVATE|MAP_ANONYMOUS, -1, 0);

    block = ptr;

    block->head = 4096;
    set_prev_in_use(block);

    /* Test small bins */

    /* Test 1.  If a smallbin has something in it, and we ask for that binsize,
     * do we get the thing we should ?*/
    

    for (i = 0; i< SMALLBINS; i++) {
        chunk_t * found;
        mark_smallmap(&state, i);
        block->link.next = &state.smallbins[i];
        block->link.prev = &state.smallbins[i];

        state.smallbins[i].link.next = block;
        state.smallbins[i].link.prev = block;

        found = find_binchunk(&state, i + bytes_to_words(MIN_BLOCK_SIZE));

        assert(found == chunk_cast(block));
        clear_smallmap(&state, i);
        
    }
    memset(&state, 0, sizeof (malloc_state_t));

    /* test 2.  If a smallbin way up has something in it, and nothing else
     * does, do we get the right thing when we request smaller sizes */

    for (i = 0; i< 200; i++) {

        block->link.next = &state.smallbins[200];
        block->link.prev = &state.smallbins[200];

        state.smallbins[200].link.next = block;
        state.smallbins[200].link.prev = block;
        mark_smallmap(&state, 200);
        assert(find_binchunk(&state, i + bytes_to_words(MIN_BLOCK_SIZE)) == chunk_cast(block));
    }

    memset(&state, 0, sizeof (malloc_state_t));

    /* test 3.  If smallbins are totally empty, but a treebin has something in it, do we get that?
     */
    tree= tree_cast(block);

    tree->list.prev = tree->list.next = tree;
    tree->link[0] = tree->link[1] = 0;
    tree->color = BLACK;
    tree->dirty.next = tree->dirty.prev = tree;
    

    state.largebins[0].root =tree; 
    state.largebins[0].max = chunk_words(block);

    mark_largemap(&state, 0);

    assert(find_binchunk(&state, 200) == chunk_cast(block));

    /* End small bin testing -- Begin largebin testing */


    memset(&state, 0, sizeof (malloc_state_t));

    /*  If A treebin has something in it, and we request that size, do we get it?
     */
    for (i = 0; i< LARGEBINS; i++) {
        tree= tree_cast(block);
        tree->list.prev = tree->list.next = tree;
        tree->link[0] = tree->link[1] = 0;
        tree->color = BLACK;
        tree->head = MAX_ALLOC;
        state.largebins[i].root = tree;
        state.largebins[i].max = chunk_words(block);

        mark_largemap(&state, i);
        assert(find_treechunk(&state, i + bytes_to_words(MIN_LARGEBLOCK_SIZE)) == chunk_cast(block));
        clear_largemap(&state, i);

    }
    memset(&state, 0, sizeof (malloc_state_t));
    
    munmap(ptr, 4096);
    UNIT_TEST_FOOTER;
}

#define POOL_SIZE 64 * 1024 * 1024
UNIT_TEST(easy_slabs)
{
    void * ptr[10];
    int i;
    uint32_t slabs[] = { 32 };
    UNIT_TEST_HEADER;

    _anr_core_init(&ut_state, 
                   0,
                   POOL_SIZE, 
                   POOL_SIZE * 2, 
                   1024,
                   sizeof (slabs) /sizeof (slabs[0]), 
                   slabs, NULL, NULL, NULL);

    for (i = 0 ; i< 300;i++) {
        ptr[i % 10] = ut_malloc(slabs[0]);
        assert(alloc_is_slice(ut_state, ptr[i %10]));
    }

    for (i=0; i< 10; i++) {
        _anr_core_mark(ut_state, ptr[i]);
    }

    _anr_core_reclaim(ut_state, NULL);
    for (i = 0; i< 10; i++) {
        ut_free(ptr[i]);
    }
    _anr_core_teardown(ut_state);
    UNIT_TEST_FOOTER;
}

UNIT_TEST(hard_slabs)
{

    void ** ptrs;
    int i;
    uint32_t slabs[] = { 20,24,32,36,128,100 };
    UNIT_TEST_HEADER;

    _anr_core_init(&ut_state, 
                   0,
                   POOL_SIZE, 
                   POOL_SIZE * 2, 
                   1024,
                   sizeof (slabs) /sizeof (slabs[0]), 
                   slabs, NULL, NULL, (AbortFunction)abort);

    ptrs = malloc(sizeof (void *) * 10000);

    for (i = 0; i< 10000; i++) {
        int idx = (i % sizeof (slabs) /sizeof (slabs[0]));
        ptrs[i] = ut_malloc(slabs[idx]);
        _anr_core_mark(ut_state, ptrs[i]);
    }

    printf ("1st pass reclaim\n");
    _anr_core_reclaim(ut_state, NULL);

    for (i=0; i< 10000;i++) {
        _anr_core_mark(ut_state, ptrs[i]);
        if (i %2)i++;
        else if (i %5 == 0)i+=3;
        else if (i%3 ==0)i+=2;
        else if (i%7==0)i+=11;
    }

    printf ("2nd pass reclaim\n");
    _anr_core_reclaim(ut_state, NULL);

    free(ptrs);

    printf ("Teardown\n");
    _anr_core_teardown(ut_state);

    UNIT_TEST_FOOTER;

}

UNIT_TEST(alignment)
{
    void ** ptrs;
    int i;
    uint32_t slabs[] = { 20,24,32,36,128,100 };
    UNIT_TEST_HEADER;

    _anr_core_init(&ut_state, 
                   0,
                   POOL_SIZE, 
                   POOL_SIZE * 2, 
                   1024,
                   sizeof (slabs) /sizeof (slabs[0]), 
                   slabs, NULL, NULL, NULL);

    ptrs = malloc(sizeof (void *) * 10000);

    for (i = 0; i< 10000; i++) {
        int idx = (i % sizeof (slabs) /sizeof (slabs[0]));
        ptrs[i] = ut_malloc(slabs[idx]);
        assert ((((uintptr_t)ptrs[i]) & ALIGNMENT_MASK) == 0);
    }

    for (i = 0; i< 10000; i++) {
        ut_free (ptrs[i]);
    }

    _anr_core_teardown(ut_state);

    _anr_core_init(&ut_state, 
                   0,
                   POOL_SIZE, 
                   POOL_SIZE * 2, 
                   1024,
                   0, 
                   NULL, NULL, NULL, NULL);

    for (i = 0; i< 10000; i++) {

        int idx = (i % sizeof (slabs) /sizeof (slabs[0]));
        ptrs[i] = ut_malloc(slabs[idx] * 10);
        assert ((((uintptr_t)ptrs[i]) & ALIGNMENT_MASK) == 0);
    }

    for (i = 0; i< 10000; i++)
        ut_free (ptrs[i]);

    _anr_core_teardown(ut_state);
    free (ptrs);
    UNIT_TEST_FOOTER;
}
UNIT_TEST(report)
{
    int i;
    void * ptr = NULL;
    UNIT_TEST_HEADER;

    printf ("sizeof (malloc_state_t) = %d\n", sizeof (malloc_state_t));

    _anr_core_default_init(&ut_state);
    _anr_core_report(ut_state, fileno(stdout), NULL);
    for (i = 1000; i> 0; i--) {
        if (ptr && i % 4 == 0) {
            void * temp = ut_malloc(10);
            ut_free(ptr);
            _anr_core_mark(ut_state, temp);
        }
        ptr = ut_malloc(words_to_bytes(i));
        UNUSED(ptr);
    }
    _anr_core_report(ut_state, fileno(stdout), NULL);
    _anr_core_reclaim(ut_state, NULL);
    _anr_core_report(ut_state, fileno(stdout), NULL);
    _anr_core_teardown(ut_state);

    UNIT_TEST_FOOTER;
}
UNIT_TEST(pool_expand_and_shrink)
{
    unsigned int expand = 8 * 1024 * 1024 / PAGESIZE_BYTES;
    unsigned int page_check;
    void * ptr;
    UNIT_TEST_HEADER;
    _anr_core_default_init(&ut_state);

    page_check = ut_state->available_pages;

    _anr_core_shrink(ut_state, expand);
    assert(((int)(page_check - expand)) == ut_state->available_pages);
    _anr_core_expand(ut_state, expand); 
    assert((int)page_check == ut_state->available_pages);


    _anr_core_shrink (ut_state, page_check - expand);

    ptr = ut_malloc((expand << 1) * PAGESIZE_BYTES);

    assert(!ptr);
    _anr_core_set_error(ut_state, false);
    _anr_core_expand(ut_state, page_check - expand);
    ptr = ut_malloc((expand << 1) * PAGESIZE_BYTES);
    assert(ptr);
    ut_free(ptr);
    
    page_check = (int)_anr_core_shrink(ut_state, 80 * 1024 * 1024 / PAGESIZE_BYTES);

    assert(ut_state->reclaimable_pages == 0);

    
    _anr_core_teardown(ut_state);
    UNIT_TEST_FOOTER;
}

#include <setjmp.h>
static jmp_buf abort_env;
static void UNIT_TEST_HELPER
verify_abort(const char * file, int line, char * message)
{
    printf ("%s:%d %s\n", file, line, message);
    longjmp(abort_env, 1);
}
UNIT_TEST(bounds_check)
{
    void * ptr;
    bool success = false;
    UNIT_TEST_HEADER;
    
    _anr_core_init(&ut_state,
                   BOUNDS_CHECK,
                   POOL_SIZE,
                   POOL_SIZE * 2,
                   1024,
                   0,
                   NULL,
                   NULL,
                   NULL,
                   verify_abort);

    if (setjmp (abort_env) == 0) {
        success = true;
        ptr = ut_malloc(100 * sizeof (int));
        /* Deliberate buffer over-run */
        ((int *)ptr)[100] = 0;
        ut_free(ptr);
    } else
        success = true;

    assert(success);

    munmap (ut_state->mappings->top, words_to_bytes(ut_state->mappings->size));
    munmap (ut_state, 2 * PAGESIZE_BYTES);
    UNIT_TEST_FOOTER;
}
UNIT_TEST(verify)
{
    void * ptr;
    bool success = false;
    UNIT_TEST_HEADER;
    
    _anr_core_init(&ut_state,
                   MALLOC_VERIFY,
                   POOL_SIZE,
                   POOL_SIZE * 2,
                   1024,
                   0,
                   NULL,
                   NULL,
                   NULL,
                   verify_abort);

    if (setjmp (abort_env) == 0) {
        ptr = ut_malloc(100 * sizeof (int));
        success = true;
        /* Deliberate buffer over-run */
        ((int *)ptr)[100] = 0;
        ut_free(ptr);
    } else
        success = true;

    assert(success);

    munmap (ut_state->mappings->top, words_to_bytes(ut_state->mappings->size));
    munmap (ut_state, 2 * PAGESIZE_BYTES);
    UNIT_TEST_FOOTER;
}

UNIT_TEST(fillwithtrash)
{
    word_t * ptr;
    unsigned int i;
    uint32_t slab_sizes[] = { 32 };
    unsigned int size;
    UNIT_TEST_HEADER;

    _anr_core_init (&ut_state, 
                    FILL_WITH_TRASH, 
                    POOL_SIZE, 
                    POOL_SIZE * 2,
                    1024, 
                    0, 
                    slab_sizes, 
                    NULL, 
                    NULL, 
                    NULL);

    ptr = ut_malloc(100);

    size = _anr_core_usable_size(ut_state, ptr);

    ut_free(ptr);

    /* this loop is correct for verifying that the "free area" has been
     * over-written.  First, we have no idea how the allocator may have
     * recombined the block on free, but since we have 1 malloc, it stands to
     * reason that we'll be a tree chunk.  All the treechunk values are
     * unverifiable WRT fillwithtrash (they should be other chunks or NULL).
     * Additionally, the last word of the allocation should hold the size of
     * the block in words, reguardless of how the allocator combined it -- this
     * is because the set_available macro writes the block size into that word.
     *
     * So, it is unsafe on a non-slab block to verify the first
     * sizeof (treechunk_t)bytes, and the last word.
     */



    for (i = bytes_to_words(sizeof (treechunk_t)); 
         i < bytes_to_words(size)-1; 
         i++)
        assert((uintptr_t)ptr[i] == (uintptr_t)0xdeadbeef);

    _anr_core_teardown(ut_state);

    _anr_core_init(&ut_state, FILL_WITH_TRASH, POOL_SIZE, POOL_SIZE * 2,
                   1024, 1, slab_sizes, NULL, NULL, NULL);

    ptr = ut_malloc(32);

    assert(alloc_is_slice(ut_state, ptr));

    ut_free(ptr);

    for (i = 0; i < bytes_to_words(32); i++)
        assert((uintptr_t)ptr[i] == (uintptr_t)0xdeadbeef);

    _anr_core_teardown(ut_state);
    UNIT_TEST_FOOTER;
}
UNIT_TEST(add_mapping)
{
    struct map_node{
        mapping_t * mapping;
        struct map_node * next;
        bool present;
    };

    int i;

    mapping_t * mapping;
    struct map_node * head = NULL;

    UNIT_TEST_HEADER;
    _anr_core_init(&ut_state,
                   0,
                   POOL_SIZE,
                   POOL_SIZE * 2,
                   1024,
                   0,
                   NULL,
                   NULL,
                   NULL,
                   NULL);
        
    for (mapping = ut_state->mappings; mapping; mapping = mapping->next) {
        struct map_node * node = ut_malloc (sizeof (struct map_node)); 
        node->next = head;
        node->present = false;
        head = node;
        node->mapping = mapping;
    }

    for (i = 0; i < 10; i++) {
        struct map_node * node;
        struct map_node * iter;
        _anr_core_add_mapping (ut_state, 
                               POOL_SIZE, 
                               POOL_SIZE *2);


        
        for (mapping = ut_state->mappings; mapping; mapping = mapping->next) {
            bool found = false;
            for (iter = head; iter; iter = iter->next) {
                if (iter->mapping == mapping) {
                    iter->present = true;
                    found = true;
                    break;
                }
            }
            
            if (!found) {
                node = ut_malloc (sizeof (struct map_node));
                node->next = head;
                head = node;
                node->mapping = mapping;
                node->present = true;
            }

        }


        for (iter = head; iter; iter = iter->next) {
            assert (iter->present);
            iter->present = false;
        }

    } 


    _anr_core_teardown(ut_state);
    UNIT_TEST_FOOTER;
}

UNIT_TEST(slab_realloc)
{
    void * ptrs[2];
    uint32_t slabs[] = { 32,104 };
    uint32_t i;
    uint32_t count;
    slab_t * slab;
    UNIT_TEST_HEADER;

    _anr_core_init (&ut_state, 
                    0,
                    POOL_SIZE, 
                    POOL_SIZE * 2, 
                    1024,
                    sizeof (slabs) / sizeof (slabs[0]), 
                    slabs, NULL, NULL, NULL);


    ptrs[0] = ut_malloc(16);
    
    assert(alloc_is_slice(ut_state,ptrs[0]));
    /* verify that we can grow in the same slice up to the slab size */
    ptrs[1] = ut_realloc(ptrs[0], 20);

    assert(alloc_is_slice(ut_state,ptrs[1]));
    assert(ptrs[0] == ptrs[1]);
    ptrs[0] = ut_realloc(ptrs[1], 24);

    assert(alloc_is_slice(ut_state, ptrs[0]));
    assert(ptrs[1] == ptrs[0]);

    ptrs[1] = ut_realloc(ptrs[0], 32);

    assert(alloc_is_slice(ut_state, ptrs[1]));
    assert(ptrs[0] == ptrs[1]);
    /* this is the slice copy case, where the slice isn't big enough */
    ptrs[0] = ut_malloc(104);

    slab = ptr_to_slab(ptrs[0]);
    ut_free(ptrs[0]);
    count = slab->pool->n_available;
    

    for (i = 0; i < count; i++) {
        if (i == count / 2)
            ptrs[1] = ut_malloc(104);
        else
            (void)ut_malloc(104);
    }

    ptrs[0] = ut_malloc(104);

    slab = ptr_to_slab(ptrs[0]);
    set_in_use(prev_chunk(slab));
    set_in_use(next_chunk(slab));
    ut_free(ptrs[1]);
    ptrs[1] = ut_realloc(ptrs[0],240);

    assert(ptrs[1] != ptrs[0]);

    set_available (prev_chunk (slab));
    set_available (next_chunk (slab));

    ut_free (ptrs[1]);

    _anr_core_reclaim (ut_state, NULL);


    assert (0 == _anr_core_verify (ut_state));

    _anr_core_teardown(ut_state);
    UNIT_TEST_FOOTER;
}

UNIT_TEST(wild_pointers)
{

    void * wild_pointer;
    void * ret;
    bool crashed = false;
    UNIT_TEST_HEADER;

    _anr_core_default_init(&ut_state);
    _abort = verify_abort;

    wild_pointer = ut_state->mappings->top; 
        
    _anr_core_mark(ut_state, wild_pointer);
    if (setjmp (abort_env) == 0) {
        crashed = false;
        ret = ut_realloc(wild_pointer, 10);
        (void)ret;
    } else 
        crashed = true;

    assert (crashed);

    crashed = false;

    if (setjmp (abort_env) == 0) {
        crashed = false;
        ut_free(wild_pointer);
    } else
        crashed = true;

    assert (crashed);
    
    assert (_anr_core_usable_size(ut_state, wild_pointer) ==0);

    UNIT_TEST_FOOTER;
} 
#undef POOL_SIZE

static jmp_buf map_env;
static void UNIT_TEST_HELPER
map_abort(const char * file, int line, char * message)
{
    printf ("%s:%d %s\n", file, line, message);
    longjmp(map_env, 1);
}

UNIT_TEST(mapping_size_failure)
{

    bool success = false;
    mapping_t * mapping;
    UNIT_TEST_HEADER;
    
    if (setjmp (map_env) == 0) {
        _anr_core_init(&ut_state,
                       0,
                       0xEFFFFFFF,
                       0xFFFFFFFF,
                       1024,
                       0,
                       NULL,
                       NULL,
                       NULL,
                       map_abort);

        success = false;
    } else
        success = true;

    assert(success);

    for (mapping = ut_state->mappings; mapping != NULL;) {
        mapping_t * next = mapping->next;
        munmap(mapping->top, words_to_bytes(mapping->size));
        mapping = next;
    }
    munmap(ut_state, 2 * PAGESIZE_BYTES);
    UNIT_TEST_FOOTER;
}

UNIT_TEST(mapping_size_fallback)
{
    mapping_t * mapping;
    UNIT_TEST_HEADER;
    
    _anr_core_init(&ut_state,
                   0,
                   HALF_GB,
                   0xFFFFFFFF,
                   1024,
                   0,
                   NULL,
                   NULL,
                   NULL,
                   NULL);

    for (mapping = ut_state->mappings; mapping != NULL;) {
        mapping_t * next = mapping->next;
        munmap(mapping->top, words_to_bytes(mapping->size));
        mapping = next;
    }
    munmap(ut_state, 2 * PAGESIZE_BYTES);
    UNIT_TEST_FOOTER;
}
UNIT_TEST(huge_mapping_overflow)
{
    mapping_t * mapping;
    UNIT_TEST_HEADER;
    _anr_core_init(&ut_state, 
                   0, 
                   1002553344/2, 
                   1002553344, 
                   1024, 
                   0, 
                   NULL, 
                   NULL, 
                   NULL, 
                   NULL);

    assert(ut_state->mappings->next == NULL);

    for (mapping = ut_state->mappings; mapping != NULL;) {
        mapping_t * next = mapping->next;
        munmap (mapping->top, words_to_bytes(mapping->size));
        mapping = next;
    }
    munmap (ut_state, 2 * PAGESIZE_BYTES);

    UNIT_TEST_FOOTER; 

}
UNIT_TEST(bitops)
{
    unsigned int i;
    unsigned int j;
    unsigned int right_mask;
    unsigned int left_mask;
    unsigned int val;
    UNIT_TEST_HEADER;
    
    assert (clz (0xffffffff) == 0);
    assert (ctz (0xffffffff) == 0);
    assert (clz (0) == 32);
    assert (ctz (0) == 32);
    
    for (i = 0; i< 32; i++) {
        val = 1 << i;
        assert(clz (val) == 32U - (i+1));
        assert(ctz (val) == i);
        assert(mask_right (val, i) == val);
        assert(mask_left (val, i) == val);
        assert((mask_right (val, i) & mask_left (val, i)) == val);

        printf ("mask_right (0x%08x, %d) = 0x%08x\n", val, i, mask_right (val, i));
        printf ("mask_left  (0x%08x, %d) = 0x%08x\n", val, i, mask_left (val, i));

        right_mask = 0xffffffff;

        left_mask = 0xffffffff;
        for (j = 0; j < i; j++) {
            right_mask &= ~(1U<<j);
        }
        for (j = 31; j > i; j--) {
            left_mask &= ~(1U <<j);
        }

        printf ("mask_right (0xffffffff, %d) = 0x%08x vs. 0x%08x\n", i, mask_right (0xffffffff, i), right_mask);
        printf ("mask_left  (0xffffffff, %d) = 0x%08x vs. 0x%08x\n", i, mask_left (0xffffffff, i), left_mask);

        assert (mask_right(0xffffffff, i) == right_mask);
        assert (mask_left (0xffffffff, i) == left_mask);
    }
    
    UNIT_TEST_FOOTER;
}
UNIT_TEST(slab_reclaim)
{
    void * ptr;
    slab_t *slab;
    int i;
    uint32_t slabs[] = {64};
    UNIT_TEST_HEADER;
    _anr_core_init(&ut_state, 
                   0, 
                   1*1024 * 1024, 
                   2*1024 * 1024, 
                   1024, 
                   1, 
                   slabs, 
                   NULL, 
                   NULL, 
                   NULL);
    
    /* we want to test a real corruption problem, which occured due to lots of
     * memory leaks from a slab. */

    ptr = ut_malloc (64);
    assert (alloc_is_slice (ut_state, ptr));
    slab = ptr_to_slab (ptr);
    mark_slice (slab, ptr);


    for (i = 1; i < slab->pool->n_slices_per_slab; i++) {
        ptr = allocate_slice (ut_state, slab);
        assert (alloc_is_slice (ut_state, ptr));
        if (i > 16)
            mark_slice (slab, ptr);
    }

    slab_reclaim (ut_state, slab, NULL);
    /* at this point, the slab is in the state that produced the bug, we have
     * free slices in the first 16 slots of the slab, but, before this commit,
     * reclaim wouldn't reset the last_bitmap_word, so we'd alloc off the end of
     * the slab. 
     */

    /* With the fix to slab_reclaim, we should actually get a slice in the slab */
    ptr = allocate_slice (ut_state, slab);
    assert (alloc_is_slice (ut_state, ptr));

    /* The second bug had to do with the fact that the last_bitmap_word was
     * rolling past the end.  And not being reset in allocate_slice.  
     * We fake that out here.
     */

    slab->last_bitmap_word = slab->pool->n_bitmap_words/2 -1; 

    ptr = allocate_slice (ut_state, slab);

    /* before the fix, both of the below tests would fail... */

    assert (alloc_is_slice(ut_state, ptr));
    assert (slab->last_bitmap_word == 0);

    /* for good measure, we include a bug found by the following, which is what
     * this unit test did initially */

    /* malloc debug catches this condition with a crash.  we need to deal with
     * that differently
     */
#if MALLOC_DEBUG == 0
    slab->last_bitmap_word = slab->pool->n_bitmap_words/2; 

    ptr = allocate_slice (ut_state, slab);

    assert (alloc_is_slice(ut_state, ptr));
    assert (slab->last_bitmap_word == 0);
#endif

    _anr_core_teardown(ut_state);
    UNIT_TEST_FOOTER;
}

#endif
