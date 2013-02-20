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
#ifndef __ANR_CORE_H_
#define __ANR_CORE_H_
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#ifndef __ANR_CALLBACKS_H__
#define __ANR_CALLBACKS_H__
typedef void (*MessageFunction)(const char *, const char *, const char *, const char *);
typedef unsigned int (*MoreMemoryFunction)(void * context, unsigned int, unsigned int );
typedef void (*MemoryFullFunction)(unsigned, unsigned, unsigned, unsigned);
typedef void (*AbortFunction)(const char *, int, char *);
#endif

/*****************************************************************************
 * Preliminaries
 ****************************************************************************/
/* in the event that wrapper code wants to add more overhead to each alloc,
 * the size of a word should be identical between core and wrapper.
 */
#define INTBYTES  4 /* sizeof(int) */
#define DOUBLEBYTES 8 /* sizeof(double)*/

#define PAGESIZE_BYTES EXEC_PAGESIZE
#define PAGE_MASK (PAGESIZE_BYTES-1)

#if defined(DOUBLEALIGNED)
typedef uint64_t word_t;
#define WORDBYTES DOUBLEBYTES
#else
typedef uint32_t word_t;
#define WORDBYTES INTBYTES
#endif

typedef uint32_t bitmap_t;
#define BYTES_PER_WORD (sizeof(word_t))
typedef struct malloc_state malloc_state_t;
typedef int (*ReportFunction)(int fd, void * ptr, size_t size);

typedef enum{
    MORE_MEM_RETRIES = 100 
} MallocStateConditions;


typedef enum{
    MALLOC_VERIFY = 1,
    BOUNDS_CHECK = 1 << 1,
    FILL_WITH_TRASH = 1 << 2
}malloc_init_flags;

/* The standard API 
   Roughly corilates to malloc/calloc, realloc, free and memalign.
 */
void * _anr_core_allocate(malloc_state_t * mstate,  
                          size_t size, void * func);
void * _anr_core_realloc( malloc_state_t * state, void * ptr, 
                                size_t size, 
                                void * func);
void _anr_core_free(malloc_state_t *, void * ptr, void * func);
void * _anr_core_memalign( malloc_state_t * self, size_t alignment, 
                                 size_t bytes);
/* Init and teardown */
int _anr_core_init(malloc_state_t ** self, 
                   int flags, 
                   unsigned int pool_size, 
                   unsigned int mapsize,
                   int dirty_page_ceiling,
                   unsigned int slabs,
                   unsigned int * slab_sizes,
                   MoreMemoryFunction gb_func, 
                   void * gb_context,
                   AbortFunction abort_func);
void _anr_core_teardown(malloc_state_t * self);
int _anr_core_default_init(malloc_state_t ** self);
/* Mark/Sweep garbage collection */
void _anr_core_mark(malloc_state_t * self,  void * ptr);
unsigned int _anr_core_reclaim(malloc_state_t * self, ReportFunction func);
/* Error state */
bool _anr_core_get_error(malloc_state_t * self);
void _anr_core_set_error(malloc_state_t * self, bool error);
/* Information about the heap/memory allocated by the heap */
void _anr_core_pointer_info(malloc_state_t * self,  void * ptr );
bool _anr_core_valid_pointer(malloc_state_t * self, void * ptr);
size_t _anr_core_usable_size(malloc_state_t * self,  void * ptr);
int _anr_core_report(malloc_state_t *, int, const char *);
void _anr_core_dump(malloc_state_t *, int, ReportFunction );
/* debug state information */
unsigned int _anr_core_event_count(malloc_state_t *);
unsigned int _anr_core_free_bytes(malloc_state_t * self);
unsigned int _anr_core_total_bytes(malloc_state_t * self);
/* Heap consistency/coherency check */
int _anr_core_verify(malloc_state_t *);
int _anr_core_check_chunks(malloc_state_t *);
/* lock the heap */
void _anr_core_lock_heap(malloc_state_t *);
int _anr_core_unlock_heap(malloc_state_t *);
bool _anr_core_trylock_heap(malloc_state_t *);
void _anr_core_prefork_prepare(malloc_state_t *);
int _anr_core_postfork(malloc_state_t *);
void _anr_core_reinit_locks(malloc_state_t *);

/* fork handling */
void _anr_core_prefork_prepare(malloc_state_t * self);
int _anr_core_postfork(malloc_state_t * self);
void _anr_core_postfork_child(malloc_state_t * self);

int _anr_core_available_pages(malloc_state_t * self);
unsigned int _anr_core_event_count(malloc_state_t *);
/* shrink or expand the heap */
unsigned int _anr_core_shrink(malloc_state_t *, unsigned int);
unsigned int  _anr_core_expand(malloc_state_t *, unsigned int);
void _anr_core_set_size(malloc_state_t *, int );
void _anr_core_add_mapping(malloc_state_t * self, unsigned int available, unsigned int bytes);
unsigned int _anr_core_mapping_overhead(unsigned int bytes);
/* For clients that can compress data (giveback clients) */
void _anr_core_block_until_free(malloc_state_t *);
/* non mallocing printf */
int malloc_printf(int fd, const char * fomat, ...);

/* internal support for fatal errors */
void _anr_core_abort(const char * file,
                     unsigned int line,
                     const char * format,
                     ...) __attribute__((noreturn))
                          __attribute__((format(printf, 3, 4)));

#define anr_assert(a) \
    do{\
        if(RARELY(!(a))){\
            _anr_core_abort(__FILE__, __LINE__, "%s", "Assertion ("#a") failed");\
        }\
    }while(0)

#define anr_crash(...)\
    _anr_core_abort(__FILE__, __LINE__, __VA_ARGS__)

/*
 * Branch Prediction.
 */
#define USUALLY(x) __builtin_expect((x), 1)
#define RARELY(x) __builtin_expect((x), 0)

#ifdef __OPTIMIZE__
#define ALWAYS_INLINE __attribute__((always_inline))
#define PURE __attribute__((pure))
#define PURE_INLINE __attribute__((pure, always_inline))
#define inline inline
#else
#define ALWAYS_INLINE
#define PURE
#define PURE_INLINE
#define inline
#endif

#endif
