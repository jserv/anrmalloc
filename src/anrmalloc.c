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
 * Allocation code for use with clients that can do co-operative memory 
 * sharing.
 */
#include <string.h> /* memset */
#include <stdint.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <unistd.h>
#include <assert.h>
#include <errno.h>
#include <pthread.h>
#include <sys/syscall.h>
#include <stdlib.h>
#ifdef HAVE_FRAME_POINTERS
#include <execinfo.h> /* backtrace */
#endif
#include "anrmalloc.h"
#include "anrcore.h"
#include "parse_config.h"

#ifndef DEBUG_OBJECTS_USE_SEPERATE_ALLOCATOR
#define DEBUG_OBJECTS_USE_SEPERATE_ALLOCATOR 0
#endif

/* Internal state flags */
enum __anr_state_flags{
    MALLOC_ERROR_BIT = 1,
    NO_GIVEBACK_BIT = 1<<1U,
    PERFORMING_GIVEBACK = 1 << 2U,
    NO_ERROR_BIT= 1<< 3U,
    TRACK_OBJECTS_BIT = 1 <<4U,
    SHOW_LEAKS_BIT = 1 << 5U,
    MEMFULL_REPORT = 1 << 6U,
    PRIMARY_TRACEBACK = 1 << 7U,
    SECONDDARY_TRACEBACK = 1 << 8U,
    TEARDOWN = 1 << 9U,
};

typedef struct debug debug_t;

/* giveback */
#define giveback_ok(mstate)\
    (!((mstate)->flags & NO_GIVEBACK_BIT))
#define start_giveback(mstate)\
    ((mstate)->flags |= PERFORMING_GIVEBACK)
#define end_giveback(mstate)\
    ((mstate)->flags &= ~PERFORMING_GIVEBACK)
#define disable_giveback(mstate)\
    ((mstate)->flags |= NO_GIVEBACK_BIT)
#define enable_giveback(mstate)\
    ((mstate)->flags &= ~NO_GIVEBACK_BIT)
#define performing_giveback(state)\
    ((state)->flags & PERFORMING_GIVEBACK)

/* reclaim print */
#define disable_reclaim_print(state)\
    ((state)->flags |= NO_RECLAIM_PRINT)
#define enable_reclaim_print(state) \
    ((state)->flags &= ~NO_RECLAIM_PRINT)
#define no_reclaim_print(state) \
    ((state)->flags & NO_RECLAIM_PRINT)

/* malloc error state */
#define disable_error(mstate) \
    ((mstate)->flags |= NO_ERROR_BIT)
#define error_ok(mstate)\
    (!((mstate)->flags & NO_ERROR_BIT))
#define enable_error(mstate)\
    ((mstate)->flags &= ~NO_ERROR_BIT)

/* track objects (debug objects) */
#define enable_track_objects(mstate)\
    ((mstate)->flags |= TRACK_OBJECTS_BIT)
#define disable_track_objects(mstate)\
    ((msate)->flags &= ~TRACK_OBJECTS_BIT)
#define track_objects(mstate)\
    ((mstate)->flags & TRACK_OBJECTS_BIT)

/* leak reporting */
#define enable_leak_report(state)\
    ((state)->flags |= SHOW_LEAKS_BIT)
#define disable_leak_report(state)\
    ((state)->flags &= ~SHOW_LEAKS_BIT)
#define report_leaks(state)\
    ((state)->flags & SHOW_LEAKS_BIT)

#define enable_memfull_report(state)\
    ((state)->flags |= MEMFULL_REPORT)
#define memfull_report(state)\
    ((state)->flags & MEMFULL_REPORT)

#define enable_teardown(state)\
    ((state)->flags |= TEARDOWN)

#define teardown(state)\
    ((state)->flags & TEARDOWN)


/* backtrace related state.  We capture the first and most
 * recent backtrace on malloc failures
 */

#define latch_primary_traceback(state)\
    ((state)->flags |= PRIMARY_TRACEBACK)

#define primary_traceback_captured(state)\
    ((state)->flags & PRIMARY_TRACEBACK)

#define secondary_traceback_captured(state)\
    ((state)->flags & SECONDDARY_TRACEBACK)

#define latch_secondary_traceback(state)\
    ((state)->flags |= SECONDDARY_TRACEBACK)

/* traceback state */
struct _traceback{
    void * backtrace[100];
    pid_t pid;
    pid_t tid;
    uint32_t count;    
    int size;
};

typedef struct _traceback traceback_t;



/* The global malloc state.
 */
struct anr_state{
    malloc_state_t * mstate;
    uint32_t flags;
    void * context;
    uint32_t overhead;
    pthread_key_t tid_key;
    MoreMemoryFunction _more_mem;
    MemoryFullFunction _mem_full;
    debug_t * dbg_pool;
#ifdef HAVE_FRAME_POINTERS
    traceback_t failures[2];
#endif
};

static struct anr_state state;


/*Debug Object management */
struct debug{
    int malloccount;
    void * function;
    pid_t tid;
    size_t size;
};
#define core_to_usr(ptr)\
    ((void *)(((uintptr_t)ptr) + state.overhead))

#define usr_to_core(ptr)\
    ((void *)(((uintptr_t)ptr) - state.overhead))


#define DEBUG_OBJECT_BIT 1U

#define is_debug_object(ptr)\
    (((uintptr_t)ptr) & DEBUG_OBJECT_BIT) 

#define set_debug_object(ptr) \
    ((void *)(((uintptr_t)ptr) | DEBUG_OBJECT_BIT))

#define debug_object(ptr) \
    ((void *)(((uintptr_t)ptr) & ~DEBUG_OBJECT_BIT))


static inline debug_t *
debug_object_allocate(void * func, size_t size) ALWAYS_INLINE;
static inline debug_t *
debug_object_allocate(void * func, size_t size)
{
    debug_t * info; 
    
    if (RARELY(track_objects(&state))) {
        if (!DEBUG_OBJECTS_USE_SEPERATE_ALLOCATOR)
            info = _anr_core_allocate(state.mstate, 
                                        sizeof(*info), 
                                        __builtin_return_address(0)); 
        else 
            info = malloc (sizeof (*info));

        if (USUALLY(info!=NULL)) {
            pid_t tid = (pid_t) pthread_getspecific (state.tid_key);
            
            if (RARELY(tid == 0)) {
                tid = syscall (SYS_gettid);
                pthread_setspecific (state.tid_key, (void *) tid);
            }
    

            info->function = func;
            info->malloccount = _anr_core_event_count(state.mstate);
            info->tid = tid;
            info->size = size;
            _anr_core_mark(state.mstate, info);

            info = set_debug_object(info);
        }
    } else 
        info = (debug_t *) func;


    return info;
}

static inline void 
debug_object_free(void * ptr, void * func)
{
    debug_t * info;
    if (USUALLY(!track_objects(&state)) || RARELY(ptr == NULL)) return;
    if ( !_anr_core_valid_pointer (state.mstate, ptr)) return;
    info = *((debug_t **) usr_to_core(ptr));
    if (DEBUG_OBJECTS_USE_SEPERATE_ALLOCATOR)
        free (debug_object(info));
    else
        _anr_core_free (state.mstate, debug_object(info), func);
}

/****************************************************************************
 * Entry Points
 ***************************************************************************/
void * 
anr_malloc(size_t size)
{
    void * ptr;
    size += state.overhead;
    ptr = _anr_core_allocate (state.mstate, size, __builtin_return_address(0));
    
    if(RARELY(ptr == NULL)) return ptr;

    *((debug_t **)ptr) = debug_object_allocate( __builtin_return_address(0), size);

    return core_to_usr(ptr);
}
void * 
anr_malloc_with_return(size_t size, void * addr)
{
    void * ptr;
    size += state.overhead;
    ptr = _anr_core_allocate (state.mstate, size, addr);
    
    if(RARELY(ptr == NULL)) return ptr;

    *((debug_t **)ptr) = debug_object_allocate(addr, size);

    return core_to_usr(ptr);
}
void * 
anr_malloc_if_available(size_t size)
{
    void * ptr;
    size += state.overhead;
    _anr_core_lock_heap(state.mstate);
    disable_giveback(&state);
    disable_error(&state);
    ptr = _anr_core_allocate( state.mstate, size, __builtin_return_address(0));
    enable_giveback(&state);
    enable_error(&state);
    _anr_core_unlock_heap(state.mstate);

    if (RARELY(ptr == NULL)) return NULL;

    *((debug_t **)ptr) = debug_object_allocate( __builtin_return_address(0), size);

    return core_to_usr(ptr);
}
void * 
anr_malloc_if_available_with_return(size_t size, void * addr)
{
    void * ptr;
    size += state.overhead;
    _anr_core_lock_heap(state.mstate);
    disable_giveback(&state);
    disable_error(&state);
    ptr = _anr_core_allocate( state.mstate, size, addr);
    enable_giveback(&state);
    enable_error(&state);
    _anr_core_unlock_heap(state.mstate);

    if (RARELY(ptr == NULL)) return NULL;

    *((debug_t **)ptr) = debug_object_allocate(addr, size);

    return core_to_usr(ptr);
}
void *
anr_malloc_if_possible(size_t size)
{
    void * ptr;
    size += state.overhead;
    _anr_core_lock_heap(state.mstate);
    disable_error(&state);
    ptr = _anr_core_allocate( state.mstate, size, __builtin_return_address(0));

    enable_error(&state);
    _anr_core_unlock_heap(state.mstate);

    if (RARELY(ptr == NULL)) return NULL;

    *((debug_t **)ptr) = debug_object_allocate( __builtin_return_address(0), size);

    return core_to_usr(ptr);
}
void *
anr_malloc_if_possible_with_return(size_t size, void * addr)
{
    void * ptr;
    size += state.overhead;
    _anr_core_lock_heap(state.mstate);
    disable_error(&state);
    ptr = _anr_core_allocate( state.mstate, size, addr);

    enable_error(&state);
    _anr_core_unlock_heap(state.mstate);

    if (RARELY(ptr == NULL)) return NULL;

    *((debug_t **)ptr) = debug_object_allocate (addr, size);

    return core_to_usr(ptr);
}
void * 
anr_realloc(void * ptr, size_t size)
{
    void * ret_ptr;
    size += state.overhead;

    if(RARELY(ptr == NULL)){
        ret_ptr = _anr_core_allocate(state.mstate, size, __builtin_return_address(0));

    } else if (RARELY( size == state.overhead)){
        debug_object_free (ptr , __builtin_return_address(0));

        _anr_core_free(state.mstate, 
                       usr_to_core(ptr), 
                       __builtin_return_address(0));
        return NULL;
    } else {
        debug_object_free (ptr, __builtin_return_address(0));
        ret_ptr = _anr_core_realloc( state.mstate, 
                                     usr_to_core(ptr), 
                                     size , 
                                     __builtin_return_address(0));
    }

    if (RARELY(ret_ptr == NULL)) return NULL;

    *((debug_t **)ret_ptr) = debug_object_allocate (__builtin_return_address(0), size);

    return core_to_usr(ret_ptr);
}

void * 
anr_realloc_with_return(void * ptr, size_t size, void * addr)
{
    void * ret_ptr;
    size += state.overhead;

    if(RARELY(ptr == NULL)){
        ret_ptr = _anr_core_allocate(state.mstate, size, addr);

    } else if (RARELY( size == state.overhead)){
        debug_object_free (ptr, addr);

        _anr_core_free(state.mstate, 
                       usr_to_core(ptr), 
                       addr);
        return NULL;
    } else {
        debug_object_free (ptr, addr);
        ret_ptr = _anr_core_realloc( state.mstate, 
                                     usr_to_core(ptr), 
                                     size , 
                                     addr);
    }

    if (RARELY(ret_ptr == NULL)) return NULL;

    *((debug_t **)ret_ptr) = debug_object_allocate (addr, size);

    return core_to_usr(ret_ptr);
}
void * 
anr_calloc(size_t n_elem, size_t elem_size)
{
    void * ptr;

    /* TODO test for overflow */
    size_t alloc_size = n_elem * elem_size;
    alloc_size += state.overhead;
    ptr = _anr_core_allocate( state.mstate, alloc_size, __builtin_return_address(0));
    if (USUALLY(ptr != NULL)) {
        *((debug_t **)ptr) = debug_object_allocate (__builtin_return_address(0), alloc_size);
        memset ( core_to_usr(ptr), 0, alloc_size - state.overhead ); 
        return core_to_usr(ptr); 
    }
    return NULL;
}
void * 
anr_calloc_with_return(size_t n_elem, size_t elem_size, void * addr)
{
    void * ptr;

    /* TODO test for overflow */
    size_t alloc_size = n_elem * elem_size;
    alloc_size += state.overhead;
    ptr = _anr_core_allocate( state.mstate, alloc_size, addr);
    if (USUALLY(ptr != NULL)) {
        *((debug_t **)ptr) = debug_object_allocate (addr, alloc_size);
        memset ( core_to_usr(ptr), 0, alloc_size - state.overhead ); 
        return core_to_usr(ptr); 
    }
    return NULL;
}

void
anr_free(void * ptr)
{
    if(RARELY(ptr == NULL)) return;
    debug_object_free (ptr, __builtin_return_address(0));
    _anr_core_free(state.mstate, 
                   usr_to_core(ptr), 
                   __builtin_return_address(0));
    return;
}
void
anr_free_with_return(void * ptr, void * caller)
{
    if(RARELY(ptr == NULL)) return;
    debug_object_free (ptr, caller);
    _anr_core_free(state.mstate, 
                   usr_to_core(ptr), 
                   caller);
    return;
}

void
anr_malloc_mark(void * ptr)
{
    if (RARELY(ptr == NULL)) return;

    _anr_core_mark(state.mstate, usr_to_core(ptr));

    if (RARELY(track_objects(&state)) && _anr_core_valid_pointer (state.mstate, ptr))
        _anr_core_mark(state.mstate, 
                       debug_object(*((debug_t **)(usr_to_core(ptr)))));
}

static int
report_one(int fd, void * ptr, size_t size, bool treat_as_leak)
{

    int ret = 0;
    if (track_objects(&state) && report_leaks(&state) && !teardown(&state)){  
        debug_t * info = *((debug_t **) ptr);
        if (is_debug_object(info)) {
            info = debug_object(info);
            ret = malloc_printf (fd, "pointer=%p size=%d location=%p count=%d tid=%d requested size=%d\n", 
                                 core_to_usr(ptr), size, 
                                 info->function, 
                                 info->malloccount, 
                                 info->tid, 
                                 info->size);
            if (treat_as_leak)
                debug_object_free (core_to_usr(ptr), &report_one);
        } else if (info) {
            void * caller = *((void **)ptr);

            if(caller)
                ret = malloc_printf (fd, "pointer=%p size=%d location=%p\n", 
                                     ptr, 
                                     size, 
                                     caller);
        } else
            ret = malloc_printf (fd, "No debug struct for %p\n", ptr);
    } else {
            void * caller = *((void **)ptr);

            if(caller)
                ret = malloc_printf (fd, "Memory Leak: %p:size=%d\n", caller, 
                                     size);
    }
    return ret;
}

static int
report(int fd, void * ptr, size_t size)
{
    return report_one (fd, ptr, size, true);
}

static int
interactive_report(int fd, void * ptr, size_t size)
{
    return report_one (fd, ptr, size, false);
}

unsigned int
anr_malloc_reclaim(void)
{
    int ret;
    enable_leak_report(&state);
    ret = _anr_core_reclaim(state.mstate, (ReportFunction)report);
    disable_leak_report(&state);
    return ret;
}

bool
anr_malloc_get_error()
{
    return _anr_core_get_error(state.mstate);
}

void
anr_malloc_set_error(bool error)
{
    _anr_core_set_error(state.mstate, error);
}

void
anr_pointer_info(void * ptr)
{
    _anr_core_pointer_info(state.mstate, usr_to_core(ptr));
    if(_anr_core_valid_pointer(state.mstate, usr_to_core(ptr))){
        enable_leak_report(&state);
        interactive_report(2, usr_to_core(ptr), _anr_core_usable_size(state.mstate, usr_to_core(ptr)));
        disable_leak_report(&state);
    }
}
int
anr_malloc_verify(void)
{
    int ret = 0;
    if (_anr_core_trylock_heap (state.mstate)) {
        ret = _anr_core_verify (state.mstate);
        _anr_core_unlock_heap (state.mstate);
    } else {
        malloc_printf (fileno(stdout), "Heap locked");
    }

    return ret;

}
unsigned int
anr_malloc_free_bytes(void)
{
    return _anr_core_free_bytes(state.mstate);
}
unsigned int
anr_malloc_total_bytes(void)
{
    return _anr_core_total_bytes(state.mstate);
}
size_t
anr_malloc_usable_size(void * ptr)
{
    size_t size = _anr_core_usable_size (state.mstate, usr_to_core(ptr));
    return size - state.overhead;
}

#ifdef HAVE_FRAME_POINTERS
static inline void 
dump_traceback(traceback_t * traceback, int fd)
{
    int i;

    malloc_printf (fd, "PID: %d\tTID: %d\tCount: %d\n", 
                   traceback->pid, 
                   traceback->tid, 
                   traceback->count);
    /* the following two calls should be functionally equilavent.  The only 
     * question remaining is if backtrace_symbols_fd calls malloc()
     */
#if 0
    
    backtrace_symbols_fd (traceback->backtrace, traceback->size, fd);
#else
    for (i = 0; i< traceback->size; i++) {
        malloc_printf (fd, "(%02d) %p\n", i, traceback->backtrace[i]);
    }
#endif

}
#endif


int
anr_malloc_report(FILE * fp, const char * params)
{
    int ret = _anr_core_report(state.mstate, fileno(fp), params);

#ifdef HAVE_FRAME_POINTERS
    if (primary_traceback_captured(&state)) {
        malloc_printf (fileno(fp), "Primary Traceback\n");
        dump_traceback (&state.failures[0], fileno(fp));
        fflush (fp);
    }

    if (secondary_traceback_captured(&state)) {
        malloc_printf (fileno(fp), "Secondary Traceback\n");
        dump_traceback (&state.failures[1], fileno(fp)); 
        fflush (fp);
    }
#endif

    return ret;
}
#ifndef HAVE_FRAME_POINTERS
#define capture_call_stack()
#else
static inline void
capture_call_stack(void)
{
    traceback_t  * traceback;

    if (!primary_traceback_captured(&state)) {
        traceback = &state.failures[0];
        latch_primary_traceback(&state);
    } else {
        traceback = &state.failures[1];
        latch_secondary_traceback(&state);
    }

    traceback->size = backtrace (traceback->backtrace, 
                      sizeof(traceback->backtrace)/sizeof(traceback->backtrace[0]));

    traceback->pid = getpid ();      
    traceback->tid = syscall (SYS_gettid);
    traceback->count = _anr_core_event_count (state.mstate);
}
#endif
static unsigned int
_anr_giveback( void * context, unsigned int attempt, unsigned int size)
{
    unsigned int reclaimed;
    (void) context;

    if (RARELY(!giveback_ok(&state) || performing_giveback(&state)))
        return 0;
    else {
        start_giveback(&state);

        reclaimed = state._more_mem(state.context, attempt, size);
        if (RARELY(reclaimed == 0
            && attempt >= MORE_MEM_RETRIES 
            && !_anr_core_get_error (state.mstate)))

        {

            capture_call_stack ();

            if (error_ok(&state)) {
                if (memfull_report(&state)) { 
                    char dump_path[100];
                    snprintf (dump_path, sizeof(dump_path), "/tmp/memfull.%d-%ld", getpid (), syscall (SYS_gettid));
                    anr_malloc_dump_path (dump_path);
                    malloc_printf (fileno (stdout), "Wrote heam memfull report to %s", dump_path);
                }
                
                _anr_core_set_error(state.mstate, true);
                if(state._mem_full)
                    state._mem_full(anr_malloc_free_bytes(),
                                    anr_malloc_total_bytes(),
                                    size, 
                                    _anr_core_event_count(state.mstate));
            }
        }

        end_giveback(&state);
    }

    return reclaimed;
}

int 
anr_malloc_init(unsigned int poolsize,
                unsigned int mapsize,
                MoreMemoryFunction gb_func, void * gb_context,
                MemoryFullFunction mf_func, AbortFunction abort_func)
{

    unsigned int slab_count = 0;
    unsigned int slabs[100];
    unsigned int dbg_words = 1;  /* we always have 1 debug word */
    unsigned int disable_slabs = 0;

    unsigned int i;
    int dirty_page_ceiling = 1024;
    int flags = 0;
    ParseConfig parser;

    memset (&state, 0, sizeof (state));
    memset (slabs, 0, sizeof (int) * sizeof (slabs)/sizeof (slabs[0]));
    memset (&parser, 0, sizeof (ParseConfig));

    if (pthread_key_create (&state.tid_key, NULL)) {
        if (abort_func) {
            abort_func (__FILE__, __LINE__, "Unable to create thread specific key");
        } else {
            abort ();
        }
    }


    /* this may seem a bit odd.  the parser simply reads all data into a
     * buffer, and will have all data in the same buffer, by maintaining offset
     * data between calls to init When requesting a value, the parser bails out
     * at the first found value.  So, anrmalloc.cfg takes precidence over
     * /var/fs/shared/anrmalloc.cfg which takes precidence over
     * /etc/anrmalloc.cfg.
     *
     * For simulation, we expect anrmalloc.cfg to exist in the current
     * directory.  (Perhaps I should get the default install path?)
     *
     * On hardware, we get the default from /etc.  If a user wishes to override
     * settings, they can rcp their anrmalloc.cfg to the printer at
     * /var/fs/shared/, which will persist across POR.  They will, of course,
     * have to POR the system to get the new configuration.
     */

    if (0 == parser_init(&parser, "anrmalloc.cfg") || 
       0 == parser_init(&parser, "/var/fs/shared/anrmalloc.cfg") || 
       0 == parser_init(&parser, "/etc/anrmalloc.cfg"))
    {
        int set;
        slab_count = parser_get_val_array (&parser, "SLAB_SIZES", 100, slabs);
        dbg_words += parser_get_val (&parser, "EXTRA_DEBUG_WORDS");
        
        set = parser_get_val (&parser, "PAGE_CEILING");
        if (set) 
            dirty_page_ceiling = set;
   
        if (1 == parser_get_val (&parser, "FILL_WITH_TRASH"))
            flags |= FILL_WITH_TRASH;
        if (1 == parser_get_val (&parser, "BOUNDS_CHECK"))
            flags |= BOUNDS_CHECK;
        if (1 == parser_get_val (&parser, "VERIFY"))
            flags |= MALLOC_VERIFY;
        if (1 == parser_get_val (&parser, "TRACK_OBJECTS"))
            enable_track_objects(&state);
        
        if (1 == parser_get_val (&parser, "MEMFULL_REPORT"))
            enable_memfull_report(&state);

        disable_slabs = parser_get_val (&parser, "DISABLE_SLABS");
    } 
    
    if (disable_slabs) {
        slab_count = 0;
    } else if (slab_count == 0 ) {
        slabs[0] = 36;
        slabs[1] = 16;
        slabs[2] = 64;
        slabs[3] = 96;
        slabs[4] = 20;
        slabs[5] = 128;
        slabs[6] = 256;
        slabs[7] = 32;
        slabs[8] = 48;
        slabs[9] = 168;
        slabs[10] = 200;
        slabs[11] = 100;
        slab_count = 12;
    }

    for(i =0; i< slab_count; i++)
        slabs[i] += BYTES_PER_WORD;

    state._mem_full = mf_func;
    state._more_mem = gb_func;
    state.context = gb_context;
    state.overhead = dbg_words * BYTES_PER_WORD;

    return _anr_core_init (&(state.mstate), 
                          flags,
                          poolsize, 
                          mapsize, 
                          dirty_page_ceiling, 
                          slab_count, 
                          slabs,
                          _anr_giveback, 
                          &state, 
                          abort_func);

}
void
anr_malloc_teardown()
{
    enable_leak_report(&state);
    enable_teardown(&state);
    malloc_printf (fileno(stdout), "Tearing down the heap -- leak report may be innaccurate\n");
    _anr_core_reclaim (state.mstate, (ReportFunction)report);
    disable_leak_report(&state);
    _anr_core_teardown(state.mstate);
}
int
anr_malloc_dump(FILE * fp, const char * args)
{
    enable_leak_report(&state);
    _anr_core_dump(state.mstate, fileno(fp), interactive_report);
    disable_leak_report(&state);

    (void) args;
    return 0;
}
void
anr_malloc_dump_path(const char * file)
{
    int fd = creat (file, S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH);

    if (fd < 0) {
        malloc_printf (fileno (stderr), "Unable to open file %s: %s\n", file, strerror (errno));
        return;
    }

    enable_leak_report(&state);
    _anr_core_dump (state.mstate, fd, interactive_report);
    disable_leak_report(&state);
    
    close (fd);

    return;
}


unsigned int
anr_malloc_expand(unsigned int pages)
{
    return _anr_core_expand(state.mstate, pages);
}
unsigned int
anr_malloc_shrink(unsigned int pages)
{
    return _anr_core_shrink(state.mstate, pages);
}
void 
anr_malloc_block_until_free(void)
{
    _anr_core_block_until_free(state.mstate);
}
void 
anr_malloc_report_leaks(bool report)
{
    if(report)
        enable_leak_report(&state);
    else
        disable_leak_report(&state);

}
void
anr_malloc_lock_heap(void)
{
    _anr_core_lock_heap(state.mstate);
}
/**
 * Attempt to lock the heap, with an optional timeout.
 * @param rel_timeout_usec  If 0, never wait, return immediately.
 *                          If nonzero, wait at most that many microseconds
 *                          for the heap lock.
 * @return True if we successfully took the lock, false if not.
 */
bool
anr_malloc_trylock_heap(unsigned int rel_timeout_usec)
{
    return _anr_core_timedlock_heap (state.mstate, rel_timeout_usec);
}
void
anr_malloc_unlock_heap(void)
{
    _anr_core_unlock_heap(state.mstate);
}

bool
anr_malloc_out_of_memory_is_error(void)
{
    return (bool) error_ok(&state);
}

bool
anr_malloc_set_out_of_memory_is_error(bool is_error)
{
    bool old_value = (bool) error_ok (&state);
    if (is_error) {
        enable_error (&state);
    } else {
        disable_error (&state);
    }
    return old_value;
}


#ifdef BUILD_TESTS

#define UNIT_TEST(name)\
    void __attribute__ ((section (".unit_tests")))\
     __unit_test_ ## name (void)

#define UNIT_TEST_HELPER __attribute__ ((section (".unit_tests")))

UNIT_TEST(anr_realloc_api)
{
    static const int POOLSIZE = 16 * 1024 * 1024;
    void * ptr;

    printf ("Begin anr_realloc_api\n");
    anr_malloc_init( POOLSIZE, POOLSIZE <<1, NULL,NULL, NULL, NULL);

    ptr = anr_realloc(NULL, 100);
    assert(ptr != NULL);

    ptr = anr_realloc(ptr, 0);

    assert(ptr == NULL);

    printf ("End anr_realloc_api\n");
    return;
}
static unsigned int UNIT_TEST_HELPER
simple_moremem(void * context, unsigned attempt, unsigned size)
{
    ((void)context);
    ((void)attempt);
    ((void) size);    
    return 0;
}
UNIT_TEST(anr_memfull_callstack)
{
    static const int POOLSIZE= 8 * 1024 * 1024;
    void * ptr;

    malloc_printf (fileno (stdout), "Begin memfull_callstack\n");
    
    anr_malloc_init( POOLSIZE, POOLSIZE <<1, simple_moremem, NULL, NULL, NULL);

    ptr = anr_malloc (POOLSIZE <<1);

    assert (ptr == NULL);
    assert(anr_malloc_get_error ());
    anr_malloc_set_error (false);

    ptr = anr_malloc (POOLSIZE << 1);

    assert(ptr == NULL);

    anr_malloc_report (stdout, NULL);

    malloc_printf (fileno (stdout), "End memfull_callstack\n");

    return;
}

#endif
