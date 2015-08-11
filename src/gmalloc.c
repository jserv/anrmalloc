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
 * Allocation code for glibc clients
 */

#include "anrcore.h"
#include "parse_config.h"
#include "gmalloc.h"
#include <errno.h>
#include <assert.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h> /* EXEC_PAGESIZE */
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>

#if HAVE_MEMBROKER
#include <membroker/mbclient.h>
#else
#define mb_register(a) 
#define mb_request_pages(pages) 0
#define mb_reserve_pages(pages) 0 
#define mb_return_pages(pages) do { } while (0)
#endif

#if HAVE_FRAME_POINTERS
#include <execinfo.h>
#endif


struct g_state{
    malloc_state_t * mstate;
    uint32_t overhead;
    uint32_t atfork_locks;
    int page_ceiling;
    void * context;
    uint32_t state;
    int dumped;
    int svc_err_on_oom;
    int dump_debug_on_oom;
    int use_membroker;
    char dump_file[PARSE_CONFIG_BUFFER_SIZE];
    pthread_key_t tid_key;
    pthread_mutex_t atfork;
    pthread_mutex_t lock;
    MoreMemoryFunction _more_mem;
    ReturnMemoryFunction _less_mem;
};

static struct g_state state;
static pthread_once_t init = PTHREAD_ONCE_INIT;

typedef void * (*MallocFunction)(size_t size, void * function);
typedef void (*DebugHook) (void *, void *);

static MallocFunction __malloc_hook;
static DebugHook __debug_hook;

static void * __malloc(size_t size, void * function);
static void * __malloc_atfork(size_t size, void * function);
static void gmalloc_out_of_memory(size_t size, void * function);

#define encode_debug(ptr) __debug_hook(ptr, __builtin_return_address(0))

static void
__no_op(void * p, void * a)
{
    uintptr_t * ptr = (uintptr_t *) p;
    uint32_t i;
    ptr += (_anr_core_usable_size (state.mstate, p) - state.overhead) / BYTES_PER_WORD;

    /* we're nulling out the debug words here */
    for (i = 0; i< state.overhead/BYTES_PER_WORD; i++) {
        ptr[i] = 0;
    }

    (void) a;
}


static void
__encode_caller (void * p, void * addr)
{

    uintptr_t * ptr = (uintptr_t *) p;
    uint32_t i;
    ptr += (_anr_core_usable_size (state.mstate, p) - state.overhead) / BYTES_PER_WORD;
    ptr[0] = (uintptr_t) addr;
    /* we want to make sure that all extra debug words are nulled out */
    for (i= 1; i< state.overhead/BYTES_PER_WORD; i++)
        ptr[i] = 0;

}


#if HAVE_FRAME_POINTERS
static void
__encode_backtrace (void * p, void * addr)
{

    uintptr_t * ptr = (uintptr_t *) p;

    ptr += (_anr_core_usable_size (state.mstate, p) - state.overhead) / BYTES_PER_WORD;

    if (state.overhead == 4) {
        ptr[0] = (uintptr_t) addr;
    } else {
        int num_frames = 0;
        int debug_words = state.overhead/BYTES_PER_WORD;

        pid_t tid = (pid_t) pthread_getspecific (state.tid_key);

        if (RARELY(0 == tid)) {
            tid = syscall (SYS_gettid);
            pthread_setspecific (state.tid_key, (void *) tid);
        }

#if HAVE_FRAME_POINTERS
        /* We start the backtrace one level back into buffer we're returning
         * to the caller, because we don't want the first frame returned.
         * We have to preserve this word for the realloc(smaller) case.
         * The alternative using a temp buffer and copying the result (yuck) */
        {
            uintptr_t temp;
            temp = ptr[-1];
            num_frames = backtrace ((void **) &ptr[-1], debug_words + 1);
            ptr[-1] = temp;
            num_frames--;  // ignore that one in caller's buffer
        }
#endif
        /* Backtrace leaves rest of my buffer uninitialized if stack was
         * too shallow to fill it, so zero it here */
        if(num_frames < debug_words) {
            memset (&ptr[num_frames], 0, (debug_words - num_frames) * BYTES_PER_WORD);
        }

        /* We're intentionally overwriting the second two words returned by
         * backtrace(). The first is an uninteresting internal allocator
         * function, and the second we overwrite with 
         * builtin_return_address(0), which should match it but is
         * generally more reliable. */
        ptr[0] = (uintptr_t) tid;
        ptr[1] = (uintptr_t) addr;
    }


}
#endif

static void
decode_debug (void * p, pid_t  * tid, uint32_t * bt_size, void ** backtrace)
{
    uintptr_t * ptr = (uintptr_t *) p;

    if (__debug_hook == __no_op)
        return;

    ptr += (_anr_core_usable_size (state.mstate, p) - state.overhead) / BYTES_PER_WORD;

    if (state.overhead == 4 || __debug_hook == __encode_caller) {
        *backtrace = (void *) &ptr[0];
        *bt_size = 1;
        *tid = 0;
    } else {
        *tid = (pid_t) ptr[0];
        *backtrace = (void *)&ptr[1];
        *bt_size = state.overhead/BYTES_PER_WORD -1;
    }

    return;
}

#define __alloc(size) \
    _anr_core_allocate (state.mstate, size + state.overhead,\
                        __builtin_return_address(0))

#define __alloc_function(size, function)\
    _anr_core_allocate (state.mstate, size + state.overhead,\
                        function)

#define __free(ptr) \
    _anr_core_free (state.mstate, ptr,\
                    __builtin_return_address(0))

#define UNUSED __attribute__((unused))


#define max(a, b) a > b ? a:b

#define ONE_MB 1024 * 1024
#define MB_MASK (ONE_MB - 1)

#define nearest_mb(size) ((size + MB_MASK) &~MB_MASK)
#define nearest_page(size) ((((size) + PAGE_MASK) & ~PAGE_MASK)/EXEC_PAGESIZE)

static void
dumpdata(int sig UNUSED)
{
    state.state = 1;
    return;
}


static unsigned int
more_memory(void * context, unsigned int attempt, unsigned int size)
{
    /* TBD: talk to the rest of the system, grow as needed */
    unsigned int pages = 0;
    struct g_state * self = (struct g_state *) context;
    if (state._more_mem) {

        size = max(1, nearest_page(size));
        pages = state._more_mem (self->context, attempt, size);
        _anr_core_expand (state.mstate, pages);

        return pages * EXEC_PAGESIZE;
    } else {
        unsigned int mapsize = _anr_core_total_bytes(state.mstate);
        mapsize += _anr_core_mapping_overhead(mapsize);
        mapsize = max( mapsize, nearest_mb(size));
        malloc_printf(fileno(stdout), "Attempting to add %f MB to the heap\n", (float)(((float)mapsize)/1024.0/1024.0));
        _anr_core_add_mapping(state.mstate, mapsize, mapsize << 1 );
        return 1;
    }
    return 0;
}

static unsigned int
membroker_more_memory(void * context UNUSED, unsigned int attempt UNUSED, unsigned int size)
{
    int pages = mb_request_pages (nearest_page(size));
    
    if (pages < 0) {
        _anr_core_set_error(state.mstate, true);
        return 0;
    }

    _anr_core_expand (state.mstate, pages);

    return pages * EXEC_PAGESIZE;

    ((void)size);

}

static void
prefork(void)
{
    int ret;
    ret = pthread_mutex_trylock (&state.atfork);

    if (ret == EDEADLK) { /* this thread owns the lock, no problem */
        state.atfork_locks++;
        return;
    } else if (ret == EBUSY) { /* some other thread got here first.  we have to wait */
        pthread_mutex_lock( &state.atfork);
    } 
    _anr_core_prefork_prepare(state.mstate);
    __malloc_hook = (MallocFunction)__malloc_atfork;
    state.atfork_locks++;
    return;
}
static void
postfork_parent(void)
{
    state.atfork_locks--;
    if (state.atfork_locks != 0)
        return;

    __malloc_hook = __malloc;
    _anr_core_postfork(state.mstate);
    pthread_mutex_unlock(&state.atfork);
    
}
static void
postfork_child(void)
{
    __malloc_hook = __malloc;

    _anr_core_postfork_child (state.mstate);
    pthread_mutex_init (&state.atfork, NULL);
    pthread_mutex_init (&state.lock, NULL);
    state.atfork_locks = 0;
    state._more_mem = 0;
    state._less_mem = 0;
}

/*
 * Set the address space for the heap to be larger than the amount of
 * memory we actually have.  Anr will make this much address space
 * available, but will not use more than the given amount of memory,
 * which allows it to avoid a majority of fragmentation problems.
 *
 * However, on systems with a lot of memory in the heap, we may not
 * actually be able to map enough address space to cover twice the
 * amount of memory we can use.  So, we play around a bit with the
 * mappings to fit.
 */
static unsigned int calculate_map_size(unsigned int size)
{
    unsigned int map_size = 0;
    int i;

    /* i = 0 -> map_size = 2 * size
     * i = 1 -> map_size = 1.5 * size
     * i = 2 -> map_size = 1.25 * size
     * i = 3 -> map_size = 1.125 * size
     * i = 4 -> map_size = 1.0625 * size
     */
    for (i = 0; i < 5 && map_size < size; i++){
        map_size = size + (size >> i);
    }

    if (map_size < size)
        map_size = size;

    return map_size;
}

static int
gmalloc_init(void)
{
    unsigned int poolsize = 0;
    unsigned int init_poolsize = 0;
    unsigned int mapsize = 0;
    unsigned int expandable_poolsize = 0;
    unsigned int no_autorequest_memory = 0;
    unsigned int slab_count = 0;
    unsigned int slabs[100];
    unsigned int dbg_words=0;
    unsigned int i;
    unsigned int disable_slabs = 0;
    unsigned int fill_with_trash = 0;
    bool is_parser_ok = false;
    const char* config_file_path = 0;
    int page_ceiling = 0;
    ParseConfig parser;
    MoreMemoryFunction more_mem_callback = more_memory;
    char * use_membroker;
    int membroker_pages = 0;

    memset (&state, 0, sizeof(state));
    memset (slabs, 0, sizeof(int) * sizeof(slabs)/sizeof(slabs[0]));
    memset (&parser, 0, sizeof (ParseConfig));

    pthread_mutex_init (&state.lock, NULL);
    pthread_mutex_init (&state.atfork, NULL);
    __malloc_hook = __malloc;
    __debug_hook = __no_op;
    pthread_atfork (prefork, postfork_parent, postfork_child);

    if (pthread_key_create (&state.tid_key, NULL)) {
        abort ();
    }


    if (getenv ("MEMORY_DEBUG") && strcmp ("1", getenv ("MEMORY_DEBUG")) == 0)
        signal (SIGUSR2, dumpdata);

    config_file_path = getenv("GMALLOC_CFG");
    if (config_file_path) {
        is_parser_ok = (0 == parser_init(&parser, config_file_path));
        if (!is_parser_ok) {
            anr_crash("Given config file path (%s) is invalid", config_file_path);
        }
    }

    is_parser_ok = (is_parser_ok ||
                    0 == parser_init(&parser, "/var/fs/shared/gmalloc.cfg") ||
                    0 == parser_init(&parser, "/etc/gmalloc.cfg") ||
                    0 == parser_init(&parser, "gmalloc.cfg"));

    if (is_parser_ok) {
        slab_count = parser_get_val_array(&parser, "SLAB_SIZES", 100, slabs);
        dbg_words = parser_get_val(&parser, "EXTRA_DEBUG_WORDS");
        poolsize = parser_get_val(&parser, "POOL_SIZE");
        mapsize = parser_get_val(&parser, "MAP_SIZE");
        page_ceiling = parser_get_val(&parser, "PAGE_CEILING");
        disable_slabs = parser_get_val(&parser, "DISABLE_SLABS");
        fill_with_trash = parser_get_val(&parser, "FILL_WITH_TRASH");
        state.svc_err_on_oom = parser_get_val(&parser, "SVC_ERR_ON_OOM");
        state.dump_debug_on_oom = parser_get_val(&parser, "DUMP_DEBUG_ON_OOM");
        expandable_poolsize = parser_get_val(&parser, "EXPANDABLE_POOL_SIZE");
        no_autorequest_memory = parser_get_val(&parser, "NO_AUTOREQUEST_MEMORY");
        parser_get_string(&parser, "DUMP_FILE", sizeof(state.dump_file),
                          state.dump_file);
    } 

    parser_destroy(&parser);


    if (slab_count == 0 && !disable_slabs) {
        slabs[0] = 36;
        slabs[1] = 16;
        slabs[2] = 64;
        slabs[3] = 96;
        slabs[4] = 20;
        slabs[5] = 128;
        slabs[6] = 256;
        slabs[7] = 32;
        slab_count = 8;
    }

    if (mapsize == 0)
        mapsize = 5 * 1024 * 1024;

    if (poolsize == 0)
        poolsize = 2 * 1024 * 1024;
    init_poolsize = poolsize;

    if (page_ceiling == 0)
        page_ceiling = 128; /* 500k seems pretty low */

    state.overhead = 1 * BYTES_PER_WORD;
    state.overhead += dbg_words * BYTES_PER_WORD;
    state.page_ceiling = page_ceiling;

    use_membroker = getenv("USE_MEMBROKER");

    if (use_membroker && strcmp("1", use_membroker) == 0){
        state.use_membroker = 1;
        mb_register(0);
        more_mem_callback = membroker_more_memory;
    }

    if (state.use_membroker && expandable_poolsize) {
#if HAVE_MEMBROKER
        /* Wait for the membroker source to connect and give pages to
         * membroker. We will set poolsize to the total number of pages
         * then shrink it so that we could potentially grow to the
         * maximum memory size. */
        while ((membroker_pages = mb_query_total()) == 0)
            usleep(20 * 1000);

        poolsize = membroker_pages * EXEC_PAGESIZE;
        mapsize = calculate_map_size(poolsize);
#else
        anr_crash("Asked for membroker but support is compiled out");
#endif
    }

    /* need to account for the 1 word of overhead we put in to track return address */ 
    for (i = 0; i< slab_count; i++)
        slabs[i] += sizeof(word_t);

    if (no_autorequest_memory)
        more_mem_callback = NULL;

    _anr_core_init(&(state.mstate),
                   fill_with_trash ? FILL_WITH_TRASH : 0,
                   poolsize, 
                   mapsize, 
                   page_ceiling/2, 
                   slab_count, 
                   slabs,
                   more_mem_callback,
                   &state, 
                   NULL);

    if (state.use_membroker) {
        int pages = 0;
        if (expandable_poolsize)
            _anr_core_shrink(state.mstate,
                             membroker_pages - (init_poolsize / EXEC_PAGESIZE));

        pages = mb_reserve_pages(init_poolsize / EXEC_PAGESIZE);
        assert(pages);
    }

    return 0;
}

typedef void (*InitFunction)(void);

int
posix_memalign(void ** ptr, size_t alignment , size_t size)
{
    
    void * ret_ptr;
    pthread_once( &init, (InitFunction)gmalloc_init);

    if (RARELY(state.state)) {
        _anr_core_report(state.mstate, fileno(stdout), NULL);
        state.state = 0;
    }

    ret_ptr = _anr_core_memalign (state.mstate, alignment, size + state.overhead);

    if (RARELY(ret_ptr == NULL)){
        *ptr = NULL;
        gmalloc_out_of_memory(size, __builtin_return_address(0));
        return ENOMEM;
    }
    encode_debug(ret_ptr);

    *ptr = ret_ptr;
    return 0;
}

void *
malloc(size_t size)
{
    void * ptr;
    int pages;
   
    pthread_once( &init, (InitFunction)gmalloc_init);
    pthread_mutex_lock (&state.lock); 

    if (RARELY(state.state)) {
        _anr_core_report(state.mstate, fileno(stdout), NULL);
        state.state = 0;
    }

    ptr = __malloc_hook (size, __builtin_return_address(0));
    pages = _anr_core_available_pages (state.mstate);
    if (state._less_mem && state.page_ceiling && pages > state.page_ceiling) {
        pages = _anr_core_shrink( state.mstate, pages - state.page_ceiling);
        state._less_mem (state.context, pages);
    }

    if (RARELY(ptr == NULL)){
        gmalloc_out_of_memory(size, __builtin_return_address(0));
        errno = ENOMEM;
    } else
        encode_debug(ptr);

    pthread_mutex_unlock (&state.lock); 
    return ptr;
}

static void *
__malloc(size_t size, void * function)
{
    return __alloc_function(size, function);
}
static void *
__malloc_atfork(size_t size, void * function)
{
    int ret = pthread_mutex_trylock (&state.atfork);
    if (ret == EBUSY) {
        pthread_mutex_lock (&state.atfork);
        pthread_mutex_unlock (&state.atfork);
    } else if (ret ==  0)
        pthread_mutex_unlock (&state.atfork);

    return __alloc_function(size, function);
}
void *
calloc(size_t n_elem, size_t size)
{
    size_t alloc_size = n_elem * size;
    void * ptr;

    pthread_once (&init, (InitFunction)gmalloc_init);
    ptr = __malloc_hook(alloc_size, __builtin_return_address(0));

    if (RARELY(!ptr)){
        gmalloc_out_of_memory(size, __builtin_return_address(0));
        errno = ENOMEM;
    } else {
        encode_debug(ptr);
        memset (ptr, 0, alloc_size); 
    }
    return ptr;
}
void * 
realloc(void * ptr, size_t size)
{
    void * ret_ptr;

    pthread_once (&init, (InitFunction)gmalloc_init);

    if (RARELY( ptr == NULL)) {
        ret_ptr = __alloc(size);
    } else if (RARELY( size == 0 )){
        __free(ptr);
        return NULL;
    } else {
        ret_ptr = _anr_core_realloc (state.mstate, ptr,
                                     size + state.overhead, 
                                     __builtin_return_address(0));
    }

    if (RARELY(ret_ptr == NULL)){
        gmalloc_out_of_memory(size, __builtin_return_address(0));
        errno = ENOMEM;
        return NULL;
    }
    encode_debug(ret_ptr);
    return (ret_ptr);
}
void
free(void * ptr)
{
    int pages;
    
    if (RARELY( ptr == NULL )) return;

    pthread_once( &init, (InitFunction)gmalloc_init);
    pthread_mutex_lock (&state.lock);

    __free(ptr);
    pages = _anr_core_available_pages (state.mstate);
    if (state._less_mem && state.page_ceiling && pages > state.page_ceiling) {
        pages = _anr_core_shrink( state.mstate, pages - state.page_ceiling);
        state._less_mem (state.context, pages);
    }

    pthread_mutex_unlock (&state.lock);
    return;
}
static int 
report(int fd, void * ptr, size_t size)
{
    void * backtrace = NULL;
    pid_t tid = 0;
    uint32_t bt_size = 0;
    decode_debug (ptr, &tid, &bt_size, &backtrace);

    malloc_printf (fd, "pointer=%p size=%d tid=%d callstack=%p", ptr, size, tid, *(uintptr_t *)backtrace);

    if (bt_size > 1) {
        uint32_t i;
        for (i = 1; i < bt_size; i++) {
            malloc_printf (fd, ",%p", ((void**)backtrace)[i]);
        }
    }

    return malloc_printf (fd, "\n");
}

static void
post_svcerr(void)
{
    malloc_printf (fileno (stderr), "gmalloc exceeded POOL_SIZE limit\n");
    if (state.dump_debug_on_oom && state.dumped)
        malloc_printf (fileno (stderr), "Debug dumped to %s\n", state.dump_file);

    abort ();
}
// ToDo: Make actions taken here configurable via config file.
static void
gmalloc_out_of_memory(size_t size, void * function)
{
    malloc_printf(fileno(stderr),
        "** gmalloc: out_of_memory. Requested size: %lu, caller: %p **\n",
        (unsigned long)size, function);

    if(state.dump_debug_on_oom && !state.dumped){
        int fd;

        state.dumped = 1;
        fd = creat(state.dump_file, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
        if (fd != -1) {
            _anr_core_report(state.mstate, fd, NULL);
            _anr_core_dump(state.mstate, fd, report);
            fsync(fd);
            close(fd);
        }
    }

    if (state.svc_err_on_oom)
        post_svcerr();
}
void
gmalloc_dump(FILE * fp, const char * params)
{
    _anr_core_dump(state.mstate, fileno (fp), report);
    (void) params;
}
void
gmalloc_report(FILE * fp, const char * params)
{
    _anr_core_report (state.mstate, fileno(fp), params);
}
int
gmalloc_expand(int pages)
{
    int pages_received = 0;

    if (state.use_membroker) {
        pages_received = mb_request_pages (pages);

        if (pages_received < pages) {
            mb_return_pages(pages_received);
            return 0;
        }
    }

    pthread_mutex_lock (&state.lock);
    pages = _anr_core_expand (state.mstate, pages);
    pthread_mutex_unlock (&state.lock);

    if (pages_received > 0) {
        int unused_pages = pages_received - pages;
        if (unused_pages > 0) {
            /* membroker gave us more than we could use.  give that
             * overrage back. */
            mb_return_pages (unused_pages);
        }
    }

    return pages;
}
int 
gmalloc_shrink(int pages)
{
    int taken_pages;
    if (state.use_membroker) {
        mb_return_pages(pages);
    }
    pthread_mutex_lock (&state.lock);
    taken_pages = _anr_core_shrink(state.mstate, pages);
    pthread_mutex_unlock (&state.lock);
    return taken_pages;
}
void
gmalloc_set_callbacks(void * context, MoreMemoryFunction more_mem, ReturnMemoryFunction less_mem)
{
    pthread_once( &init, (InitFunction)gmalloc_init);

    pthread_mutex_lock (&state.lock);

    state._more_mem = more_mem;
    state._less_mem = less_mem;
    state.context = context;
    
    pthread_mutex_unlock (&state.lock);

}
unsigned int
gmalloc_total_bytes()
{
    return _anr_core_total_bytes(state.mstate);
}

unsigned int
gmalloc_free_bytes ()
{
    return _anr_core_free_bytes (state.mstate);
}

void
gmalloc_enable_debug ()
{
    pthread_once( &init, (InitFunction)gmalloc_init);
#if HAVE_FRAME_POINTERS
    if (state.overhead == 4)
        __debug_hook = __encode_caller;
    else {
        void * bt[2];
        backtrace ((void **)&bt, 1);
        __debug_hook = __encode_backtrace;
    }
#else
    __debug_hook = __encode_caller;
#endif
}
