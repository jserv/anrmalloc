/* -*- mode: c; indent-tabs-mode: nil; c-basic-offset: 4; -*- */
/* vim: set expandtab shiftwidth=4 softtabstop=4 : */

#ifndef __ANRMALLOC_H__
#define __ANRMALLOC_H__

#include <stddef.h>
#include <stdbool.h>
#include <stdio.h> 
#ifdef __cplusplus
extern "C" {
#endif

#ifndef __ANR_CALLBACKS_H__
#define __ANR_CALLBACKS_H__
typedef void (*MessageFunction)(const char *, const char *, const char *, const char *);
typedef unsigned int (*MoreMemoryFunction)(void * context, unsigned int, unsigned int );
typedef void (*MemoryFullFunction)(unsigned, unsigned, unsigned, unsigned);
typedef void (*AbortFunction)(const char *, int, char *);
#endif

int anr_malloc_init(unsigned int poolsize, unsigned int mapsize, 
                MoreMemoryFunction gb_func, void * gb_context,
                MemoryFullFunction mem_full, AbortFunction abort_func);

int anr_malloc_default_init(void);
void anr_malloc_teardown(void);

/* Standard memory management entry points */
void * anr_malloc (size_t size);
void * anr_malloc_if_possible (size_t size);
void * anr_malloc_if_available (size_t size);
void * anr_realloc (void * ptr, size_t size);
void * anr_calloc (size_t n_elements, size_t elem_size);
void anr_free (void * ptr);

/* The entry point for malloc like functions */
void * anr_malloc_with_return (size_t size, void * caller);
void * anr_malloc_if_possible_with_return (size_t size, void * caller);
void * anr_malloc_if_available_with_return (size_t size, void * caller);
void * anr_realloc_with_return (void * ptr, size_t size, void * caller);
void * anr_calloc_with_return (size_t n_elements, size_t elem_size, void * caller);
void anr_free_with_return (void * ptr, void * caller);

void anr_malloc_mark (void * ptr);

bool anr_malloc_get_error();
void anr_malloc_set_error(bool error);
unsigned int anr_malloc_reclaim(void);
unsigned anr_malloc_total_bytes(void);
unsigned anr_malloc_free_bytes(void);
void anr_malloc_block_until_free(void);
size_t anr_malloc_usable_size(void *);

int anr_malloc_verify(void);
void anr_pointer_info( void * ptr );
int anr_malloc_report(FILE *, const char *);
int anr_malloc_dump(FILE *, const char *);
void anr_malloc_dump_path(const char * file);
unsigned int anr_malloc_expand(unsigned int pages);
unsigned int anr_malloc_shrink(unsigned int pages);
void anr_malloc_block_until_free(void);
void anr_malloc_report_leaks(bool);
void anr_malloc_lock_heap(void);
void anr_malloc_unlock_heap(void);
bool anr_malloc_out_of_memory_is_error(void);
bool anr_malloc_set_out_of_memory_is_error(bool is_error);

#ifdef __cplusplus
}
#endif

#endif
