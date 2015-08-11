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

#ifndef __G_MALLOC_H__
#define __G_MALLOC_H__

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

typedef int (*ReturnMemoryFunction)(void * context, unsigned int pages);
typedef void (*SetCallbacksFunction)(void * context, MoreMemoryFunction more_mem, ReturnMemoryFunction less_mem);
typedef unsigned int (*TotalMemoryFunction)(void);
typedef int (*ShrinkFunction)(int pages);

void gmalloc_set_callbacks(void * context, MoreMemoryFunction more_mem, ReturnMemoryFunction less_mem);
int gmalloc_expand(int pages);
int gmalloc_shrink(int pages);
unsigned int gmalloc_total_bytes(void);
/* Getting the free bytes is only intended as diagnostic information.
 *
 * Algorithms that depend on using free bytes won't work very well, if at all.
 * In particular, a call such as malloc (gmalloc_free_bytes ()) will almost
 * certainly fail.  Here's just a couple of the problems you'd have:
 *
 * 1. the free bytes on the system isn't the same as the free bytes available
 * for your program to use.  For example, the memory allocator may use some
 * memory for internal use for each allocation.
 * 2. You'd have race conditions with other things malloc'ing and free'ing
 * memory, causing lots of intermittent behavioral changes.
 *
 * In short, don't make algorithms that rely on free bytes.  Only use it for
 * diagnostic information. */
unsigned int gmalloc_free_bytes(void);
void gmalloc_report(FILE * fp, const char * args);
void gmalloc_dump(FILE * fp, const char * args);
void gmalloc_enable_debug (void);

#define GMALLOC_SET_CALLBACKS  "gmalloc_set_callbacks"
#define GMALLOC_TOTAL_BYTES  "gmalloc_total_bytes"
#define GMALLOC_FREE_BYTES  "gmalloc_free_bytes"
#define GMALLOC_SHRINK "gmalloc_shrink"
#define GMALLOC_INFO "gmalloc_report"


#ifdef __cplusplus
}
#endif

#endif
