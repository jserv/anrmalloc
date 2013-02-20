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


#ifndef _PARSE_CONFIG_H_
#define _PARSE_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#define PARSE_CONFIG_BUFFER_SIZE 512

typedef struct parse_config ParseConfig;
struct parse_config{
    int offset;
    char buf[PARSE_CONFIG_BUFFER_SIZE];
};    

int parser_init(ParseConfig *, const char *);
void parser_destroy(ParseConfig *);
unsigned int parser_get_val(ParseConfig *, const char *);
unsigned int parser_get_val_array(ParseConfig *, const char * key, unsigned int size, unsigned int * val);
void parser_get_string(ParseConfig * parser, const char * key,
                       unsigned int size, char *val);
#ifdef __cplusplus
}
#endif
#endif
