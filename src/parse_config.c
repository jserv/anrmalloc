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
#include "parse_config.h"
#include <ctype.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

int
parser_init(ParseConfig * parser, const char * file)
{
    int fd;
    fd = open (file, O_RDONLY);

    if (fd < 0)
        return -1;

    parser->offset += read (fd, parser->buf + parser->offset, PARSE_CONFIG_BUFFER_SIZE - parser->offset);
    close (fd);
    return 0;
}
void
parser_destroy(ParseConfig * parser)
{
    ((void)parser);
}

/* poor man's sscanf */
static inline unsigned int
int_convert(const char * data)
{
    unsigned int i;
    unsigned int value =0;

    for(i = 0; data[i] != '\n' && i< strlen(data); i++){
        if(isdigit((int)data[i])){
            value *= 10;
            value += data[i] - '0';
        }
    }

    return value;
}
static inline void
readline(char ** start, char ** end, char * buf)
{
    *start = *end;
    *end = strchr( *start, '\n');

    if(!*end){
        *start = NULL;
        return;
    }

    while(*start != *end){
        *buf = **start;
        *start+=1;
        buf+=1;
    }
    
    *buf = '\0';

    *end+=1;
}

unsigned int 
parser_get_val(ParseConfig * parser, const char * key)
{
    char * start;
    char * end = parser->buf;
    char buf[PARSE_CONFIG_BUFFER_SIZE];

    readline(&start, &end, buf); 

    while(end != NULL){
        if(buf[0] != '#'){
            char * needle = strstr(buf, key);
            if(needle){
                return int_convert(buf + strlen(key));
            }

        }

        readline(&start, &end, buf); 
    }

    return 0;
}
unsigned int
parser_get_val_array(ParseConfig  * parser, 
                     const char * key, 
                     unsigned int size, 
                     unsigned int * val)
{

    char * start;
    char * end = parser->buf;
    char buf[PARSE_CONFIG_BUFFER_SIZE];

    readline(&start, &end, buf); 

    while(end != NULL){
        if(buf[0] != '#'){
            char * needle = strstr(buf, key);
            if(needle){
                unsigned int idx =0;
                int i =0;
                unsigned int value=0;
                needle += strlen(key);
                while(!isdigit(*needle)) needle++;
                for(; needle[i] != '\0' && idx < size ; i++){
                    if(needle[i] == ' '){
                        val[idx++] = value;
                        value = 0;
                        continue;
                    }
                    value *=10; 
                    value += needle[i] - '0';

                }
                if(val[idx] != value)
                    val[idx++] = value;
                return idx;
            }
        }
        readline(&start, &end, buf); 
    }

    return 0;
}

void parser_get_string(ParseConfig * parser,
                       const char * key,
                       unsigned int size,
                       char *val)
{
    char * start;
    char * end = parser->buf;
    char buf[PARSE_CONFIG_BUFFER_SIZE];

    if (size > 0) val[0] = '\0';

    readline(&start, &end, buf);

    while(end != NULL){
        if(buf[0] != '#'){
            char * needle = strstr(buf, key);

            if(needle){
                needle += strlen(key);
                while (*needle != '\0' && *needle == ' ')
                    needle++;

                snprintf(val, size, "%s", needle);
            }
        }

        readline(&start, &end, buf);
    }
}
