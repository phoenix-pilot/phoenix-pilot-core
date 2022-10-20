/*
 * Phoenix-Pilot
 *
 * Parser for config files
 *
 * Copyright 2022 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef PARSER_H
#define PARSER_H

#include <hmap.h>
#include <regex.h>


#define MAX_HEADER_LEN 16
#define MAX_FIELD_LEN  16
#define MAX_VALUE_LEN  64


typedef struct parser_t parser_t;

/* Returns a pointer to correctly allocated parser object. `headerNo` - max number of headers to add to this parser. */
extern parser_t *parser_alloc(int maxHeadersNb, int maxFieldsNb);


/* Deallocates parser object and removes added headers */
extern void parser_free(parser_t *p);


/* Adds new header to parser. `headerName` - literal string with name of a header from file without `@`, `converter` - function which gets parsing results */
extern int parser_headerAdd(parser_t *p, const char *headerName, int (*converter)(const hmap_t *));


/* Performs the parsing at file specified by `path` */
extern int parser_execute(parser_t *p, const char *path);


/* Removes all added headers */
extern void parser_clear(parser_t *p);


#endif /* PARSER_H */
