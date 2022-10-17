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


/* Result of parsing is a table of such structures */
typedef struct {
    int structFlag;     /* Flag for structure recognition. Same as specified in `parser_headerAdd` function */
    void* data;         /* Pointer to filled stucture of type specified by `structFlag` */
} parser_result_t;


/* Stucture of the main parser object */
typedef struct {
	regex_t headerRegex;
    regex_t fieldRegex;
    regex_t commentRegex;
    
    hmap_t* headersFunMap;

    int maxFieldsNo;
} parser_t;


/* Returns a pointer to correctly allocated parser object. `headerNo` - max number of headers to add to this parser. */
extern parser_t* parser_alloc(int headersNo);


/* Deallocates parser object and removes added headers */
extern void parser_free(parser_t* p);


/* 
 * Allows to add new header.
 * `headerName` - name of header from parsing file
 * `fieldsNo` - number of fields in this header
 * `structFlag` - flag for structure recognition, it will be used in the result
 * `converter` - function, which gets hashmap with fields names as keys and values from file as strings and return allocated structure
 * 
 * Return value:
 *  0 - upon successful completion
 *  non zero - in other case
 */
extern int parser_headerAdd(parser_t* p, const char* headerName, unsigned int fieldsNo, int structFlag, void* (*converter)(hmap_t*));


/* Performs the parsing at file specified by `path`. Returns table of `resultLen` length */
extern int parser_parse(parser_t* p, const char* path, parser_result_t** result,  int* resultLen);


/* Removes added headers */
extern void parser_clear(parser_t* p);


#endif /* PARSER_H */
