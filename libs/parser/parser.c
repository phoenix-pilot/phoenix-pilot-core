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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "parser.h"

#define WORD "[A-Za-z0-9_.,-]"


/*
 * Patterns for regex expressions
 *
 * If you have changed this pattern check if these function work properly:
 * - parser_nextHeaderGet
 * - parser_nextFieldNameGet
 * - parser_nextFieldGet
 * 
 * Especially be aware of `regmatch_t` stuctures.
 */
#define HEADER_PATTERN "^[[:space:]]*@("WORD"+)[[:space:]]*(#.*)?[[:space:]]*$"
#define HEADER_SUBEXPRESSIONS 1

#define FIELD_PATTERN "^[[:space:]]*("WORD"+)[ =]+("WORD"+)[[:space:]]*(#.*)?[[:space:]]*$"
#define FIELD_SUBEXPRESSIONS 2

#define COMMENT_PATTERN "^[[:space:]]*#.*$|^[[:space:]]*$"

/* Structure contains information about defined header */
typedef struct {
    int structFlag;
	int fieldsNo;
    void* (*converter)(hmap_t*);
} parser_headerInfo_t;


parser_t* parser_alloc(int headersNo)
{
	parser_t* result;
	
	if (headersNo <= 0) {
		return NULL;
	}

	result = malloc(sizeof(parser_t));
	if (result == NULL) {
		return NULL;
	}

	result->headersFunMap = hmap_init(headersNo);
	if (result->headersFunMap == NULL) {
		free(result);
		return NULL;
	}

	/* Compiling regular expressions */
	if (regcomp(&(result->commentRegex), COMMENT_PATTERN, REG_EXTENDED) != 0) {
		hmap_free(result->headersFunMap);
		free(result);
		return NULL;
	}
	if (regcomp(&(result->fieldRegex), FIELD_PATTERN, REG_EXTENDED) != 0) {
		hmap_free(result->headersFunMap);
		regfree(&(result->commentRegex));
		free(result);
		return NULL;
	}
	if (regcomp(&(result->headerRegex), HEADER_PATTERN, REG_EXTENDED) != 0) {
		hmap_free(result->headersFunMap);
		regfree(&(result->commentRegex));
		regfree(&(result->fieldRegex));
		free(result);
		return NULL;
	}

	result->maxFieldsNo = 0;
	return result;
}


/*  Function deallocates elements from hash map */
static void parser_mapElemsFree(hmap_t* hmap) {
	void* elem;
	unsigned int i = 0;
	
	do
	{
		elem = hmap_next(hmap, &i);
		free(elem);
	} while (elem != NULL);
}


void parser_free(parser_t* p)
{
	if(p == NULL) {
		return;
	}
	
	parser_mapElemsFree(p->headersFunMap);
	hmap_free(p->headersFunMap);

	regfree(&(p->commentRegex));
	regfree(&(p->fieldRegex));
	regfree(&(p->headerRegex));

	free(p);
}


int parser_headerAdd(parser_t* p, const char* headerName, unsigned int fieldsNo, int structFlag, void* (*converter)(hmap_t*))
{
	parser_headerInfo_t* newElem;

	if (p == NULL || converter == NULL || fieldsNo == 0) {
		return EXIT_FAILURE;
	}
	
	newElem = malloc(sizeof(parser_headerInfo_t));
	if (newElem == NULL) {
		return EXIT_FAILURE;
	}

	newElem->structFlag = structFlag;
	newElem->fieldsNo = fieldsNo;
	newElem->converter = converter;
	
	if (hmap_insert(p->headersFunMap, headerName, newElem) == -1) {
		free(newElem);
		return EXIT_FAILURE;
	}

	if (p->maxFieldsNo < fieldsNo) {
		p->maxFieldsNo = fieldsNo;
	}

	return EXIT_SUCCESS;
}


/* Checks if this line from parsing file contains a header */
static inline int parser_isHeader(parser_t* p, char* line) {
	return regexec(&(p->headerRegex), line, 0, NULL, 0);
}


/* Checks if this line from parsing file contains a comment */
static inline int parser_isComment(parser_t* p, char* line) {
	return regexec(&(p->commentRegex), line, 0, NULL, 0);
}


/* Checks if this line from parsing file contains a field */
static inline int parser_isField(parser_t* p, char* line) {
	return regexec(&(p->fieldRegex), line, 0, NULL, 0);
}


/* Allocates new string of length equal to `len` and containing substring of `source` starting from `source[startIndex]` */
static char* parser_newCopiedStr(const char* source, unsigned int startIndex, size_t len)
{
	char* result = malloc(sizeof(char) * (len + 1));
	if (result == NULL) {
		return NULL;
	}

	strncpy(result, &(source[startIndex]), len);
	result[len] = '\0';

	return result;
}


/* Returns next header name. If error occurred or field was found returns NULL */
static char* parser_nextHeaderGet(parser_t* p, FILE* file)
{
	size_t bufLen = 0;
	ssize_t lineLen;
	char* buf = NULL;
	char* result;
	regmatch_t regmatch[HEADER_SUBEXPRESSIONS + 1];
	int headerLen;

	for (;;)
	{
		lineLen = getline(&buf, &bufLen, file);
		if(lineLen < 0) {
			break;
		}

		if(parser_isComment(p, buf) == 0) {
			continue;
		}

		if(parser_isField(p, buf) == 0) {
			break;
		}

		if(regexec(&(p->headerRegex), buf, 2, regmatch, 0) == 0) {
			headerLen = regmatch[1].rm_eo - regmatch[1].rm_so;

			result = parser_newCopiedStr(buf, regmatch[1].rm_so, headerLen);
			if (result == NULL) {
				break;
			}
			
			free(buf);
			return result;
		}

		break;
	}
	free(buf);

	return NULL;
}


/* Returns next field name. If error occurred or header was found returns NULL */
static char* parser_nextFieldNameGet(parser_t* p, FILE* file)
{
	char* buf = NULL;
	size_t bufLen = 0;
	ssize_t lineLen;
	regmatch_t regmatch[FIELD_SUBEXPRESSIONS + 1];
	char* result;
	int resultLen;
	
	for (;;)
	{
		lineLen = getline(&buf, &bufLen, file);
		if(lineLen < 0) {
			break;
		}

		if(parser_isComment(p, buf) == 0) {
			continue;
		}

		if (parser_isHeader(p, buf) == 0) {
			break;
		}

		if(regexec(&(p->fieldRegex), buf, 3, regmatch, 0) == 0) {
			resultLen = regmatch[1].rm_eo - regmatch[1].rm_so;

			result = parser_newCopiedStr(buf, regmatch[1].rm_so, resultLen);
			if (result == NULL) {
				break;
			}

			free(buf);
			return result;
		}
	}
	free(buf);

	return NULL;
}


/* Checks if file if syntactically correct */
static int parser_fileCheck(parser_t* p, FILE* file, int* headersNo)
{
	char* header, *field;
	hmap_t* fieldsMap;
	parser_headerInfo_t* headerInfo;
	int i;

	*headersNo = 0;

	fieldsMap = hmap_init(p->maxFieldsNo);
	if (fieldsMap == NULL) {
		return EXIT_FAILURE;
	}

	for (;;)
	{
		/* Gets header name and checks if there is no fields left */
		header = parser_nextHeaderGet(p, file);
		if (header == NULL) {
			hmap_free(fieldsMap);
			if(feof(file) != 0) {
				return EXIT_SUCCESS;
			}
			return EXIT_FAILURE;
		}

		(*headersNo)++;

		/* Checks if such a header was defined */
		headerInfo = hmap_get(p->headersFunMap, header);
		if (headerInfo == NULL) {
			hmap_free(fieldsMap);
			free(header);
			return EXIT_FAILURE;
		}

		for (i=0 ; i< headerInfo->fieldsNo; i++) {
			field = parser_nextFieldNameGet(p, file);
			if (field == NULL) {
				hmap_free(fieldsMap);
				free(header);
				return EXIT_FAILURE;
			}

			/* Checks if there is no fields with the same names. Header is a placeholder. */
			if (hmap_insert(fieldsMap, field, header) != 0) {
				hmap_free(fieldsMap);
				free(header);
				free(field);
				return EXIT_FAILURE;
			}

			free(field);
		}

		free(header);
		hmap_clear(fieldsMap);
	}

	hmap_free(fieldsMap);
}


/* In case of 0 and name of the next field and it's value in case of success. In other case returns non zero value */
static int parser_nextFieldGet(parser_t* p, FILE* file, char** fieldName, char** value)
{
	size_t bufLen = 0;
	ssize_t lineLen;
	char* buf = NULL;
	regmatch_t regmatch[3];
	int fieldLen, valueLen;

	for (;;)
	{
		lineLen = getline(&buf, &bufLen, file);
		if(lineLen < 0) {
			break;
		}

		if(parser_isComment(p, buf) == 0) {
			continue;
		}

		if (parser_isHeader(p, buf) == 0) {
			break;
		}

		if(regexec(&(p->fieldRegex), buf, 3, regmatch, 0) == 0) {
			fieldLen = regmatch[1].rm_eo - regmatch[1].rm_so;
			valueLen = regmatch[2].rm_eo - regmatch[2].rm_so;

			*fieldName = parser_newCopiedStr(buf, regmatch[1].rm_so, fieldLen);
			if (*fieldName == NULL) {
				break;
			}

			*value = parser_newCopiedStr(buf, regmatch[2].rm_so, valueLen);
			if (*value == NULL) {
				free(*fieldName);
				break;
			}
			
			free(buf);
			return EXIT_SUCCESS;
		}

		break;
	}
	free(buf);

	return EXIT_FAILURE;
}


int parser_parse(parser_t* p, const char* path, parser_result_t** result,  int* resultLen)
{
	FILE* file;
	int headersNo, i, j;
	hmap_t* fieldsMap;
	char* header, *field, *value;
	parser_headerInfo_t* headerInfo;

	if (resultLen == NULL || p == NULL) {
		return EXIT_FAILURE;
	}

	*resultLen = 0;

	if (result == NULL) {
		return EXIT_FAILURE;
	}

	file = fopen(path, "r");
	if (file == NULL) {
		return EXIT_FAILURE;
	}

	if (parser_fileCheck(p, file, &headersNo) != 0) {
		fclose(file);
		return EXIT_FAILURE;
	}

	if (headersNo == 0) {
		fclose(file);
		return EXIT_SUCCESS;
	}

	fieldsMap = hmap_init(p->maxFieldsNo);
	if (fieldsMap == NULL) {
		fclose(file);
		return EXIT_FAILURE;
	}

	*result = malloc(sizeof(parser_result_t) * headersNo);
	if (*result == NULL) {
		fclose(file);
		hmap_free(fieldsMap);
		return EXIT_FAILURE;
	}

	if (fseek(file, 0, SEEK_SET) != 0) {
		fclose(file);
		hmap_free(fieldsMap);
		free(*result);
		return EXIT_FAILURE;
	}
	
	for(i=0; i<headersNo; i++) {
		header = parser_nextHeaderGet(p, file);
		if (header == NULL) {
			fclose(file);
			hmap_free(fieldsMap);
			if (*resultLen == 0) {
				free(*result);
			}
			return EXIT_FAILURE;
		}

		headerInfo = hmap_get(p->headersFunMap, header);
		if (headerInfo == NULL) {
			free(header);
			fclose(file);
			hmap_free(fieldsMap);
			if (*resultLen == 0) {
				free(*result);
			}
			return EXIT_FAILURE;
		}

		free(header);
		(*result)[i].structFlag = headerInfo->structFlag;
		
		for (j = 0; j < headerInfo->fieldsNo; j++) {
			if (parser_nextFieldGet(p, file, &field, &value) != 0) {
				fclose(file);
				parser_mapElemsFree(fieldsMap);
				hmap_free(fieldsMap);
				if (*resultLen == 0) {
					free(*result);
				}
				return EXIT_FAILURE;
			}

			if (hmap_insert(fieldsMap, field, value) != 0) {
				fclose(file);
				parser_mapElemsFree(fieldsMap);
				hmap_free(fieldsMap);
				if (*resultLen == 0) {
					free(*result);
				}
				return EXIT_FAILURE;
			}

			free(field);
		}

		(*result)[i].data = headerInfo->converter(fieldsMap);
		if ((*result)[i].data == NULL) {
			parser_mapElemsFree(fieldsMap);
			hmap_free(fieldsMap);
			fclose(file);
			if (*resultLen == 0) {
				free(*result);
			}
			return EXIT_FAILURE;
		}

		(*resultLen)++;

		parser_mapElemsFree(fieldsMap);
		hmap_clear(fieldsMap);
	}

	fclose(file);
	hmap_free(fieldsMap);

	return EXIT_SUCCESS;
}


void parser_clear(parser_t* p)
{
	if (p == NULL) {
		return;
	}
	
	parser_mapElemsFree(p->headersFunMap);
	hmap_clear(p->headersFunMap);

	p->maxFieldsNo = 0;
}
