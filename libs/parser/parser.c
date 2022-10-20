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

#define MAX_HEADER_LEN 16
#define MAX_FIELD_LEN  16
#define MAX_VALUE_LEN  64

#define WORD "[A-Za-z0-9_.,-]"


/*
 * Patterns for regex expressions
 *
 * If you have changed this pattern check if these function work properly:
 * - parser_nextHeaderGet
 * - parser_nextFieldNameGet
 * - parser_nextFieldGet
 *
 * Especially be aware of `regmatch_t` structures.
 */
#define HEADER_PATTERN        "^[[:space:]]*@(" WORD "+)[[:space:]]*(#.*)?[[:space:]]*$"
#define HEADER_SUBEXPRESSIONS 2

#define FIELD_PATTERN        "^[[:space:]]*(" WORD "+)[ =]+(" WORD "+)[[:space:]]*(#.*)?[[:space:]]*$"
#define FIELD_SUBEXPRESSIONS 3

#define COMMENT_PATTERN "^[[:space:]]*#.*$|^[[:space:]]*$"

/* Structure contains information about defined header */
typedef struct {
	char headerName[MAX_HEADER_LEN + 1];
	int fieldsNo;
	int (*converter)(hmap_t *);
} parser_headerInfo_t;

/* Structure of the main parser object */
struct parser_t {
	parser_headerInfo_t *headerInfos;
	hmap_t *headersInfosMap;

	int maxHeadersNb;
	int defHeaders;
	int maxFieldsNb;
};

typedef struct {
	char field[MAX_FIELD_LEN + 1];
	char value[MAX_VALUE_LEN + 1];
} parser_field_t;


static int instancesNb = 0;

static regex_t headerRegex;
static regex_t fieldRegex;
static regex_t commentRegex;


parser_t *parser_alloc(int headersNo)
{
	parser_t *result;

	if (headersNo <= 0) {
		return NULL;
	}

	result = malloc(sizeof(parser_t));
	if (result == NULL) {
		return NULL;
	}

	result->headerInfos = malloc(sizeof(parser_headerInfo_t) * headersNo);
	if (result->headerInfos == NULL) {
		free(result);
		return NULL;
	}

	result->headersInfosMap = hmap_init(headersNo);
	if (result->headersInfosMap == NULL) {
		free(result->headerInfos);
		free(result);
		return NULL;
	}

	if (instancesNb == 0) {
		/* Compiling regular expressions */
		if (regcomp(&commentRegex, COMMENT_PATTERN, REG_EXTENDED) != 0) {
			hmap_free(result->headersInfosMap);
			free(result);
			return NULL;
		}
		if (regcomp(&fieldRegex, FIELD_PATTERN, REG_EXTENDED) != 0) {
			hmap_free(result->headersInfosMap);
			regfree(&commentRegex);
			free(result);
			return NULL;
		}
		if (regcomp(&headerRegex, HEADER_PATTERN, REG_EXTENDED) != 0) {
			hmap_free(result->headersInfosMap);
			regfree(&(commentRegex));
			regfree(&fieldRegex);
			free(result);
			return NULL;
		}
	}

	instancesNb++;
	result->defHeaders = 0;
	result->maxHeadersNb = headersNo;
	result->maxFieldsNb = 0;
	return result;
}


void parser_free(parser_t *p)
{
	if (p == NULL) {
		return;
	}

	free(p->headerInfos);
	hmap_free(p->headersInfosMap);

	free(p);

	instancesNb--;
	if (instancesNb == 0) {
		regfree(&(commentRegex));
		regfree(&(fieldRegex));
		regfree(&(headerRegex));
	}
}


int parser_headerAdd(parser_t *p, const char *headerName, unsigned int fieldsNo, int (*converter)(hmap_t *))
{
	if (p == NULL || converter == NULL || headerName == NULL) {
		return -1;
	}

	if (p->maxHeadersNb <= p->defHeaders) {
		return -1;
	}

	if (strlen(headerName) > MAX_HEADER_LEN) {
		return -1;
	}

	p->headerInfos[p->defHeaders].converter = converter;
	p->headerInfos[p->defHeaders].fieldsNo = fieldsNo;
	strcpy(p->headerInfos[p->defHeaders].headerName, headerName);

	if (hmap_insert(p->headersInfosMap, p->headerInfos[p->defHeaders].headerName, &(p->headerInfos[p->defHeaders])) == -1) {
		return -1;
	}

	p->defHeaders++;

	if (p->maxFieldsNb < fieldsNo) {
		p->maxFieldsNb = fieldsNo;
	}

	return 0;
}


/* Checks if this line from parsing file contains a header */
static inline int parser_isHeader(char *line)
{
	return regexec(&headerRegex, line, 0, NULL, 0);
}


/* Checks if this line from parsing file contains a comment */
static inline int parser_isComment(char *line)
{
	return regexec(&commentRegex, line, 0, NULL, 0);
}


/* Checks if this line from parsing file contains a field */
static inline int parser_isField(char *line)
{
	return regexec(&fieldRegex, line, 0, NULL, 0);
}


/* Returns next header name. If error occurred or field was found returns NULL */
static int parser_nextHeaderGet(FILE *file, char *result)
{
	size_t bufLen = 0;
	ssize_t lineLen;
	char *buf = NULL;
	regmatch_t regmatch[HEADER_SUBEXPRESSIONS];
	int headerLen;

	for (;;) {
		lineLen = getline(&buf, &bufLen, file);
		if (lineLen < 0) {
			break;
		}

		if (parser_isComment(buf) == 0) {
			continue;
		}

		if (parser_isField(buf) == 0) {
			break;
		}

		if (regexec(&(headerRegex), buf, HEADER_SUBEXPRESSIONS, regmatch, 0) == 0) {
			headerLen = regmatch[1].rm_eo - regmatch[1].rm_so;

			if (headerLen > MAX_HEADER_LEN) {
				break;
			}

			strcpy(result, &buf[regmatch[1].rm_so]);

			free(buf);
			return 0;
		}

		break;
	}

	free(buf);

	return -1;
}


/* In case of 0 and name of the next field and it's value in case of success. In other case returns non zero value */
static int parser_nextFieldGet(FILE *file, parser_field_t *result)
{
	size_t bufLen = 0;
	ssize_t lineLen;
	char *buf = NULL;
	regmatch_t regmatch[FIELD_SUBEXPRESSIONS];
	int fieldLen, valueLen;

	for (;;) {
		lineLen = getline(&buf, &bufLen, file);
		if (lineLen < 0) {
			break;
		}

		if (parser_isComment(buf) == 0) {
			continue;
		}

		if (parser_isHeader(buf) == 0) {
			break;
		}

		if (regexec(&fieldRegex, buf, FIELD_SUBEXPRESSIONS, regmatch, 0) == 0) {
			fieldLen = regmatch[1].rm_eo - regmatch[1].rm_so;
			valueLen = regmatch[2].rm_eo - regmatch[2].rm_so;

			if (fieldLen > MAX_FIELD_LEN || valueLen > MAX_VALUE_LEN) {
				break;
			}

			strcpy(result->field, &buf[regmatch[1].rm_so]);
			strcpy(result->value, &buf[regmatch[2].rm_so]);

			free(buf);
			return 0;
		}

		break;
	}
	free(buf);

	return -1;
}


int parser_parse(parser_t *p, const char *path)
{
	FILE *file;
	parser_field_t *fields;
	hmap_t *fieldsMap;
	char header[MAX_HEADER_LEN + 1];
	parser_headerInfo_t *headerInfo;
	int i, result;
	;

	if (p == NULL) {
		return -1;
	}

	file = fopen(path, "r");
	if (file == NULL) {
		return -1;
	}

	fields = malloc(sizeof(parser_field_t) * p->maxFieldsNb);
	if (fields == NULL) {
		fclose(file);
		return -1;
	}

	fieldsMap = hmap_init(p->maxFieldsNb);
	if (fieldsMap == NULL) {
		fclose(file);
		free(fields);
		return -1;
	}

	for (;;) {
		if (parser_nextHeaderGet(file, header) != 0) {
			fclose(file);
			free(fields);
			hmap_free(fieldsMap);
			return -1;
		}

		headerInfo = hmap_get(p->headersInfosMap, header);
		if (headerInfo == NULL) {
			fclose(file);
			free(fields);
			hmap_free(fieldsMap);
			return -1;
		}

		for (i = 0; i < headerInfo->fieldsNo; i++) {

			if (parser_nextFieldGet(file, &(fields[i])) != 0) {
				fclose(file);
				free(fields);
				hmap_free(fieldsMap);
				return -1;
			}

			if (hmap_insert(fieldsMap, fields[i].field, &(fields[i].value)) != 0) {
				fclose(file);
				free(fields);
				hmap_free(fieldsMap);
				return -1;
			}
		}

		if (headerInfo->converter(fieldsMap) != 0) {
			fclose(file);
			free(fields);
			hmap_free(fieldsMap);
			return -1;
		}

		hmap_clear(fieldsMap);
	}

	if (feof(file) != 0) {
		result = 0;
	}
	else {
		result = -1;
	}

	fclose(file);
	free(fields);
	hmap_free(fieldsMap);

	return result;
}


void parser_clear(parser_t *p)
{
	if (p == NULL) {
		return;
	}

	hmap_clear(p->headersInfosMap);

	p->defHeaders = 0;
	p->maxFieldsNb = 0;
}
