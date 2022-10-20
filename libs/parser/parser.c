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
#include <stdint.h>

#include "parser.h"


#define FUNCTION_ALLOC      "parser_alloc"
#define FUNCTION_HEADER_ADD "parser_headerAdd"
#define FUNCTION_EXECUTE    "parser_execute"

#define WORD "[A-Za-z0-9_.,-]"

/*
 * Patterns for regex expressions
 *
 * If you have changed these patterns check if these function work properly:
 * - parser_headerGet
 * - parser_fieldGet
 *
 * Especially be aware of `regmatch_t` structures.
 */
#define HEADER_PATTERN        "^[[:space:]]*@(" WORD "+)[[:space:]]*(#.*)?[[:space:]]*$"
#define HEADER_SUBEXPRESSIONS 2

#define FIELD_PATTERN        "^[[:space:]]*(" WORD "+)[ =]+(" WORD "+)([[:space:]]+#.*)?[[:space:]]*$"
#define FIELD_SUBEXPRESSIONS 3

#define COMMENT_PATTERN "^[[:space:]]*#.*$|^[[:space:]]*$"


/* Static variables */
struct {
	int regexCompErr;

	regex_t headerRegex;
	regex_t fieldRegex;
	regex_t commentRegex;
} parser_common;


/* Structure contains information about defined header */
typedef struct {
	const char *headerName;
	int (*converter)(const hmap_t *);
} parser_headerInfo_t;


/* Contains information about particular field from file */
typedef struct {
	char field[MAX_FIELD_LEN + 1];
	char value[MAX_VALUE_LEN + 1];
} parser_field_t;


/* Structure of the main parser object */
struct parser_t {
	parser_headerInfo_t *headerInfos;
	hmap_t *headersMap;

	parser_field_t *fields;
	hmap_t *fieldsMap;
};


enum parser_lineType { type_Header, type_Field, type_Comment, type_InvalidLine };


parser_t *parser_alloc(int maxHeadersNb, int maxFieldsNb)
{
	parser_t *result;

	if (parser_common.regexCompErr != 0) {
		fprintf(stderr, "%s: error on Regex compilation.\n", FUNCTION_ALLOC);
		return NULL;
	}

	if (maxHeadersNb <= 0 || maxFieldsNb <= 0) {
		fprintf(stderr, "%s: invalid maxHeadersNb/maxFieldsNb arguments\n", FUNCTION_ALLOC);
		return NULL;
	}

	if (SIZE_MAX / maxHeadersNb < sizeof(parser_headerInfo_t)) {
		fprintf(stderr, "%s: maximum header number exceeded limit value\n", FUNCTION_ALLOC);
		return NULL;
	}

	if (SIZE_MAX / maxFieldsNb < sizeof(parser_field_t)) {
		fprintf(stderr, "%s: maximum fields number exceeded limit value\n", FUNCTION_ALLOC);
		return NULL;
	}

	result = malloc(sizeof(parser_t));
	if (result == NULL) {
		fprintf(stderr, "%s: malloc error\n", FUNCTION_ALLOC);
		return NULL;
	}

	result->headerInfos = malloc(sizeof(parser_headerInfo_t) * maxHeadersNb);
	if (result->headerInfos == NULL) {
		fprintf(stderr, "%s: malloc error\n", FUNCTION_ALLOC);
		free(result);
		return NULL;
	}

	result->headersMap = hmap_init(maxHeadersNb);
	if (result->headersMap == NULL) {
		fprintf(stderr, "%s: headers map initialization error\n", FUNCTION_ALLOC);
		free(result->headerInfos);
		free(result);
		return NULL;
	}

	result->fields = malloc(sizeof(parser_field_t) * maxFieldsNb);
	if (result->fields == NULL) {
		fprintf(stderr, "%s: malloc error\n", FUNCTION_ALLOC);
		hmap_free(result->headersMap);
		free(result->headerInfos);
		free(result);
		return NULL;
	}

	result->fieldsMap = hmap_init(maxFieldsNb);
	if (result->fieldsMap == NULL) {
		fprintf(stderr, "%s: fields map initialization error\n", FUNCTION_ALLOC);
		free(result->fields);
		hmap_free(result->headersMap);
		free(result->headerInfos);
		free(result);
		return NULL;
	}

	return result;
}


void parser_free(parser_t *p)
{
	if (p == NULL) {
		return;
	}

	hmap_free(p->headersMap);
	free(p->headerInfos);

	hmap_free(p->fieldsMap);
	free(p->fields);

	free(p);
}


int parser_headerAdd(parser_t *p, const char *headerName, int (*converter)(const hmap_t *))
{
	unsigned int id;

	if (p == NULL || headerName == NULL || converter == NULL) {
		fprintf(stderr, "%s: invalid arguments\n", FUNCTION_HEADER_ADD);
		return -1;
	}

	/* Checking if there is enough memory for a new header */
	if (p->headersMap->capacity == p->headersMap->size) {
		fprintf(stderr, "%s: maximum number of header was exceeded.\n", FUNCTION_HEADER_ADD);
		return -1;
	}

	if (strlen(headerName) > MAX_HEADER_LEN) {
		fprintf(stderr, "%s: header name \"%s\" is too long\n", FUNCTION_HEADER_ADD, headerName);
		return -1;
	}

	id = p->headersMap->size;

	p->headerInfos[id].converter = converter;
	p->headerInfos[id].headerName = headerName;

	if (hmap_insert(p->headersMap,
			p->headerInfos[id].headerName,
			&p->headerInfos[id]) == -1) {
		fprintf(stderr, "%s: header %s already exists!\n", FUNCTION_HEADER_ADD, headerName);
		return -1;
	}

	return 0;
}


/* Returns a type of line from file */
enum parser_lineType parser_lineTypeGet(char *line)
{
	enum parser_lineType result = type_InvalidLine;

	if (regexec(&parser_common.fieldRegex, line, 0, NULL, 0) == 0) {
		result = type_Field;
	}
	else if (regexec(&parser_common.commentRegex, line, 0, NULL, 0) == 0) {
		result = type_Comment;
	}
	else if (regexec(&parser_common.headerRegex, line, 0, NULL, 0) == 0) {
		result = type_Header;
	}

	return result;
}


/* Returns next header name. If succeeded returns 0. */
static int parser_headerGet(char *line, char *result)
{
	regmatch_t regmatch[HEADER_SUBEXPRESSIONS];
	int len;
	int err = -1;

	if (regexec(&parser_common.headerRegex, line, HEADER_SUBEXPRESSIONS, regmatch, 0) == 0) {
		len = regmatch[1].rm_eo - regmatch[1].rm_so;

		if (len <= MAX_HEADER_LEN) {
			strncpy(result, &line[regmatch[1].rm_so], len);
			result[len] = '\0';
			err = 0;
		}
	}

	return err;
}


/* Fills `parser_result_t` structure. If succeeded returns 0. */
static int parser_fieldGet(char *line, parser_field_t *result)
{
	regmatch_t regmatch[FIELD_SUBEXPRESSIONS];
	size_t fieldLen, valueLen;
	int err = -1;

	if (regexec(&parser_common.fieldRegex, line, FIELD_SUBEXPRESSIONS, regmatch, 0) == 0) {
		fieldLen = regmatch[1].rm_eo - regmatch[1].rm_so;
		valueLen = regmatch[2].rm_eo - regmatch[2].rm_so;

		if (fieldLen <= MAX_FIELD_LEN && valueLen <= MAX_VALUE_LEN) {
			strncpy(result->field, &line[regmatch[1].rm_so], fieldLen);
			strncpy(result->value, &line[regmatch[2].rm_so], valueLen);

			result->field[fieldLen] = '\0';
			result->value[valueLen] = '\0';

			err = 0;
		}
	}

	return err;
}


int parser_execute(parser_t *p, const char *path)
{
	FILE *file;

	char *line = NULL;
	size_t lineLen = 0;

	char header[MAX_HEADER_LEN + 1];
	parser_headerInfo_t *headerInfo = NULL;

	int res, fieldsCnt = 0, err = 0;

	if (p == NULL || path == NULL) {
		fprintf(stderr, "%s: invalid arguments.\n", FUNCTION_EXECUTE);
		return -1;
	}

	file = fopen(path, "r");
	if (file == NULL) {
		fprintf(stderr, "%s: error on file \"%s\" opening.\n", FUNCTION_EXECUTE, path);
		return -1;
	}

	while (getline(&line, &lineLen, file) != -1) {
		switch (parser_lineTypeGet(line)) {
			case type_Header:
				/* Invoking converter for the previous header. If this is first header, then `headerInfo` is NULL */
				if (headerInfo != NULL) {
					err = headerInfo->converter(p->fieldsMap);
					hmap_clear(p->fieldsMap);
					if (err != 0) {
						fprintf(stderr, "%s: error on converter invocation\n", FUNCTION_EXECUTE);
						break;
					}
				}
				fieldsCnt = 0;

				if (parser_headerGet(line, header) != 0) {
					fprintf(stderr, "%s: error on parsing header \"%s\"\n", FUNCTION_EXECUTE, line);
					err = -1;
					break;
				}

				headerInfo = hmap_get(p->headersMap, header);
				if (headerInfo == NULL) {
					fprintf(stderr, "%s: undefined header \"%s\"\n", FUNCTION_EXECUTE, header);
					err = -1;
					break;
				}

				break;

			case type_Field:
				/* Checking the case when there is no header above field in file */
				if (headerInfo == NULL) {
					fprintf(stderr, "%s: file is incorrect. Field without header\n\"%s\"\n", FUNCTION_EXECUTE, line);
					err = -1;
					break;
				}

				if (fieldsCnt >= p->fieldsMap->capacity) {
					fprintf(stderr, "%s: maximum number of fields was exceeded\n", FUNCTION_EXECUTE);
					err = -1;
					break;
				}

				if (parser_fieldGet(line, &p->fields[fieldsCnt]) != 0) {
					fprintf(stderr, "%s: error on field parsing \"%s\"\n", FUNCTION_EXECUTE, line);
					err = -1;
					break;
				}

				if (hmap_insert(p->fieldsMap, p->fields[fieldsCnt].field, p->fields[fieldsCnt].value) != 0) {
					fprintf(stderr, "%s: incorrect file. Header \"%s\" with multiple fields \"%s\"\n", FUNCTION_EXECUTE, header, p->fields[fieldsCnt].field);
					err = -1;
					break;
				}

				fieldsCnt++;
				break;

			case type_Comment:
				break;

			case type_InvalidLine:
			default:
				err = -1;
				break;
		}

		if (err != 0) {
			hmap_clear(p->fieldsMap);
			fclose(file);
			free(line);
			return -1;
		}
	}

	res = 0;

	if (feof(file) == 0) {
		fprintf(stderr, "%s: error on file parsing\n", FUNCTION_EXECUTE);
		err = -1;
	}

	if (res == 0 && headerInfo != NULL) {
		err = headerInfo->converter(p->fieldsMap);

		if (err != 0) {
			fprintf(stderr, "%s: error on converter invocation\n", FUNCTION_EXECUTE);
		}
	}

	hmap_clear(p->fieldsMap);
	fclose(file);
	free(line);

	return err;
}


void parser_clear(parser_t *p)
{
	if (p == NULL) {
		return;
	}

	hmap_clear(p->headersMap);
}


__attribute__((constructor)) static void parser_compileRegex(void)
{
	if (regcomp(&parser_common.commentRegex, COMMENT_PATTERN, REG_EXTENDED) != 0) {
		parser_common.regexCompErr = -1;
		return;
	}

	if (regcomp(&parser_common.fieldRegex, FIELD_PATTERN, REG_EXTENDED) != 0) {
		regfree(&parser_common.commentRegex);
		parser_common.regexCompErr = -1;
		return;
	}

	if (regcomp(&parser_common.headerRegex, HEADER_PATTERN, REG_EXTENDED) != 0) {
		regfree(&parser_common.commentRegex);
		regfree(&parser_common.fieldRegex);
		parser_common.regexCompErr = -1;
		return;
	}
}


__attribute__((destructor)) static void parser_regexFree(void)
{
	if (parser_common.regexCompErr == 0) {
		regfree(&parser_common.commentRegex);
		regfree(&parser_common.fieldRegex);
		regfree(&parser_common.headerRegex);
	}
}
