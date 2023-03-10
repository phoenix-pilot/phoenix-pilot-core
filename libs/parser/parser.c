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
#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>

#include "parser.h"


/*
 * Patterns for regex expressions
 *
 * If you have changed these patterns check if these function work properly:
 * - parser_headerGet
 * - parser_fieldFill
 *
 * Especially be aware of `regmatch_t` structures.
 */
#define WORD "([A-Za-z0-9_]+)"

#define HEADER_PATTERN        "^[[:space:]]*@" WORD "[[:space:]]*(#.*)?$"
#define HEADER_SUBEXPRESSIONS 2

#define FIELD_VALUE          "([A-Za-z0-9_,.+-]+)"
#define FIELD_PATTERN        "^[[:space:]]*" WORD "( +| *= *)" FIELD_VALUE "[[:space:]]*(#.*)?$"
#define FIELD_SUBEXPRESSIONS 4


/* Static variables */
struct {
	int regexCompErr;

	regex_t headerRegex;
	regex_t fieldRegex;
	regex_t headerNameRegex;
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
		fprintf(stderr, "%s: error on Regex compilation.\n", __FUNCTION__);
		return NULL;
	}

	if (maxHeadersNb <= 0 || maxFieldsNb <= 0) {
		fprintf(stderr, "%s: invalid maxHeadersNb/maxFieldsNb arguments\n", __FUNCTION__);
		return NULL;
	}

	if (SIZE_MAX / maxHeadersNb < sizeof(parser_headerInfo_t)) {
		fprintf(stderr, "%s: maximum header number exceeded limit value\n", __FUNCTION__);
		return NULL;
	}

	if (SIZE_MAX / maxFieldsNb < sizeof(parser_field_t)) {
		fprintf(stderr, "%s: maximum fields number exceeded limit value\n", __FUNCTION__);
		return NULL;
	}

	result = malloc(sizeof(parser_t));
	if (result == NULL) {
		fprintf(stderr, "%s: malloc error\n", __FUNCTION__);
		return NULL;
	}

	result->headerInfos = malloc(sizeof(parser_headerInfo_t) * maxHeadersNb);
	if (result->headerInfos == NULL) {
		fprintf(stderr, "%s: malloc error\n", __FUNCTION__);
		free(result);
		return NULL;
	}

	result->headersMap = hmap_init(maxHeadersNb);
	if (result->headersMap == NULL) {
		fprintf(stderr, "%s: headers map initialization error\n", __FUNCTION__);
		free(result->headerInfos);
		free(result);
		return NULL;
	}

	result->fields = malloc(sizeof(parser_field_t) * maxFieldsNb);
	if (result->fields == NULL) {
		fprintf(stderr, "%s: malloc error\n", __FUNCTION__);
		hmap_free(result->headersMap);
		free(result->headerInfos);
		free(result);
		return NULL;
	}

	result->fieldsMap = hmap_init(maxFieldsNb);
	if (result->fieldsMap == NULL) {
		fprintf(stderr, "%s: fields map initialization error\n", __FUNCTION__);
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


/* Returns 0 if headerName contains valid name for a header. In other case returns -1 */
static int parser_headerNameCheck(const char *headerName)
{
	if (regexec(&parser_common.headerNameRegex, headerName, 0, NULL, 0) == 0) {
		return 0;
	}

	return -1;
}


int parser_headerAdd(parser_t *p, const char *headerName, int (*converter)(const hmap_t *))
{
	unsigned int id;

	if (p == NULL || headerName == NULL || converter == NULL) {
		fprintf(stderr, "%s: invalid arguments\n", __FUNCTION__);
		return -1;
	}

	if (parser_headerNameCheck(headerName) != 0) {
		fprintf(stderr, "%s: invalid header name\n", __FUNCTION__);
		return -1;
	}

	/* Checking if there is enough memory for a new header */
	if (p->headersMap->capacity == p->headersMap->size) {
		fprintf(stderr, "%s: maximum number of header was exceeded.\n", __FUNCTION__);
		return -1;
	}

	if (strlen(headerName) > MAX_HEADER_LEN) {
		fprintf(stderr, "%s: header name \"%s\" is too long\n", __FUNCTION__, headerName);
		return -1;
	}

	id = p->headersMap->size;

	p->headerInfos[id].converter = converter;
	p->headerInfos[id].headerName = headerName;

	if (hmap_insert(p->headersMap,
			p->headerInfos[id].headerName,
			&p->headerInfos[id]) == -1) {
		fprintf(stderr, "%s: header %s already exists!\n", __FUNCTION__, headerName);
		return -1;
	}

	return 0;
}


/* Returns a type of line from file */
static enum parser_lineType parser_lineTypeGet(const char *line)
{
	enum parser_lineType result = type_InvalidLine;

	const char *s = line;
	while (isspace(*s)) {
		s++;
	}

	if (*s == '\0' || *s == '#') {
		result = type_Comment;
	}
	else if (*s == '@') {
		result = type_Header;
	}
	else if (isalnum(*s) || *s == '_') {
		result = type_Field;
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
static int parser_fieldFill(char *line, parser_field_t *result)
{
	regmatch_t regmatch[FIELD_SUBEXPRESSIONS];
	size_t fieldLen, valueLen;
	int err = -1;

	if (regexec(&parser_common.fieldRegex, line, FIELD_SUBEXPRESSIONS, regmatch, 0) == 0) {
		fieldLen = regmatch[1].rm_eo - regmatch[1].rm_so;
		valueLen = regmatch[3].rm_eo - regmatch[3].rm_so;

		if (fieldLen <= MAX_FIELD_LEN && valueLen <= MAX_VALUE_LEN) {
			strncpy(result->field, &line[regmatch[1].rm_so], fieldLen);
			strncpy(result->value, &line[regmatch[3].rm_so], valueLen);

			result->field[fieldLen] = '\0';
			result->value[valueLen] = '\0';

			err = 0;
		}
	}

	return err;
}


int parser_execute(parser_t *p, const char *path, unsigned int mode)
{
	FILE *file;

	char *line = NULL;
	size_t lineLen = 0;

	char header[MAX_HEADER_LEN + 1] = { 0 };
	parser_headerInfo_t *headerInfo = NULL;

	int fieldsCnt = 0, err = 0;

	if (p == NULL || path == NULL) {
		fprintf(stderr, "%s: invalid arguments.\n", __FUNCTION__);
		return -1;
	}

	file = fopen(path, "r");
	if (file == NULL) {
		fprintf(stderr, "%s: error on file \"%s\" opening.\n", __FUNCTION__, path);
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
						fprintf(stderr, "%s: error on converter invocation\n", __FUNCTION__);
						break;
					}
				}
				fieldsCnt = 0;

				if (parser_headerGet(line, header) != 0) {
					fprintf(stderr, "%s: error on parsing header \"%s\"\n", __FUNCTION__, line);
					err = -1;
					break;
				}

				headerInfo = hmap_get(p->headersMap, header);
				if (headerInfo == NULL && (mode & PARSER_EXEC_ALL_HEADERS) == PARSER_EXEC_ALL_HEADERS) {
					fprintf(stderr, "%s: undefined header \"%s\"\n", __FUNCTION__, header);
					err = -1;
				}

				break;

			case type_Field:
				/* Checking the case when there is no header above field in file */
				if (header[0] == '\0') {
					fprintf(stderr, "%s: file is incorrect. Field without header\n\"%s\"\n", __FUNCTION__, line);
					err = -1;
					break;
				}

				/* Ignoring field */
				if (headerInfo == NULL) {
					break;
				}

				if (fieldsCnt >= p->fieldsMap->capacity) {
					fprintf(stderr, "%s: maximum number of fields was exceeded\n", __FUNCTION__);
					err = -1;
					break;
				}

				if (parser_fieldFill(line, &p->fields[fieldsCnt]) != 0) {
					fprintf(stderr, "%s: error on field parsing \"%s\"\n", __FUNCTION__, line);
					err = -1;
					break;
				}

				if (hmap_insert(p->fieldsMap, p->fields[fieldsCnt].field, p->fields[fieldsCnt].value) != 0) {
					fprintf(stderr, "%s: incorrect file. Header \"%s\" with multiple fields \"%s\"\n", __FUNCTION__, header, p->fields[fieldsCnt].field);
					err = -1;
					break;
				}

				fieldsCnt++;
				break;

			case type_Comment:
				break;

			case type_InvalidLine:
				printf("Invalid line \"%s\"\n", line);
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

	if (feof(file) == 0) {
		fprintf(stderr, "%s: error on parsing %s\n", __FUNCTION__, path);
		err = -1;
	}

	if (err == 0 && headerInfo != NULL) {
		err = headerInfo->converter(p->fieldsMap);

		if (err != 0) {
			fprintf(stderr, "%s: error on converter invocation\n", __FUNCTION__);
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
	if (regcomp(&parser_common.fieldRegex, FIELD_PATTERN, REG_EXTENDED) != 0) {
		parser_common.regexCompErr = -1;
		return;
	}

	if (regcomp(&parser_common.headerRegex, HEADER_PATTERN, REG_EXTENDED) != 0) {
		regfree(&parser_common.fieldRegex);
		parser_common.regexCompErr = -1;
		return;
	}

	if (regcomp(&parser_common.headerNameRegex, "^" WORD "$", REG_EXTENDED) != 0) {
		regfree(&parser_common.fieldRegex);
		regfree(&parser_common.headerRegex);
		parser_common.regexCompErr = -1;
		return;
	}
}


__attribute__((destructor)) static void parser_regexFree(void)
{
	if (parser_common.regexCompErr == 0) {
		regfree(&parser_common.fieldRegex);
		regfree(&parser_common.headerRegex);
		regfree(&parser_common.headerNameRegex);
	}
}


int parser_fieldGet(const hmap_t *h, const char *fieldName, void *target, parser_fieldType fieldType)
{
	char *valueStr, *endptr;

	if (target == NULL) {
		fprintf(stderr, "%s: argument `target` cannot be NULL\n", __FUNCTION__);
	}

	valueStr = hmap_get(h, fieldName);
	if (valueStr == NULL) {
		fprintf(stderr, "%s: no \"%s\" field in header\n", __FUNCTION__, fieldName);
		return -1;
	}

	switch (fieldType) {
		case parser_int:
			*(int *)target = strtol(valueStr, &endptr, 10);
			break;
		case parser_float:
			*(float *)target = strtof(valueStr, &endptr);
			break;
		default:
			fprintf(stderr, "%s: invalid field type\n", __FUNCTION__);
			return -1;
	}

	/* Checking if field was parsed successfully */
	if (endptr[0] != '\0') {
		fprintf(stderr, "%s: invalid \"%s\" value in header\n", __FUNCTION__, fieldName);
		return -1;
	}

	return 0;
}
