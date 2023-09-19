/*
 * Phoenix-Pilot
 *
 * EKF config file modification tool
 *
 * Copyright 2023 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include "config_file_handler.h"

#include <stdio.h>
#include <ctype.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>


#define EKF_CONFIG_FILE     "etc/ekf.conf"
#define OLD_EKF_CONFIG_FILE "tmp/old_ekf.conf"

typedef struct {
	char header[MAX_HEADER_LEN + 1];
	int headerLen;

	char fieldName[MAX_FIELD_LEN + 1];
	int fieldNameLen;
} file_entry_t;


static inline int ekftests_isWordChar(char c)
{
	return (isalnum(c) || c == '_');
}


/* Returns 0 and fills entry when the new one was found. On EOF returns -1. On error returns -1 and sets errno. */
static int ekftests_nextEntry(FILE *file, char **line, size_t *len, file_entry_t *entry)
{
	char *c;
	int i;

	while (getline(line, len, file) != -1) {
		c = *line;

		while (isspace(*c) != 0) {
			c++;
		}

		if (*c == '@') {
			/* New header was found */
			c++;
			i = 0;
			entry->headerLen = 0;

			/* Header name writing */
			while (ekftests_isWordChar(*c)) {
				if (i >= MAX_HEADER_LEN) {
					fprintf(stderr, "Too long header: %s\n", *line);
					errno = EINVAL;
					return -1;
				}

				entry->header[i++] = *(c++);
				entry->headerLen++;
			}

			entry->header[i] = '\0';

			continue;
		}

		if (ekftests_isWordChar(*c)) {
			/* New field was found */
			entry->fieldNameLen = 0;
			i = 0;

			do {
				if (i >= MAX_FIELD_LEN) {
					fprintf(stderr, "Too long field name: %s\n", *line);
					errno = EINVAL;
					return -1;
				}

				entry->fieldName[i++] = *(c++);
				entry->fieldNameLen++;
			} while (ekftests_isWordChar(*c));

			entry->fieldName[i] = '\0';

			return 0;
		}
	}

	return -1;
}


/* Returns 0 if `entry` from file fits `field` specifier. In other case returns -1 */
static bool ekftests_fieldMatch(const file_entry_t *entry, const char *field)
{
	if (strncmp(entry->header, field, entry->headerLen) != 0) {
		return -1;
	}

	if (field[entry->headerLen] != '/') {
		return -1;
	}

	return strncmp(entry->fieldName, field + entry->headerLen + 1, entry->fieldNameLen);
}


int ekftests_configPrepare(const char **fields, int fieldsCnt)
{
	FILE *cfgOld, *cfgNew;
	char *line = NULL;
	const char *strToWrite;
	size_t lineLen = 0;
	file_entry_t entry;
	char lastHeader[MAX_HEADER_LEN] = "";
	int i;

	if (rename(EKF_CONFIG_FILE, OLD_EKF_CONFIG_FILE) != 0) {
		fprintf(stderr, "Cannot rename file from %s to %s", EKF_CONFIG_FILE, OLD_EKF_CONFIG_FILE);
		return -1;
	}

	cfgOld = fopen(OLD_EKF_CONFIG_FILE, "r");
	if (cfgOld == NULL) {
		ekftests_restoreConfig();
		return -1;
	}

	cfgNew = fopen(EKF_CONFIG_FILE, "w");
	if (cfgNew == NULL) {
		fclose(cfgOld);
		ekftests_restoreConfig();
		return -1;
	}

	/* Clearing errno to distinguish between EOF and error in `ekftests_nextEntry` */
	errno = 0;

	while (ekftests_nextEntry(cfgOld, &line, &lineLen, &entry) == 0) {
		if (strcmp(lastHeader, entry.header) != 0) {
			strcpy(lastHeader, entry.header);

			if (fprintf(cfgNew, "@%s\n", entry.header) < 0) {
				break;
			}
		}

		strToWrite = line;

		for (i = 0; i < fieldsCnt; i++) {
			if (ekftests_fieldMatch(&entry, fields[i]) == 0) {
				strToWrite = &fields[i][entry.headerLen + 1];
				break;
			}
		}

		if (fprintf(cfgNew, "%s\n", strToWrite) < 0) {
			break;
		}
	}

	fclose(cfgOld);
	fclose(cfgNew);
	free(line);

	return errno == 0 ? 0 : -1;
}


int ekftests_restoreConfig(void)
{
	if (access(EKF_CONFIG_FILE, F_OK) == 0) {
		if (remove(EKF_CONFIG_FILE) != 0) {
			return -1;
		}
	}

	return rename(OLD_EKF_CONFIG_FILE, EKF_CONFIG_FILE);
}
