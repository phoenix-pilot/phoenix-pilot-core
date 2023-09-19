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

#ifndef CONFIG_FILE_HANDLER_H
#define CONFIG_FILE_HANDLER_H


#include <parser.h>


#define MAX_FILE_TO_CHANGE_STR_LEN (MAX_HEADER_LEN + MAX_FIELD_LEN + MAX_VALUE_LEN + 2)


/*
 * Prepares EKF config file.
 * `fields` is a table of strings, which specifies changes to config file.
 * These string should follow format "<header>/<field_name>=<value_to_use>"
 * `fieldsCnt` is a number of elements in table
 */
int ekftests_configPrepare(const char **fields, int fieldsCnt);


/* Restores config file changed with `ekftests_configPrepare` function */
int ekftests_restoreConfig(void);


#endif
