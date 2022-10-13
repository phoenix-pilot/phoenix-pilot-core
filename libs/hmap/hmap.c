/*
 * Phoenix-Pilot
 *
 * hmap.c
 * 
 * String keyed hashmap for pointer storage implementation.
 *
 * Copyright 2022 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#include <stdlib.h>
#include <errno.h>
#include <string.h>

#include "hmap.h"


/* 
 * Hashing algorithm was created for sdbm (a public-domain reimplementation of
 * ndbm) database library. It was found to do well in scrambling bits, causing
 * better distribution of the keys and fewer splits. it also happens to be a
 * good general hashing function with good distribution.
 *
 * Slightly rewritten. 
 * Source: http://www.cs.yorku.ca/~oz/hash.html with no license restrictions.
 */
static unsigned long hmap_hash(const char *key)
{
	unsigned long h = 0;

	while (*key != 0) {
		h = (h << 6) + (h << 16) - h + (unsigned long)*key++;
	}

	return h;
}


void *hmap_next(hmap_t *hm, unsigned int *iter)
{
	void *val = NULL;

	if (iter != NULL) {
		while (*iter < hm->capacity && val == NULL) {
			val = hm->arr[(*iter)++].value;
		}
	}

	return val;
}


void *hmap_get(const hmap_t *hm, const char *key)
{
	unsigned long hash, i;

	if (hm == NULL || key == NULL) {
		errno = EINVAL;
		return NULL;
	}

	/* hash the key with modulo size to not overflow the array */
	hash = hmap_hash(key) % hm->capacity;
	i = hash;

	while (hm->arr[i].value != NULL) {
		if (hm->arr[i].hash == hash) {
			if (strcmp(hm->arr[i].key, key) == 0) {
				return hm->arr[i].value;
			}
		}

		/* loop the index */
		if (++i == hm->capacity) {
			i = 0;
		}

		if (i == hash) {
			/* whole hashmap traversed without empty bucket */
			break;
		}
	}

	errno = ENOENT;
	return NULL;
}


int hmap_insert(hmap_t *hm, const char *key, void *val)
{
	unsigned long i, hash;
	char *keyCpy;

	if (hm == NULL || key == NULL || val == NULL) {
		errno = EINVAL;
		return -1;
	}

	/* hash the key with modulo size to not overflow the array */
	hash = hmap_hash(key) % hm->capacity;
	i = hash;

	/* check for collision */
	if (hm->arr[i].value != NULL) {
		/* traverse hashmap to find next free bucket */
		do {
			if (hash == hm->arr[i].hash) {
				if (strcmp(key, hm->arr[i].key) == 0) {
					/* There is already an entry with such a key */
					return -1;
				}
			}

			if (++i == hm->capacity) {
				i = 0;
			}

			if (i == hash) {
				/* whole hashmap traversed without empty bucket */
				errno = ENOMEM;
				return -1;
			}

		} while (hm->arr[i].value != NULL);
	}

	keyCpy = malloc(strlen(key) + 1);
	if (keyCpy == NULL) {
		return -1;
	}
	strcpy(keyCpy, key);

	/* fill empty entry */
	hm->arr[i].value = val;
	hm->arr[i].key = keyCpy;
	hm->arr[i].hash = hash;

	hm->size++;

	return 0;
}


void hmap_clear(hmap_t *hm)
{
	size_t i;

	if (hm != NULL) {
		for (i = 0; i < hm->capacity; i++) {
			if (hm->arr[i].value != NULL) {
				free(hm->arr[i].key);
			}
			hm->arr[i].value = NULL;
		}

		hm->size = 0;
	}
}


void hmap_free(hmap_t *hm)
{
	int i;

	if (hm != NULL) {
		for (i = 0; i < hm->capacity; i++) {
			if (hm->arr[i].value != NULL) {
				free(hm->arr[i].key);
			}
		}

		free(hm->arr);
		free(hm);
	}
}


hmap_t *hmap_init(size_t capacity)
{
	hmap_t *hm;
	size_t i;

	hm = malloc(sizeof(hmap_t));
	if (hm == NULL) {
		errno = ENOMEM;
		return NULL;
	}

	hm->capacity = capacity;
	hm->size = 0;

	hm->arr = calloc(capacity, sizeof(hmap_entry_t));
	if (hm->arr == NULL) {
		free(hm);
		errno = ENOMEM;
		return NULL;
	}

	for (i = 0; i < capacity; i++) {
		hm->arr[i].value = NULL;
	}

	return hm;
}
