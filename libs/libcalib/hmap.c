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


void *hmap_next(hmap *hm, unsigned int *iter)
{
	void *val = NULL;

	while (*iter < hm->nitems && val == NULL) {
		val = hm->list[(*iter)++].value;
	}

	return val;
}


void *hmap_get(const hmap *hm, const char *key)
{
	unsigned long hash, i;
	const hmap_entry_t *curr;

	i = hash = hmap_hash(key) % hm->nitems;

	do {
		curr = &hm->list[i];

		if (curr->value == NULL) {
			break;
		}

		if (curr->hash == hash) {
			if (strcmp(key, curr->key) == 0) {
				return curr->value;
			}
		}

		if (++i == hm->nitems) {
			i = 0;
		}
	} while (i != hash);

	errno = ENOENT;
	return NULL;
}


int hmap_insert(hmap *hm, const char *key, void *val)
{
	unsigned long i, hash;
	hmap_entry_t *curr;

	hash = i = hmap_hash(key) % hm->nitems;
	do {
		curr = &hm->list[i];

		/* collision check */
		if (curr->value == NULL) {
			/* fill entry */
			curr->value = val;
			curr->key = key;
			curr->hash = hash;

			hm->used++;

			return 0;
		}

		if (++i == hm->nitems) {
			i = 0;
		}
	} while (i != hash);

	errno = ENOMEM;
	return -1;
}


void hmap_clear(hmap *hm)
{
	size_t i;

	for (i = 0; i < hm->nitems; i++) {
		hm->list[i].value = NULL;
	}

	hm->used = 0;
}


void hmap_free(hmap *hm)
{
	if (hm != NULL) {
		free(hm->list);
		free(hm);
	}
}


hmap *hmap_init(size_t nitems)
{
	hmap *hm;
	hmap_entry_t *entries;
	size_t i;

	entries = calloc(nitems, sizeof(hmap_entry_t));
	hm = malloc(sizeof(hmap));

	if (entries == NULL || hm == NULL) {
		free(entries);
		free(hm);

		errno = ENOMEM;
		return NULL;
	}

	for (i = 0; i < nitems; i++) {
		entries[i].value = NULL;
	}

	hm->nitems = nitems;
	hm->list = entries;
	hm->used = 0;

	return hm;
}
