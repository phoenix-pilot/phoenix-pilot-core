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

#ifndef HMAP_H
#define HMAP_H


#include <stdlib.h>

/* Hashmap entry representing key/value pair */
typedef struct _hmap_entry_t {
	char *key;          /* key of a hashmap entry */
	void *value;        /* value referenced by 'key' */
	unsigned long hash; /* (hashed key) mod (hasmap capacity) */
} hmap_entry_t;


/* Hashmap metadata structure */
typedef struct {
	hmap_entry_t *arr; /* array of hashmap elements */
	size_t capacity;   /* maximum capacity of hashmap */
	size_t size;       /* used capacity of hashmap */
} hmap_t;


/* 
* Iterates over hashmap values using 'i' iterator. Returns NULL if iteration ended. 
* 'i'=0 restarts iteration. 'i' does not correspond to hashmap elements order.
*/
extern void *hmap_next(hmap_t *hm, unsigned int *i);


/* returns value for 'key', NULL if 'key' not found */
extern void *hmap_get(const hmap_t *hm, const char *key);


/* Inserts 'key'/'value' into hashmap. Return 0 on success, -1 otherwise */
extern int hmap_insert(hmap_t *hm, const char *key, void *val);


/* Clear the hashmap */
extern void hmap_clear(hmap_t *hm);


/* Deallocates hashmap pointed by `hm` */
extern void hmap_free(hmap_t *hm);


/* Allocates a hashmap of `capacity` capacity */
extern hmap_t *hmap_init(size_t capacity);


#endif
