# `parser_alloc()`

```c
parser_t* parser_alloc(int maxHeadersNb, int maxFieldsNb)
```

Allocates an instance of `parser_t` structure with `maxHeadersNb` capacity for headers, which each can have no more than `maxFieldsNb` fields.

**Return value:**
- On success, a pointer to the correctly allocated `parser_t` structure is returned.
- On error, `NULL` is returned.

## Example:

```c
parser_t* parser = parser_alloc(5, 5);
```


# `parser_free()`

```c
void parser_free(parser_t* p)
```
Frees structure `p` allocated by the `parser_alloc` function. If `p` is equal to `NULL`, function does nothing.


# `parser_headerAdd()`

```c
int parser_headerAdd(parser_t *p, const char *headerName, int (*converter)(hmap_t *))
```

Defines a new header in parser `p`. The library can only parse headers declared via this function. Header names are unique within `p`. Headers are distinguished by their names specified by `headerName`. It is a literal string without the `@` at the beginning. This name must be the same as in parsing files. Parser after recognition of header invocates function pointed by `converter`.

For more information about `converter` see [there](api.md#converter-handler).

**Return value:**
- On success, zero is returned.
- On error non-zero value is returned.


# `parser_execute()`

```c
int parser_execute(parser_t *p, const char *path, unsigned int mode)
```
Parses the file specified by the `path`. It invocates a `converter` function for every found header in a parsed file. Argument `mode` allows to specify behavior in case of an undefined header:
- `PARSER_EXEC_ALL_HEADERS` - returns an error,
- `PARSER_IGN_UNKNOWN_HEADERS` - ignores such headers.

Returned integer signalizes if the function parsed the whole file.

**Return value**
- On success, zero is returned.
- On error non-zero value is returned.


# `parser_clear()`


```c
void parser_clear(parser_t* p)
```
Removes all defined headers from the parser. After usage, it is possible to add new headers using `parser_headerAdd`.

The function does not change the maximum number of headers or fields.

**Arguments:**
- `p` - pointer to structure


# Defines

- `MAX_HEADER_LEN` is equal to maximum header length.
- `MAX_FIELD_LEN` is equal to maximum field name length.
- `MAX_VALUE_LEN` is equal to maximum field value length.
- `PARSER_EXEC_ALL_HEADERS` allows to specify behavior of `parser_execute`.
- `PARSER_IGN_UNKNOWN_HEADERS` allows to specify behavior of `parser_execute`.


# Converter Handler

```c
int converter(hmap_t* hmap)
```

This is a function that has to be defined by the user to add a definition of a new header.

Hash map `hmap` contains names of fields from files as keys and values of these fields as map values. These values are strings the same as those from a parsing file.

Important is that `converter` have to copy values `hamp` because these string will be deallocated after parsing completion.

In case of an error during execution function should return a non-zero value. This stops parsing.

In case of successful execution, the converter should return zero.

## Example:

Assume that we want to define a new header, which in a file has such a syntax:

```
@NEW_HEADER
    int_field = 3
    float_field = 2.5
```

We want to parse such a header to this structure:

```c
typedef struct {
    int intValue;
    float floatValue;
} example_t;

```

Then a good converter can look like this:

```c
/* Global variable */
static example_t result;

void* converter(hamp_t* hmap) {
    char *intValue, *floatValue; 
    
    if (hmap->size > 2) {
        return -1;
    }

    intValue = hmap_get(hmap, "int_field");
    floatValue = hmap_get(hmap, "float_field");

    if (intValue == NULL || floatValue == NULL) {
        return -1;
    }

    result.intValue = atoi(intValue);
    result.floatValue = atof(floatValue)

    return 0;
}
```

# See also:
- [Table of contents](README.md)
