# Files syntax

The parser recognizes only the syntax described below. Non-compliance with the convention causes errors during the parsing process.

## Example file

```
# Habitants of the house

@Person
  Name = Jim
  Surname = Morris
  Age = 29

@Person
  Name = Emily
  Surname = Simpson
  Age = 24

# Their dog
@Dog
  Name = Max
  Age = 2
```

## File syntax

Every line from a correct file can be assigned to one of four categories:
- headers
- fields
- comments
- white lines

### Headers

---

Headers specify the kinds of data that are stored in a file. They are analogical to structure names from programming languages.

The header has to begin with `@` sign and then without any white characters, there should be a header name. In this name, it is possible to use big and small letters as well as digits and `_` (underscore) character without white signs. The maximum length of a header is 16 characters.

After a header name comment is possible.

Before `@` possible are spaces or tabulators.

```
@Person

@Header1

@New_header   # This is a comment
```

### Fields

---

Fields define all the properties of the nearest header, which is above the field. Such a header has to exist. They are analogical to members of a structure.

The order of fields does not matter. Between fields connected with one header comments and empty lines are possible.

Fields have to consist of two parts. The field name and value of this field. Between them, there should be at least one white space or `=` (both of them are also possible).

```
[Filed name]( |=)[Value]
```

The field name defines the kinds of value, which comes after it. Value stores data in form of a string, which during parsing can be converted to other data types. Syntax of fields allows using big and small letters, digits, `_` (underscore), `,` (comma) and `.` (dot). The maximum length of a field name is 16 characters and for a value, it is 64 characters.

After value, it is possible to add a comment.

Before the field, name white spaces are possible.

```
name Jim

length = 2.15m

    Age = 18 # This is a comment
```

### Comments

---

Comments are parts of the file that are omitted during parsing. They can provide additional information for a human reader.

This section of the file can be added to another, but comments have to be at the end of the line.

Comments start from a `#`. Before it multiple white spaces are possible. Every character from `#` to `\n` (new line sign) is considered a comment and will not affect the parsing process.

```
# Comment

    # Comment with tabulator at the beginning

  # Comment with spaces at the beginning
```

### White line

---

The white line consists of only white signs such as spaces or tabulators. Such lines will be omitted the same as it is with comments.

# See also:
- [Table of contents](README.md)
