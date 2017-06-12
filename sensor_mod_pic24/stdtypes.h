
#ifndef STDTYPES_H
#define STDTYPES_H

#ifndef false
#define false (0)
#endif
#ifndef true
#define true  (!false)
#endif

typedef void *             pvoid;
typedef char *             cstring;
typedef unsigned char      bool;

typedef unsigned int       uint;
typedef unsigned char      uchar;

typedef signed char        int8;
typedef unsigned char      uint8;
typedef signed short       int16;
typedef unsigned short     uint16;
typedef signed long        int32;
typedef unsigned long      uint32;
typedef signed long long   int64;
typedef unsigned long long uint64;

#endif  /* STDTYPES_H */

