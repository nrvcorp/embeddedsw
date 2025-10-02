#include <stdint.h>
#include "xil_types.h"

#ifndef REGVAL_LIST_DECLARED
#define REGVAL_LIST_DECLARED 1
struct regval_list {
    u16 Address;  /* 16-bit register address */
    u8  Data;     /* 8-bit register value    */
};

typedef struct regval_list regval_list;

#endif
