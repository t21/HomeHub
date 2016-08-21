/*
 * fifo.c
 *
 *  Created on: 17 aug. 2016
 *      Author: thomas
 */

#include "fifo.h"

void fifo_init (fifo_t *_this)
{
    /*****
      The following clears:
        -> buf
        -> head
        -> tail
        -> count
      and sets head = tail
    ***/
    memset (_this, 0, sizeof (*_this));
}


