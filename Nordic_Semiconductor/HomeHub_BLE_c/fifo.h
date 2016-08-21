/*
 * fifo.h
 *
 *  Created on: 17 aug. 2016
 *      Author: thomas
 */

#ifndef FIFO_H_
#define FIFO_H_

    #define MAX_BUF_SIZE    256

    typedef struct  fifo_s
    {
      unsigned char buf[MAX_BUF_SIZE];
      int head;
      int tail;
      int count;
    } fifo_t;

    #ifdef  __cplusplus
      extern  "C" {
    #endif
      void  fifo_init  (fifo_t *_this);
      int   fifo_empty (fifo_t *_this);
      int   fifo_full  (fifo_t *_this);
      int   fifo_get   (fifo_t *_this);
      void  fifo_put   (fifo_t *_this, const unsigned char c);
      void  fifo_flush (fifo_t *_this, const int clearBuffer);
    #ifdef  __cplusplus
      }
    #endif




#endif /* FIFO_H_ */

