#include <linux/string.h>
#include <linux/module.h>

#include "emuc_parse.h"



extern bool show_debug_pars;
extern bool trace_func_pars;
extern void print_func_trace (bool is_trace, int line, const char *func);

/* static function prototype */
static void chk_sum_end_byte (unsigned char *frame, int size);


/*---------------------------------------------------------------------------------------*/
void EMUCSendHex (EMUC_CAN_FRAME *frame)
{
  /*=======================================================*/
  print_func_trace(trace_func_pars, __LINE__, __FUNCTION__);
  /*=======================================================*/

  unsigned char  *p;
  unsigned char   func = 0x00;

  p = frame->com_buf;
  memset(p, 0, sizeof(frame->com_buf));

  /* head - byte 0 */
  *p = CMD_HEAD_SEND;

  /* func - byte 1 */
  func += frame->CAN_port + 1;
  func += ((frame->id_type - 1) << 2);
  func += frame->rtr << 3;
  func += frame->dlc << 4;

  *(p+1) = func;

  /* id - byte 2 ~ byte 5 */
  memcpy(p+2, frame->id, ID_LEN);

  /* data - byte 6 ~ byte 13 */
  memcpy(p+6, frame->data, DATA_LEN);

  /* chk sum & end byte - byte 14 ~ byte 16 */
  chk_sum_end_byte(p, COM_BUF_LEN);


} /* END: EMUCSendHex() */



/*---------------------------------------------------------------------------------------*/
int EMUCRevHex (EMUC_CAN_FRAME *frame)
{
  /*=======================================================*/
  print_func_trace(trace_func_pars, __LINE__, __FUNCTION__);
  /*=======================================================*/

  int             i;
  unsigned char   chk_sum = 0x00;
  unsigned char  *p;

  p = frame->com_buf;

  /*==========================*/
  if(show_debug_pars)
  {
    for(i=0; i<COM_BUF_LEN; i++)
      printk("%02X ", *(p + i));
    printk("\n");    
  }
  /*==========================*/

  /* head - byte 0 */
  if(*p != CMD_HEAD_RECV)
    return -1;

  /* check sum - byte 14 */
  for(i=0; i<COM_BUF_LEN-3; i++)
    chk_sum = chk_sum + *(p + i);

  if(chk_sum != *(p+14))
    return -2;

  /* func - byte 1 */
  frame->CAN_port = ((int) ( *(p+1) & 0x03)) - 1;
  frame->id_type  = ((int) ((*(p+1) & 0x04) >> 2)) + 1;
  frame->rtr      =  (int) ((*(p+1) & 0x08) >> 3);
  frame->dlc      =  (int) ((*(p+1) & 0xF0) >> 4);

  /* id - byte 2 ~ byte 5 */
  memcpy(frame->id, p+2, ID_LEN);

  /* data - byte 6 ~ byte 13 */
  memcpy(frame->data, p+6, DATA_LEN);

  return 0;


} /* END: EMUCRevHex() */


/*---------------------------------------------------------------------------------------*/
static void chk_sum_end_byte (unsigned char *frame, int size)
{
  /*=======================================================*/
  print_func_trace(trace_func_pars, __LINE__, __FUNCTION__);
  /*=======================================================*/

  int            i;
  unsigned char  chk_sum = 0x00;

  for(i=0; i<size-3; i++)
    chk_sum = chk_sum + *(frame + i);

  *(frame + size - 3) = chk_sum;
  *(frame + size - 2) = 0x0D;
  *(frame + size - 1) = 0x0A;


} /* END: chk_sum_end_byte() */