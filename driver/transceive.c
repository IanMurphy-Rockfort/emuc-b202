#include <linux/skbuff.h>
#include <linux/version.h>
#include <linux/tty.h>
#include <linux/mutex.h>

#include "transceive.h"

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
  #include <linux/can/skb.h>
#endif


extern bool trace_func_tran;
extern bool show_debug_tran;
extern void print_func_trace (bool is_trace, int line, const char *func);



/*-----------------------------------------------------------------------*/
void emuc_unesc (EMUC_RAW_INFO *info, unsigned char s)
{
  /*=======================================================*/
  print_func_trace(trace_func_tran, __LINE__, __FUNCTION__);
  /*=======================================================*/

  if(!test_bit(SLF_ERROR, &info->flags))
  {
    if(info->rcount < EMUC_MTU)
      info->rbuff[info->rcount] = s;
    else
    {
      info->devs[0]->stats.rx_over_errors++;
      info->devs[1]->stats.rx_over_errors++;
      set_bit(SLF_ERROR, &info->flags);
    }
  }

  info->rcount++;

  if(info->rcount == EMUC_MTU)
  {
    emuc_bump(info);
    info->rcount = 0;
  }


} /* END: emuc_unesc() */



/*-----------------------------------------------------------------------*/
void emuc_bump (EMUC_RAW_INFO *info)
{
  /*=======================================================*/
  print_func_trace(trace_func_tran, __LINE__, __FUNCTION__);
  /*=======================================================*/

  EMUC_CAN_FRAME     frame;
  int                i, ret;
  struct sk_buff    *skb;
  struct can_frame   cf;

  memset(&frame, 0, sizeof(frame));
  memcpy(frame.com_buf, info->rbuff, info->rcount);

  if((ret = EMUCRevHex(&frame)) < 0)
  {
    printk("emuc : bump : parse fail %d.\n", ret);
    return;
  }


  /*===============================*/
  if(show_debug_tran)
  {
    static int cnt = 1;

    printk("%d. Data: ", cnt++);
    for(i=0; i<DATA_LEN; i++)
      printk("%02X ", frame.data[i]);
    printk("| ");
    printk("ID: ");
    for(i=0; i<ID_LEN; i++)
      printk("%02X ", frame.id[i]);
    printk("\n");
  }
  /*===============================*/


  frame.CAN_port = frame.CAN_port + 1;
  frame.id_type  = frame.id_type - 1;

  cf.can_id = 0;

  if(frame.rtr)
    cf.can_id |= CAN_RTR_FLAG;

  if(frame.id_type)
    cf.can_id |= CAN_EFF_FLAG;

  for(i=0; i<ID_LEN; i++)
    cf.can_id |= (frame.id[ID_LEN - 1 - i] << (i * 8));

  /* RTR frames may have a dlc > 0 but they never have any data bytes */
  *(u64 *)(&cf.data) = 0;

  if (!frame.rtr)
  {
    cf.can_dlc = frame.dlc;

    for(i=0; i<DATA_LEN; i++)
      cf.data[i] = frame.data[i];
  }
  else
    cf.can_dlc = 0;

  #if LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
    skb = dev_alloc_skb(sizeof(struct can_frame) + sizeof(struct can_skb_priv));
  #else
    skb = dev_alloc_skb(sizeof(struct can_frame));
  #endif

  if(!skb)
  {
    return;
  }

  skb->dev       = info->devs[frame.CAN_port - 1];
  skb->protocol  = htons(ETH_P_CAN);
  skb->pkt_type  = PACKET_BROADCAST;
  skb->ip_summed = CHECKSUM_UNNECESSARY;

  #if LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
    can_skb_reserve(skb);
    can_skb_prv(skb)->ifindex = info->devs[frame.CAN_port - 1]->ifindex;
  #endif

  #if LINUX_VERSION_CODE >= KERNEL_VERSION(4,1,5)
    can_skb_prv(skb)->skbcnt = 0;
  #endif

  memcpy(skb_put(skb, sizeof(struct can_frame)), &cf, sizeof(struct can_frame));

  info->devs[frame.CAN_port - 1]->stats.rx_packets++;
  info->devs[frame.CAN_port - 1]->stats.rx_bytes += cf.can_dlc;

  netif_rx_ni(skb);

} /* END: emuc_bump() */



/*-----------------------------------------------------------------------*/
void emuc_encaps (EMUC_RAW_INFO *info, int channel, struct can_frame *cf)
{
  /*=======================================================*/
  print_func_trace(trace_func_tran, __LINE__, __FUNCTION__);
  /*=======================================================*/

  int             i;
  int             len = COM_BUF_LEN;
  int             actual;
  canid_t         id = cf->can_id;
  EMUC_CAN_FRAME  emuc_can_frame;

  memset(&emuc_can_frame, 0, sizeof(EMUC_CAN_FRAME));
  emuc_can_frame.CAN_port = channel;
  emuc_can_frame.rtr = (cf->can_id & CAN_RTR_FLAG) ? 1: 0;

  if (cf->can_id & CAN_EFF_FLAG)
  {
    emuc_can_frame.id_type = EMUC_EID;
    id &= cf->can_id & CAN_EFF_MASK;
  }
  else
  {
    emuc_can_frame.id_type = EMUC_SID;
    id &= cf->can_id & CAN_SFF_MASK;
  }

  for(i=ID_LEN-1; i >= 0; i--)
  {
    emuc_can_frame.id[i] = id & 0xff;
    id >>= 8;
  }
  
  emuc_can_frame.dlc = cf->can_dlc;

  for(i=0; i<cf->can_dlc; i++)
    emuc_can_frame.data[i] = cf->data[i];

  EMUCSendHex(&emuc_can_frame);
  memcpy(info->xbuff, emuc_can_frame.com_buf, len);


  /* Order of next two lines is *very* important.
   * When we are sending a little amount of data,
   * the transfer may be completed inside the ops->write()
   * routine, because it's running with interrupts enabled.
   * In this case we *never* got WRITE_WAKEUP event,
   * if we did not request it before write operation.
   *       14 Oct 1994  Dmitry Gorodchanin.
   */

  set_bit(TTY_DO_WRITE_WAKEUP, &info->tty->flags);
  actual = info->tty->ops->write(info->tty, info->xbuff, len);

  info->xleft = len - actual;
  info->xhead = info->xbuff + actual;
  info->devs[channel]->stats.tx_bytes += cf->can_dlc;


  /* v2.2: for fixing tx_packet bug */
  info->current_channel = channel;

} /* END: emuc_encaps() */



/*-----------------------------------------------------------------------*/
void emuc_transmit (struct work_struct *work)
{
  /*=======================================================*/
  print_func_trace(trace_func_tran, __LINE__, __FUNCTION__);
  /*=======================================================*/

  int             actual;
  EMUC_RAW_INFO  *info = container_of(work, EMUC_RAW_INFO, tx_work);

  spin_lock_bh(&info->lock);

  /* First make sure we're connected. */
  if(!info->tty || info->magic != EMUC_MAGIC || (!netif_running(info->devs[0]) && !netif_running(info->devs[1])))
  {
    spin_unlock_bh(&info->lock);
    return;
  }

  if(info->xleft <= 0)
  {
    /* v2.1 */
    #if 0
      /* Now serial buffer is almost free & we can start
       * transmission of another packet */
      if (netif_running(info->devs[0]))
        info->devs[0]->stats.tx_packets++;
      if (netif_running(info->devs[1]))
        info->devs[1]->stats.tx_packets++;
    #endif


    /* v2.2 */
    info->devs[info->current_channel]->stats.tx_packets++;


    clear_bit(TTY_DO_WRITE_WAKEUP, &info->tty->flags);
    spin_unlock_bh(&info->lock);
    
    if (netif_running(info->devs[0]))
      netif_wake_queue(info->devs[0]);
    if (netif_running(info->devs[1]))
      netif_wake_queue(info->devs[1]);
    return;
  }

  actual = info->tty->ops->write(info->tty, info->xhead, info->xleft);
  info->xleft -= actual;
  info->xhead += actual;
  spin_unlock_bh(&info->lock);


} /* END: emuc_transmit() */
