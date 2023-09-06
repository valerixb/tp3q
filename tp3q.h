#ifndef TP3Q_H
#define TP3Q_H

#include <linux/ioctl.h>

/* ------------------------------------------
 *     TPCMD MM2S FIFO register offsets
 * ------------------------------------------
 */

#define XLLF_ISR_OFFSET  0x00000000  /* Interrupt Status */
#define XLLF_IER_OFFSET  0x00000004  /* Interrupt Enable */

#define XLLF_TDFR_OFFSET 0x00000008  /* Transmit Reset */
#define XLLF_TDFV_OFFSET 0x0000000c  /* Transmit Vacancy */
#define XLLF_TDFD_OFFSET 0x00000010  /* Transmit Data */
#define XLLF_TLR_OFFSET  0x00000014  /* Transmit Length */

#define XLLF_RDFR_OFFSET 0x00000018  /* Receive Reset */
#define XLLF_RDFO_OFFSET 0x0000001c  /* Receive Occupancy */
#define XLLF_RDFD_OFFSET 0x00000020  /* Receive Data */
#define XLLF_RLR_OFFSET  0x00000024  /* Receive Length */
#define XLLF_SRR_OFFSET  0x00000028  /* Local Link Reset */
#define XLLF_TDR_OFFSET  0x0000002C  /* Transmit Destination */
#define XLLF_RDR_OFFSET  0x00000030  /* Receive Destination */


/* ------------------------------------------
 *     UDP PKTZR register offsets
 * ------------------------------------------
 */

#define PKTZR_STATUSW_OFFSET       0x00000000  /* Read only - 32 bitfields - unused*/
#define PKTZR_CONTROLW_OFFSET      0x00000004  /* W(/R) - 32 bitfields - control word */
#define PKTZR_SRC_IP_OFFSET        0x00000008  /* W(/R) - uint32 - Source IP address */
#define PKTZR_SRC_MAC_LO_OFFSET    0x0000000C  /* W(/R) - uint32 - 32 LSBs of Source MAC */
#define PKTZR_SRC_MAC_HI_OFFSET    0x00000010  /* W(/R) - uint16 - 16 MSBs of Source MAC */
#define PKTZR_SRC_UDP_PORT_OFFSET  0x00000014  /* W(/R) - uint32 - Source UDP port # */
#define PKTZR_DEST_IP_OFFSET       0x00000018  /* W(/R) - uint32 - Destination IP address */
#define PKTZR_DEST_MAC_LO_OFFSET   0x0000001C  /* W(/R) - uint32 - 32 LSBs of Destination MAC */
#define PKTZR_DEST_MAC_HI_OFFSET   0x00000020  /* W(/R) - uint16 - 16 MSBs of Destination MAC */
#define PKTZR_DEST_UDP_PORT_OFFSET 0x00000024  /* W(/R) - uint32 - Destination UDP port # */
#define PKTZR_WDOG_TIMEOUT_OFFSET  0x00000028  /* W(/R) - uint32 - Watchdog timeout in 156.25 MHz clock ticks */


/* ------------------------------------------
 *     UDP PKTZR register bitmasks
 * ------------------------------------------
 */

#define PKTZR_ENABLE_MASK     0x00000001ul


/* ----------------------------
 *     reset register masks
 * ----------------------------
 */

#define XLLF_RDFR_RESET_MASK        0x000000a5 /* receive reset value */
#define XLLF_TDFR_RESET_MASK        0x000000a5 /* Transmit reset value */
#define XLLF_SRR_RESET_MASK         0x000000a5 /* Local Link reset value */

/* ----------------------------
 *       interrupt masks
 * ----------------------------
 */

#define XLLF_INT_RPURE_MASK       0x80000000 /* Receive under-read */
#define XLLF_INT_RPORE_MASK       0x40000000 /* Receive over-read */
#define XLLF_INT_RPUE_MASK        0x20000000 /* Receive underrun (empty) */
#define XLLF_INT_TPOE_MASK        0x10000000 /* Transmit overrun */
#define XLLF_INT_TC_MASK          0x08000000 /* Transmit complete */
#define XLLF_INT_RC_MASK          0x04000000 /* Receive complete */
#define XLLF_INT_TSE_MASK         0x02000000 /* Transmit length mismatch */
#define XLLF_INT_TRC_MASK         0x01000000 /* Transmit reset complete */
#define XLLF_INT_RRC_MASK         0x00800000 /* Receive reset complete */
#define XLLF_INT_TFPF_MASK        0x00400000 /* Tx FIFO Programmable Full */
#define XLLF_INT_TFPE_MASK        0x00200000 /* Tx FIFO Programmable Empty */
#define XLLF_INT_RFPF_MASK        0x00100000 /* Rx FIFO Programmable Full */
#define XLLF_INT_RFPE_MASK        0x00080000 /* Rx FIFO Programmable Empty */
#define XLLF_INT_ALL_MASK         0xfff80000 /* All the ints */
#define XLLF_INT_ERROR_MASK       0xf2000000 /* Error status ints */
#define XLLF_INT_RXERROR_MASK     0xe0000000 /* Receive Error status ints */
#define XLLF_INT_TXERROR_MASK     0x12000000 /* Transmit Error status ints */

/* ----------------------------
 *      ioctls
 * ----------------------------
 */
#define AXIS_FIFO_IOCTL_MAGIC 'Q'
#define AXIS_FIFO_NUM_IOCTLS 14


struct axis_fifo_kern_regInfo{
        uint32_t regNo;
        uint32_t regVal;
        };

#define AXIS_FIFO_GET_REG         _IOR(AXIS_FIFO_IOCTL_MAGIC, 0, struct axis_fifo_kern_regInfo)
#define AXIS_FIFO_SET_REG         _IOW(AXIS_FIFO_IOCTL_MAGIC, 1, struct axis_fifo_kern_regInfo)
#define AXIS_FIFO_GET_TX_MAX_PKT  _IOR(AXIS_FIFO_IOCTL_MAGIC, 2, uint32_t)
#define AXIS_FIFO_SET_TX_MAX_PKT  _IOW(AXIS_FIFO_IOCTL_MAGIC, 3,  uint32_t)
#define AXIS_FIFO_GET_RX_MIN_PKT  _IOR(AXIS_FIFO_IOCTL_MAGIC, 4,  uint32_t)
#define AXIS_FIFO_SET_RX_MIN_PKT  _IOW(AXIS_FIFO_IOCTL_MAGIC, 5,  uint32_t)
#define AXIS_FIFO_RESET_IP        _IO(AXIS_FIFO_IOCTL_MAGIC,6)
#define AXIS_FIFO_GET_FPGA_ADDR   _IOR(AXIS_FIFO_IOCTL_MAGIC,7, uint32_t)
#define AXIS_FIFO_GET_TX_VACANCY  _IOR(AXIS_FIFO_IOCTL_MAGIC, 8, uint32_t)
#define AXIS_FIFO_GET_RX_OCCUPANCY _IOR(AXIS_FIFO_IOCTL_MAGIC, 9, uint32_t)
#define AXIS_FIFO_GET_TX_PKTS_SENT _IOR(AXIS_FIFO_IOCTL_MAGIC, 10, uint32_t)
#define AXIS_FIFO_GET_TX_BYTES_SENT _IOR(AXIS_FIFO_IOCTL_MAGIC, 11, uint32_t)
#define AXIS_FIFO_GET_RX_PKTS_READ _IOR(AXIS_FIFO_IOCTL_MAGIC, 12, uint32_t)
#define AXIS_FIFO_GET_RX_BYTES_READ _IOR(AXIS_FIFO_IOCTL_MAGIC, 13, uint32_t)


/* ----------------------------
 *      subsys identifiers
 * ----------------------------
 */

#define TP3Q_TPCMD_SUBSYS     1
#define TP3Q_PKTZR_SUBSYS     2
#define TP3Q_STREAMGEN_SUBSYS 3



#endif /* AXIS_FIFO_H */
