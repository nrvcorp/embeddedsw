/******************************************************************************
 * Copyright (C) 2010 - 2021 Xilinx, Inc.  All rights reserved.
 * SPDX-License-Identifier: MIT
 ******************************************************************************/

/**
 *
 * @file xaxiethernet_example.h
 *
 * Defines common data types, prototypes, and includes the proper headers
 * for use with the Axi Ethernet example code residing in this directory.
 *
 * This file along with xaxiethernet_example_util.c are utilized with the
 * specific example code in the other source code files provided.
 *
 * These examples are designed to be compiled and utilized within the EDK
 * standalone BSP development environment. The readme file contains more
 * information on build requirements needed by these examples.
 *
 * <pre>
 * MODIFICATION HISTORY:
 *
 * Ver   Who  Date     Changes
 * ----- ---- -------- -------------------------------------------------------
 * 1.00a asa  4/30/10 First release based on the ll temac driver
 * 3.02a srt  4/26/13 Added function prototype for *_ConfigureInternalPhy().
 *
 * </pre>
 *
 ******************************************************************************/
#ifndef XAXIETHERNET_EXAMPLE_H
#define XAXIETHERNET_EXAMPLE_H

/***************************** Include Files *********************************/

#include "xparameters.h" // defines XPAR values
#include "xaxidma.h"
#include "xaxiethernet.h" // defines Axi Ethernet APIs
#include "stdio.h"        // stdio

#include "xtime_l.h"

/************************** Constant Definitions ****************************/
#define AXIETHERNET_LOOPBACK_SPEED 100       // 100Mb/s for Mii
#define AXIETHERNET_LOOPBACK_SPEED_1G 1000   // 1000Mb/s for GMii
#define AXIETHERNET_LOOPBACK_SPEED_2p5G 2500 // 2p5G for 2.5G MAC
#define AXIETHERNET_PHY_DELAY_SEC 4          // Amount of time to delay waiting on PHY to reset.

#define MAX_MULTICAST_ADDR (1 << 23) // Maximum number of multicast ethernet mac addresses.
#ifndef TESTAPP_GEN
#define NUM_PACKETS 50
#else
#define NUM_PACKETS 1
#endif

#define NUM_OF_BUFFER 512 // Caution!!! This should be power of 2
#define SQRT_NUM_OF_BUFFER 9

// #define BUFFER_SIZE			0x8000000	// Equal to 128MBytes -> 8 Buffers = 1GBytes
// #define BUFFER_SIZE			0x1FFE00	//  1456 * 1440 for CIS -(DMA)-> MEM = About 2MBytes / 1080p frame
#define BUFFER_SIZE 0x2AA80 //  1456 * 120 for DVS -(DMA)-> MEM

#define AXIETHERNET_DEVICE_ID XPAR_AXIETHERNET_0_DEVICE_ID
#define AXIDMA_DEVICE_ID XPAR_AXIDMA_1_DEVICE_ID
#define AXIETHERNET_IRPT_INTR XPAR_FABRIC_AXIETHERNET_0_VEC_ID
#define DMA_TX_IRPT_INTR XPAR_FABRIC_AXIDMA_1_MM2S_INTROUT_VEC_ID
#define BD_ALIGNMENT XAXIDMA_BD_MINIMUM_ALIGNMENT
// #define FRAME_SIZE				0x1FA400	// Equal to 1920 * 1080 Bytes
#define FRAME_SIZE 0x2A300 // Equal to (960 * 720 * 2 / 8) Bytes
#define PAYLOAD_SIZE 1440
#define TX_PACKET_CNT (FRAME_SIZE / PAYLOAD_SIZE)       // Number of packets per FHD frame
#define FRAMES_THRESHOLD 120                            // Highest divisible number for `TX_PACKET_CNT` between 1 ~ 255
#define TXBD_CNT (2 * TX_PACKET_CNT)                    // Number of TxBDs to use
#define TX_BURST_CNT (TX_PACKET_CNT / FRAMES_THRESHOLD) // Number of iterations per FHD frame

#define HEADER_SIZE (XAE_HDR_SIZE + 2)
#define PACKET_SIZE (HEADER_SIZE + PAYLOAD_SIZE)
// PACKET_SIZE_RUP is individual packet size rounded up to multiple of 64
#if PACKET_SIZE % BD_ALIGNMENT == 0
#define PACKET_SIZE_RUP PACKET_SIZE
#else
#define PACKET_SIZE_RUP (PACKET_SIZE + (BD_ALIGNMENT - (PACKET_SIZE % BD_ALIGNMENT)))
#endif
/*
 * Number of bytes to reserve for BD space for the number of BDs desired
 */
#define TXBD_SPACE_BYTES (XAxiDma_BdRingMemCalc(BD_ALIGNMENT, TXBD_CNT))

/***************** Macros (Inline Functions) Definitions *********************/

/**************************** Type Definitions ******************************/

/*
 * Define an aligned data type for an ethernet frame. This declaration is
 * specific to the GNU compiler
 */
// typedef unsigned char EthernetFrame[NUM_PACKETS * XAE_MAX_JUMBO_FRAME_SIZE] __attribute__ ((aligned(64)));
typedef unsigned char EthernetFrame[PACKET_SIZE_RUP] __attribute__((aligned(64)));

/************************** Function Prototypes *****************************/
int SetupIntrCoalescingSetting();
int SetupEthernet();
void TxCallBack(XAxiDma_BdRing *TxRingPtr);
void TxIntrHandler(XAxiDma_BdRing *TxRingPtr);
void AxiEthernetErrorHandler(XAxiEthernet *AxiEthernet);

float NRV_GetTime(XTime t1, XTime t2);
int NRV_IntTime(XTime t1, XTime t2);

////// Utility functions implemented in xaxiethernet_example_util.c //////
void AxiEthernetUtilFrameHdrVlanFormatVid(EthernetFrame *FramePtr, u32 VlanNumber, u32 Vid);
void AxiEthernetUtilFrameHdrVlanFormatType(EthernetFrame *FramePtr, u16 FrameType, u32 VlanNumber);
void AxiEthernetUtilFrameSetVlanPayloadData(EthernetFrame *FramePtr, int PayloadSize, u32 VlanNumber);
int AxiEthernetUtilFrameVerify(EthernetFrame *CheckFrame, EthernetFrame *ActualFrame);
void AxiEthernetUtilFrameMemClear(EthernetFrame *FramePtr);
int AxiEthernetUtilEnterLoopback(XAxiEthernet *AxiEthernetInstancePtr, int Speed);
void AxiEthernetUtilErrorTrap(char *Message);
void AxiEthernetUtilPhyDelay(unsigned int Seconds);
int AxiEthernetUtilConfigureInternalPhy(XAxiEthernet *AxiEthernetInstancePtr, int Speed);
int AxiEtherentConfigureTIPhy(XAxiEthernet *AxiEthernetInstancePtr, u32 PhyAddr);

int SendFrameSingleTxBurstSetup(u8 *SendingAddress);
int SendFrameSingleTxBurstSend(void);

LONG EthFrameSetTxHeaderOnly(void);

void FrmRdDoneCallBack();

/************************** Variable Definitions ****************************/

extern char AxiEthernetMAC[];     // Local MAC address
extern char AxiEthernetDestMAC[]; // Dest MAC address
extern char TxBdSpace[];

extern int DeviceErrors; // Num of errors detected in the device
extern int Padding;
extern int ExternalLoopback;

extern u32 *g_FrmBuf[];
extern int g_wr_ptr;
extern int g_rd_setup_ptr;
extern int g_rd_work_ptr;
extern int NumOfEmptyBuf;
extern int NumOfReadyTxBDSet;
extern u8 TxTriggeredFlag;
extern u8 IsInTxCriticalSection;

extern XTime intStart, intEnd;   // Eth Tx Interrupt ~ Next send
extern volatile int accumTime[]; // Duration check
extern volatile int accumCntr[]; // Checked count

extern XAxiEthernet AxiEthernetInstance;
extern XAxiDma EthernetDmaInstancePtr;

#endif /* XAXIETHERNET_EXAMPLE_H */
