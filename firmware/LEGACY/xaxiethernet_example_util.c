/******************************************************************************
* Copyright (C) 2010 - 2021 Xilinx, Inc.  All rights reserved.
* SPDX-License-Identifier: MIT
******************************************************************************/

/**
*
* @file xaxiethernet_example_util.c
*
* This file implements the utility functions for the Axi Ethernet example code.
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who  Date     Changes
* ----- ---- -------- -------------------------------------------------------
* 1.00a asa  6/30/10 First release based on the ll temac driver
* 3.01a srt  02/03/13 Added support for SGMII mode (CR 676793).
*	     02/14/13 Added support for Zynq (CR 681136).
* 3.02a srt  04/24/13 Modified parameter *_SGMII_PHYADDR to *_PHYADDR, the
*                     config parameter C_PHYADDR applies to SGMII/1000BaseX
*	              modes of operation and added support for 1000BaseX mode
*		      (CR 704195). Added function *_ConfigureInternalPhy()
*		      for this purpose.
*	     04/24/13 Added support for RGMII mode.
* 3.02a srt  08/06/13 Fixed CR 717949:
*			Configures external Marvel 88E1111 PHY based on the
*			axi ethernet physical interface type and allows to
*			operate in specific interface mode without changing
*			jumpers on the Microblaze board.
* 5.4	adk  07/12/16  Added Support for TI PHY DP83867.
*       ms   04/05/17  Added tabspace for return statements in functions
*                      for proper documentation while generating doxygen.
* </pre>
*
******************************************************************************/

/***************************** Include Files *********************************/

#include "xaxiethernet_example.h"
#if !defined (__MICROBLAZE__) && !defined(__PPC__)
#include "sleep.h"
#endif

/************************** Variable Definitions ****************************/

// Local MAC address
char AxiEthernetMAC[6] = { 0x00, 0x0A, 0x35, 0x01, 0x02, 0x03 };
// char AxiEthernetDestMAC[6] = {0xD8, 0x5E, 0xD3, 0xA3, 0xA0, 0x69}; // destination mac address
//char AxiEthernetDestMAC[6] = { 0x9C, 0xEB, 0xE8, 0xAF, 0x12, 0x28 }; // Dongtan
//char AxiEthernetDestMAC[6] = { 0xA8, 0xA1, 0x59, 0xFB, 0x8C, 0x4B }; // Dongtan-CHA
//char AxiEthernetDestMAC[6] = { 0xD8, 0xBB, 0xC1, 0x70, 0x68, 0x20 }; // tskim-PC. 2.5Gbps
char AxiEthernetDestMAC[6] = { 0x00, 0xE0, 0x4C, 0x68, 0x00, 0x0E }; // tskim-PC-USB2.5Gbps
//char AxiEthernetDestMAC[6] = { 0xD8, 0xBB, 0xC1, 0xA2, 0x50, 0x4E }; // tskim-4090x1
//char AxiEthernetDestMAC[6] = { 0x98, 0xFC, 0x84, 0xEC, 0x76, 0x04 }; // tskim-USB-C Hub 1Gbps

// Aligned memory segments to be used for buffer descriptors
char TxBdSpace[TXBD_SPACE_BYTES] __attribute__ ((aligned(BD_ALIGNMENT)));
char InitializedTxBdSpace[3][TXBD_SPACE_BYTES] __attribute__ ((aligned(BD_ALIGNMENT)));

// Counters to be incremented by callbacks
int DeviceErrors;     // Num of errors detected in the device
int Padding;          // For 1588 Packets we need to pad 8 bytes time stamp value
int ExternalLoopback; // Variable for External loopback

XAxiEthernet AxiEthernetInstance;
XAxiDma EthernetDmaInstancePtr;

EthernetFrame TxFrameBuf[FRAMES_THRESHOLD]; // Transmit Frame buffer
EthernetFrame TxFrame;	// Transmit Frame

XTime intStart, intEnd;

/*****************************************************************************/
/**
 * This function sets Ethernet Coalescing Options and enables the send
 * interrupts.
 *
 * @return	XST_SUCCESS if successful, else XST_FAILURE.
 *****************************************************************************/
int SetupIntrCoalescingSetting()
{
	xil_printf("Ethernet Coalescing Setup Start...\r\n");
	int Status;
	u16 Threshold = FRAMES_THRESHOLD;
	XAxiEthernet* AxiEthernetInstancePtr = &AxiEthernetInstance;
	XAxiDma* EthernetDmaPtr = &EthernetDmaInstancePtr;
	XAxiDma_BdRing* TxRingPtr = XAxiDma_GetTxRing(EthernetDmaPtr);

	// clear variables shared with callbacks
	DeviceErrors = 0;

	// Make sure Tx is enabled
	Status = XAxiEthernet_SetOptions(AxiEthernetInstancePtr, XAE_TRANSMITTER_ENABLE_OPTION);
	if (Status != XST_SUCCESS) {
		AxiEthernetUtilErrorTrap("Error setting options");
		return XST_FAILURE;
	}

	// we don't care about the receive channel for this example,
	// so turn it off, in case it was turned on earlier
	Status = XAxiEthernet_ClearOptions(AxiEthernetInstancePtr, XAE_RECEIVER_ENABLE_OPTION);
	if (Status != XST_SUCCESS) {
		AxiEthernetUtilErrorTrap("Error clearing option");
		return XST_FAILURE;
	}

	/*
	 * Set the interrupt coalescing parameters for the test. The waitbound
	 * timer is set to 1 (ms) to catch the last few frames.
	 *
	 * If you set variable Threshold to some value larger than TXBD_CNT,
	 * then there can never be enough frames sent to meet the threshold.
	 * In this case the waitbound timer will always cause the interrupt to
	 * occur.
	 */
	Status = XAxiDma_BdRingSetCoalesce(TxRingPtr, Threshold, 255);
	if (Status != XST_SUCCESS) {
		AxiEthernetUtilErrorTrap("Error setting coalescing settings");
		return XST_FAILURE;
	}

	/*
	 * Enable the send interrupts. Nothing should be transmitted yet as the
	 * device has not been started
	 */
	XAxiDma_BdRingIntEnable(TxRingPtr, XAXIDMA_IRQ_ALL_MASK);

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
 * This function sets uo Axi Ethernet and Axi DMA with respective configs.
 *
 * @return	XST_SUCCESS if successful, else XST_FAILURE.
 *****************************************************************************/
int SetupEthernet()
{
	int Status;
	int LoopbackSpeed;
	XAxiEthernet_Config *MacCfgPtr;
	XAxiDma_Config* DmaConfig;
	XAxiEthernet* AxiEthernetInstancePtr = &AxiEthernetInstance;
	XAxiDma* EthernetDmaPtr = &EthernetDmaInstancePtr;
	XAxiDma_BdRing* TxRingPtr = XAxiDma_GetTxRing(EthernetDmaPtr);
	XAxiDma_Bd BdTemplate;

	MacCfgPtr = XAxiEthernet_LookupConfig(AXIETHERNET_DEVICE_ID);
	if (MacCfgPtr->AxiDevType != XPAR_AXI_DMA) {
		AxiEthernetUtilErrorTrap("Device HW not configured for SGDMA mode\r\n");
		return XST_FAILURE;
	}

	DmaConfig = XAxiDma_LookupConfig(AXIDMA_DEVICE_ID);
	Status = XAxiDma_CfgInitialize(EthernetDmaPtr, DmaConfig);
	if (Status != XST_SUCCESS) {
		AxiEthernetUtilErrorTrap("Error initializing DMA\r\n");
		return XST_FAILURE;
	}

	Status = XAxiEthernet_CfgInitialize(AxiEthernetInstancePtr, MacCfgPtr, MacCfgPtr->BaseAddress);
	if (Status != XST_SUCCESS) {
		AxiEthernetUtilErrorTrap("Error initializing AxiEthernet\r\n");
		return XST_FAILURE;
	}

	Status = XAxiEthernet_SetMacAddress(AxiEthernetInstancePtr,	AxiEthernetMAC);
	if (Status != XST_SUCCESS) {
		AxiEthernetUtilErrorTrap("Error setting MAC address\r\n");
		return XST_FAILURE;
	}

	// Create the TxBD ring
	Status = XAxiDma_BdRingCreate(TxRingPtr, (UINTPTR) &TxBdSpace, (UINTPTR) &TxBdSpace, BD_ALIGNMENT, TXBD_CNT);
	if (Status != XST_SUCCESS) {
		AxiEthernetUtilErrorTrap("Error setting up TxBD space");
		return XST_FAILURE;
	}

	// We reuse the bd template, as the same one will work for both Rx and Tx.
	Status = XAxiDma_BdRingClone(TxRingPtr, &BdTemplate);
	if (Status != XST_SUCCESS) {
		AxiEthernetUtilErrorTrap("Error initializing TxBD space");
		return XST_FAILURE;
	}

	if (XAxiEthernet_GetPhysicalInterface(AxiEthernetInstancePtr) == XAE_PHY_TYPE_MII)
		LoopbackSpeed = AXIETHERNET_LOOPBACK_SPEED;
	else
		LoopbackSpeed = AXIETHERNET_LOOPBACK_SPEED_1G;

	AxiEthernetUtilEnterLoopback(AxiEthernetInstancePtr, LoopbackSpeed);



	// Set PHY<-->MAC data clock
	Status = XAxiEthernet_SetOperatingSpeed(AxiEthernetInstancePtr, (u16) LoopbackSpeed);
	if (Status != XST_SUCCESS)
		return XST_FAILURE;

	/*
	 * Setting the operating speed of the MAC needs a delay.  There
	 * doesn't seem to be register to poll, so please consider this
	 * during your application design.
	 */
	AxiEthernetUtilPhyDelay(2);

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
 *
 * This is the DMA TX callback function to be called by TX interrupt handler.
 * This function handles BDs finished by hardware.
 *
 * @param    TxRingPtr is a pointer to TX channel of the DMA engine.
 *
 * @return   None.
 *
 * @note     None.
 *
 *****************************************************************************/
void TxCallBack(XAxiDma_BdRing *TxRingPtr)
{
 //	XAxiDma_BdRingIntDisable(TxRingPtr, XAXIDMA_IRQ_ALL_MASK);
	int Status;
	int NumBd;
	int Threshold = TX_PACKET_CNT;
	XAxiDma_Bd *BdPtr;

	if (!XAxiDma_Busy(&EthernetDmaInstancePtr, XAXIDMA_DMA_TO_DEVICE)) {
		if(!IsInTxCriticalSection) {
			BdPtr = (XAxiDma_Bd*) TxRingPtr->HwTail;
			u32 BdSts = XAxiDma_BdRead(BdPtr, XAXIDMA_BD_STS_OFFSET);
			if (!(BdSts & XAXIDMA_BD_STS_COMPLETE_MASK)) xil_printf("ISR: ERROR, TxDma is idle even before all TxBds are not completed \r\n");

			XTime_GetTime(&intStart);
			accumTime[6] = NRV_IntTime(intEnd, intStart) / TX_BURST_CNT;
			++accumCntr[6];

			FrmRdDoneCallBack();
			//xil_printf("ISR  : NumOfEmptyBuf/rd_setup_ptr/rd_work_ptr = %d,%d, %d\r\n",NumOfEmptyBuf,g_rd_setup_ptr,g_rd_work_ptr);

			if (NumOfEmptyBuf < NUM_OF_BUFFER) {	//To be more strict, but slow, use this -> if (!RdDmaStopCond())
				if(TxTriggeredFlag) xil_printf("TxCallBack: Tx Procedure ERROR, FrmRdDoneCallBack() may not be called after Tx Done\r\n");
				Status = SendFrameSingleTxBurstSend();
				if (Status != XST_SUCCESS) {
					AxiEthernetUtilErrorTrap("Frame transmit failed");
					return XST_FAILURE;
				}
				TxTriggeredFlag = 1;
			}

			int NumBd = XAxiDma_BdRingFromHw(TxRingPtr, TX_PACKET_CNT, &BdPtr);
			if (NumBd != TX_PACKET_CNT) {
				xil_printf("Requesting NumBd: %d, ", NumBd);
				AxiEthernetUtilErrorTrap("Error in interrupt coalescing");
			} else {
				// Don't bother to check the BDs status, just free them
				Status = XAxiDma_BdRingFree(TxRingPtr, TX_PACKET_CNT, BdPtr);
				if (Status != XST_SUCCESS)
					AxiEthernetUtilErrorTrap("Error freeing TxBDs");
			}

			Status = SendFrameSingleTxBurstSetup((u8 *) g_FrmBuf[g_rd_setup_ptr]);
			if (Status != XST_SUCCESS) {
				xil_printf("Next Ethernet TxBD Setup failed\r\n");
				return XST_FAILURE;
			}
			NumOfReadyTxBDSet++;
			g_rd_setup_ptr = (g_rd_setup_ptr == (NUM_OF_BUFFER-1)) ? 0 : g_rd_setup_ptr + 1;
			//xil_printf("Main : NumOfEmptyBuf/rd_setup_ptr/rd_work_ptr = %d,%d, %d\r\n",NumOfEmptyBuf,g_rd_setup_ptr,g_rd_work_ptr);
		}
	} else {
		// NOTE: `nop`: Resume with enabled interrupts
	 //	XAxiDma_BdRingIntEnable(TxRingPtr, XAXIDMA_IRQ_ALL_MASK);
	}

}

/*****************************************************************************/
/**
 *
 * This is the DMA TX Interrupt handler function.
 *
 * @param	TxRingPtr is a pointer to TX channel of the DMA engine.
 *
 * @return	None.
 *
 * @note		This Interrupt handler MUST clear pending interrupts before
 *		handling them by calling the call back. Otherwise the following
 *		corner case could raise some issue:
 *
 *		A packet got transmitted and a TX interrupt got asserted. If
 *		the interrupt handler calls the callback before clearing the
 *		interrupt, a new packet may get transmitted in the callback.
 *		This new packet then can assert one more TX interrupt before
 *		the control comes out of the callback function. Now when
 *		eventually control comes out of the callback function, it will
 *		never know about the second new interrupt and hence while
 *		clearing the interrupts, would clear the new interrupt as well
 *		and will never process it.
 *		To avoid such cases, interrupts must be cleared before calling
 *		the callback.
 *
 *****************************************************************************/
void TxIntrHandler(XAxiDma_BdRing *TxRingPtr)
{
	u32 IrqStatus;

	// Read pending interrupts
	IrqStatus = XAxiDma_BdRingGetIrq(TxRingPtr);

	// Acknowledge pending interrupts
	XAxiDma_BdRingAckIrq(TxRingPtr, IrqStatus);
	/* If no interrupt is asserted, raise error flag, reset the
	 * hardware to recover from the error, and return with no further
	 * processing.
	 */
	if (!(IrqStatus & XAXIDMA_IRQ_ALL_MASK)) {
		DeviceErrors++;
		AxiEthernetUtilErrorTrap("AXIDma: No interrupts asserted in TX status register");
		XAxiDma_Reset(&EthernetDmaInstancePtr);
		if (!XAxiDma_ResetIsDone(&EthernetDmaInstancePtr))
			AxiEthernetUtilErrorTrap ("AxiDMA: Error: Could not reset\n");

		return;
	}

	/* If error interrupt is asserted, raise error flag, reset the
	 * hardware to recover from the error, and return with no further
	 * processing.
	 */
	if ((IrqStatus & XAXIDMA_IRQ_ERROR_MASK)) {

		AxiEthernetUtilErrorTrap("AXIDMA: TX Error interrupts\n");

		/* Reset should never fail for transmit channel
		 */
		XAxiDma_Reset(&EthernetDmaInstancePtr);
		if (!XAxiDma_ResetIsDone(&EthernetDmaInstancePtr))
			AxiEthernetUtilErrorTrap ("AXIDMA: Error: Could not reset\n");

		return;
	}

	/* If Transmit done interrupt is asserted, call TX call back function
	 * to handle the processed BDs and raise the according flag
	 */
	// complete�ÿ��� interrupt �߻��ϵ���
	if ((IrqStatus & (XAXIDMA_IRQ_IOC_MASK))) {
//	if ((IrqStatus & (XAXIDMA_IRQ_DELAY_MASK | XAXIDMA_IRQ_IOC_MASK))) {
		TxCallBack(TxRingPtr);
	}
}

/*****************************************************************************/
/**
 *
 * This is the Error handler callback function and this function increments the
 * the error counter so that the main thread knows the number of errors.
 *
 * @param	AxiEthernet is a reference to the Axi Ethernet device instance.
 *
 * @return	None.
 *
 * @note		None.
 *
 *****************************************************************************/
void AxiEthernetErrorHandler(XAxiEthernet *AxiEthernet)
{
	u32 Pending = XAxiEthernet_IntPending(AxiEthernet);

	if (Pending & XAE_INT_RXRJECT_MASK)
		AxiEthernetUtilErrorTrap("AxiEthernet: Rx packet rejected");

	if (Pending & XAE_INT_RXFIFOOVR_MASK)
		AxiEthernetUtilErrorTrap("AxiEthernet: Rx fifo over run");

	XAxiEthernet_IntClear(AxiEthernet, Pending);

	// Bump counter
	DeviceErrors++;
}

/******************************************************************************/
/**
*
* Set the frame VLAN info for the specified frame.
*
* @param	FramePtr is the pointer to the frame.
* @param	VlanNumber is the VlanValue insertion position to set in frame.
* @param	Vid  is the 4 bytes Vlan value (TPID, Priority, CFI, VID)
*		to be set in frame.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void AxiEthernetUtilFrameHdrVlanFormatVid(EthernetFrame *FramePtr, u32 VlanNumber,	u32 Vid)
{
	char *Frame = (char *) FramePtr;

	// Increment to type field
	Frame = Frame + 12 + (VlanNumber * 4);

	Vid = Xil_Htonl(Vid);

	// Set the type
	*(u32 *) Frame = Vid;
}

/******************************************************************************/
/**
*
* Set the frame type for the specified frame.
*
* @param	FramePtr is the pointer to the frame.
* @param	FrameType is the Type to set in frame.
* @param	VlanNumber is the VLAN friendly adjusted insertion position to
*		set in frame.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void AxiEthernetUtilFrameHdrVlanFormatType(EthernetFrame *FramePtr, u16 FrameType, u32 VlanNumber)
{
	char *Frame = (char *) FramePtr;

	// Increment to type field
	Frame = Frame + 12 + (VlanNumber * 4);

	FrameType = Xil_Htons(FrameType);

	// Set the type
	*(u16 *) Frame = FrameType;
}

/******************************************************************************/
/**
* This function places a pattern in the payload section of a frame. The pattern
* is a  8 bit incrementing series of numbers starting with 0.
* Once the pattern reaches 256, then the pattern changes to a 16 bit
* incrementing pattern:
* <pre>
*   0, 1, 2, ... 254, 255, 00, 00, 00, 01, 00, 02, ...
* </pre>
*
* @param	FramePtr is a pointer to the frame to change.
* @param	PayloadSize is the number of bytes in the payload that will be set.
* @param	VlanNumber is the VLAN friendly adjusted insertion position to
*		set in frame.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void AxiEthernetUtilFrameSetVlanPayloadData(EthernetFrame *FramePtr, int PayloadSize, u32 VlanNumber)
{
	unsigned BytesLeft = PayloadSize;
	u8 *Frame;
	u16 Counter = 0;

	// Set the frame pointer to the start of the payload area
	Frame = (u8 *) FramePtr + XAE_HDR_SIZE + (VlanNumber * 4);

	// Insert 8 bit incrementing pattern
	while (BytesLeft && (Counter < 256)) {
		*Frame++ = (u8) Counter++;
		BytesLeft--;
	}

	// Switch to 16 bit incrementing pattern
	while (BytesLeft) {
		*Frame++ = (u8) (Counter >> 8);	// high
		BytesLeft--;

		if (!BytesLeft)
			break;

		*Frame++ = (u8) Counter++;	// low
		BytesLeft--;
	}
}

/******************************************************************************/
/**
* This function verifies the frame data against a CheckFrame.
*
* Validation occurs by comparing the ActualFrame to the header of the
* CheckFrame. If the headers match, then the payload of ActualFrame is
* verified for the same pattern Util_FrameSetPayloadData() generates.
*
* @param	CheckFrame is a pointer to a frame containing the 14 byte header
*		that should be present in the ActualFrame parameter.
* @param	ActualFrame is a pointer to a frame to validate.
*
* @return
*		- XST_SUCCESS if successful.
*		- XST_FAILURE in case of failure.
*
* @note		None.
*
******************************************************************************/
int AxiEthernetUtilFrameVerify(EthernetFrame * CheckFrame, EthernetFrame * ActualFrame)
{
	unsigned char *CheckPtr = (unsigned char *) CheckFrame;
	unsigned char *ActualPtr = (unsigned char *) ActualFrame;
	u16 BytesLeft;
	u16 Counter;
	int Index;

	CheckPtr = CheckPtr + Padding;
	ActualPtr = ActualPtr + Padding;

	// Compare the headers
	for (Index = 0; Index < XAE_HDR_SIZE; Index++) {
		if (CheckPtr[Index] != ActualPtr[Index]) {
			return XST_FAILURE;
		}
	}

	Index = 0;

	BytesLeft = *(u16 *) &ActualPtr[12];
	BytesLeft = Xil_Ntohs(BytesLeft);
	/*
	 * Get the length of the payload, do not use VLAN TPID here.
	 * TPID needs to be verified.
	 */
	while ((0x8100 == BytesLeft) || (0x88A8 == BytesLeft) ||
	       (0x9100 == BytesLeft) || (0x9200 == BytesLeft)) {
		Index++;
		BytesLeft = *(u16 *) &ActualPtr[12+(4*Index)];
		BytesLeft = Xil_Ntohs(BytesLeft);
	}

	// Validate the payload
	Counter = 0;
	ActualPtr = &ActualPtr[14+(4*Index)];

	// Check 8 bit incrementing pattern
	while (BytesLeft && (Counter < 256)) {
		if (*ActualPtr++ != (u8) Counter++) {

			return XST_FAILURE;
		}
		BytesLeft--;
	}

	// Check 16 bit incrementing pattern
	while (BytesLeft) {
		if (*ActualPtr++ != (u8) (Counter >> 8)) {	// high
			return XST_FAILURE;
		}

		BytesLeft--;

		if (!BytesLeft)
			break;

		if (*ActualPtr++ != (u8) Counter++) {	// low
			return XST_FAILURE;
		}

		BytesLeft--;
	}

	return XST_SUCCESS;
}

/******************************************************************************/
/**
* This function sets all bytes of a frame to 0.
*
* @param	FramePtr is a pointer to the frame itself.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void AxiEthernetUtilFrameMemClear(EthernetFrame * FramePtr)
{
	u32 *Data32Ptr = (u32 *) FramePtr;
	u32 WordsLeft = sizeof(EthernetFrame) / sizeof(u32);

	// Frame should be an integral number of words
	while (WordsLeft--) {
		*Data32Ptr++ = 0;
	}
}

// Use MII register 1 (MII status register) to detect PHY
#define PHY_DETECT_REG  1

/* Mask used to verify certain PHY features (or register contents)
 * in the register above:
 *  0x1000: 10Mbps full duplex support
 *  0x0800: 10Mbps half duplex support
 *  0x0008: Auto-negotiation support
 */
#define PHY_DETECT_MASK 0x1808

/******************************************************************************/
/**
*
* This function detects the PHY address by looking for successful MII status
* register contents (PHY register 1). It looks for a PHY that supports
* auto-negotiation and 10Mbps full-duplex and half-duplex.  So, this code
* won't work for PHYs that don't support those features, but it's a bit more
* general purpose than matching a specific PHY manufacturer ID.
*
* Note also that on some (older) Xilinx ML4xx boards, PHY address 0 does not
* properly respond to this query.  But, since the default is 0 and assuming
* no other address responds, then it seems to work OK.
*
* @param	AxiEthernetInstancePtr is the Axi Ethernet driver instance
*
* @return	The address of the PHY (defaults to 0 if none detected)
*
* @note		None.
*
******************************************************************************/
u32 AxiEthernetDetectPHY(XAxiEthernet * AxiEthernetInstancePtr)
{
	u16 PhyReg;
	int PhyAddr;

	for (PhyAddr = 31; PhyAddr >= 0; PhyAddr--) {
		XAxiEthernet_PhyRead(AxiEthernetInstancePtr, PhyAddr, PHY_DETECT_REG, &PhyReg);

		if ((PhyReg != 0xFFFF) && ((PhyReg & PHY_DETECT_MASK) == PHY_DETECT_MASK)) {
			// Found a valid PHY address
			return PhyAddr;
		}
	}

	return 0;		// Default to zero
}

/******************************************************************************/
/**
* Set PHY to loopback mode. This works with the marvell PHY common on ML40x
* evaluation boards
*
* @param Speed is the loopback speed 10, 100, or 1000 Mbit
*
******************************************************************************/
// IEEE PHY Specific definitions
#define PHY_R0_CTRL_REG		0
#define PHY_R3_PHY_IDENT_REG	3

#define PHY_R0_RESET         0x8000
#define PHY_R0_LOOPBACK      0x4000
#define PHY_R0_ANEG_ENABLE   0x1000
#define PHY_R0_DFT_SPD_MASK  0x2040
#define PHY_R0_DFT_SPD_10    0x0000
#define PHY_R0_DFT_SPD_100   0x2000
#define PHY_R0_DFT_SPD_1000  0x0040
#define PHY_R0_DFT_SPD_2500  0x0040
#define PHY_R0_ISOLATE       0x0400

// Marvel PHY 88E1111 Specific definitions
#define PHY_R20_EXTND_CTRL_REG	20
#define PHY_R27_EXTND_STS_REG	27

#define PHY_R20_DFT_SPD_10    	0x20
#define PHY_R20_DFT_SPD_100   	0x50
#define PHY_R20_DFT_SPD_1000  	0x60
#define PHY_R20_RX_DLY		0x80

#define PHY_R27_MAC_CONFIG_GMII      0x000F
#define PHY_R27_MAC_CONFIG_MII       0x000F
#define PHY_R27_MAC_CONFIG_RGMII     0x000B
#define PHY_R27_MAC_CONFIG_SGMII     0x0004

// Marvel PHY 88E1116R Specific definitions
#define PHY_R22_PAGE_ADDR_REG	22
#define PHY_PG2_R21_CTRL_REG	21

#define PHY_REG21_10      0x0030
#define PHY_REG21_100     0x2030
#define PHY_REG21_1000    0x0070

// Marvel PHY flags
#define MARVEL_PHY_88E1111_MODEL	0xC0
#define MARVEL_PHY_88E1116R_MODEL	0x240
#define PHY_MODEL_NUM_MASK		0x3F0

// TI PHY flags
#define TI_PHY_IDENTIFIER		0x2000
#define TI_PHY_MODEL			0x230
#define TI_PHY_CR			0xD
#define TI_PHY_PHYCTRL			0x10
#define TI_PHY_CR_SGMII_EN		0x0800
#define TI_PHY_ADDDR			0xE
#define TI_PHY_CFGR2			0x14
#define TI_PHY_SGMIITYPE		0xD3
#define TI_PHY_CFGR2_SGMII_AUTONEG_EN	0x0080
#define TI_PHY_SGMIICLK_EN		0x4000
#define TI_PHY_CR_DEVAD_EN		0x001F
#define TI_PHY_CR_DEVAD_DATAEN		0x4000

/******************************************************************************/
/**
*
* This function sets the PHY to loopback mode. This works with the marvell PHY
* common on ML40x evaluation boards.
*
* @param	AxiEthernetInstancePtr is a pointer to the instance of the
*		AxiEthernet component.
* @param	Speed is the loopback speed 10, 100, or 1000 Mbit.
*
* @return
*		- XST_SUCCESS if successful.
*		- XST_FAILURE, in case of failure..
*
* @note		None.
*
******************************************************************************/
int AxiEthernetUtilEnterLoopback(XAxiEthernet *AxiEthernetInstancePtr, int Speed)
{
	u16 PhyReg0;
	signed int PhyAddr;
	u8 PhyType;
	u16 PhyModel;
	u16 PhyReg20;	// Extended PHY specific Register (Reg 20) of Marvell 88E1111 PHY
	u16 PhyReg21;	// Control Register MAC (Reg 21) of Marvell 88E1116R PHY

	// Get the Phy Interface
	PhyType = XAxiEthernet_GetPhysicalInterface(AxiEthernetInstancePtr);

	// Detect the PHY address
	if (PhyType != XAE_PHY_TYPE_1000BASE_X)
		PhyAddr = AxiEthernetDetectPHY(AxiEthernetInstancePtr);
	else
		PhyAddr = XPAR_AXIETHERNET_0_PHYADDR;

	XAxiEthernet_PhyRead(AxiEthernetInstancePtr, PhyAddr, PHY_R3_PHY_IDENT_REG, &PhyModel);
	PhyModel = PhyModel & PHY_MODEL_NUM_MASK;

	// Clear the PHY of any existing bits by zeroing this out
	PhyReg0 = PhyReg20 = PhyReg21 = 0;

	switch (Speed) {
		case XAE_SPEED_10_MBPS:
			PhyReg0 |= PHY_R0_DFT_SPD_10;
			PhyReg20 |= PHY_R20_DFT_SPD_10;
			PhyReg21 |= PHY_REG21_10;
			break;

		case XAE_SPEED_100_MBPS:
			PhyReg0 |= PHY_R0_DFT_SPD_100;
			PhyReg20 |= PHY_R20_DFT_SPD_100;
			PhyReg21 |= PHY_REG21_100;
			break;

		case XAE_SPEED_1000_MBPS:
			PhyReg0 |= PHY_R0_DFT_SPD_1000;
			PhyReg20 |= PHY_R20_DFT_SPD_1000;
			PhyReg21 |= PHY_REG21_1000;
			break;

		case XAE_SPEED_2500_MBPS:
			PhyReg0 |= PHY_R0_DFT_SPD_2500;
			PhyReg20 |= PHY_R20_DFT_SPD_1000;
			PhyReg21 |= PHY_REG21_1000;
			break;

		default:
			AxiEthernetUtilErrorTrap("Intg_LinkSpeed not 10, 100, or 1000 mbps");
			return XST_FAILURE;
	}

	// RGMII mode Phy specific registers initialization
	if ((PhyType == XAE_PHY_TYPE_RGMII_2_0) || (PhyType == XAE_PHY_TYPE_RGMII_1_3)) {
		if (PhyModel == MARVEL_PHY_88E1111_MODEL) {
			PhyReg20 |= PHY_R20_RX_DLY;
			// Adding Rx delay. Configuring loopback speed.
			XAxiEthernet_PhyWrite(AxiEthernetInstancePtr, PhyAddr,
						PHY_R20_EXTND_CTRL_REG, PhyReg20);
		} else if (PhyModel == MARVEL_PHY_88E1116R_MODEL) {
			// Switching to PAGE2
			XAxiEthernet_PhyWrite(AxiEthernetInstancePtr, PhyAddr,
						PHY_R22_PAGE_ADDR_REG, 2);
			// Adding Tx and Rx delay. Configuring loopback speed.
			XAxiEthernet_PhyWrite(AxiEthernetInstancePtr, PhyAddr,
						PHY_PG2_R21_CTRL_REG, PhyReg21);
			// Switching to PAGE0
			XAxiEthernet_PhyWrite(AxiEthernetInstancePtr, PhyAddr,
						PHY_R22_PAGE_ADDR_REG, 0);
		}
		PhyReg0 &= (~PHY_R0_ANEG_ENABLE);
	}

	// Configure interface modes
	if (PhyModel == MARVEL_PHY_88E1111_MODEL) {
		if ((PhyType == XAE_PHY_TYPE_RGMII_2_0) || (PhyType == XAE_PHY_TYPE_RGMII_1_3))  {
			XAxiEthernet_PhyWrite(AxiEthernetInstancePtr, PhyAddr,
						PHY_R27_EXTND_STS_REG, PHY_R27_MAC_CONFIG_RGMII);
		} else if (PhyType == XAE_PHY_TYPE_SGMII) {
			XAxiEthernet_PhyWrite(AxiEthernetInstancePtr, PhyAddr,
						PHY_R27_EXTND_STS_REG, PHY_R27_MAC_CONFIG_SGMII);
		} else if ((PhyType == XAE_PHY_TYPE_GMII) || (PhyType == XAE_PHY_TYPE_MII)) {
			XAxiEthernet_PhyWrite(AxiEthernetInstancePtr, PhyAddr,
						PHY_R27_EXTND_STS_REG, PHY_R27_MAC_CONFIG_GMII );
		}
	}

	// Set the speed and put the PHY in reset, then put the PHY in loopback
	XAxiEthernet_PhyWrite(AxiEthernetInstancePtr, PhyAddr, PHY_R0_CTRL_REG, PhyReg0 | PHY_R0_RESET);
	AxiEthernetUtilPhyDelay(AXIETHERNET_PHY_DELAY_SEC);
	XAxiEthernet_PhyRead(AxiEthernetInstancePtr, PhyAddr, PHY_R0_CTRL_REG, &PhyReg0);
	if (!ExternalLoopback) {
		XAxiEthernet_PhyWrite(AxiEthernetInstancePtr, PhyAddr,
					PHY_R0_CTRL_REG, PhyReg0 & ~PHY_R0_LOOPBACK);
	}

	if ((PhyModel == TI_PHY_MODEL) && (PhyType == XAE_PHY_TYPE_SGMII)) {
		XAxiEthernet_PhyRead(AxiEthernetInstancePtr, PhyAddr,
				    PHY_R0_CTRL_REG, &PhyReg0);
		PhyReg0 &= (~PHY_R0_ANEG_ENABLE);
		XAxiEthernet_PhyWrite(AxiEthernetInstancePtr, PhyAddr,
					PHY_R0_CTRL_REG, PhyReg0 );
		AxiEtherentConfigureTIPhy(AxiEthernetInstancePtr, PhyAddr);
	}

	if ((PhyType == XAE_PHY_TYPE_SGMII) || (PhyType == XAE_PHY_TYPE_1000BASE_X))
		AxiEthernetUtilConfigureInternalPhy(AxiEthernetInstancePtr, Speed);

	AxiEthernetUtilPhyDelay(1);

	return XST_SUCCESS;
}

/******************************************************************************/
/**
*
* This function is called by example code when an error is detected. It
* can be set as a breakpoint with a debugger or it can be used to print out the
* given message if there is a UART or STDIO device.
*
* @param	Message is the text explaining the error
*
* @return	None
*
* @note		None
*
******************************************************************************/
void AxiEthernetUtilErrorTrap(char *Message)
{
	static int Count = 0;

	Count++;

#ifdef STDOUT_BASEADDRESS
	xil_printf("%s\r\n", Message);
#endif
}

/******************************************************************************/
/**
*
* For Microblaze we use an assembly loop that is roughly the same regardless of
* optimization level, although caches and memory access time can make the delay
* vary.  Just keep in mind that after resetting or updating the PHY modes,
* the PHY typically needs time to recover.
*
* @param	Seconds is the number of seconds to delay
*
* @return	None
*
* @note		None
*
******************************************************************************/
void AxiEthernetUtilPhyDelay(unsigned int Seconds)
{
#if defined (__MICROBLAZE__) || defined(__PPC__)
	static int WarningFlag = 0;

	/* If MB caches are disabled or do not exist, this delay loop could
	 * take minutes instead of seconds (e.g., 30x longer).  Print a warning
	 * message for the user (once).  If only MB had a built-in timer!
	 */
	if (((mfmsr() & 0x20) == 0) && (!WarningFlag)) {
		WarningFlag = 1;
	}

#define ITERS_PER_SEC   (XPAR_CPU_CORE_CLOCK_FREQ_HZ / 6)
    asm volatile ("\n"
			"1:               \n\t"
			"addik r7, r0, %0 \n\t"
			"2:               \n\t"
			"addik r7, r7, -1 \n\t"
			"bneid  r7, 2b    \n\t"
			"or  r0, r0, r0   \n\t"
			"bneid %1, 1b     \n\t"
			"addik %1, %1, -1 \n\t"
			:: "i"(ITERS_PER_SEC), "d" (Seconds));
#else
    sleep(Seconds);
#endif
}

/******************************************************************************/
/**
*
* This function configures the internal phy for SGMII and 1000baseX modes.
* *
* @param	AxiEthernetInstancePtr is a pointer to the instance of the
*		AxiEthernet component.
* @param	Speed is the loopback speed 10, 100, or 1000 Mbit.
*
* @return
*		- XST_SUCCESS if successful.
*		- XST_FAILURE, in case of failure..
*
* @note		None.
*
******************************************************************************/
int AxiEthernetUtilConfigureInternalPhy(XAxiEthernet *AxiEthernetInstancePtr, int Speed)
{
	u16 PhyReg0;
	signed int PhyAddr;

	PhyAddr = XPAR_AXIETHERNET_0_PHYADDR;

	// Clear the PHY of any existing bits by zeroing this out
	PhyReg0 = 0;
	XAxiEthernet_PhyRead(AxiEthernetInstancePtr, PhyAddr, PHY_R0_CTRL_REG, &PhyReg0);

	PhyReg0 &= (PHY_R0_ANEG_ENABLE);
	PhyReg0 &= (PHY_R0_ISOLATE);

	switch (Speed) {
		case XAE_SPEED_10_MBPS:
			PhyReg0 |= PHY_R0_DFT_SPD_10;
			break;
		case XAE_SPEED_100_MBPS:
			PhyReg0 |= PHY_R0_DFT_SPD_100;
			break;
		case XAE_SPEED_1000_MBPS:
			PhyReg0 |= PHY_R0_DFT_SPD_1000;
			break;
		case XAE_SPEED_2500_MBPS:
			PhyReg0 |= PHY_R0_DFT_SPD_2500;
			break;
		default:
			AxiEthernetUtilErrorTrap("Intg_LinkSpeed not 10, 100, or 1000 mbps\n\r");
			return XST_FAILURE;
	}

	AxiEthernetUtilPhyDelay(1);
	XAxiEthernet_PhyWrite(AxiEthernetInstancePtr, PhyAddr, PHY_R0_CTRL_REG, PhyReg0);
	return XST_SUCCESS;
}

int AxiEtherentConfigureTIPhy(XAxiEthernet *AxiEthernetInstancePtr, u32 PhyAddr)
{
	u16 PhyReg14;

	// Enable SGMII Clock
	XAxiEthernet_PhyWrite(AxiEthernetInstancePtr, PhyAddr,
				TI_PHY_CR, TI_PHY_CR_DEVAD_EN);
	XAxiEthernet_PhyWrite(AxiEthernetInstancePtr, PhyAddr,
				TI_PHY_ADDDR, TI_PHY_SGMIITYPE);
	XAxiEthernet_PhyWrite(AxiEthernetInstancePtr, PhyAddr,
				TI_PHY_CR, TI_PHY_CR_DEVAD_EN | TI_PHY_CR_DEVAD_DATAEN);
	XAxiEthernet_PhyWrite(AxiEthernetInstancePtr, PhyAddr,
				TI_PHY_ADDDR, TI_PHY_SGMIICLK_EN);

	// Enable SGMII
	XAxiEthernet_PhyWrite(AxiEthernetInstancePtr, PhyAddr,
				TI_PHY_PHYCTRL, TI_PHY_CR_SGMII_EN);
	XAxiEthernet_PhyRead(AxiEthernetInstancePtr, PhyAddr,
				TI_PHY_CFGR2, &PhyReg14);
	XAxiEthernet_PhyWrite(AxiEthernetInstancePtr, PhyAddr,
				TI_PHY_CFGR2, PhyReg14 & (~TI_PHY_CFGR2_SGMII_AUTONEG_EN));
	XAxiEthernet_PhyRead(AxiEthernetInstancePtr, PhyAddr,
				TI_PHY_CFGR2, &PhyReg14);

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
 * This function is used to check elapsed time between two `XTime` time stamps.
 * Both time stamps are assumed to be taken at any point prior to calling this
 * function.
 *
 * This function was originally planned as an inline function, however the
 * Vitis compiler unfortunately throws an error when using inline functions.
 * :(
 *
 * @param	t1 is the first time stamp
 * @param	t2 is the second time stamp
 *
 * @return	elapsed time in microseconds as a `float`
 *
 *****************************************************************************/
float NRV_GetTime(XTime t1, XTime t2)
{
   return 1.0 * (t2 - t1) / (COUNTS_PER_SECOND / 1000000);
}

/*****************************************************************************/
/**
 * This function is used to check elapsed time between two `XTime` time stamps.
 * Both time stamps are assumed to be taken at any point prior to calling this
 * function. Unlike `NRV_GetTime()`, this function returns an integer value.
 *
 * @param	t1 is the first time stamp
 * @param	t2 is the second time stamp
 *
 * @return	elapsed time in microseconds as an `int`
 *
 *****************************************************************************/
int NRV_IntTime(XTime t1, XTime t2)
{
   return (t2 - t1) / (COUNTS_PER_SECOND / 1000000);
}

/*****************************************************************************/
/**
 * This function sets up TxBDs for the Ethernet DMA engine. It allocates all
 * BDs and assigns them to the same buffer, according to a pre-defined size
 * `FRAMES_THRESHOLD`, which must remain (by restriction of the Ethernet device)
 * less than 255.
 *
 * This function has been split with `SendFrameSingleTxBurstSend()` to allow
 * the user to set buffer descriptors for the next iteration during the
 * busy-wait time for the current iteration's Ethernet transmission.
 * To send the buffer descriptors allocated and set up here, the user must call
 * `SendFrameSingleTxBurstSend()`.
 *
 * @param	SendingAddress is the starting address of the packet data to be sent
 * 			by the Ethernet DMA engine.
 *
 * @return	XST_SUCCESS if successful, else XST_FAILURE.
 *****************************************************************************/
int SendFrameSingleTxBurstSetup(u8 *SendingAddress)
{
	int Status;
	u32 TxFrameLength;
	u32 Index;
	u32 NumBd;
	int PayloadSize = PAYLOAD_SIZE;
	int Threshold = TX_PACKET_CNT;
	XAxiDma_Bd *BdPtr;
	XAxiDma_Bd *BdCurPtr;
	XAxiEthernet* AxiEthernetInstancePtr = &AxiEthernetInstance;
	XAxiDma* EthernetDmaPtr = &EthernetDmaInstancePtr;
	XAxiDma_BdRing* TxRingPtr = XAxiDma_GetTxRing(EthernetDmaPtr);
	XTime start, end; // DEBUG: Time debuggers

	// clear variables shared with callbacks
	DeviceErrors = 0;

	// calculate the frame length (not including FCS)
	TxFrameLength = PACKET_SIZE;

	XTime_GetTime(&start);
	// Prime the engine, allocate all BDs and assign them to the same buffer
	Status = XAxiDma_BdRingAlloc(TxRingPtr, Threshold, &BdPtr);
	if (Status != XST_SUCCESS) {
		AxiEthernetUtilErrorTrap("Error allocating TxBDs prior to starting");
		return XST_FAILURE;
	}
	XTime_GetTime(&end);
	accumTime[0] = NRV_IntTime(start, end);
	++accumCntr[0];

	// Setup the TxBDs
	XTime_GetTime(&start);
	BdCurPtr = BdPtr;
	for (Index = 0; Index < Threshold; Index++) {
		XAxiDma_BdSetBufAddr(BdCurPtr, (UINTPTR) SendingAddress);
		SendingAddress += PACKET_SIZE;
		XAxiDma_BdSetLength(BdCurPtr, TxFrameLength, TxRingPtr->MaxTransferLen);
		XAxiDma_BdSetCtrl(BdCurPtr, XAXIDMA_BD_CTRL_TXSOF_MASK | XAXIDMA_BD_CTRL_TXEOF_MASK);
		BdCurPtr = (XAxiDma_Bd *) XAxiDma_BdRingNext(TxRingPtr, BdCurPtr);
	}
	XTime_GetTime(&end);
	accumTime[1] = NRV_IntTime(start, end);
	++accumCntr[1];

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
 * This function takes previously set up TxBDs and sends them to the Ethernet
 * DMA engine. It is assumed that the user has already called a corresponding
 * `SendFrameSingleTxBurstSetup()` to set up the BDs.
 *
 * This function has been split with `SendFrameSingleTxBurstSetup()` to allow
 * the user to set buffer descriptors for the next iteration during the
 * busy-wait time for the current iteration's Ethernet transmission.
 *
 * Notably, this function neither retrieves used buffer descriptors from the
 * DMA engine nor frees them. This is done by the callback function
 * `TxCallBack()` instead, which is called by the DMA engine when it finishes
 * a single batch transmission.
 *
 * @param	none.
 *
 * @return	XST_SUCCESS if successful, else XST_FAILURE.
 *****************************************************************************/
int SendFrameSingleTxBurstSend(void)
{
	int Status;
	XAxiEthernet* AxiEthernetInstancePtr = &AxiEthernetInstance;
	XAxiDma* EthernetDmaPtr = &EthernetDmaInstancePtr;
	XAxiDma_BdRing* TxRingPtr = XAxiDma_GetTxRing(EthernetDmaPtr);
	int Threshold = TX_PACKET_CNT;
	XAxiDma_Bd *BdPtr = XAxiDma_GetTxRing(&EthernetDmaInstancePtr)->PreHead;
	XTime start, end; // DEBUG: Time debuggers

	//  Enqueue all TxBDs to HW
	XTime_GetTime(&start);
	Status = XAxiDma_BdRingToHw(TxRingPtr, Threshold, BdPtr);
	if (Status != XST_SUCCESS) {
		AxiEthernetUtilErrorTrap("Error committing TxBDs prior to starting");
		return XST_FAILURE;
	}

	XTime_GetTime(&end);
	accumTime[2] = NRV_IntTime(start, end);
	++accumCntr[2];

	// Start DMA TX channel. Transmission starts at once
	XTime_GetTime(&start);
	Status = XAxiDma_BdRingStart(TxRingPtr);
	if (Status != XST_SUCCESS)
		return XST_FAILURE;

	// Start the device. Transmission commences now!
	XAxiEthernet_Start(AxiEthernetInstancePtr);
	XAxiDma_BdRingIntEnable(TxRingPtr, XAXIDMA_IRQ_ALL_MASK);
	XTime_GetTime(&intEnd);
	accumTime[3] = NRV_IntTime(start, intEnd);
	++accumCntr[3];
	if (intEnd > intStart) {
		accumTime[5] = NRV_IntTime(intStart, intEnd);
		++accumCntr[5];
	}

	return XST_SUCCESS;
}
