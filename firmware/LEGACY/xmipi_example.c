/******************************************************************************
* Copyright (C) 2017 - 2020 Xilinx, Inc.  All rights reserved.
* SPDX-License-Identifier: MIT
 *****************************************************************************/

/*****************************************************************************/
/**
 *
 * @file xmipi_example.c
 *
 * This file demonstrates the Xilinx MIPI CSI2 Rx Subsystem and MIPI DSI2 Tx
 * Subsystem. The video pipeline is created by connecting an IMX274 Camera
 * sensor to the MIPI CSI2 Rx Subsystem. The sensor is programmed to generate
 * RAW10 type de bayered data as per the pipeline configuration. The raw pixels
 * are fed to Xilinx Demosaic, Gamma lut and v_proc_ss IPs to convert pixel
 * to RGB format. The RGB pixels are then sent across to a data Video
 * Test Pattern Generator. In a pass through mode, the camera data is passed
 * to an AXI Stream broadcaster. This sends across video stream to along
 * HDMI Tx Subsystem and a Video Processing Subsystem configured as Scalar.
 * The output of the scalar is connected to the DSI2 Tx Subsystem. The DSI2
 * output is connected to AUO Asus Display panel with 1920x1200 fixed resolution
 *
 *
 * <pre>
 * MODIFICATION HISTORY:
 *
 * Ver   Who    Date     Changes
 * ----- ------ -------- --------------------------------------------------
 * 1.00  pg    12/07/17 Initial release.
 * </pre>
 *
 *****************************************************************************/

/***************************** Include Files *********************************/

#include <stdio.h>
#include "platform.h"
#include "xparameters.h"
#include "xil_cache.h"
#include "xiic.h"
#include "xil_io.h"
#include "xuartps.h"
#include "xil_types.h"
#include "xil_exception.h"
#include "string.h"
#include "si5324drv.h"
#include "xvidc.h"
#include "xvidc_edid.h"
#include "sleep.h"
#include "xgpio.h"
#include "xscugic.h"
#include "sensor_cfgs.h"
#include "xmipi_menu.h"
#include "pipeline_program.h"
#include "xil_mmu.h"

#include "xaxidma.h"
#include "xtmrctr.h"

// for Ethernet
#include "xaxiethernet_example.h"



/************************** Constant Definitions *****************************/

#define RX_DMA_CYCLIC_MODE	1	//ALWAYS 1

#define SLOW_DOWN_TX 0
#define TX_GO_AHEAD_WITHOUT_CHECKING_RX_DATA_IS_READY	0

#define DEBUG_FEATURES 1



#define MIPI_CONTROLLER_mWriteReg(BaseAddress, RegOffset, Data) \
  	Xil_Out32((BaseAddress) + (RegOffset), (u32)(Data))
#define MIPI_CONTROLLER_mReadReg(BaseAddress, RegOffset) \
  	Xil_In32((BaseAddress) + (RegOffset))

#define MIPI_CONTROLLER_S00_AXI_SLV_REG0_OFFSET 0
#define MIPI_CONTROLLER_S00_AXI_SLV_REG1_OFFSET 4
#define MIPI_CONTROLLER_S00_AXI_SLV_REG2_OFFSET 8
#define MIPI_CONTROLLER_S00_AXI_SLV_REG3_OFFSET 12 // frame_dump_num_reg_val
#define MIPI_CONTROLLER_S00_AXI_SLV_REG4_OFFSET 16 // frame_control_reg_val
#define MIPI_CONTROLLER_S00_AXI_SLV_REG5_OFFSET 20 // first fifo fps val
#define MIPI_CONTROLLER_S00_AXI_SLV_REG6_OFFSET 24 // second fifo fps val

/***************** Macros (Inline Functions) Definitions *********************/
#define UART_BASEADDR	XPAR_XUARTPS_0_BASEADDR

#define IIC_SENSOR_INTR_ID	XPAR_FABRIC_AXI_IIC_0_IIC2INTC_IRPT_INTR

#define GPIO_SENSOR		XPAR_AXI_GPIO_0_BASEADDR

#ifdef XPAR_PSU_ACPU_GIC_DEVICE_ID
#define PSU_INTR_DEVICE_ID	XPAR_PSU_ACPU_GIC_DEVICE_ID
#endif

#ifdef XPAR_PSU_RCPU_GIC_DEVICE_ID
#define PSU_INTR_DEVICE_ID	XPAR_PSU_RCPU_GIC_DEVICE_ID
#endif

#define DMA_DEV_ID		XPAR_AXIDMA_0_DEVICE_ID
#define RX_INTR_ID		XPAR_FABRIC_AXI_DMA_0_S2MM_INTROUT_INTR

#define TMRCTR_DEVICE_ID	XPAR_TMRCTR_0_DEVICE_ID
#define TMRCTR_INTERRUPT_ID	XPAR_FABRIC_TMRCTR_0_VEC_ID

#define RESET_TIMEOUT_COUNTER	10000

#define INTC		XScuGic
#define INTC_HANDLER	XScuGic_InterruptHandler

#define TIMER_CNTR_0 0 // 1 seconds for 200MHz
#define RESET_VALUE	 0xF4143DFF // 1 seconds for 249.975021MHz (250MHz set with IOPLL) PL Clock

#define NUM_TIMER_LOOP 10

#define PLATFORM_EMAC_BASEADDR XPAR_XEMACPS_0_BASEADDR

#define TIMER_DEVICE_ID     XPAR_XTTCPS_0_DEVICE_ID
#define TIMER_IRPT_INTR     XPAR_XTTCPS_0_INTR
#define INTC_BASE_ADDR      XPAR_SCUGIC_0_CPU_BASEADDR
#define INTC_DIST_BASE_ADDR XPAR_SCUGIC_0_DIST_BASEADDR

#define PLATFORM_TIMER_INTR_RATE_HZ 2

// for CIS Sensor 6 Ignored area of effective pixel + 4 Effective margin for ISP + 3240 BDs + 4 Effective margin for ISP
#define NUMBER_OF_BDS_TO_TRANSFER	120	//1440 byte * 120 = 960*720*2 bit
#define SINGLE_VIDEO_FRAME_RXBD_BYTES	(XAxiDma_BdRingMemCalc(BD_ALIGNMENT, NUMBER_OF_BDS_TO_TRANSFER))
#define RXBD_SPACE_BYTES	(SINGLE_VIDEO_FRAME_RXBD_BYTES * NUM_OF_BUFFER)

#define DDR_BASE_ADDR	XPAR_PSU_DDR_0_S_AXI_BASEADDR
#define MEM_BASE_ADDR		(DDR_BASE_ADDR + 0x01000000 + RXBD_SPACE_BYTES)	//16 MBytes Code Space + Rx BD Space
#define BUFFER_BASE_ADDR	(MEM_BASE_ADDR + RXBD_SPACE_BYTES)

//CIS BD Setup Help defines
#define BD_LEN_1440		0x5A0	// 1440 bytes
#define BD_LEN_960		0x3C0	// 960 bytes
#define BD_LEN_480		0x1E0	// 480 bytes
#define BD_LEN_12		0xC	// 12 bytes
#define BD_LEN_1932		0x78C	// 1932 bytes
#define BD_LEN_644		0x284	// 644 bytes

/**************************** Type Definitions *******************************/

/************************** Function Prototypes ******************************/


/************************** Variable Definitions *****************************/

XScuGic Intc;

u8 TxRestartColorbar; /* TX restart flag is set when the TX cable has
                       * been reconnected and the TX colorbar was showing. */
XMipi_Menu xMipiMenu; // Menu structure

XPipeline_Cfg Pipeline_Cfg;
XPipeline_Cfg New_Cfg;

//extern XIic IicSensor; /* The instance of the IIC device. */

XAxiDma AxiDma;

XTmrCtr TimerCounterInst;   /* The instance of the Timer Counter */

u32 g_Transmitted_Frame;

#define DEBUG_COUNT 7
u8 dbg_idx = 0;

int FramesTx;		// Num of frames that have been sent

//BUFFER & Flags
int NumOfEmptyBuf = NUM_OF_BUFFER;
int NumOfReadyTxBDSet = 0;
int NumOfReadyRxBDSet = 0;
int g_wr_setup_ptr = 0;
int g_wr_work_ptr = 0;

int g_rd_setup_ptr = 0;
int g_rd_work_ptr = 0;

int g_rd_work_loop_ptr = 0;
int g_wr_work_loop_ptr = 0;

char RxBdSpace[RXBD_SPACE_BYTES] __attribute__ ((aligned(BD_ALIGNMENT)));

uint8_t StartTx;

volatile int accumTime[DEBUG_COUNT] = { 0 };
volatile int accumCntr[DEBUG_COUNT] = { 0 };

u32 *g_FrmBuf[NUM_OF_BUFFER];

u32 g_IOCIntrCnt = 0;

u32 g_cut_data[3];
u8 g_ignored_data[1932];
u8 g_last_ignored_data[1932];

u8 RxStatusFlag;
u8 TxTriggeredFlag;
u8 IsInTxCriticalSection = 0;

#if DEBUG_FEATURES
u32 NumOfTxFrames = 0;
#endif

int Err_Cnt = 0;
/************************** Function Definitions *****************************/

extern void config_csi_cap_path();

// For Timer
int TmrCtrIntrRun(INTC *IntcInstancePtr, XTmrCtr *InstancePtr, u16 DeviceId, u16 IntrId, u8 TmrCtrNumber);

static int TmrCtrSetupIntrSystem(INTC *IntcInstancePtr, XTmrCtr *InstancePtr, u16 DeviceId, u16 IntrId);

static void TimerCounterHandler(void *CallBackRef, u8 TmrCtrNumber);

static void TmrCtrDisableIntr(INTC *IntcInstancePtr, u16 IntrId);

/*****************************************************************************/
/**
 *
 * This function setups the interrupt system.
 *
 * @return	XST_SUCCESS if interrupt setup was successful else error code
 *
 * @note	None.
 *
 *****************************************************************************/
int SetupInterruptSystem(void) {
	int Status;

	XScuGic *IntcInstPtr = &Intc;

	XAxiDma * AxiDmaPtr = &AxiDma;
	XAxiDma_BdRing *RxRingPtr = XAxiDma_GetRxRing(AxiDmaPtr);

	XAxiDma* EthernetDmaPtr = &EthernetDmaInstancePtr;
	XAxiDma_BdRing* TxRingPtr = XAxiDma_GetTxRing(EthernetDmaPtr);
	XAxiEthernet* AxiEthernetInstancePtr = &AxiEthernetInstance;

	/* Initialize the interrupt controller driver so that it's ready to
	 * use, specify the device ID that was generated in xparameters.h */

	XScuGic_Config *IntcCfgPtr;
	IntcCfgPtr = XScuGic_LookupConfig(PSU_INTR_DEVICE_ID);
	if (IntcCfgPtr == NULL) {
		print("ERR:: Interrupt Controller not found");
		return (XST_DEVICE_NOT_FOUND);
	}

	Status = XScuGic_CfgInitialize(IntcInstPtr, IntcCfgPtr, IntcCfgPtr->CpuBaseAddress);
	if (Status != XST_SUCCESS) {
		xil_printf("Intc initialization failed!\r\n");
		return XST_FAILURE;
	}

	/* Start the interrupt controller such that interrupts are recognized
	 * and handled by the processor */
	XScuGic_SetPriorityTriggerType(IntcInstPtr, DMA_TX_IRPT_INTR, 0xA0, 0x3);
	XScuGic_SetPriorityTriggerType(IntcInstPtr, AXIETHERNET_IRPT_INTR, 0xA0, 0x3);

	Status = XScuGic_Connect(IntcInstPtr, XPAR_XIICPS_1_INTR,
			(Xil_InterruptHandler)XIicPs_MasterInterruptHandler,
			(void *)&IicPsInstance);
	if (Status != XST_SUCCESS) {
		return Status;
	}

	Status = XScuGic_Connect(IntcInstPtr, DMA_TX_IRPT_INTR,
				(Xil_InterruptHandler) TxIntrHandler,
				TxRingPtr);
	if (Status != XST_SUCCESS)
		return Status;

	Status = XScuGic_Connect(IntcInstPtr, AXIETHERNET_IRPT_INTR,
				(Xil_InterruptHandler) AxiEthernetErrorHandler,
				AxiEthernetInstancePtr);
	if (Status != XST_SUCCESS)
		return Status;

//	Status = XScuGic_Connect(IntcInstPtr, IIC_SENSOR_INTR_ID,
//				(XInterruptHandler) XIic_InterruptHandler,
//				(void *) &IicSensor);
//	if (Status != XST_SUCCESS)
//		return XST_FAILURE;

	/* Enable IO expander and sensor IIC interrupts */
//	XScuGic_Enable(IntcInstPtr, IIC_SENSOR_INTR_ID);
	XScuGic_Enable(IntcInstPtr, XPAR_XIICPS_1_INTR);

	XScuGic_Enable(IntcInstPtr, DMA_TX_IRPT_INTR);
	XScuGic_Enable(IntcInstPtr, AXIETHERNET_IRPT_INTR);

	Xil_ExceptionInit();

	/* Register the interrupt controller handler with the exception table. */
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
				(Xil_ExceptionHandler) XScuGic_InterruptHandler,
				(XScuGic *) IntcInstPtr);

	return (XST_SUCCESS);
}

/*****************************************************************************/
/**
* This function asserts a callback error.
*
* @param	File is current file name.
* @param	Line is line number of the asserted callback.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void Xil_AssertCallbackRoutine(u8 *File, s32 Line) {
	xil_printf("Assertion in File %s, on line %0d\n\r", File, Line);
}

/*****************************************************************************/
/**
* This function resets IMX274 camera sensor.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void CamReset(void)
{
	Xil_Out32(GPIO_SENSOR, 0x07);
	Xil_Out32(GPIO_SENSOR, 0x06);
	Xil_Out32(GPIO_SENSOR, 0x07);
}


#if RX_DMA_CYCLIC_MODE
static int RxBDRingInit(XAxiDma * AxiDmaInstPtr, char* RxBDBufBase)
{
	XAxiDma_BdRing *RxRingPtr;
	int Status;
	XAxiDma_Bd BdTemplate;
	XAxiDma_Bd *BdPtr;
	XAxiDma_Bd *BdCurPtr;
	int BdCount;

	UINTPTR RxBufferPtr;
	int Index;
	int BdIterPatternIdx;
	int BdLength;
	int BdBufIdx;

	RxRingPtr = XAxiDma_GetRxRing(AxiDmaInstPtr);

	/* Setup Rx BD space */
	BdCount = XAxiDma_BdRingCntCalc(XAXIDMA_BD_MINIMUM_ALIGNMENT, RXBD_SPACE_BYTES);

	Status = XAxiDma_BdRingCreate(RxRingPtr, RxBDBufBase, RxBDBufBase, XAXIDMA_BD_MINIMUM_ALIGNMENT, BdCount);
	if (Status != XST_SUCCESS) {
		xil_printf("Rx bd create failed with %d\r\n", Status);
		return XST_FAILURE;
	}

	// Setup a BD template for the Rx channel. Then copy it to every RX BD.
	XAxiDma_BdClear(&BdTemplate);
	Status = XAxiDma_BdRingClone(RxRingPtr, &BdTemplate);
	if (Status != XST_SUCCESS) {
		xil_printf("Rx bd clone failed with %d\r\n", Status);
		return XST_FAILURE;
	}

	RxRingPtr = XAxiDma_GetRxRing(AxiDmaInstPtr);

	/* Disable all RX interrupts before RxBD space setup */
	XAxiDma_BdRingIntDisable(RxRingPtr, XAXIDMA_IRQ_ALL_MASK);


	/* Setup Rx BD space */
	BdCount = XAxiDma_BdRingCntCalc(XAXIDMA_BD_MINIMUM_ALIGNMENT, RXBD_SPACE_BYTES);

	Status = XAxiDma_BdRingCreate(RxRingPtr, RxBdSpace,
			RxBdSpace, XAXIDMA_BD_MINIMUM_ALIGNMENT, BdCount);
	if (Status != XST_SUCCESS) {
		xil_printf("Rx bd create failed with %d\r\n", Status);
		return XST_FAILURE;
	}

	// Setup a BD template for the Rx channel. Then copy it to every RX BD.
	XAxiDma_BdClear(&BdTemplate);
	Status = XAxiDma_BdRingClone(RxRingPtr, &BdTemplate);
	if (Status != XST_SUCCESS) {
		xil_printf("Rx bd clone failed with %d\r\n", Status);
		return XST_FAILURE;
	}

	for(BdBufIdx = 0 ; BdBufIdx < NUM_OF_BUFFER; BdBufIdx++) {

		Status = XAxiDma_BdRingAlloc(RxRingPtr, NUMBER_OF_BDS_TO_TRANSFER, &BdPtr);
		if (Status != XST_SUCCESS) {
			xil_printf("Rx bd alloc failed with %d\r\n", Status);
			return XST_FAILURE;
		}

		BdCurPtr = BdPtr;

		RxBufferPtr = (UINTPTR) g_FrmBuf[BdBufIdx];
		BdIterPatternIdx = 0;

		for (Index = 0; Index < NUMBER_OF_BDS_TO_TRANSFER; Index++) {


			RxBufferPtr += (XAE_HDR_SIZE + 2);	//Skip Ethernet Header Space
			Status = XAxiDma_BdSetBufAddr(BdCurPtr, RxBufferPtr);
			if (Status != XST_SUCCESS) {
				xil_printf("Rx set buffer addr %x on BD %x failed %d\r\n",
							(unsigned int) RxBufferPtr, (UINTPTR) BdCurPtr, Status);
				return XST_FAILURE;
			}

			BdLength = PAYLOAD_SIZE;
			Status = XAxiDma_BdSetLength(BdCurPtr, BdLength, RxRingPtr->MaxTransferLen);
			if (Status != XST_SUCCESS) {
				xil_printf("Rx set length %d on BD %x failed %d\r\n", BdLength, (UINTPTR) BdCurPtr, Status);
				return XST_FAILURE;
			}

			XAxiDma_BdSetCtrl(BdCurPtr, 0);
			XAxiDma_BdSetId(BdCurPtr, RxBufferPtr);

			RxBufferPtr += BdLength;

			BdCurPtr = (XAxiDma_Bd *) XAxiDma_BdRingNext(RxRingPtr, BdCurPtr);

		}
		//xil_printf("In BDBuf: BdBufIdx = %d, BdAddr = %x, RxBufAddr = %x\r\n",BdBufIdx,BdPtr,XAxiDma_BdRead((u32)BdPtr+(u32)640, XAXIDMA_BD_BUFA_OFFSET));
	}

	return XST_SUCCESS;
}

static int RxCyclicStart(XAxiDma * AxiDmaInstPtr)
{
	XAxiDma_BdRing *RxRingPtr;
	int Status;
	XAxiDma_Bd *BdPtr;

	RxRingPtr = XAxiDma_GetRxRing(AxiDmaInstPtr);
	BdPtr = RxRingPtr->PreHead;

	Status = XAxiDma_BdRingToHw(RxRingPtr, NUMBER_OF_BDS_TO_TRANSFER * NUM_OF_BUFFER, BdPtr);
	if (Status != XST_SUCCESS) {
		xil_printf("Rx ToHw failed with %d\r\n", Status);
		return XST_FAILURE;
	}

	/* Enable Cyclic DMA mode */
	XAxiDma_BdRingEnableCyclicDMA(RxRingPtr);
	XAxiDma_SelectCyclicMode(AxiDmaInstPtr, XAXIDMA_DEVICE_TO_DMA, 1);

	/* Start RX DMA channel */
	Status = XAxiDma_BdRingStart(RxRingPtr);
	if (Status != XST_SUCCESS) {
		xil_printf("Rx start BD ring failed with %d\r\n", Status);
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}
#endif	//#if RX_DMA_CYCLIC_MODE


void FrmRdDoneCallBack()
{
#if DEBUG_FEATURES
	NumOfTxFrames++;
#endif

#if SLOW_DOWN_TX && TX_GO_AHEAD_WITHOUT_CHECKING_RX_DATA_IS_READY
	xil_printf("SLOW_DOWN_TX & TX_GO_AHEAD_WITHOUT_CHECKING_RX_DATA_IS_READY must not be 1 at the same time.\r\n");
#endif


#if SLOW_DOWN_TX
	usleep(20000);
#endif

	g_rd_work_ptr = g_rd_work_ptr==(NUM_OF_BUFFER-1) ? 0 : g_rd_work_ptr + 1;
	TxTriggeredFlag = 0;

	g_rd_work_loop_ptr = g_rd_work_loop_ptr==(2*NUM_OF_BUFFER-1) ? 0 : g_rd_work_loop_ptr + 1;
	Xil_Out32((UINTPTR) (XPAR_AXIS_OVERFLOWDATADIS_0_S_AXI_LITE_BASEADDR + 0x0C), g_rd_work_loop_ptr);	//Write Current Tx ptr to AXIS discarder

 	g_wr_work_loop_ptr = Xil_In32((UINTPTR)XPAR_AXIS_OVERFLOWDATADIS_0_S_AXI_LITE_BASEADDR + 0x28);	//Read Current Rx ptr from AXIS discarder
	if(g_wr_work_loop_ptr >= g_rd_work_loop_ptr)
		NumOfEmptyBuf = (NUM_OF_BUFFER) - (g_wr_work_loop_ptr - g_rd_work_loop_ptr);
	else
		NumOfEmptyBuf = (NUM_OF_BUFFER) - ((g_wr_work_loop_ptr + 2*NUM_OF_BUFFER) - g_rd_work_loop_ptr);

#if TX_GO_AHEAD_WITHOUT_CHECKING_RX_DATA_IS_READY
	NumOfEmptyBuf = 0;
#endif

	//xil_printf("NumOfEmptyBuf,g_wr_work_loop_ptr,g_rd_work_loop_ptr = %d,%d,%d\r\n",NumOfEmptyBuf,g_wr_work_loop_ptr,g_rd_work_loop_ptr);

}

/*****************************************************************************/
/**
* This function does a minimal test on the timer counter device and driver as a
* design example.  The purpose of this function is to illustrate how to use the
* XTmrCtr component.  It initializes a timer counter and then sets it up in
* compare mode with auto reload such that a periodic interrupt is generated.
*
* This function uses interrupt driven mode of the timer counter.
*
* @param	IntcInstancePtr is a pointer to the Interrupt Controller
*		driver Instance
* @param	TmrCtrInstancePtr is a pointer to the XTmrCtr driver Instance
* @param	DeviceId is the XPAR_<TmrCtr_instance>_DEVICE_ID value from
*		xparameters.h
* @param	IntrId is XPAR_<INTC_instance>_<TmrCtr_instance>_INTERRUPT_INTR
*		value from xparameters.h
* @param	TmrCtrNumber is the number of the timer to which this
*		handler is associated with.
*
* @return	XST_SUCCESS if the Test is successful, otherwise XST_FAILURE
*
* @note		This function contains an infinite loop such that if interrupts
*		are not working it may never return.
*
*****************************************************************************/
int TmrCtrIntrRun(INTC *IntcInstancePtr, XTmrCtr *TmrCtrInstancePtr, u16 DeviceId, u16 IntrId, u8 TmrCtrNumber)
{
	int Status;
 //	int LastTimerExpired = 0;

	/* Initialize the timer counter so that it's ready to use,
	 * specify the device ID that is generated in xparameters.h
	 */
	Status = XTmrCtr_Initialize(TmrCtrInstancePtr, DeviceId);
	if (Status != XST_SUCCESS)
		return XST_FAILURE;

	/* Perform a self-test to ensure that the hardware was built
	 * correctly, use the 1st timer in the device (0)
	 */
	Status = XTmrCtr_SelfTest(TmrCtrInstancePtr, TmrCtrNumber);
	if (Status != XST_SUCCESS)
		return XST_FAILURE;

	/* Connect the timer counter to the interrupt subsystem such that
	 * interrupts can occur.  This function is application specific.
	 */
	Status = TmrCtrSetupIntrSystem(IntcInstancePtr, TmrCtrInstancePtr, DeviceId, IntrId);
	if (Status != XST_SUCCESS)
		return XST_FAILURE;

	/* Setup the handler for the timer counter that will be called from the
	 * interrupt context when the timer expires, specify a pointer to the
	 * timer counter driver instance as the callback reference so the
	 * handler is able to access the instance data
	 */
	XTmrCtr_SetHandler(TmrCtrInstancePtr, TimerCounterHandler, TmrCtrInstancePtr);

	/* Enable the interrupt of the timer counter so interrupts will occur
	 * and use auto reload mode such that the timer counter will reload
	 * itself automatically and continue repeatedly, without this option
	 * it would expire once only
	 */
	XTmrCtr_SetOptions(TmrCtrInstancePtr, TmrCtrNumber, XTC_INT_MODE_OPTION | XTC_AUTO_RELOAD_OPTION);

	/* Set a reset value for the timer counter such that it will expire
	 * eariler than letting it roll over from 0, the reset value is loaded
	 * into the timer counter when it is started
	 */
	XTmrCtr_SetResetValue(TmrCtrInstancePtr, TmrCtrNumber, RESET_VALUE);

	/* Start the timer counter such that it's incrementing by default,
	 * then wait for it to timeout a number of times
	 */
	XTmrCtr_Start(TmrCtrInstancePtr, TmrCtrNumber);
/*
	while (1) {
		xil_printf("Timer Operation... TimerExpired = %d\r\n",TimerExpired);

		while (TimerExpired == LastTimerExpired) {
		}
		LastTimerExpired = TimerExpired;

		if (TimerExpired == NUM_TIMER_LOOP) {

			XTmrCtr_Stop(TmrCtrInstancePtr, TmrCtrNumber);
			break;
		}
	}

	TmrCtrDisableIntr(IntcInstancePtr, IntrId);g_Transmitted_Frame
*/
	return XST_SUCCESS;
}

/*****************************************************************************/
/**
* This function is the handler which performs processing for the timer counter.
* It is called from an interrupt context such that the amount of processing
* performed should be minimized.  It is called when the timer counter expires
* if interrupts are enabled.
*
* This handler provides an example of how to handle timer counter interrupts
* but is application specific.
*
* @param	CallBackRef is a pointer to the callback function
* @param	TmrCtrNumber is the number of the timer to which this
*		handler is associated with.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void TimerCounterHandler(void *CallBackRef, u8 TmrCtrNumber)
{
 //	u8 dbg_count = DEBUG_COUNT;
/*
	int i;
	UINTPTR pBufBase;
	u32 BufVal[8];

	pBufBase = (UINTPTR)g_FrmBuf[g_rd_setup_ptr];

	xil_printf("Buffer 1st 32 Pixels in Buffer %d:\r\n",g_rd_setup_ptr);
	for(i = 0; i < 8;i++){
		BufVal[i] = Xil_In32(pBufBase+i*4);
	}
	for(i = 0; i < 8;i++){
		xil_printf("%02X %02X %02X %02X ",BufVal[i]&0xFF,(BufVal[i]&0xFF00)>>8,(BufVal[i]&0xFF0000)>>16,(BufVal[i]&0xFF000000)>>24);
	}
	xil_printf("\r\n");

	xil_printf("Transmitted_Frame = %d  \r\n",g_Transmitted_Frame);
*/
 //	xil_printf("NumOfEmptyBuf = %d  \r\n",NumOfEmptyBuf);
	Err_Cnt = 0;
	dbg_idx = (dbg_idx == DEBUG_COUNT - 1) ? 0 : dbg_idx + 1;
	//dbg_idx = 4;
	switch (dbg_idx) {
		case 0:  // Printing XAxiDma_BdRingAlloc
			xil_printf("XAxiDma_BdRingAlloc: %d us, %d times\r\n",
						accumTime[dbg_idx], accumCntr[dbg_idx]);
			break;
		case 1:  // Printing BD Setting For Loop
			xil_printf("BD Setting For Loop: %d us, %d times\r\n",
						accumTime[dbg_idx], accumCntr[dbg_idx]);
			break;
		case 2:  // Printing XAxiDma_BdRingToHw
			xil_printf("XAxiDma_BdRingToHw: %d us, %d times\r\n",
						accumTime[dbg_idx], accumCntr[dbg_idx]);
			break;
		case 3:  // Printing XAxiDma_BdRingStart
			xil_printf("XAxiDma_BdRingStart: %d us, %d times\r\n",
						accumTime[dbg_idx], accumCntr[dbg_idx]);
			break;
		case 4:  // Printing RxSetup Duration
			xil_printf("RxSetup Duration: %d us, %d times\r\n",
						accumTime[dbg_idx], accumCntr[dbg_idx]);
			break;
		case 5:  // Printing Interrupt-to-Start
			xil_printf("Interrupt-to-Start: %d us, %d times\r\n",
						accumTime[dbg_idx], accumCntr[dbg_idx]);
			break;
		case 6:  // Printing Start-to-Interrupt
			xil_printf("Start-to-Interrupt: %d us, %d times\r\n",
						accumTime[dbg_idx], accumCntr[dbg_idx]);
			break;
		default: // Error!
			break;
	}

	accumTime[dbg_idx] = 0;
	accumCntr[dbg_idx] = 0;
	xil_printf("NumOfEmptyBuf = %d, RxBusy = %d, TxTriggeredFlag = %d\r\n", NumOfEmptyBuf,XAxiDma_Busy(&AxiDma, XAXIDMA_DEVICE_TO_DMA),TxTriggeredFlag);
	int mipi_error_reg = MIPI_CONTROLLER_mReadReg(XPAR_MIPI_CONTROLLER_0_BASEADDR, MIPI_CONTROLLER_S00_AXI_SLV_REG2_OFFSET);
	xil_printf("error_status = %x\r\n", mipi_error_reg);
	int first_fifo_fps = MIPI_CONTROLLER_mReadReg(XPAR_MIPI_CONTROLLER_0_BASEADDR, MIPI_CONTROLLER_S00_AXI_SLV_REG5_OFFSET);
	xil_printf("first fifo fps = %i fps\r\n", first_fifo_fps);
	int second_fifo_fps = MIPI_CONTROLLER_mReadReg(XPAR_MIPI_CONTROLLER_0_BASEADDR, MIPI_CONTROLLER_S00_AXI_SLV_REG6_OFFSET);
	xil_printf("second fifo fps = %i fps\r\n", second_fifo_fps);
	CsiRxPrintRegStatus();
 //	int CoalCnt;
 //	CoalCnt = XAxiDma_ReadReg(XAxiDma_GetRxRing(&AxiDma)->ChanBase, XAXIDMA_SR_OFFSET) & XAXIDMA_COALESCE_MASK;
 //	CoalCnt = CoalCnt >> XAXIDMA_COALESCE_SHIFT;
 //	xil_printf("Timer, SR coal counter = %d  \r\n",CoalCnt);
 //	CheckAfterRx(&AxiDma);
 //	if (XAxiDma_Busy(&EthernetDmaInstancePtr, XAXIDMA_DMA_TO_DEVICE))
 //		xil_printf("Tx DMA is Busy!\r\n");
 //	if(XAxiDma_Busy(&AxiDma, XAXIDMA_DEVICE_TO_DMA))
 //		xil_printf("Rx DMA is Busy!\r\n");
 //	xil_printf("NumOfEmptyBuf = %d, g_wr_setup_ptr = %d, g_wr_work_ptr = %d \r\n",NumOfEmptyBuf,g_wr_setup_ptr,g_wr_work_ptr);

}

/*****************************************************************************/
/**
* This function setups the interrupt system such that interrupts can occur
* for the timer counter. This function is application specific since the actual
* system may or may not have an interrupt controller.  The timer counter could
* be directly connected to a processor without an interrupt controller.  The
* user should modify this function to fit the application.
*
* @param	IntcInstancePtr is a pointer to the Interrupt Controller
*		driver Instance.
* @param	TmrCtrInstancePtr is a pointer to the XTmrCtr driver Instance.
* @param	DeviceId is the XPAR_<TmrCtr_instance>_DEVICE_ID value from
*		xparameters.h.
* @param	IntrId is XPAR_<INTC_instance>_<TmrCtr_instance>_VEC_ID
*		value from xparameters.h.
*
* @return
*		- XST_SUCCESS if the Test is successful
*		- XST_FAILURE if the Test is not successful
*
* @note		This function contains an infinite loop such that if interrupts
*		are not working it may never return.
*
******************************************************************************/
static int TmrCtrSetupIntrSystem(INTC *IntcInstancePtr, XTmrCtr *TmrCtrInstancePtr, u16 DeviceId, u16 IntrId)
{
	 int Status;
/*	NOTE: The XScuGic driver instance passed in by the caller must have already
 *	been initialized in SetupInterruptSystem() at line 1176 of main(), which is
 *	faster than this function call during main() line 1260.
	 XScuGic_Config *IntcConfig;
	 IntcConfig = XScuGic_LookupConfig(PSU_INTR_DEVICE_ID);
	if (NULL == IntcConfig) {
		return XST_FAILURE;
	}

	Status = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig,
					IntcConfig->CpuBaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
*/
	XScuGic_SetPriorityTriggerType(IntcInstancePtr, TMRCTR_INTERRUPT_ID, 0xA0, 0x3);

	Status = XScuGic_Connect(IntcInstancePtr, TMRCTR_INTERRUPT_ID,
					(Xil_InterruptHandler)XTmrCtr_InterruptHandler,
					(void *)TmrCtrInstancePtr);
	if (Status != XST_SUCCESS)
		return Status;

	XScuGic_Enable(IntcInstancePtr, TMRCTR_INTERRUPT_ID);

 //	If interrupt still doesn't work, the following lines until
 //	Xil_ExceptionEnable() are **also duplicate** of SetupInterruptSystem().
 //	Removing the functions below may cause some more changes.
	Xil_ExceptionInit();
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
			(Xil_ExceptionHandler)XScuGic_InterruptHandler,
			(void *)IntcInstancePtr);

	Xil_ExceptionEnable();

	return XST_SUCCESS;
}

/******************************************************************************/
/**
*
* This function disables the interrupts for the Timer.
*
* @param	IntcInstancePtr is a reference to the Interrupt Controller
*		driver Instance.
* @param	IntrId is XPAR_<INTC_instance>_<Timer_instance>_VEC_ID
*		value from xparameters.h.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void TmrCtrDisableIntr(INTC *IntcInstancePtr, u16 IntrId)
{
	// Disable the interrupt for the timer counter
	XScuGic_Disconnect(IntcInstancePtr, IntrId);
}

/*****************************************************************************/
/**
 *
 * Main function to initialize the video pipleline and process user input
 *
 * @return	XST_SUCCESS if MIPI example was successful else XST_FAILURE
 *
 * @note	None.
 *
 *****************************************************************************/
int main(void)
{
	u8 Response;
	u32 Status;
	XAxiDma_Config *Config;
	u32 *RxBuf;
	u8 TxBusy;
	u8 BDInitCnt = 0;
	u32 CoalCnt;
	XAxiDma_BdRing *RxRingPtr = XAxiDma_GetRxRing(&AxiDma);
	XAxiDma_Bd *BdPtr;
	XAxiDma_Bd *RxTailBdPtr;
	u32 LastBdSts;
	u32 BdSts;

	for(int i = 0 ; i < NUM_OF_BUFFER; i ++ ){
		g_FrmBuf[i] = (u32 *) (BUFFER_BASE_ADDR + BUFFER_SIZE * i);
	}

	TxBusy = 0;
	g_Transmitted_Frame = 0;

	if(NUM_OF_BUFFER * BUFFER_SIZE + RXBD_SPACE_BYTES > 2000000000) xil_printf("ERROR!!!! DRAM Usage OVER about 2GB!!!\r\n");
	else if(NUM_OF_BUFFER * BUFFER_SIZE + RXBD_SPACE_BYTES > 1500000000) xil_printf("WARNING!!!! DRAM Usage OVER about 1.5GB!!! Please check it is OK.\r\n");
	else if(NUM_OF_BUFFER * BUFFER_SIZE + RXBD_SPACE_BYTES > 1000000000) xil_printf("WARNING!!!! DRAM Usage OVER about 1GB!!! Please check it is OK.\r\n");

#if DEBUG_FEATURES
	xil_printf("Addr of g_first_ignored_data/g_last_ignored_data = %x, %x\r\n",
				&g_ignored_data[0], &g_last_ignored_data[0]);
	xil_printf("Addr of NumOfEmptyBuf, g_wr_setup_ptr, g_wr_work_ptr, g_rd_setup_ptr, g_rd_work_ptr, TxTriggeredFlag = %x,%x,%x,%x,%x,%x\r\n",
			&NumOfEmptyBuf,&g_wr_setup_ptr,&g_wr_work_ptr,&g_rd_setup_ptr, &g_rd_work_ptr,&TxTriggeredFlag);
	xil_printf("Addr of NumOfTxFrames = %x\r\n",&NumOfTxFrames);
	xil_printf("Addr of RxBdSpace[0]/RxBdSpace[LAST], Size of RxBdSpace(Byte) = %x/%x, %d\r\n",RxBdSpace,&RxBdSpace[RXBD_SPACE_BYTES-1],RXBD_SPACE_BYTES);
#endif

	/************************** ethernet setup ******************************/
	xil_printf("Ethernet Link Initializing...\r\n");
	XAxiEthernet* AxiEthernetInstancePtr = &AxiEthernetInstance;
	XAxiDma* EthernetDmaPtr = &EthernetDmaInstancePtr;
	XAxiDma_BdRing* TxRingPtr = XAxiDma_GetTxRing(EthernetDmaPtr);
	Status = SetupEthernet();
	if (Status != XST_SUCCESS) {
		xil_printf("Error initializing AxiEthernet\r\n");
		return XST_FAILURE;
	}
	/************************** ethernet setup ******************************/

	/* Setup the default pipeline configuration parameters:
	 * Look for Default ColorDepth manual setting just after SetColorDepth
	 */
	Pipeline_Cfg.ActiveLanes = 4;
	Pipeline_Cfg.VideoSrc = XVIDSRC_SENSOR;

	/* Default DSI */
	Pipeline_Cfg.VideoDestn = XVIDDES_DSI;

	Pipeline_Cfg.Live = TRUE;

	/* Vertical and Horizontal flip don't work */
	Pipeline_Cfg.Vflip = FALSE;
	Pipeline_Cfg.Hflip = FALSE;

	/* Video pipeline configuration from user */
	Pipeline_Cfg.CameraPresent = TRUE;
	Pipeline_Cfg.DSIDisplayPresent = TRUE;

	/* Default Resolution that to be displayed */
	Pipeline_Cfg.VideoMode = XVIDC_VM_1920x1080_60_P;

	Xil_DCacheDisable();

	xil_printf("\n\r\n\r");
	xil_printf(TXT_GREEN);
	xil_printf("--------------------------------------------------\r\n");
	xil_printf("------  MIPI Reference Pipeline Design  ----------\r\n");
	xil_printf("---------  (c) 2017 by Xilinx, Inc.  -------------\r\n");
	xil_printf("--------------------------------------------------\r\n");
	xil_printf(TXT_RST);

	xil_printf(TXT_YELLOW);
	xil_printf("--------------------------------------------------\r\n");
	xil_printf("Build %s - %s\r\n", __DATE__, __TIME__);
	xil_printf("--------------------------------------------------\r\n");
	xil_printf(TXT_RST);

	xil_printf("Please answer the following questions about the hardware setup.");
	xil_printf("\r\n");

	do {
		xil_printf("Is the camera sensor connected? (Y/N)\r\n");

		Response = XUartPs_RecvByte(UART_BASEADDR);

		XUartPs_SendByte(UART_BASEADDR, Response);

		if ((Response == 'Y') || (Response == 'y')) {
			Pipeline_Cfg.CameraPresent = TRUE;
			break;
		} else if ((Response == 'N') || (Response == 'n')) {
			Pipeline_Cfg.CameraPresent = FALSE;
			break;
		}

	} while (1);

	if (Pipeline_Cfg.CameraPresent)
		print(TXT_GREEN);
	else
		print(TXT_RED);

	xil_printf("\r\nCamera sensor is set as %s\r\n",
			(Pipeline_Cfg.CameraPresent) ? "Connected" : "Disconnected");
	print(TXT_RST);

	if (!Pipeline_Cfg.CameraPresent) {
		Pipeline_Cfg.VideoSrc = XVIDSRC_TPG;
		xil_printf("Setting TPG as source in absence of Camera sensor.\r\n");
	}

	Pipeline_Cfg.DSIDisplayPresent = FALSE;

	if (Pipeline_Cfg.DSIDisplayPresent)
		xil_printf(TXT_GREEN);
	else
		xil_printf(TXT_RED);

	xil_printf("\r\nDSI Display panel is set as %s\r\n",
			(Pipeline_Cfg.DSIDisplayPresent) ? "Connected" : "Disconnected");
	xil_printf(TXT_RST);


	/* Initialize platform */
	init_platform();

	/* Initialize IIC */
	Status = InitIIC();
	if (Status != XST_SUCCESS) {
		xil_printf(TXT_RED "\n\rIIC Init Failed \n\r" TXT_RST);
		return XST_FAILURE;
 	} else {
 		print(TXT_GREEN "IIC init SUCCEEDED.\n\r" TXT_RST);
	}

	/* Initialize IRQ */
	Status = SetupInterruptSystem();
	if (Status == XST_FAILURE) {
		print(TXT_RED "IRQ init failed.\n\r" TXT_RST);
		return XST_FAILURE;
 	} else {
 		print(TXT_GREEN "IRQ init SUCCEEDED.\n\r" TXT_RST);
	}

	/* Turn on DVS Sensor */
	Status = StartDVSSensor();
	if (Status != XST_SUCCESS) {
		print(TXT_RED "START DVS error.\n\r" TXT_RST);
		return XST_FAILURE;
	} else {
		print(TXT_GREEN "START DVS SUCCEEDED.\n\r" TXT_RST);
	}

//	/* IIC interrupt handlers */
//	SetupIICIntrHandlers();

	/* Initialize CSIRXSS  */
//	Status = InitializeCsiRxSs();
	Status = InitializeDphy();
	if (Status != XST_SUCCESS) {
		xil_printf(TXT_RED "CSI Rx Ss Init failed status = %x.\r\n" TXT_RST, Status);
		return XST_FAILURE;
	}

	//Set Rx DMA
	print(TXT_GREEN "Start Set DMA.\n\r" TXT_RST);
	Config = XAxiDma_LookupConfig(DMA_DEV_ID);
	if (!Config) {
		xil_printf("No config found for %d\r\n", DMA_DEV_ID);
		return XST_FAILURE;
	}


	XAxiDma_CfgInitialize(&AxiDma, Config);
	if (Status != XST_SUCCESS) {
		xil_printf(TXT_RED "DMA Init failed status = %x.\r\n" TXT_RST, Status);
		return XST_FAILURE;
 //	} else {
 //		print(TXT_GREEN "DMA Init SUCCEEDED.\n\r" TXT_RST);
	}

	/* MIPI colour depth in bits per clock */
//	SetColorDepth();

	print("---------------------------------\r\n");

	/* Enable exceptions. */
	Xil_AssertSetCallback((Xil_AssertCallback) Xil_AssertCallbackRoutine);
	Xil_ExceptionEnable();

	/* Reset Camera Sensor module through GPIO */
//	xil_printf("Disable CAM_RST of Sensor through GPIO\r\n");
//	CamReset();
//	xil_printf("Sensor is Enabled\r\n");
	CsiRxPrintRegStatus();

	/* Program Camera sensor */
	MIPI_CONTROLLER_mWriteReg(XPAR_MIPI_CONTROLLER_0_BASEADDR,
			MIPI_CONTROLLER_S00_AXI_SLV_REG0_OFFSET,
			0);
	sleep(1);
	MIPI_CONTROLLER_mWriteReg(XPAR_MIPI_CONTROLLER_0_BASEADDR,
			MIPI_CONTROLLER_S00_AXI_SLV_REG0_OFFSET,
			2);
	// 1. control fifo with drop rate-----------------------------
//	MIPI_CONTROLLER_mWriteReg(XPAR_MIPI_CONTROLLER_0_BASEADDR,
//			MIPI_CONTROLLER_S00_AXI_SLV_REG3_OFFSET,
//			10);

	// 2. control fifo with fifo control -------------------------
	MIPI_CONTROLLER_mWriteReg(XPAR_MIPI_CONTROLLER_0_BASEADDR,
			MIPI_CONTROLLER_S00_AXI_SLV_REG3_OFFSET,
			0);
	MIPI_CONTROLLER_mWriteReg(XPAR_MIPI_CONTROLLER_0_BASEADDR,
			MIPI_CONTROLLER_S00_AXI_SLV_REG4_OFFSET,
			1);

	//------------------------Ethernet Setup ----------------------------//
	Status = SetupIntrCoalescingSetting();
	if (Status != XST_SUCCESS) {
		xil_printf("Failed to setup Interrupt Coalescing\r\n");
		return XST_FAILURE;
	}
	//----------------------Ethernet Setup finish -------------------------//

//	// Initialize menu
//	XMipi_MenuInitialize(&xMipiMenu, UART_BASEADDR);
//	// Print the Pipe line configuration
//	PrintPipeConfig();

	//Turn on Timer Interrupt
	Status = TmrCtrIntrRun(&Intc, &TimerCounterInst, TMRCTR_DEVICE_ID, TMRCTR_INTERRUPT_ID, TIMER_CNTR_0);

	Status = EthFrameSetTxHeaderOnly();
	if (Status != XST_SUCCESS) {
		xil_printf("Failed to set frame header\r\n");
		return XST_FAILURE;
	}

#if RX_DMA_CYCLIC_MODE
	Status = RxBDRingInit(&AxiDma, RxBdSpace);
	if (Status != XST_SUCCESS) {
		xil_printf(TXT_RED "Failed RxBDRingInit(), " TXT_RST);
		return XST_FAILURE;
	}

	Status = RxCyclicStart(&AxiDma);
	if (Status != XST_SUCCESS) {
		xil_printf(TXT_RED "Failed RxCyclicStart(), " TXT_RST);
		return XST_FAILURE;
	}

	//AXIS Discarder Setup
	Xil_Out32((UINTPTR) XPAR_AXIS_OVERFLOWDATADIS_0_S_AXI_LITE_BASEADDR, 0x00000001);	//Turn On/Off : When off, bypass AXIS
	Xil_Out32((UINTPTR) (XPAR_AXIS_OVERFLOWDATADIS_0_S_AXI_LITE_BASEADDR + 0x04), SQRT_NUM_OF_BUFFER);
	Xil_Out32((UINTPTR) (XPAR_AXIS_OVERFLOWDATADIS_0_S_AXI_LITE_BASEADDR + 0x08), 0x00000000);	//1 time TLAST minus 1 / Frame for DVS
	Xil_Out32((UINTPTR) (XPAR_AXIS_OVERFLOWDATADIS_0_S_AXI_LITE_BASEADDR + 0x0C), g_rd_work_ptr);	//Current Read Pointer
	Xil_Out32((UINTPTR) (XPAR_AXIS_OVERFLOWDATADIS_0_S_AXI_LITE_BASEADDR + 0x10), 0x0000A8BF);	//Debug Monitor : # of AXIS transaction minus 1 /Frame for DVS  0xA8BF = 43200 - 1
#endif

//	Status = SetupCameraSensor();
//	if (Status != XST_SUCCESS) {
//		xil_printf("Failed to setup Camera sensor\r\n");
//		return XST_FAILURE;
//	}
	//================ DVS sensor program ===================//
	Status = ProgramDVSSensor();
	if (Status != XST_SUCCESS) {
		print(TXT_RED "PROGRAM DVS error.\n\r" TXT_RST);
		return XST_FAILURE;
	} else {
		print(TXT_GREEN "PROGRAM DVS SUCCEEDED.\n\r" TXT_RST);
	}

	StartTx = 0;
	TxTriggeredFlag = 0;

	/* Main loop */
	do {
		IsInTxCriticalSection = 1;
		{
#if RX_DMA_CYCLIC_MODE
		 	g_wr_work_loop_ptr = Xil_In32((UINTPTR)XPAR_AXIS_OVERFLOWDATADIS_0_S_AXI_LITE_BASEADDR + 0x28);
			if(g_wr_work_loop_ptr >= g_rd_work_loop_ptr)
				NumOfEmptyBuf = (NUM_OF_BUFFER) - (g_wr_work_loop_ptr - g_rd_work_loop_ptr);
			else
				NumOfEmptyBuf = (NUM_OF_BUFFER) - ((g_wr_work_loop_ptr + 2*NUM_OF_BUFFER) - g_rd_work_loop_ptr);
	#if TX_GO_AHEAD_WITHOUT_CHECKING_RX_DATA_IS_READY
			NumOfEmptyBuf = 0;
	#endif
	#if DEBUG_FEATURES
		if(Xil_In32((UINTPTR)XPAR_AXIS_OVERFLOWDATADIS_0_S_AXI_LITE_BASEADDR + 0x2C)!=0)
			xil_printf(TXT_RED "Discarder # of Input Transaction Failed. It should be zero\r\n" TXT_RST);
	#endif
		}
		IsInTxCriticalSection = 0;
#endif

		u8* sending_address;
		if (!StartTx) {
			if(NumOfEmptyBuf < NUM_OF_BUFFER) {
				sending_address = (u8 *) g_FrmBuf[g_rd_setup_ptr];
				Status = SendFrameSingleTxBurstSetup(sending_address);
				if (Status != XST_SUCCESS) {
					xil_printf(TXT_RED "Next Ethernet TxBD Setup failed\r\n" TXT_RST);
					return XST_FAILURE;
				}
				NumOfReadyTxBDSet++;
				g_rd_setup_ptr = (g_rd_setup_ptr == (NUM_OF_BUFFER-1)) ? 0 : g_rd_setup_ptr + 1;

				sending_address = (u8 *) g_FrmBuf[g_rd_setup_ptr];
				Status = SendFrameSingleTxBurstSetup(sending_address);
				if (Status != XST_SUCCESS) {
					xil_printf(TXT_RED "Next Ethernet TxBD Setup failed\r\n" TXT_RST);
					return XST_FAILURE;
				}
				NumOfReadyTxBDSet++;
				g_rd_setup_ptr = (g_rd_setup_ptr == (NUM_OF_BUFFER-1)) ? 0 : g_rd_setup_ptr + 1;

				Status = SendFrameSingleTxBurstSend();
				if (Status != XST_SUCCESS) {
					xil_printf(TXT_RED "Frame transmit failed\r\n" TXT_RST);
					return XST_FAILURE;
				}
				TxTriggeredFlag = 1;
				StartTx = 1;
			}
		}
		else {
			IsInTxCriticalSection = 1;
			{
				if(TxTriggeredFlag) {
					TxBusy = XAxiDma_Busy(&EthernetDmaInstancePtr, XAXIDMA_DMA_TO_DEVICE);
					if(!TxBusy) {

						do {
							BdPtr = (XAxiDma_Bd*) TxRingPtr->HwTail;
							BdSts = XAxiDma_BdRead(BdPtr, XAXIDMA_BD_STS_OFFSET);
							if(!(BdSts & XAXIDMA_BD_STS_COMPLETE_MASK)) {
								xil_printf("Main Loop : WARNING, TxDma is idle even before all TxBDs are not completed.\r\n");
								xil_printf("For Safety, this loop repeats until tail TxBD complete bit is detected.\r\n");
							}
							//This warning may occur if the time when TxBusy is turned off is earlier than when the BD completed bit is written.
							//However, this is just an assumption. Need to be confirmed.
						} while (!(BdSts & XAXIDMA_BD_STS_COMPLETE_MASK));

						FrmRdDoneCallBack();

						if(NumOfEmptyBuf < NUM_OF_BUFFER) {	//To be more strict, but slow, use this -> if(!RdDmaStopCond()) {
							Status = SendFrameSingleTxBurstSend();
							if (Status != XST_SUCCESS) {
								xil_printf(TXT_RED "Frame transmit failed\r\n" TXT_RST);
								return XST_FAILURE;
							}
							TxTriggeredFlag = 1;
						}

						int NumBd = XAxiDma_BdRingFromHw(TxRingPtr, TX_PACKET_CNT, &BdPtr);
						if (NumBd != TX_PACKET_CNT) {
							xil_printf("Main Loop: Requesting NumBd: %d, ", NumBd);
							AxiEthernetUtilErrorTrap("Error in interrupt coalescing");
						} else {
							// Don't bother to check the BDs status, just free them
							Status = XAxiDma_BdRingFree(TxRingPtr, TX_PACKET_CNT, BdPtr);
							if (Status != XST_SUCCESS)
								AxiEthernetUtilErrorTrap("Error freeing TxBDs");
						}

						Status = SendFrameSingleTxBurstSetup((u8 *) g_FrmBuf[g_rd_setup_ptr]);
						if (Status != XST_SUCCESS) {
							xil_printf("Main Loop: Next Ethernet TxBD Setup failed\r\n");
							xil_printf("NumOfEmptyBuf : %d, g_rd_setup_ptr,g_rd_work_ptr : %d/%d, g_wr_setup_ptr,g_wr_work_ptr : %d,%d, TxTriggeredFlag : %d \r\n",
										NumOfEmptyBuf,g_rd_setup_ptr,g_rd_work_ptr,g_wr_setup_ptr,g_wr_work_ptr,TxTriggeredFlag);
							return XST_FAILURE;
						}
						NumOfReadyTxBDSet++;
						g_rd_setup_ptr = (g_rd_setup_ptr == (NUM_OF_BUFFER-1)) ? 0 : g_rd_setup_ptr + 1;
					}
				}
			}
			IsInTxCriticalSection = 0;

			if(!TxTriggeredFlag && NumOfEmptyBuf < NUM_OF_BUFFER) {	//To be more strict, but slow, use this -> if(!TxTriggeredFlag && !RdDmaStopCond()) {
				Status = SendFrameSingleTxBurstSend();
				if (Status != XST_SUCCESS) {
					xil_printf(TXT_RED "Frame transmit failed\r\n" TXT_RST);
					return XST_FAILURE;
				}
				TxTriggeredFlag = 2;
			}

		}

	} while (1);

	if (XAxiDma_Busy(&AxiDma, XAXIDMA_DEVICE_TO_DMA))
		CheckAfterRx(&AxiDma);

	int Checked = 0;
	//Avoid main() Done
	while (1) {
		if (!XAxiDma_Busy(&AxiDma, XAXIDMA_DEVICE_TO_DMA) && !Checked) {
			CheckAfterRx(&AxiDma);
			++Checked;
		}
	 //	CoalCnt = XAxiDma_ReadReg(XAxiDma_GetRxRing(&AxiDma)->ChanBase, XAXIDMA_SR_OFFSET) & XAXIDMA_COALESCE_MASK;
	 //	CoalCnt = CoalCnt >> XAXIDMA_COALESCE_SHIFT;
	 //	xil_printf("while, SR coal counter = %d  \r\n",CoalCnt);
	};

	return 0;
}

/*****************************************************************************/
/**
 * This function sets header bytes for 1440 packet frames in `g_FrmBuf`, the
 * global MIPI FHD frame buffer array. Ideally, this function is only called
 * once, while the `g_FrmBuf` is still empty. At each iteration of the main
 * loop, the AxiDMA engine fills the remaining payload bytes with data from the
 * CIS sensor, which completes the frame buffer for a single FHD frame with
 * compliance to a contiguous array format of Ethernet packets.
 *
 * @param	none
 *
 * @return	XST_SUCCESS if successful, else XST_FAILURE.
 *****************************************************************************/
LONG EthFrameSetTxHeaderOnly(void)
{
	if(NUM_OF_BUFFER > 16) xil_printf("WARNING!!!! Ethernet Packet Order Bit length is only 4 bit (Maximum NUM_OF_BUFFER may be 16). Is it OK?\r\n");
	for (u16 i = 0; i < NUM_OF_BUFFER; ++i) {

		// Pointer to the EthernetFrame (incremented every iteration)
		u8 *Frame = (u8 *) g_FrmBuf[i];

		// Size of FULL TxFrame is given as CONSTANT -> Can set as STATIC!
		for (u16 iter = 0; iter < 1440; ++iter) {
			//	1. Set Src/Dest MAC Addresses (Taken from `EmacPsUtilFrameHdrFormatMAC()`)

			memcpy(Frame, AxiEthernetDestMAC, XAE_MAC_ADDR_SIZE); // Destination address
			Frame += XAE_MAC_ADDR_SIZE;

			memcpy(Frame, AxiEthernetMAC, XAE_MAC_ADDR_SIZE); // Source address
			Frame += XAE_MAC_ADDR_SIZE;

			//	2. Set Frame Type (Taken from `EmacPsUtilFrameHdrFormatType()`)

			u16 FrameType = PAYLOAD_SIZE + 2; // + ORDER_OF_PACKET; // Compute `FrameType` as Packet Total Length
			*Frame++ = (u8) (FrameType >> 8);	// Higher 8 bits
			*Frame++ = (u8)  FrameType;			// Lower  8 bits

			//	3. Set Order Bytes Manually

			*Frame++ = (u8) ((iter >> 8) + ((i&0xFF) << 4)); // Higher 8 bits
			// NOTE: Higher 4 bits contain information on frame buffer index (up to 16 buffers)
			*Frame++ = (u8)   iter; // Lower  8 bits

			//	NOTE: Use `memset()` to reset all DATA bytes!
			memset(Frame, 0, PACKET_SIZE - HEADER_SIZE);

			//	4. Increment `Frame` and `send_addr` pointers
			Frame += (PACKET_SIZE - HEADER_SIZE); // Move `Frame` to start of next frame
		}
	}

 //	return EmacPsUtilFrameSetTxHeadersDebug();
	return XST_SUCCESS;
}
