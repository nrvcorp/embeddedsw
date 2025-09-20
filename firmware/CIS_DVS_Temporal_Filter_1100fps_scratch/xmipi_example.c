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
#include "xvprocss.h"
#include "sensor_cfgs.h"
#include "xmipi_menu.h"
#include "pipeline_program.h"
#include "xv_frmbufwr_l2.h"
#include "xiicps.h"
#include "xaxidma.h"

//#include "xaxidma_bdring.h"
#define XAXIDMA_VIRT_TO_PHYS(BdPtr) \
	((UINTPTR)(BdPtr) + (RingPtr->FirstBdPhysAddr - RingPtr->FirstBdAddr))

#include "xil_printf.h"
#include "dma_example.h"
#include "DVS_5x5x5_Filter.h"

#include "xtime_l.h"
#include <stdlib.h>


/************************** Constant Definitions *****************************/
#define MIPI_CONTROLLER_mWriteReg(BaseAddress, RegOffset, Data) \
  	Xil_Out32((BaseAddress) + (RegOffset), (u32)(Data))
#define MIPI_CONTROLLER_mReadReg(BaseAddress, RegOffset) \
  	Xil_In32((BaseAddress) + (RegOffset))

#define MIPI_CONTROLLER_BASEADDR XPAR_MIPI_RX_SUBSYSTEM_TOP_0_BASEADDR
#define MIPI_CONTROLLER_S00_AXI_SLV_REG0_OFFSET 0
#define MIPI_CONTROLLER_S00_AXI_SLV_REG1_OFFSET 4
#define MIPI_CONTROLLER_S00_AXI_SLV_REG2_OFFSET 8
#define MIPI_CONTROLLER_S00_AXI_SLV_REG3_OFFSET 12 // frame_dump_num_reg_val
#define MIPI_CONTROLLER_S00_AXI_SLV_REG4_OFFSET 16 // frame_control_reg_val
#define MIPI_CONTROLLER_S00_AXI_SLV_REG5_OFFSET 20 // first fifo fps val
#define MIPI_CONTROLLER_S00_AXI_SLV_REG6_OFFSET 24 // second fifo fps val

// Device ID & Address
#ifdef XPAR_PSU_ACPU_GIC_DEVICE_ID
#define PSU_INTR_DEVICE_ID	XPAR_PSU_ACPU_GIC_DEVICE_ID
#endif

#define	DMA_FIL_TX_IRPT_INTR		XPAR_FABRIC_DMA_FIL_MM2S_INTROUT_INTR
#define	DMA_FIL_RX_IRPT_INTR		XPAR_FABRIC_DMA_FIL_S2MM_INTROUT_INTR

//DVS frame info: pixel and bit counts
#define DVS_FRAME_WIDTH		960
#define DVS_FRAME_HEIGHT	720
#define BIT_PER_PEL			2
#define DVS_LINE_SIZE_IN_BYTE 	DVS_FRAME_WIDTH * BIT_PER_PEL / 8	//240
#define DVS_FRAME_SIZE_IN_BYTE	DVS_FRAME_WIDTH * DVS_FRAME_HEIGHT * BIT_PER_PEL / 8	//172800


//Memory Address Map
//#define DDR_BASE_ADDR	 XPAR_PSU_DDR_0_S_AXI_BASEADDR
/*
 * pipeline_program.h
 */


#define FIL_BUF_SIZE		DVS_FRAME_SIZE_IN_BYTE	//(960*720*2)/8 byte
#define FIL_BUF_TOTAL_SIZE	FIL_BUF_SIZE * NUM_OF_FIL_FRAME_BUFFER

#define FIL_BUF_END_ADDR 	FIL_BUF_END_ADDR+FIL_BUF_TOTAL_SIZE-1

#define	NUM_OF_FIL_FRAME_BUFFER	16

//DMA BD Settings (FIL TX, FIL RX)
// BD LEN = BD SIZE in Byte
#define	DMA_FIL_SINGLE_TXDATA_LEN	DVS_LINE_SIZE_IN_BYTE //240		//1 line transaction at a time (1-line = 240 byte)
#define	DMA_FIL_TXBD_NUM	DVS_FRAME_HEIGHT * 6  //4320	//4320 lines in 6 dvs frames for filter input
#define	DMA_FIL_TXBD_NUM_PER_FRAME	DVS_FRAME_HEIGHT  //720
#define	DMA_FIL_TXDATA_LEN	DMA_FIL_SINGLE_TXDATA_LEN*DMA_FIL_TXBD_NUM //960*2/8 *720*6
#define	DMA_FIL_TXDATA_LEN_PER_FRAME  DMA_FIL_TXDATA_LEN / 6
#define	DMA_FIL_TX_IRQ_THRESHOLD	1	//1 Interrupt per one dma frame

#define	DMA_FIL_SINGLE_RXDATA_LEN	DVS_LINE_SIZE_IN_BYTE //240		//1 line transaction at a time (1-line = 240 byte)
#define	DMA_FIL_RXBD_NUM	DVS_FRAME_HEIGHT * 2 // 1440	//1440 lines in 2 dvs frames for filter output
#define	DMA_FIL_RXBD_NUM_PER_FRAME	DVS_FRAME_HEIGHT //720
#define	DMA_FIL_RXDATA_LEN	DMA_FIL_SINGLE_RXDATA_LEN*DMA_FIL_RXBD_NUM //960*2/8 *720*2
#define	DMA_FIL_RXDATA_LEN_PER_FRAME  DMA_FIL_RXDATA_LEN / 2
#define	DMA_FIL_RX_IRQ_THRESHOLD	1	//1 Interrupt per one frame


#define DMA_FIL_SINGLE_TXBD_LENGTH XAXIDMA_BD_MINIMUM_ALIGNMENT*DMA_FIL_TXBD_NUM
#define DMA_FIL_SINGLE_RXBD_LENGTH XAXIDMA_BD_MINIMUM_ALIGNMENT*DMA_FIL_RXBD_NUM

//  DVSBufNum/2 cases
#define DMA_FIL_TXBDRING_LENGTH (DVS_BUFFER_NUM/2)*DMA_FIL_SINGLE_TXBD_LENGTH
// FilBufNum/2 cases
#define DMA_FIL_RXBDRING_LENGTH (NUM_OF_FIL_FRAME_BUFFER/2)*DMA_FIL_SINGLE_RXBD_LENGTH



/***************** Macros (Inline Functions) Definitions *********************/

#define TRX_RX 0
#define TRX_TX 1

#define DEBUG    0

#define COMP 1

#define INFO     2
#define WARNING  4
#define ERROR    6
#define CRITICAL 8
#define MESSAGE 10

#define CURRENT_LOG_LEVEL MESSAGE

#define LOG_LEVEL_STR(level) \
    ((level) == DEBUG ? "DEBUG: " : \
    (level) == INFO ? "INFO: " : \
    (level) == WARNING ? "WARNING: " : \
    (level) == ERROR ? "ERROR: " : \
    (level) == CRITICAL ? "CRITICAL: " :  \
	(level) == COMP ? "COMPARE: " : "MESSAGE: ")

#define DEBUG_PRINT(level, fmt, ...) \
    do { \
        if (CURRENT_LOG_LEVEL <= level) { \
            xil_printf("%s" fmt, LOG_LEVEL_STR(level), ##__VA_ARGS__); \
        } \
    } while (0)


#define XAXIDMA_RING_SEEKAHEAD(RingPtr, BdPtr, NumBd)                \
{                                                                \
        UINTPTR Addr = (UINTPTR)(void *)(BdPtr);                                     \
                                                                     \
        Addr += ((RingPtr)->Separation * (NumBd));                   \
        if ((Addr > (RingPtr)->LastBdAddr) || ((UINTPTR)(BdPtr) > Addr)) \
        {                                                            \
            Addr -= (RingPtr)->Length;                               \
        }                                                            \
                                                                     \
        (BdPtr) = (XAxiDma_Bd*)(void *)Addr;                                 \
}



#define XAxiDma_ReadReg(BaseAddress, RegOffset)             \
    XAxiDma_In32((BaseAddress) + (RegOffset))


#define LOOPBACK_MODE_EN	0
#define XPAR_CPU_CORE_CLOCK_FREQ_HZ	100000000
#define UART_BASEADDR	XPAR_XUARTPS_0_BASEADDR
#define I2C_MUX_ADDR	0x74  /**< I2C Mux Address */
#define I2C_CLK_ADDR	0x68  /**< I2C Clk Address */

#define IIC_SENSOR_INTR_ID	XPAR_FABRIC_SENSOR_CTRL_AXI_IIC_0_IIC2INTC_IRPT_INTR


#define GPIO_TPG_RESET_DEVICE_ID	XPAR_GPIO_3_DEVICE_ID

#define GPIO_SENSOR			XPAR_SENSOR_CTRL_AXI_GPIO_0_BASEADDR

#ifdef XPAR_PSU_ACPU_GIC_DEVICE_ID
#define PSU_INTR_DEVICE_ID	XPAR_PSU_ACPU_GIC_DEVICE_ID
#endif

#define XPAR_INTC_0_V_FRMBUF_WR_0_VEC_ID XPAR_FABRIC_V_FRMBUF_WR_0_VEC_ID

#define RESET_TIMEOUT_COUNTER	10000
/**************************** Type Definitions *******************************/

/************************** Function Prototypes ******************************/

int I2cMux(void);
int I2cClk(u32 InFreq, u32 OutFreq);

extern u32 InitStreamMuxGpio(void);

void Info(void);

void CloneTxEdid(void);

void SendVSInfoframe(void);

/************************** Variable Definitions *****************************/

// Interrupt
XScuGic Intc;				/* The instance of the Interrupt Controller. */

XAxiDma	AxiDmaFILTx, AxiDmaFILRx;

XAxiDma_BdRing *FILTxRingPtr,*FILRxRingPtr;


char DMA_FIL_TxBdTable[DMA_FIL_TXBDRING_LENGTH] __attribute__ ((aligned(XAXIDMA_BD_MINIMUM_ALIGNMENT)));
char DMA_FIL_RxBdTable[DMA_FIL_RXBDRING_LENGTH] __attribute__ ((aligned(XAXIDMA_BD_MINIMUM_ALIGNMENT)));

u8 *g_FILBuf[NUM_OF_FIL_FRAME_BUFFER];
u64 fil_frame_rdy[NUM_OF_FIL_FRAME_BUFFER];

u32 NumOfEmptyDVSBuf;
u32 NumOfEmptyFILBuf;


// TODO: Pack data into a 8 byte at the bit level  __attribute__ ((aligned(cashe line size)))

u32 g_dma_fil_rd_setup_ptr;
u32 g_dma_fil_rd_work_ptr;
u32 g_dma_fil_wr_setup_ptr;
u32 g_dma_fil_wr_work_ptr;


// TODO: Pack data into a single byte at the bit level
u8 g_FILTxRunning;
u8 g_FILRxRunning;
u8 g_DVSRxRunning;



u8 IsPassThrough; /**< Demo mode 0-colorbar 1-pass through */
u8 StartTxAfterRxFlag;
u8 TxBusy;         /* TX busy flag is set while the TX is initialized */
u8 TxRestartColorbar; 	/* TX restart flag is set when the TX cable has
			 * been reconnected and the TX colorbar was showing.
			 */
u32 Index;

XPipeline_Cfg Pipeline_Cfg;
XPipeline_Cfg New_Cfg;


XMipi_Menu FilterMenu;


extern XV_FrmbufWr_l2	frmbufwr;
extern XAxiDma			DVSDma;

extern XIic IicSensor; /* The instance of the IIC device. */

extern XVprocSs scaler_new_inst;

extern u32 frm_cnt;
extern u32 dvs_frm_cnt; // culc fps


u32 fil_frm_cnt;  // culc fps

extern u32 dvs_wr_ptr; //
extern u64 dvs_frame_array[DVS_BUFFER_NUM];
extern u64 dvs_frame_rdy[DVS_BUFFER_NUM];

/************************** Function Definitions *****************************/
extern void config_csi_cap_path();
extern int config_dvs_cap_path();

XTime debug_t;
int t_count=0;


void DMARxIntrHandler(void *Callback)
{
//	xil_printf("DMA Interrupt Handler!!");
	XAxiDma_BdRing *RxRingPtr = (XAxiDma_BdRing *) Callback;

	u32 IrqStatus;
	int TimeOut;

	/* Read pending interrupts */
	IrqStatus = XAxiDma_BdRingGetIrq(RxRingPtr);

	/* Acknowledge pending interrupts */
	XAxiDma_BdRingAckIrq(RxRingPtr, IrqStatus);

	/*
	 * If no interrupt is asserted, we do not do anything
	 */
	if (!(IrqStatus & XAXIDMA_IRQ_ALL_MASK)) {
		return;
	}

	/*
	 * If error interrupt is asserted, raise error flag, reset the
	 * hardware to recover from the error, and return with no further
	 * processing.
	 */
	if ((IrqStatus & XAXIDMA_IRQ_ERROR_MASK)) {

		XAxiDma_BdRingDumpRegs(RxRingPtr);


		/* Reset could fail and hang
		 * NEED a way to handle this or do not call it??
		 */
		XAxiDma_Reset(&DVSDma);

		TimeOut = RESET_TIMEOUT_COUNTER;

		while (TimeOut) {
			if(XAxiDma_ResetIsDone(&DVSDma)) {
				break;
			}

			TimeOut -= 1;
		}

		return;
	}

	/*
	 * If completion interrupt is asserted, call RX call back function
	 * to handle the processed BDs and then raise the according flag.
	 */
	if ((IrqStatus & (XAXIDMA_IRQ_IOC_MASK))) {
		DmaWriteDoneCallback(RxRingPtr);
	}
}

/*****************************************************************************/
/**
 *
 * This function setup SI5324 clock generator over IIC.
 *
 * @return	The number of bytes sent.
 *
 * @note	None.
 *
 *****************************************************************************/
int I2cMux(void) {
	u8 Buffer;
	int Status;

	/* Select SI5324 clock generator */
	Buffer = 0x80;
	Status = XIic_Send((XPAR_IIC_0_BASEADDR), (I2C_MUX_ADDR),
				(u8 *) &Buffer, 1, (XIIC_STOP));

	return Status;
}

/*****************************************************************************/
/**
 *
 * This function setup SI5324 clock generator either in free or locked mode.
 *
 * @param	InFreq specifies an input frequency for the si5324.
 * @param	OutFreq specifies the output frequency of si5324.
 *
 * @return	Zero if error in programming external clock ele '1' if success
 *
 * @note	None.
 *
 *****************************************************************************/
int I2cClk(u32 InFreq, u32 OutFreq) {
	int Status;

	/* Free running mode */
	if (InFreq == 0) {

		Status = Si5324_SetClock((XPAR_IIC_0_BASEADDR),
						(I2C_CLK_ADDR),
						(SI5324_CLKSRC_XTAL),
						(SI5324_XTAL_FREQ),
						OutFreq);

		if (Status != (SI5324_SUCCESS)) {
			print("Error programming free mode SI5324\n\r");
			return 0;
		}
	}

	/* Locked mode */
	else {
		Status = Si5324_SetClock((XPAR_IIC_0_BASEADDR),
						(I2C_CLK_ADDR),
						(SI5324_CLKSRC_CLK1),
						InFreq,
						OutFreq);

		if (Status != (SI5324_SUCCESS)) {
			print("Error programming locked mode SI5324\n\r");
			return 0;
		}
	}

	return 1;
}
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

	XAxiDma * AxiDmaFILTxPtr = &AxiDmaFILTx;
	XAxiDma * AxiDmaFILRxPtr = &AxiDmaFILRx;

	XAxiDma_BdRing *DmaFILTxRingPtr = XAxiDma_GetTxRing(AxiDmaFILTxPtr);
	XAxiDma_BdRing *DmaFILRxRingPtr = XAxiDma_GetRxRing(AxiDmaFILRxPtr);

	XAxiDma_BdRing *RxRingPtr = XAxiDma_GetRxRing(&DVSDma);

	XScuGic *IntcInstPtr = &Intc;

	/*
	 * Initialize the interrupt controller driver so that it's ready to
	 * use, specify the device ID that was generated in xparameters.h
	 */

	XScuGic_Config *IntcCfgPtr;
	IntcCfgPtr = XScuGic_LookupConfig(PSU_INTR_DEVICE_ID);
	if (IntcCfgPtr == NULL) {
		print("ERR:: Interrupt Controller not found");
		return (XST_DEVICE_NOT_FOUND);
	}
	Status = XScuGic_CfgInitialize(IntcInstPtr, IntcCfgPtr,
			IntcCfgPtr->CpuBaseAddress);

	if (Status != XST_SUCCESS) {
		xil_printf("Intc initialization failed!\r\n");
		return XST_FAILURE;
	}

	XScuGic_SetPriorityTriggerType(IntcInstPtr, XPAR_FABRIC_AXIDMA_1_VEC_ID, 0xA0, 0x3);


	XScuGic_SetPriorityTriggerType(IntcInstPtr, DMA_FIL_TX_IRPT_INTR, 0xA0, 0x3);
	XScuGic_SetPriorityTriggerType(IntcInstPtr, DMA_FIL_RX_IRPT_INTR, 0xA0, 0x3);


	/*
	 * Start the interrupt controller such that interrupts are recognized
	 * and handled by the processor
	 */

	Status = XScuGic_Connect(IntcInstPtr, IIC_SENSOR_INTR_ID,
				(XInterruptHandler) XIic_InterruptHandler,
				(void *) &IicSensor);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	Status = XScuGic_Connect(IntcInstPtr, XPAR_INTC_0_V_FRMBUF_WR_0_VEC_ID,
					(XInterruptHandler) XVFrmbufWr_InterruptHandler,
					(void *) &frmbufwr);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	Status = XScuGic_Connect(IntcInstPtr, DMA_FIL_TX_IRPT_INTR,
						(Xil_InterruptHandler) DmaFILTxIntrHandler,DmaFILTxRingPtr);
	if (Status != XST_SUCCESS) return Status;
	Status = XScuGic_Connect(IntcInstPtr, DMA_FIL_RX_IRPT_INTR,
						(Xil_InterruptHandler) DmaFILRxIntrHandler,DmaFILRxRingPtr);
	if (Status != XST_SUCCESS) return Status;

	/*
	 * Connect the device driver handler that will be called when an
	 * interrupt for the device occurs, the handler defined above performs
	 * the specific interrupt processing for the device.
	 */
	Status = XScuGic_Connect(IntcInstPtr, XPAR_XIICPS_1_INTR,
			(Xil_InterruptHandler)XIicPs_MasterInterruptHandler,
			(void *) &IicPsInstance);
	if (Status != XST_SUCCESS) return Status;
	Status = XScuGic_Connect(IntcInstPtr, XPAR_FABRIC_AXIDMA_1_VEC_ID,
			(Xil_InterruptHandler) DMARxIntrHandler,
			RxRingPtr);
	if (Status != XST_SUCCESS) return Status;


	/* Enable IO expander and sensor IIC interrupts */
	XScuGic_Enable(IntcInstPtr, IIC_SENSOR_INTR_ID);
	XScuGic_Enable(IntcInstPtr, XPAR_INTC_0_V_FRMBUF_WR_0_VEC_ID);

	XScuGic_Enable(IntcInstPtr, DMA_FIL_TX_IRPT_INTR);
	XScuGic_Enable(IntcInstPtr, DMA_FIL_RX_IRPT_INTR);

	XScuGic_Enable(IntcInstPtr, XPAR_XIICPS_1_INTR);
	XScuGic_Enable(IntcInstPtr, XPAR_FABRIC_AXIDMA_1_VEC_ID);

	Xil_ExceptionInit();

	/*Register the interrupt controller handler with the exception table.*/
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
			(Xil_ExceptionHandler) XScuGic_InterruptHandler,
			(XScuGic *) IntcInstPtr);


	Xil_ExceptionEnable();

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

/*****************************************************************************/
/**
* This function resets image processing pipe.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void Reset_IP_Pipe(void)
{

//	Xil_Out32(GPIO_IP_RESET1, 0x00);
//	Xil_Out32(GPIO_IP_RESET2, 0x00);
//	usleep(1000);
//	Xil_Out32(GPIO_IP_RESET1, 0x01);
//	Xil_Out32(GPIO_IP_RESET2, 0x03);

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


	/* Setup the default pipeline configuration parameters */
	/* Look for Default ColorDepth manual setting just after
	 * SetColorDepth
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

	/* Default Resolution that to be displayed */
	Pipeline_Cfg.VideoMode =  XVIDC_VM_1920x1080_60_P;

	configure_buffer_system();
	//configure_fil_buffer

		u64 fil_frame_addr = FIL_BUF_BASE_ADDR;
		u64 fil_frame_rdy_addr = FIL_BUFFER_RDY;
		for (int i=0; i < NUM_OF_FIL_FRAME_BUFFER; ++i) {
			g_FILBuf[i] = (u8 *)(UINTPTR)(fil_frame_addr);
			fil_frame_rdy[i] = fil_frame_rdy_addr;
			fil_frame_addr += DVS_FRAME_SIZE_IN_BYTE;
			fil_frame_rdy_addr += 1;
		}
		Xil_DCacheInvalidateRange((UINTPTR)FIL_BUF_BASE_ADDR, NUM_OF_FIL_FRAME_BUFFER*DVS_FRAME_SIZE_IN_BYTE);
		Xil_DCacheInvalidateRange((UINTPTR)FIL_BUFFER_RDY, NUM_OF_FIL_FRAME_BUFFER);

		FrameBufferPointerInit();


	memset((u64 *)fil_frame_rdy[0], 0b0, NUM_OF_FIL_FRAME_BUFFER);
	memset(g_FILBuf[0], 0b01010101, NUM_OF_FIL_FRAME_BUFFER*DVS_FRAME_SIZE_IN_BYTE);


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

	StartTxAfterRxFlag = (FALSE);
	TxBusy = (FALSE);
	TxRestartColorbar = (FALSE);

	/* Start in color bar */
	IsPassThrough = 0;

	/* Initialize platform */
	init_platform();



	/* Initialize IIC */
	Status = InitIIC();
	if (Status != XST_SUCCESS) {
		xil_printf(TXT_RED "\n\rIIC Init Failed \n\r" TXT_RST);
		return XST_FAILURE;
	}

	Status = InitDVSIIC();
	if (Status != XST_SUCCESS) {
		xil_printf(TXT_RED "\n\rDVS IIC Init Failed \n\r" TXT_RST);
		return XST_FAILURE;
	} else {
		print(TXT_GREEN "IIC init SUCCEEDED.\n\r" TXT_RST);
	}

	//=========================================================//
	//=========================================================//
		//Filter Setup & Run
		// Reg1: Set Threshold  ==>> [13:7]OffTh,  [6:0]OnTh
		// Reg0: Run Filter ==>> 1: Run
		//u32 Threshold = 0b00000100000010;
	u32 Threshold = 0b00000000000000;
		//u32 Threshold = 0b11111111111111;
	DVS_5X5X5_FILTER_mWriteReg(XPAR_DVS_5X5X5_FILTER_0_AXI_LITE_BASEADDR, DVS_5X5X5_FILTER_AXI_Lite_SLV_REG1_OFFSET, Threshold);
	DVS_5X5X5_FILTER_mWriteReg(XPAR_DVS_5X5X5_FILTER_0_AXI_LITE_BASEADDR, DVS_5X5X5_FILTER_AXI_Lite_SLV_REG0_OFFSET, 1);

		/*Status = DVS_5X5X5_FILTER_Reg_SelfTest(XPAR_DVS_5X5X5_FILTER_0_AXI_LITE_BASEADDR);
		if (Status != XST_SUCCESS) xil_printf("DVS Filter Reg SelfTest failed.\n\r");	else xil_printf("DVS Filter Reg SelfTest succeed.\n\r");*/
	//=========================================================//
	//=========================================================//

	/* Turn on DVS Sensor */
	Status = StartDVSSensor();
	if (Status != XST_SUCCESS) {
		print(TXT_RED "START DVS error.\n\r" TXT_RST);
		return XST_FAILURE;
	} else {
		print(TXT_GREEN "START DVS SUCCEEDED.\n\r" TXT_RST);
	}


	/* Initialize IRQ */
	Status = SetupInterruptSystem();
	if (Status == XST_FAILURE) {
		print(TXT_RED "IRQ init failed.\n\r" TXT_RST);
		return XST_FAILURE;
	}


	// DMA Initialization - 1st stage : Get DMA Hardware Information & put into instance and BdRing
	Status = GetDMAHardwareSpecificationAndPutIntoInstance(&AxiDmaFILTx, XPAR_DMA_FIL_DEVICE_ID);
	if (Status != XST_SUCCESS) DEBUG_PRINT(ERROR, "Get DMA FIL Hardware Specification failed.\n\r");
	Status = GetDMAHardwareSpecificationAndPutIntoInstance(&AxiDmaFILRx, XPAR_DMA_FIL_DEVICE_ID);
	if (Status != XST_SUCCESS) DEBUG_PRINT(ERROR, "Get DMA FIL Hardware Specification failed.\n\r");

	// Get BD Ring Pointers
	FILTxRingPtr = XAxiDma_GetTxRing(&AxiDmaFILTx);
	FILRxRingPtr = XAxiDma_GetRxRing(&AxiDmaFILRx);

	DEBUG_PRINT(DEBUG,"FILTxRingFreeCnt: %d\n\r",FILTxRingPtr->FreeCnt);
	DEBUG_PRINT(DEBUG,"FILRxRingFreeCnt: %d\n\r",FILRxRingPtr->FreeCnt);


	XAxiDma_BdRingIntEnable(FILTxRingPtr, XAXIDMA_IRQ_ALL_MASK);
	XAxiDma_BdRingIntEnable(FILRxRingPtr, XAXIDMA_IRQ_ALL_MASK);
	// Enable Cyclic DMA mode
	XAxiDma_BdRingEnableCyclicDMA(FILTxRingPtr);
	XAxiDma_BdRingEnableCyclicDMA(FILRxRingPtr);
	XAxiDma_SelectCyclicMode(&AxiDmaFILTx, XAXIDMA_DMA_TO_DEVICE, 1);
	XAxiDma_SelectCyclicMode(&AxiDmaFILRx, XAXIDMA_DEVICE_TO_DMA, 1);

	/* Start RX DMA channel */
	Status = XAxiDma_BdRingStart(FILTxRingPtr);
	Status = XAxiDma_BdRingStart(FILRxRingPtr);

	// DMA Initialization - 2nd stage : Create BdRing
	Status = TxBDRingInit(FILTxRingPtr, (char*)&DMA_FIL_TxBdTable, DMA_FIL_TXBD_NUM, DVS_BUFFER_NUM/2);
	if (Status != XST_SUCCESS) DEBUG_PRINT(ERROR, "FIL DMA TxBDRingInit failed.\n\r");
	Status = RxBDRingInit(FILRxRingPtr, (char*)&DMA_FIL_RxBdTable, DMA_FIL_RXBD_NUM, NUM_OF_FIL_FRAME_BUFFER/2);
	if (Status != XST_SUCCESS) DEBUG_PRINT(ERROR, "FIL DMA RxBDRingInit failed.\n\r");


	// DMA Setup & Run
	DEBUG_PRINT(INFO,"==========DMA TRxSetup......==========\n\r");
	Status = GenerateAllBDTable();
	if (Status != XST_SUCCESS) DEBUG_PRINT(ERROR,"DMA TRxSetup failed.\n\r");	else DEBUG_PRINT(INFO, "DMA TRxSetup succeed.\n\r");



	/* IIC interrupt handlers */
	SetupIICIntrHandlers();

	/* Reset Demosaic, Gamma_Lut and CSC IPs */
	Reset_IP_Pipe();

	/* Initialize VProcSS Scalar IP */
	InitVprocSs_Scaler(1);
	xil_printf("\r\nInitVprocSs_Scaler Done \n\r");

	/* Initialize CSIRXSS  */
	Status = InitializeCsiRxSs();
	if (Status != XST_SUCCESS) {
		xil_printf(TXT_RED "CSI Rx Ss Init failed status = %x.\r\n"
				 TXT_RST, Status);
		return XST_FAILURE;
	}

	Status = InitializeDphy();
	if (Status != XST_SUCCESS) {
		xil_printf(TXT_RED "DVS DPHY Init failed status = %x.\r\n" TXT_RST, Status);
		return XST_FAILURE;
	} else {
		xil_printf(TXT_GREEN "DVS DPHY Init Success\r\n" TXT_RST);
	}
	config_csi_cap_path();
	config_dvs_cap_path();

	print("---------------------------------\r\n");

	/* Enable exceptions. */
	Xil_AssertSetCallback((Xil_AssertCallback) Xil_AssertCallbackRoutine);
	Xil_ExceptionEnable();

	/* Reset Camera Sensor module through GPIO */
	xil_printf("Disable CAM_RST of Sensor through GPIO\r\n");
	CamReset();
	xil_printf("Sensor is  Enabled\r\n");

	/* Program Camera sensor */
	Status = SetupCameraSensor();
	if (Status != XST_SUCCESS) {
		xil_printf("Failed to setup Camera sensor\r\n");
		return XST_FAILURE;
	}


	/*************** Program DVS sensor **************/
	// frame header setting
	MIPI_CONTROLLER_mWriteReg(MIPI_CONTROLLER_BASEADDR,
				MIPI_CONTROLLER_S00_AXI_SLV_REG0_OFFSET,
				0);
	MIPI_CONTROLLER_mWriteReg(MIPI_CONTROLLER_BASEADDR,
				MIPI_CONTROLLER_S00_AXI_SLV_REG1_OFFSET,
				0);
	start_dvs_cap_pipe();
	Status = ProgramDVSSensor();
	if (Status != XST_SUCCESS) {
		print(TXT_RED "PROGRAM DVS error.\n\r" TXT_RST);
		return XST_FAILURE;
	} else {
		print(TXT_GREEN "PROGRAM DVS SUCCEEDED.\n\r" TXT_RST);
	}
	/*************************************************/

	start_csi_cap_pipe(Pipeline_Cfg.VideoMode);

	InitImageProcessingPipe();

	/* Start Camera Sensor to capture video */
	StartSensor();

	XMipi_MenuInitialize(&FilterMenu, UART_BASEADDR);

	New_Cfg = Pipeline_Cfg;

	/* Print the Pipe line configuration */
	PrintPipeConfig();

	g_FILTxRunning = 0;
	//g_FILRxRunning = 1;

	DEBUG_PRINT(INFO, "DMA setup is done and will be run.\r\n");


	Status = RxBDRun(FILRxRingPtr, DMA_FIL_RXBD_NUM);
	if (Status != XST_SUCCESS) xil_printf("FIL RxBDRun failed.\n\r");


	XTime start_t, start_t2, end_t;
	XTime_GetTime(&start_t);
	XTime_GetTime(&start_t2);

	fil_frm_cnt =0;


	do {
//		sleep(1);
//		CsiRxPrintRegStatus();

		XTime_GetTime(&end_t);
		if(end_t-start_t>COUNTS_PER_SECOND){
			XTime_GetTime(&start_t);
			/*int rdy_sum=0;
			for(int i=0;i<16;i++){
				rdy_sum += Xil_In8(fil_frame_rdy[i]);
			}*/
			xil_printf("CIS frame count: %d, DVS frame count: %d, fil_frm_cnt: %d\r\n",frm_cnt, dvs_frm_cnt,fil_frm_cnt);

			xil_printf("DVS frame count-fil_frm_cnt: %d\r\n",dvs_frm_cnt-fil_frm_cnt);

			//xil_printf("rdy count: %d\r\n",rdy_sum);

		}

		/*if(end_t-start_t2>(COUNTS_PER_SECOND/1000)){
			XTime_GetTime(&start_t2);
			Xil_Out8(fil_frame_rdy[ g_dma_fil_wr_setup_ptr % NUM_OF_FIL_FRAME_BUFFER ], 0x1);
			Xil_Out8(fil_frame_rdy[ (g_dma_fil_wr_setup_ptr % NUM_OF_FIL_FRAME_BUFFER) +1 ], 0x1);
			UpdateFilBufPtr(&g_dma_fil_wr_setup_ptr,2);
			fil_frm_cnt+=2;
		}*/


		XMipi_MenuProcess(&FilterMenu);

		if(g_FILTxRunning==0 && NumOfEmptyDVSBuf<=(DVS_BUFFER_NUM-FIL_RUN_MARGIN)){
			g_FILTxRunning = 1;

			Status = TxBDRun(FILTxRingPtr, DMA_FIL_TXBD_NUM);
			if (Status != XST_SUCCESS) DEBUG_PRINT(ERROR, "TxBDRun failed.\n\r");
			/*if(t_count==0){
							t_count++;
							XTime_GetTime(&debug_t);
			}
			else if(t_count==1){
							t_count++;
							XTime tmp = debug_t;
							XTime_GetTime(&debug_t);
							xil_printf("tx intr -> tx run: %d \r\n", debug_t-tmp);
			}
			else t_count++;*/
		}

		/*if(g_FILTxRunning==0){
			dvs_ready = Xil_In8(dvs_frame_rdy[ (g_dma_fil_rd_setup_ptr % NUM_OF_FIL_FRAME_BUFFER) +1 ]) ;
			if( dvs_ready == 0x1 )
				g_FILTxRunning = 1;
				Status = TxBDRun(FILTxRingPtr, DMA_FIL_TXBD_NUM);
				if (Status != XST_SUCCESS) DEBUG_PRINT(ERROR, "TxBDRun failed.\n\r");

				Xil_Out8(dvs_frame_rdy[ g_dma_fil_rd_setup_ptr % NUM_OF_FIL_FRAME_BUFFER ], 0x0);
				Xil_Out8(dvs_frame_rdy[ (g_dma_fil_rd_setup_ptr % NUM_OF_FIL_FRAME_BUFFER) +1 ], 0x0);
		}*/

		/*if(NumOfEmptyFILBuf<NUM_OF_FIL_FRAME_BUFFER){
			//UpdateFilBufPtr(&g_dma_fil_send_work_ptr,1);
			if( (g_dma_fil_wr_work_ptr>=NUM_OF_FIL_FRAME_BUFFER) == (g_dma_fil_send_work_ptr>=NUM_OF_FIL_FRAME_BUFFER))
				NumOfEmptyFILBuf = ( NUM_OF_FIL_FRAME_BUFFER - (g_dma_fil_wr_work_ptr - g_dma_fil_send_work_ptr) % NUM_OF_FIL_FRAME_BUFFER ) ;
			else
				NumOfEmptyFILBuf = ( g_dma_fil_send_work_ptr - g_dma_fil_wr_work_ptr) % NUM_OF_FIL_FRAME_BUFFER;

			DEBUG_PRINT(DEBUG, "NumOfEmptyFILBuf: %d \r\n",NumOfEmptyFILBuf);

			if( g_FILRxRunning == 0 && NumOfEmptyFILBuf >= 2){
				g_FILRxRunning = 1;
				DEBUG_PRINT(DEBUG, "FIL Rx ReRun\n\r");

				Status = RxBDRun(FILRxRingPtr, DMA_FIL_RXBD_NUM);
				if (Status != XST_SUCCESS) DEBUG_PRINT(ERROR, "RxBDRun failed.\n\r");
			}
		}*/

	} while (1);

	cleanup_platform();

	return 0;
}




static int RxBDRun(XAxiDma_BdRing* RxRingPtr, u32 NumBDsPerSingleRx)
{
	int Status;
	XAxiDma_Bd *BdPtr = (XAxiDma_Bd*) RxRingPtr->PreHead;

	// To ensure proper operation, confirm that the DMA is turned off before restart.
	// !!HERE!! : Set Current Descriptor Buffer Here
	UINTPTR RegBase = RxRingPtr->ChanBase;
	XAxiDma_WriteReg(RegBase, XAXIDMA_CR_OFFSET,XAxiDma_ReadReg(RegBase, XAXIDMA_CR_OFFSET)	& (~XAXIDMA_CR_RUNSTOP_MASK));
	XAxiDma_WriteReg(RegBase, XAXIDMA_CDESC_OFFSET, ((UINTPTR)BdPtr & (UINTPTR)XAXIDMA_DESC_LSB_MASK));

	// Enqueue all RxBDs to HW
	Status = XAxiDma_CustomBdRingToHw(RxRingPtr, NumBDsPerSingleRx, BdPtr);
	if (Status != XST_SUCCESS) {
		DEBUG_PRINT(ERROR, "Error committing RxBDs prior to starting\r\n");
		return XST_FAILURE;
	}

	// Start DMA RX channel. Transmission starts at once
	//Status = XAxiDma_BdRingStart(RxRingPtr);	//From Example Original Code
	//if (Status != XST_SUCCESS) return XST_FAILURE;

	Status = XAxiDma_StartBdRingHw(RxRingPtr);	// Omit setting the CDESC pointer from XAxiDma_BdRingStart() as it has already been set !!HERE!!.
	if (Status != XST_SUCCESS) return XST_FAILURE;

	return XST_SUCCESS;
}

static int TxBDRun(XAxiDma_BdRing* TxRingPtr, u32 NumBDsPerSingleTx)
{
	int Status;
	XAxiDma_Bd *BdPtr = (XAxiDma_Bd*) TxRingPtr->PreHead;

	// To ensure proper operation, confirm that the DMA is turned off before restart.
	// !!HERE!! : Set Current Descriptor Buffer Here
	UINTPTR RegBase = TxRingPtr->ChanBase;
	XAxiDma_WriteReg(RegBase, XAXIDMA_CR_OFFSET,XAxiDma_ReadReg(RegBase, XAXIDMA_CR_OFFSET)	& (~XAXIDMA_CR_RUNSTOP_MASK));
	XAxiDma_WriteReg(RegBase, XAXIDMA_CDESC_OFFSET, ((UINTPTR)BdPtr & (UINTPTR)XAXIDMA_DESC_LSB_MASK));


	//  Enqueue all TxBDs to HW
	Status = XAxiDma_CustomBdRingToHw(TxRingPtr, NumBDsPerSingleTx, BdPtr);
	if (Status != XST_SUCCESS) {
		DEBUG_PRINT(ERROR, "Error committing TxBDs prior to starting\r\n");
		return XST_FAILURE;
	}

	// Start DMA TX channel. Transmission starts at once
	//Status = XAxiDma_BdRingStart(TxRingPtr);	//From Example Original Code
	//if (Status != XST_SUCCESS) return XST_FAILURE;

	Status = XAxiDma_StartBdRingHw(TxRingPtr);	// Omit setting the CDESC pointer from XAxiDma_BdRingStart() as it has already been set !!HERE!!.
	if (Status != XST_SUCCESS) return XST_FAILURE;

	return XST_SUCCESS;
}


static int RxBDRetrieve(XAxiDma_BdRing* RxRingPtr, u32 NumBDsPerSingleRx)
{

	/*int Status, NumRetrievedBd;
	XAxiDma_Bd *BdPtr;

	NumRetrievedBd = XAxiDma_BdRingFromHw(RxRingPtr, NumBDsPerSingleRx, &BdPtr);
	if (NumRetrievedBd != NumBDsPerSingleRx) {
		DEBUG_PRINT(ERROR, "Requesting NumRetrievedBd: %d, \r\n", NumRetrievedBd);
		DEBUG_PRINT(ERROR, "Error in interrupt coalescing");
	} else {
		// Don't bother to check the BDs status, just free them
		Status = XAxiDma_BdRingFree(RxRingPtr, NumBDsPerSingleRx, BdPtr);
		if (Status != XST_SUCCESS)
			DEBUG_PRINT(ERROR, "Error freeing RxBDs\r\n");
	}*/

	RxRingPtr->HwCnt -= NumBDsPerSingleRx;
	//RxRingPtr->PostCnt += NumBDsPerSingleRx;
	XAXIDMA_RING_SEEKAHEAD(RxRingPtr, RxRingPtr->HwHead, NumBDsPerSingleRx);

	RxRingPtr->FreeCnt += NumBDsPerSingleRx;
	//RxRingPtr->PostCnt -= NumBDsPerSingleRx;
	XAXIDMA_RING_SEEKAHEAD(RxRingPtr, RxRingPtr->PostHead, NumBDsPerSingleRx);

	return XST_SUCCESS;
}

static int TxBDRetrieve(XAxiDma_BdRing* TxRingPtr, u32 NumBDsPerSingleTx)
{
	//int Status;//, NumRetrievedBd;
	//XAxiDma_Bd *BdPtr;

	//NumRetrievedBd = XAxiDma_BdRingFromHw(TxRingPtr, NumBDsPerSingleTx, &BdPtr);
	/*if (NumRetrievedBd != NumBDsPerSingleTx) {
		DEBUG_PRINT(ERROR, "Requesting NumRetrievedBd: %d, \r\n", NumRetrievedBd);
		DEBUG_PRINT(ERROR, "Error in interrupt coalescing\r\n");
	} else {
		// Don't bother to check the BDs status, just free them
		Status = XAxiDma_BdRingFree(TxRingPtr, NumBDsPerSingleTx, BdPtr);
		if (Status != XST_SUCCESS)
			DEBUG_PRINT(ERROR, "Error freeing TxBDs\r\n");
	}*/


	TxRingPtr->HwCnt -= NumBDsPerSingleTx;
	//TxRingPtr->PostCnt += NumBDsPerSingleTx;
	XAXIDMA_RING_SEEKAHEAD(TxRingPtr, TxRingPtr->HwHead, NumBDsPerSingleTx);

    TxRingPtr->FreeCnt += NumBDsPerSingleTx;
    //TxRingPtr->PostCnt -= NumBDsPerSingleTx;
    XAXIDMA_RING_SEEKAHEAD(TxRingPtr, TxRingPtr->PostHead, NumBDsPerSingleTx);

	return XST_SUCCESS;
}

static int TrxSetup_v2(u8 Mode)
{
	switch(Mode){
	case TRX_TX:{
		DEBUG_PRINT(DEBUG, "FIL_TX_BD_SET\r\n");
		FILTxRingPtr->FreeCnt -= DMA_FIL_TXBD_NUM;
		FILTxRingPtr->PreCnt += DMA_FIL_TXBD_NUM;
		break;
	}
	case TRX_RX:{
		DEBUG_PRINT(DEBUG, "FIL_RX_BD_SET\r\n");
		FILRxRingPtr->FreeCnt -= DMA_FIL_RXBD_NUM;
		FILRxRingPtr->PreCnt += DMA_FIL_RXBD_NUM;
		break;
	}
	default: break;
	}
	return XST_SUCCESS;
}


static int RxBDRingInit(XAxiDma_BdRing *RxRingPtr, char* RxBDBufBase, u32 NumBDsPerSingleRx, u32 NumOfBDGroup)
{
	int Status;
	XAxiDma_Bd BdTemplate;
	int BdCount;

	/* Disable all RX interrupts before RxBD space setup */
	XAxiDma_BdRingIntDisable(RxRingPtr, XAXIDMA_IRQ_ALL_MASK);

	// Setup Rx BD space
	BdCount = NumOfBDGroup * NumBDsPerSingleRx;
	Status = XAxiDma_BdRingCreate(RxRingPtr, (UINTPTR)RxBDBufBase, (UINTPTR)RxBDBufBase, XAXIDMA_BD_MINIMUM_ALIGNMENT, BdCount);
	if (Status != XST_SUCCESS) {
		DEBUG_PRINT(ERROR, "Rx bd create failed with %d\r\n", Status);
		return XST_FAILURE;
	}

	// Setup a BD template for the Rx channel. Then copy it to every RX BD.
	XAxiDma_BdClear(&BdTemplate);
	Status = XAxiDma_BdRingClone(RxRingPtr, &BdTemplate);
	if (Status != XST_SUCCESS) {
		DEBUG_PRINT(ERROR, "Rx bd clone failed with %d\r\n", Status);
		return XST_FAILURE;
	}

	Status = XAxiDma_BdRingSetCoalesce(RxRingPtr, (u8)DMA_FIL_RX_IRQ_THRESHOLD, 0);	//0 means no timeout interrupt
	if (Status != XST_SUCCESS) {
		DEBUG_PRINT(ERROR, "Error setting coalescing settings\r\n");
		return XST_FAILURE;
	}

	/*
	 * Enable the send interrupts. Nothing should be transmitted yet as the
	 * device has not been started
	 */
	XAxiDma_BdRingIntEnable(RxRingPtr, XAXIDMA_IRQ_ALL_MASK);

	return XST_SUCCESS;

}

static int TxBDRingInit(XAxiDma_BdRing *TxRingPtr, char* TxBDBufBase, u32 NumBDsPerSingleTx, u32 NumOfBDGroup)
{
	int Status;
	XAxiDma_Bd BdTemplate;
	int BdCount;

	/* Disable all TX interrupts before TxBD space setup */
	XAxiDma_BdRingIntDisable(TxRingPtr, XAXIDMA_IRQ_ALL_MASK);

	// Setup Tx BD space
	BdCount = NumOfBDGroup * NumBDsPerSingleTx;
	Status = XAxiDma_BdRingCreate(TxRingPtr, (UINTPTR)TxBDBufBase, (UINTPTR)TxBDBufBase, XAXIDMA_BD_MINIMUM_ALIGNMENT, BdCount);
	if (Status != XST_SUCCESS) {
		DEBUG_PRINT(ERROR, "Tx bd create failed with %d\r\n", Status);
		return XST_FAILURE;
	}

	// Setup a BD template for the Tx channel. Then copy it to every TX BD.
	XAxiDma_BdClear(&BdTemplate);
	Status = XAxiDma_BdRingClone(TxRingPtr, &BdTemplate);
	if (Status != XST_SUCCESS) {
		DEBUG_PRINT(ERROR, "Tx bd clone failed with %d\r\n", Status);
		return XST_FAILURE;
	}

	Status = XAxiDma_BdRingSetCoalesce(TxRingPtr, (u8)DMA_FIL_TX_IRQ_THRESHOLD, 0);	//0 means no timeout interrupt
	if (Status != XST_SUCCESS) {
		DEBUG_PRINT(ERROR, "Error setting coalescing settings\r\n");
		return XST_FAILURE;
	}

	/*
	 * Enable the send interrupts. Nothing should be transmitted yet as the
	 * device has not been started
	 */
	XAxiDma_BdRingIntEnable(TxRingPtr, XAXIDMA_IRQ_ALL_MASK);

	return XST_SUCCESS;

}

static int GetDMAHardwareSpecificationAndPutIntoInstance(XAxiDma * AxiDmaInstPtr, u32 DeviceId)
{
	XAxiDma_Config *Config;
	Config = XAxiDma_LookupConfig(DeviceId);
	if (!Config) {
		DEBUG_PRINT(ERROR, "No config found for %d\r\n", DeviceId);
		return XST_FAILURE;
	}
	XAxiDma_CfgInitialize(AxiDmaInstPtr, Config);

	return XST_SUCCESS;
}



static void DmaFILTxIntrHandler(XAxiDma_BdRing *TxRingPtr) {

	u32 IrqStatus;

	/*if(t_count==1){
		XTime tmp = debug_t;
		XTime_GetTime(&debug_t);
		xil_printf("tx run -> tx intr: %d \r\n", debug_t-tmp);
	}
	else if(t_count==1000){
		t_count = 0;
	}*/
	DEBUG_PRINT(DEBUG, "DmaFILTxIntrHandler is called\r\n");

	// Read pending interrupts
	IrqStatus = XAxiDma_BdRingGetIrq(TxRingPtr);

	// Acknowledge pending interrupts
	XAxiDma_BdRingAckIrq(TxRingPtr, IrqStatus);
	if (!(IrqStatus & XAXIDMA_IRQ_ALL_MASK)) {
		DEBUG_PRINT(ERROR, "AXIDma: No interrupts asserted in TX status register\r\n");
		return;
	}
	if ((IrqStatus & XAXIDMA_IRQ_ERROR_MASK)) {
		DEBUG_PRINT(ERROR, "AXIDMA: TX Error interrupts\r\n");
		return;
	}
	if ((IrqStatus & (XAXIDMA_IRQ_DELAY_MASK | XAXIDMA_IRQ_IOC_MASK))) {
		DmaFILTxDoneCallBack(TxRingPtr);
	}
	return ;
}

static void DmaFILTxDoneCallBack(XAxiDma_BdRing *TxRingPtr){
	int Status;

	DEBUG_PRINT(DEBUG, "DmaFilTxDoneCallBack is called\r\n");

	UpdateFilRDBufPtr(&g_dma_fil_rd_work_ptr,2);
	if((dvs_wr_ptr>=DVS_BUFFER_NUM) == (g_dma_fil_rd_work_ptr>=DVS_BUFFER_NUM))
		NumOfEmptyDVSBuf = ( DVS_BUFFER_NUM - (dvs_wr_ptr - g_dma_fil_rd_work_ptr) % DVS_BUFFER_NUM ) ;
	else
		NumOfEmptyDVSBuf = ( g_dma_fil_rd_work_ptr - dvs_wr_ptr) % DVS_BUFFER_NUM;

	DEBUG_PRINT(DEBUG, "NumOfEmptyDVSBuf: %d \r\n",NumOfEmptyDVSBuf);


	g_FILTxRunning = 0;
	if(NumOfEmptyDVSBuf <= (DVS_BUFFER_NUM-FIL_RUN_MARGIN)){
		g_FILTxRunning = 1;
		Status = TxBDRun(TxRingPtr, DMA_FIL_TXBD_NUM);
		if (Status != XST_SUCCESS) DEBUG_PRINT(ERROR, "FIL TxBDRun failed.\n\r");

		/*if(t_count==0){
						t_count++;
						XTime_GetTime(&debug_t);
		}
		else if(t_count==1){
						t_count++;
						XTime tmp = debug_t;
						XTime_GetTime(&debug_t);
						xil_printf("tx intr -> tx run: %d \r\n", debug_t-tmp);
		}
		else t_count++;*/

	}
	else{
		DEBUG_PRINT(INFO, "There is not enough data accumulated in the DVS Buffer, FIL Tx Stopped (fil read work pointer: 0x%x)\r\n", g_dma_fil_rd_work_ptr%DVS_BUFFER_NUM);
	}
	//Status = TxBDRun(FILTxRingPtr, DMA_FIL_TXBD_NUM);
	//if (Status != XST_SUCCESS) DEBUG_PRINT(ERROR, "TxBDRun failed.\n\r");

	Status = TxBDRetrieve(TxRingPtr, DMA_FIL_TXBD_NUM);
	if (Status != XST_SUCCESS) DEBUG_PRINT(ERROR, "FIL TxBDRetrieve failed.\n\r");

	TrxSetup_v2(TRX_TX);

	//UpdateFilRDBufPtr(&g_dma_fil_rd_setup_ptr,2);
	//g_FILTxRunning = 0;


	return;
}

static void DmaFILRxIntrHandler(XAxiDma_BdRing *RxRingPtr) {

	int Status;

	DEBUG_PRINT(DEBUG, "DmaFILRxIntrHandler is called\r\n");

	// Read pending interrupts
	Status = XAxiDma_BdRingGetIrq(RxRingPtr);

	// Acknowledge pending interrupts
	XAxiDma_BdRingAckIrq(RxRingPtr, Status);
	if (!(Status & XAXIDMA_IRQ_ALL_MASK)) {
		DEBUG_PRINT(ERROR, "AXIDma: No interrupts asserted in RX status register\r\n");
		return;
	}
	if ((Status & XAXIDMA_IRQ_ERROR_MASK)) {
		DEBUG_PRINT(ERROR, "AXIDMA: RX Error interrupts\n");
		return;
	}

	if ((Status & (XAXIDMA_IRQ_DELAY_MASK | XAXIDMA_IRQ_IOC_MASK))) {
		DmaFILRxDoneCallBack(RxRingPtr);
	}
	else DEBUG_PRINT(ERROR, "Passed.\r\n");
	return ;

}

static void DmaFILRxDoneCallBack(XAxiDma_BdRing *RxRingPtr){

	int Status;

	DEBUG_PRINT(DEBUG, "DmaFILRxDoneCallBack is called\r\n");

	//Run & Retrieve & Setup Dma
	//UpdateFilBufPtr(&g_dma_fil_wr_work_ptr,2);
	/*if((g_dma_fil_wr_work_ptr>=NUM_OF_FIL_FRAME_BUFFER) == (g_dma_fil_send_work_ptr>=NUM_OF_FIL_FRAME_BUFFER))
		NumOfEmptyFILBuf = ( NUM_OF_FIL_FRAME_BUFFER - (g_dma_fil_wr_work_ptr - g_dma_fil_send_work_ptr) % NUM_OF_FIL_FRAME_BUFFER ) ;
	else
		NumOfEmptyFILBuf = ( g_dma_fil_send_work_ptr - g_dma_fil_wr_work_ptr) % NUM_OF_FIL_FRAME_BUFFER;
*/
	DEBUG_PRINT(DEBUG, "NumOfEmptyFILBuf: %d \r\n",NumOfEmptyFILBuf);

	//if(NumOfEmptyFILBuf < 2){
	//	g_FILRxRunning = 0;
	//	DEBUG_PRINT(INFO, "There is no empty space in the FIL Buffer, FIL Rx Stopped (fil write work pointer: 0x%x)\r\n", g_dma_fil_wr_work_ptr%NUM_OF_FIL_FRAME_BUFFER);
	//}
	//else{
	//	g_FILRxRunning = 1;
	//	Status = RxBDRun(RxRingPtr, DMA_FIL_RXBD_NUM);
	//	if (Status != XST_SUCCESS) DEBUG_PRINT(ERROR, "RxBDRun failed.\n\r");
	//}

	Status = RxBDRun(RxRingPtr, DMA_FIL_RXBD_NUM);
	if (Status != XST_SUCCESS) DEBUG_PRINT(ERROR, "RxBDRun failed.\n\r");

	Status = RxBDRetrieve(RxRingPtr, DMA_FIL_RXBD_NUM);
	if (Status != XST_SUCCESS) DEBUG_PRINT(ERROR, "RxBDRetrieve failed.\n\r");


	TrxSetup_v2(TRX_RX);

	Xil_Out8(fil_frame_rdy[ g_dma_fil_wr_setup_ptr % NUM_OF_FIL_FRAME_BUFFER ], 0x1);
	Xil_Out8(fil_frame_rdy[ (g_dma_fil_wr_setup_ptr % NUM_OF_FIL_FRAME_BUFFER) +1 ], 0x1);

	UpdateFilWRBufPtr(&g_dma_fil_wr_setup_ptr,2);

	fil_frm_cnt+=2;

	return;
}


static void FrameBufferPointerInit()
{
	g_dma_fil_rd_setup_ptr = 0;
	g_dma_fil_rd_work_ptr = 0;
	g_dma_fil_wr_setup_ptr = 0;
	g_dma_fil_wr_work_ptr = 0;

	NumOfEmptyDVSBuf = DVS_BUFFER_NUM   ;
	NumOfEmptyFILBuf = NUM_OF_FIL_FRAME_BUFFER;
}

static void UpdateFilWRBufPtr(u32 *pCurBufPtr, u32 uiStep)
{
	u32 uiNextBufPtrCand;
	static u32 uiBufPtrMaxPlus1 = 2*NUM_OF_FIL_FRAME_BUFFER;
	uiNextBufPtrCand = *pCurBufPtr + uiStep;


	if(uiNextBufPtrCand > (uiBufPtrMaxPlus1 - 1)) *pCurBufPtr = uiNextBufPtrCand - uiBufPtrMaxPlus1;
	else *pCurBufPtr = uiNextBufPtrCand;
}
static void UpdateFilRDBufPtr(u32 *pCurBufPtr, u32 uiStep)
{
	u32 uiNextBufPtrCand;
	static u32 uiBufPtrMaxPlus1 = 2*DVS_BUFFER_NUM;
	uiNextBufPtrCand = *pCurBufPtr + uiStep;


	if(uiNextBufPtrCand > (uiBufPtrMaxPlus1 - 1)) *pCurBufPtr = uiNextBufPtrCand - uiBufPtrMaxPlus1;
	else *pCurBufPtr = uiNextBufPtrCand;
}


static int GenerateAllBDTable(void){

	int Status;
	int i;


	// All FIL Tx
	for(i=0; i<DVS_BUFFER_NUM; i+=2){
		Status = GenerateBDTable(FILTxRingPtr, TRX_TX, (u8 *)(INTPTR)dvs_frame_array[i], DMA_FIL_TXDATA_LEN_PER_FRAME, DMA_FIL_SINGLE_TXDATA_LEN, DMA_FIL_TXBD_NUM, DMA_FIL_TXBD_NUM_PER_FRAME, 6);
	}


	// All FIL Rx
	for(i=0; i<NUM_OF_FIL_FRAME_BUFFER; i+=2){
		Status = GenerateBDTable(FILRxRingPtr, TRX_RX, g_FILBuf[i], DMA_FIL_RXDATA_LEN_PER_FRAME, DMA_FIL_SINGLE_RXDATA_LEN, DMA_FIL_RXBD_NUM, DMA_FIL_RXBD_NUM_PER_FRAME, 2);
	}


	return Status;
}

static int GenerateBDTable(XAxiDma_BdRing* RingPtr, u8 TrxType, u8 *BaseAddr, u32 DataLengthPerFrm, u32 SingleDataLength, u32 NumBDsPerSingleTrx, u32 NumOfBDPerFrm, u8 NumOfAlternatingFrm)
{

	int Status;
	u32 NumOfSetBD,FrmIdx,BDIdxInFrm,RegVal;
	UINTPTR TargetAddr;
	XAxiDma_Bd *BdPtr;
	XAxiDma_Bd *BdCurPtr;

	// Prime the engine, allocate all BDs and assign them to the same buffer
	Status = XAxiDma_BdRingAlloc(RingPtr, NumBDsPerSingleTrx, &BdPtr);
	if (Status != XST_SUCCESS) {
		if((Status == XST_INVALID_PARAM)){
			DEBUG_PRINT(ERROR, "BdRingAlloc: negative BD "
					"number %d\r\n", NumBDsPerSingleTrx);
		}
		else if((Status == XST_FAILURE)){
			DEBUG_PRINT(ERROR,
					"Not enough BDs to alloc %d/%d\r\n", NumBDsPerSingleTrx, RingPtr->FreeCnt);
		}
		DEBUG_PRINT(ERROR, "Error allocating TrxBDs prior to starting\r\n");
		return XST_FAILURE;
	}

	// Setup the TrxBDs
	NumOfSetBD = 0;
	BdCurPtr = BdPtr;

	if (TrxType==TRX_RX){
		for (BDIdxInFrm = 0; BDIdxInFrm < NumOfBDPerFrm; BDIdxInFrm++) {
					for (FrmIdx = 0; FrmIdx < NumOfAlternatingFrm; FrmIdx++) {
						TargetAddr = (UINTPTR)((BaseAddr + FrmIdx * DataLengthPerFrm + BDIdxInFrm * (DataLengthPerFrm/NumOfBDPerFrm)) );

						XAxiDma_BdSetBufAddr(BdCurPtr, TargetAddr);
						XAxiDma_BdSetLength(BdCurPtr, SingleDataLength, RingPtr->MaxTransferLen);

						/*RegVal = TrxType==TRX_RX ? 0 : NumBDsPerSingleTrx==1 ? (XAXIDMA_BD_CTRL_TXSOF_MASK | XAXIDMA_BD_CTRL_TXEOF_MASK) :
								NumOfSetBD==0 ? XAXIDMA_BD_CTRL_TXSOF_MASK : (NumOfSetBD==(NumBDsPerSingleTrx-1) ? XAXIDMA_BD_CTRL_TXEOF_MASK : 0);
						XAxiDma_BdSetCtrl(BdCurPtr, RegVal);*/
						XAxiDma_BdSetCtrl(BdCurPtr, 0);
						BdCurPtr = (XAxiDma_Bd *) XAxiDma_BdRingNext(RingPtr, BdCurPtr);

						NumOfSetBD++;
					}
			}
	}
	else{ // TRX_TX
		for (BDIdxInFrm = 0; BDIdxInFrm < NumOfBDPerFrm; BDIdxInFrm++) {
				for (FrmIdx = 0; FrmIdx < NumOfAlternatingFrm; FrmIdx++) {
					TargetAddr = (UINTPTR)((BaseAddr + FrmIdx * DataLengthPerFrm + BDIdxInFrm * (DataLengthPerFrm/NumOfBDPerFrm)) );
					if (TargetAddr > DVS_BUFFER_HIGH){
						TargetAddr -= DVS_BUFFER_TOTAL_SIZE;
					}

					XAxiDma_BdSetBufAddr(BdCurPtr, TargetAddr);
					XAxiDma_BdSetLength(BdCurPtr, SingleDataLength, RingPtr->MaxTransferLen);

					RegVal = NumBDsPerSingleTrx==1 ? (XAXIDMA_BD_CTRL_TXSOF_MASK | XAXIDMA_BD_CTRL_TXEOF_MASK) :
							NumOfSetBD==0 ? XAXIDMA_BD_CTRL_TXSOF_MASK : (NumOfSetBD==(NumBDsPerSingleTrx-1) ? XAXIDMA_BD_CTRL_TXEOF_MASK : 0);
					XAxiDma_BdSetCtrl(BdCurPtr, RegVal);

					BdCurPtr = (XAxiDma_Bd *) XAxiDma_BdRingNext(RingPtr, BdCurPtr);

					NumOfSetBD++;
				}
		}
	}

	return XST_SUCCESS;
}



int XAxiDma_CustomBdRingToHw(XAxiDma_BdRing * RingPtr, int NumBd,
	XAxiDma_Bd * BdSetPtr)
{
	XAxiDma_Bd *CurBdPtr;
	int i;
	u32 BdCr;
	u32 BdSts;
	int RingIndex = RingPtr->RingIndex;

	//if (NumBd < 0) {

	//	xdbg_printf(XDBG_DEBUG_ERROR, "BdRingToHw: negative BD number "
	//		"%d\r\n", NumBd);

	//	return XST_INVALID_PARAM;
	//}

	/* If the commit set is empty, do nothing */
	//if (NumBd == 0) {
	//	return XST_SUCCESS;
	//}

	/* Make sure we are in sync with XAxiDma_BdRingAlloc() */
	//if ((RingPtr->PreCnt < NumBd) || (RingPtr->PreHead != BdSetPtr)) {

	//	xdbg_printf(XDBG_DEBUG_ERROR, "Bd ring has problems\r\n");
	//	return XST_DMA_SG_LIST_ERROR;
	//}

	CurBdPtr = BdSetPtr;
	BdCr = XAxiDma_BdGetCtrl(CurBdPtr);
	BdSts = XAxiDma_BdGetSts(CurBdPtr);

	/* In case of Tx channel, the first BD should have been marked
	 * as start-of-frame
	 */
	//if (!(RingPtr->IsRxChannel) && !(BdCr & XAXIDMA_BD_CTRL_TXSOF_MASK)) {

	//	xdbg_printf(XDBG_DEBUG_ERROR, "Tx first BD does not have "
	//							"SOF\r\n");
	//	return XST_FAILURE;
	//}

	/* Clear the completed status bit
	 */
	//for (i = 0; i < NumBd - 1; i++) {

		/* Make sure the length value in the BD is non-zero. */
		/*if (XAxiDma_BdGetLength(CurBdPtr,
				RingPtr->MaxTransferLen) == 0) {

			xdbg_printf(XDBG_DEBUG_ERROR, "0 length bd\r\n");

			return XST_FAILURE;
		}*/

		//BdSts &=  ~XAXIDMA_BD_STS_COMPLETE_MASK;
		//XAxiDma_BdWrite(CurBdPtr, XAXIDMA_BD_STS_OFFSET, BdSts);
		//XAxiDma_BdWrite(CurBdPtr, XAXIDMA_BD_STS_OFFSET, 0);

		/* Flush the current BD so DMA core could see the updates */
		//XAXIDMA_CACHE_FLUSH(CurBdPtr);

	//	CurBdPtr = (XAxiDma_Bd *)((void *)XAxiDma_BdRingNext(RingPtr, CurBdPtr));
		//BdSts = XAxiDma_BdRead(CurBdPtr, XAXIDMA_BD_STS_OFFSET);
	//}

	CurBdPtr = (XAxiDma_Bd *)((UINTPTR)CurBdPtr + RingPtr->Separation*(UINTPTR)(NumBd-1));
	if((UINTPTR)CurBdPtr > RingPtr->LastBdAddr) {
		CurBdPtr = (XAxiDma_Bd *)((UINTPTR)CurBdPtr -(UINTPTR)(RingPtr)->Length + RingPtr->Separation);
	}

	/*BdCr = XAxiDma_BdRead(CurBdPtr, XAXIDMA_BD_CTRL_LEN_OFFSET);
	// In case of Tx channel, the last BD should have EOF bit set
	if (!(RingPtr->IsRxChannel) && !(BdCr & XAXIDMA_BD_CTRL_TXEOF_MASK)) {

		xdbg_printf(XDBG_DEBUG_ERROR, "Tx last BD does not have "
								"EOF\r\n");

		return XST_FAILURE;
	}*/

	/* Make sure the length value in the last BD is non-zero. */
	//if (XAxiDma_BdGetLength(CurBdPtr,
	//		RingPtr->MaxTransferLen) == 0) {

	//	xdbg_printf(XDBG_DEBUG_ERROR, "0 length bd\r\n");

	//	return XST_FAILURE;
	//}

	/* The last BD should also have the completed status bit cleared
	 */
	BdSts &= ~XAXIDMA_BD_STS_COMPLETE_MASK;
	XAxiDma_BdWrite(CurBdPtr, XAXIDMA_BD_STS_OFFSET, BdSts);

	/* Flush the last BD so DMA core could see the updates */
	XAXIDMA_CACHE_FLUSH(CurBdPtr);
	DATA_SYNC;

	/* This set has completed pre-processing, adjust ring pointers and
	 * counters
	 */
	XAXIDMA_RING_SEEKAHEAD(RingPtr, RingPtr->PreHead, NumBd);
	RingPtr->PreCnt -= NumBd;
	RingPtr->HwTail = CurBdPtr;
	RingPtr->HwCnt += NumBd;

	/* If it is running, signal the engine to begin processing */
	if (RingPtr->RunState == AXIDMA_CHANNEL_NOT_HALTED) {
			if (RingPtr->Cyclic) {
				XAxiDma_WriteReg(RingPtr->ChanBase,
						 XAXIDMA_TDESC_OFFSET,
						 (u32)XAXIDMA_VIRT_TO_PHYS(RingPtr->CyclicBd));
				if (RingPtr->Addr_ext)
					XAxiDma_WriteReg(RingPtr->ChanBase,
							 XAXIDMA_TDESC_MSB_OFFSET,
							 UPPER_32_BITS(XAXIDMA_VIRT_TO_PHYS(RingPtr->CyclicBd)));
				return XST_SUCCESS;
			}

			if (RingPtr->IsRxChannel) {
				if (!RingIndex) {
					XAxiDma_WriteReg(RingPtr->ChanBase,
							XAXIDMA_TDESC_OFFSET, (XAXIDMA_VIRT_TO_PHYS(RingPtr->HwTail) & XAXIDMA_DESC_LSB_MASK));
					if (RingPtr->Addr_ext)
						XAxiDma_WriteReg(RingPtr->ChanBase, XAXIDMA_TDESC_MSB_OFFSET,
								 UPPER_32_BITS(XAXIDMA_VIRT_TO_PHYS(RingPtr->HwTail)));
				}
				else {
					XAxiDma_WriteReg(RingPtr->ChanBase,
						(XAXIDMA_RX_TDESC0_OFFSET +
						(RingIndex - 1) * XAXIDMA_RX_NDESC_OFFSET),
						(XAXIDMA_VIRT_TO_PHYS(RingPtr->HwTail) & XAXIDMA_DESC_LSB_MASK ));
					if (RingPtr->Addr_ext)
						XAxiDma_WriteReg(RingPtr->ChanBase,
							(XAXIDMA_RX_TDESC0_MSB_OFFSET +
							(RingIndex - 1) * XAXIDMA_RX_NDESC_OFFSET),
							UPPER_32_BITS(XAXIDMA_VIRT_TO_PHYS(RingPtr->HwTail)));
				}
			}
			else {
				XAxiDma_WriteReg(RingPtr->ChanBase,
							XAXIDMA_TDESC_OFFSET, (XAXIDMA_VIRT_TO_PHYS(RingPtr->HwTail) & XAXIDMA_DESC_LSB_MASK));
				if (RingPtr->Addr_ext)
					XAxiDma_WriteReg(RingPtr->ChanBase, XAXIDMA_TDESC_MSB_OFFSET,
								UPPER_32_BITS(XAXIDMA_VIRT_TO_PHYS(RingPtr->HwTail)));
			}
	}

	return XST_SUCCESS;
}


