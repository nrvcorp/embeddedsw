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



/************************** Constant Definitions *****************************/
#define DEBUG_FEATURES 1


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

/**************************** Type Definitions *******************************/

/************************** Function Prototypes ******************************/


/************************** Variable Definitions *****************************/

XScuGic Intc;

u8 TxRestartColorbar; /* TX restart flag is set when the TX cable has
                       * been reconnected and the TX colorbar was showing. */
XMipi_Menu xMipiMenu; // Menu structure

XPipeline_Cfg Pipeline_Cfg;
XPipeline_Cfg New_Cfg;


#define DEBUG_COUNT 7
u8 dbg_idx = 0;

int FramesTx;		// Num of frames that have been sent


uint8_t StartTx;

volatile int accumTime[DEBUG_COUNT] = { 0 };
volatile int accumCntr[DEBUG_COUNT] = { 0 };



u8 RxStatusFlag;
u8 TxTriggeredFlag;
u8 IsInTxCriticalSection = 0;

#if DEBUG_FEATURES
u32 NumOfTxFrames = 0;
#endif

int Err_Cnt = 0;
/************************** Function Definitions *****************************/

extern void config_csi_cap_path();


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
	u32 *RxBuf;
	u8 TxBusy;
	u8 BDInitCnt = 0;
	u32 CoalCnt;
	u32 LastBdSts;
	u32 BdSts;

	TxBusy = 0;

#if DEBUG_FEATURES
	xil_printf("Addr of NumOfTxFrames = %x\r\n",&NumOfTxFrames);
#endif

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
 		// 2. setup interrupt system
		Status = SetupIicPsInterruptSystem();
		if(Status != XST_SUCCESS){
			return XST_FAILURE;
		}


		/*
		 * 3. Setup the handlers for the IIC that will be called from the
		 * interrupt context when data has been sent and received, specify a
		 * pointer to the IIC driver instance as the callback reference so
		 * the handlers are able to access the instance data.
		 */
		XIicPs_SetStatusHandler(&IicPsInstance, (void *) &IicPsInstance, Handler);

		/*
		 * 4. Set the IIC serial clock rate.
		 */
		XIicPs_SetSClk(&IicPsInstance, IIC_SCLK_RATE);
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
//		sleep(1);
//		CsiRxPrintRegStatus();
	} while (1);


	return 0;
}
