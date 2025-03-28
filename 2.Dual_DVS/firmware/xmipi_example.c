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

/***************** Macros (Inline Functions) Definitions *********************/
#define UART_BASEADDR	XPAR_XUARTPS_0_BASEADDR

#define IIC_SENSOR_INTR_ID	XPAR_FABRIC_AXI_IIC_0_IIC2INTC_IRPT_INTR

#define GPIO_SENSOR		XPAR_AXI_GPIO_0_BASEADDR

#ifdef XPAR_PSU_ACPU_GIC_DEVICE_ID
#define PSU_INTR_DEVICE_ID	XPAR_PSU_ACPU_GIC_DEVICE_ID
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

/************************** Function Definitions *****************************/
void DMARxIntrHandler(void *Callback)
{
//		xil_printf("DMA Interrupt Handler!!");
	DVSCamera *DVSCamera_t = (DVSCamera *)Callback;
	XAxiDma_BdRing *RxRingPtr = XAxiDma_GetRxRing(&DVSCamera_t->axi_dma);

	u32 IrqStatus;
	int TimeOut;

	/* Read pending interrupts */
	IrqStatus = XAxiDma_BdRingGetIrq(RxRingPtr);

	/* Acknowledge pending interrupts */
	XAxiDma_BdRingAckIrq(RxRingPtr, IrqStatus);

	/*
	 * If no interrupt is asserted, we do not do anything
	 */
	if (!(IrqStatus & XAXIDMA_IRQ_ALL_MASK))
	{
		return;
	}

	/*
	 * If error interrupt is asserted, raise error flag, reset the
	 * hardware to recover from the error, and return with no further
	 * processing.
	 */
	if ((IrqStatus & XAXIDMA_IRQ_ERROR_MASK))
	{

		XAxiDma_BdRingDumpRegs(RxRingPtr);

		/* Reset could fail and hang
		 * NEED a way to handle this or do not call it??
		 */
		XAxiDma_Reset(&DVSCamera_t->axi_dma);

		TimeOut = RESET_TIMEOUT_COUNTER;

		while (TimeOut)
		{
			if (XAxiDma_ResetIsDone(&DVSCamera_t->axi_dma))
			{
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
	if ((IrqStatus & (XAXIDMA_IRQ_IOC_MASK)))
	{
		DVSDmaWriteDoneCallback(DVSCamera_t);
	}
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
int SetupInterruptSystem(void)
{
	int Status;

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


	for (int cam_id = 0; cam_id < DVS_NUM; ++cam_id) {
		XScuGic_SetPriorityTriggerType(IntcInstPtr, dvs_cameras[cam_id].axi_dma_intr_id, 0xA8, 0x3);
	}

	/*
	 * Connect the device driver handler that will be called when an
	 * interrupt for the device occurs, the handler defined above performs
	 * the specific interrupt processing for the device.
	 */
	Status = XScuGic_Connect(IntcInstPtr, XPAR_XIICPS_1_INTR,
							 (Xil_InterruptHandler)XIicPs_MasterInterruptHandler,
							 (void *)&IicPsInstance);
	if (Status != XST_SUCCESS) {
		return Status;
	}
	for (int cam_id = 0; cam_id < DVS_NUM; ++cam_id) {
		Status = XScuGic_Connect(IntcInstPtr, dvs_cameras[cam_id].axi_dma_intr_id,
								 (Xil_InterruptHandler)DMARxIntrHandler,
								 &dvs_cameras[cam_id]);
		if (Status != XST_SUCCESS) {
			return Status;
		}
	}

	/* Enable IO expander and sensor IIC interrupts */
	XScuGic_Enable(IntcInstPtr, XPAR_XIICPS_1_INTR);
	for (int cam_id = 0; cam_id < DVS_NUM; ++cam_id) {
		XScuGic_Enable(IntcInstPtr, dvs_cameras[cam_id].axi_dma_intr_id);
	}

	Xil_ExceptionInit();

	/*Register the interrupt controller handler with the exception table.*/
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
								 (Xil_ExceptionHandler)XScuGic_InterruptHandler,
								 (XScuGic *)IntcInstPtr);

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
	initializeDVSCameras();

	/* Initialize IIC */
	Status = InitPsIIC();
	if (Status != XST_SUCCESS) {
		xil_printf(TXT_RED "\n\r PS IIC Init Failed \n\r" TXT_RST);
		return XST_FAILURE;
	}


	/* Turn on DVS Sensor */
	for (int cam_id = 0; cam_id < DVS_NUM; ++cam_id) {
		Status = StartDVSSensor(dvs_cameras[cam_id]);
		if (Status != XST_SUCCESS) {
			xil_printf(TXT_RED "START DVS(%d) error.\n\r" TXT_RST, cam_id);
			return XST_FAILURE;
		} else {
			xil_printf(TXT_GREEN "START DVS(%d) SUCCEEDED.\n\r" TXT_RST, cam_id);
		}

		config_dvs_cap_path(&dvs_cameras[cam_id]);
	}

	/* Initialize IRQ */
	Status = SetupInterruptSystem();
	if (Status == XST_FAILURE)
	{
		xil_printf(TXT_RED "IRQ init failed. \r\n" TXT_RST);
		return XST_FAILURE;
	}


	print("---------------------------------\r\n");
	/* Enable exceptions. */
	Xil_AssertSetCallback((Xil_AssertCallback) Xil_AssertCallbackRoutine);
	Xil_ExceptionEnable();
	//================ DVS sensor program ===================//
	for (int cam_id = 0; cam_id < DVS_NUM; ++cam_id) {
		start_dvs_cap_pipe(&dvs_cameras[cam_id]);
		Status = ProgramDVSSensor(dvs_cameras[cam_id]);
		if (Status != XST_SUCCESS) {
			xil_printf(TXT_RED "PROGRAM DVS(%d) error.\n\r" TXT_RST, cam_id);
			return XST_FAILURE;
		} else {
			xil_printf(TXT_GREEN "PROGRAM DVS(%d) SUCCEEDED.\n\r" TXT_RST, cam_id);
		}
	}

	/* Main loop */
	do {
		sleep(1);
		for (int cam_id = 0; cam_id < DVS_NUM; cam_id++){
			xil_printf("DVS_Camera(%d) Frame_cnt: %d  ", cam_id, dvs_cameras[cam_id].frm_cnt);
		}
		xil_printf("\r\n");
	} while (1);


	return 0;
}
