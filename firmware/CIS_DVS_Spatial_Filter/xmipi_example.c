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
#include "DVS_Spatial_Filter.h"

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


#define DVS_SPATIAL_FILTER_BASEADDR	XPAR_DVS_STREAM_DVS_SPATIAL_FILTER_0_AXI_LITE_BASEADDR
/***************** Macros (Inline Functions) Definitions *********************/
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

#ifdef XPAR_PSU_RCPU_GIC_DEVICE_ID
#define PSU_INTR_DEVICE_ID	XPAR_PSU_RCPU_GIC_DEVICE_ID
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

XScuGic Intc;

u8 IsPassThrough; /**< Demo mode 0-colorbar 1-pass through */
u8 StartTxAfterRxFlag;
u8 TxBusy;         /* TX busy flag is set while the TX is initialized */
u8 TxRestartColorbar; 	/* TX restart flag is set when the TX cable has
			 * been reconnected and the TX colorbar was showing.
			 */
u32 Index;

XPipeline_Cfg Pipeline_Cfg;
XPipeline_Cfg New_Cfg;


extern XV_FrmbufWr_l2	frmbufwr;
extern XAxiDma			DVSDma;

extern XIic IicSensor; /* The instance of the IIC device. */

extern XVprocSs scaler_new_inst;

extern u32 frm_cnt;
extern u32 dvs_frm_cnt;

/************************** Function Definitions *****************************/
extern void config_csi_cap_path();
extern void config_dvs_cap_path();

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

	XAxiDma_BdRing *RxRingPtr = XAxiDma_GetRxRing(&DVSDma);
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

	XScuGic_SetPriorityTriggerType(IntcInstPtr, XPAR_FABRIC_AXIDMA_0_VEC_ID, 0xA8, 0x3);

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


	/*
	 * Connect the device driver handler that will be called when an
	 * interrupt for the device occurs, the handler defined above performs
	 * the specific interrupt processing for the device.
	 */
	Status = XScuGic_Connect(IntcInstPtr, XPAR_XIICPS_1_INTR,
			(Xil_InterruptHandler)XIicPs_MasterInterruptHandler,
			(void *) &IicPsInstance);
	if (Status != XST_SUCCESS) {
		return Status;
	}
	Status = XScuGic_Connect(IntcInstPtr, XPAR_FABRIC_AXIDMA_0_VEC_ID,
			(Xil_InterruptHandler) DMARxIntrHandler,
			RxRingPtr);
	if (Status != XST_SUCCESS) {
		return Status;
	}


	/* Enable IO expander and sensor IIC interrupts */
	XScuGic_Enable(IntcInstPtr, IIC_SENSOR_INTR_ID);
	XScuGic_Enable(IntcInstPtr, XPAR_INTC_0_V_FRMBUF_WR_0_VEC_ID);
	XScuGic_Enable(IntcInstPtr, XPAR_XIICPS_1_INTR);
	XScuGic_Enable(IntcInstPtr, XPAR_FABRIC_AXIDMA_0_VEC_ID);

	Xil_ExceptionInit();

	/*Register the interrupt controller handler with the exception table.*/
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
//	XVphy_Config *XVphyCfgPtr;
//	XVidC_VideoStream *HdmiTxSsVidStreamPtr;

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
	Pipeline_Cfg.VideoMode = XVIDC_VM_1920x1080_60_P;

	configure_buffer_system();

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

	/*************** Program & Run DVS Spatial Filter **************/
	DVS_SPATIAL_FILTER_mWriteReg(DVS_SPATIAL_FILTER_BASEADDR,
					DVS_SPATIAL_FILTER_AXI_Lite_SLV_REG1_OFFSET,2);
	DVS_SPATIAL_FILTER_mWriteReg(DVS_SPATIAL_FILTER_BASEADDR,
						DVS_SPATIAL_FILTER_AXI_Lite_SLV_REG2_OFFSET,2);
	DVS_SPATIAL_FILTER_mWriteReg(DVS_SPATIAL_FILTER_BASEADDR,
							DVS_SPATIAL_FILTER_AXI_Lite_SLV_REG0_OFFSET,1);
	/***************************************************************/

	/*************** Program DVS sensor **************/
	// frame header setting
	MIPI_CONTROLLER_mWriteReg(MIPI_CONTROLLER_BASEADDR,
				MIPI_CONTROLLER_S00_AXI_SLV_REG0_OFFSET,
				1);
	MIPI_CONTROLLER_mWriteReg(MIPI_CONTROLLER_BASEADDR,
				MIPI_CONTROLLER_S00_AXI_SLV_REG1_OFFSET,
				1);
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

	New_Cfg = Pipeline_Cfg;

	/* Print the Pipe line configuration */
	PrintPipeConfig();


	int OnTh, OffTh;
	OnTh = 0;
	OffTh = 0;
	/* Main loop */
	do {
//		sleep(1);
//		CsiRxPrintRegStatus();
		sleep(2);
		xil_printf("CIS frame count: %d, DVS frame count:  %d\r\n",frm_cnt, dvs_frm_cnt);



		DVS_SPATIAL_FILTER_mWriteReg(DVS_SPATIAL_FILTER_BASEADDR,
						DVS_SPATIAL_FILTER_AXI_Lite_SLV_REG1_OFFSET,OnTh);
		DVS_SPATIAL_FILTER_mWriteReg(DVS_SPATIAL_FILTER_BASEADDR,
						DVS_SPATIAL_FILTER_AXI_Lite_SLV_REG2_OFFSET,OffTh);

		if(OnTh==0) OnTh = 2;
		else OnTh = 0;
		if(OffTh==0) OffTh = 2;
		else OffTh = 0;

	} while (1);

	return 0;
}
