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
#include "sensor_cfg_input.h"
#include "sensor_cfg_types.h"

#include "xmipi_menu.h"				//modified

#include "pipeline_program.h"
#include "xv_frmbufwr_l2.h"
#include "xiicps.h"
#include "xaxidma.h"


#include "xil_printf.h"
#include "dma_example.h"
#include "DVS_Adaptive_Filter.h"
#include "axi4_read_dma.h"
#include "axi4_write_dma.h"

#include "isr_interval_monitor.h"
#include "debug_config.h"
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

#define	DMA_FIL_TX_IRPT_INTR		XPAR_FABRIC_AXI4_READ_DMA_0_IRQ_DMA_DONE_INTR
#define	DMA_FIL_RX_IRPT_INTR		XPAR_FABRIC_AXI4_WRITE_DMA_0_IRQ_DMA_DONE_INTR


//Memory Address Map
//#define DDR_BASE_ADDR	 XPAR_PSU_DDR_0_S_AXI_BASEADDR
/*
 * pipeline_program.h
 */

/***************** Macros (Inline Functions) Definitions *********************/


/**********************************************************/
/**********************************************************/




#define LOOPBACK_MODE_EN 0
#define XPAR_CPU_CORE_CLOCK_FREQ_HZ 100000000
#define UART_BASEADDR XPAR_XUARTPS_0_BASEADDR
#define I2C_MUX_ADDR 0x74 /**< I2C Mux Address */
#define I2C_CLK_ADDR 0x68 /**< I2C Clk Address */

#define IIC_SENSOR_INTR_ID XPAR_FABRIC_SENSOR_CTRL_AXI_IIC_0_IIC2INTC_IRPT_INTR

#define GPIO_TPG_RESET_DEVICE_ID XPAR_GPIO_3_DEVICE_ID

#define GPIO_SENSOR XPAR_SENSOR_CTRL_AXI_GPIO_0_BASEADDR
#define GPIO_IP_RESET1 XPAR_CIS_STREAM_AXI_GPIO_RST_BASEADDR
#define GPIO_IP_RESET2 XPAR_CIS_STREAM_AXI_GPIO_0_BASEADDR

#ifdef XPAR_PSU_ACPU_GIC_DEVICE_ID
#define PSU_INTR_DEVICE_ID XPAR_PSU_ACPU_GIC_DEVICE_ID
#endif

#ifdef XPAR_PSU_RCPU_GIC_DEVICE_ID
#define PSU_INTR_DEVICE_ID XPAR_PSU_RCPU_GIC_DEVICE_ID
#endif

#define XPAR_INTC_0_V_FRMBUF_WR_0_VEC_ID XPAR_FABRIC_V_FRMBUF_WR_0_VEC_ID

#define RESET_TIMEOUT_COUNTER 10000

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
XScuGic Intc;

#if(ENABLE_DVS_FILTER)
int *FilTxPtr;
int *FilRxPtr;

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
#endif


u8 IsPassThrough; /**< Demo mode 0-colorbar 1-pass through */
u8 StartTxAfterRxFlag;
u8 TxBusy;			  /* TX busy flag is set while the TX is initialized */
u8 TxRestartColorbar; /* TX restart flag is set when the TX cable has
			 * been reconnected and the TX colorbar was showing.
			 */
u32 Index;

XPipeline_Cfg Pipeline_Cfg;
XPipeline_Cfg New_Cfg;

#if	(ENABLE_MENU && AUTO_TEST_MODE==0)
XMipi_Menu XmipiMenu;
extern u8 is_user_input_active;
#endif

#if(ENABLE_DVS_RESET)
u8 dvs_reset=0;
#endif

extern XV_FrmbufWr_l2 frmbufwr;
extern XAxiDma DVSDma;

extern XIic IicSensor; /* The instance of the IIC device. */

extern XVprocSs scaler_new_inst;

extern u32 frm_cnt;
extern u32 dvs_frm_cnt;
extern u32 wr_ptr;
extern u32 dvs_wr_ptr;

#if(ENABLE_DVS_FILTER)
u32 fil_frm_cnt;  // culc fps


extern u64 dvs_frame_array[DVS_BUFFER_NUM];
extern u64 dvs_frame_rdy[DVS_BUFFER_NUM];

volatile u64 fil_frame_rdy[FIL_BUFFER_NUM];
volatile u64 fil_frame_array[FIL_BUFFER_NUM];
#endif

/************************** Function Definitions *****************************/
extern void config_csi_cap_path();
extern int config_dvs_cap_path();

XTime debug_t;
int t_count=0;


volatile u32 slv_reg2=0, slv_reg3=0, slv_reg4=0, slv_reg5=0;
u32 slv_reg2_old=0, slv_reg3_old=0, slv_reg4_old=0, slv_reg5_old=0;


#if(CHECK_FILTER_DMA_READ_WRITE_SYNC)
u8 prev_isr = 1;
#endif

#if(CHECK_DVS_BUFFER_HOST_DRAIN)
extern int host_delay_count_dvs;
#endif
#if(CHECK_FIL_BUFFER_HOST_DRAIN)
int host_delay_count_filter=0;
#endif

#if(CHECK_DVS_FRAME_DROP)
extern u8 frame_drop_check_init;
	#if(!DVS_FRAME_DROP_LOG_IMMEDIATE)
extern int frame_drop_count_dvs;
		#if(CHECK_DVS_MULTIPLE_FRAME_DROP)
extern int multiple_frame_drop_count_dvs;
		#endif
	#endif
#endif

#if(USE_EXTENDED_DVS_FRAME_HEADER)
volatile u64 test_header=0;
#endif

#if (CHECK_FIL_EVENT_COUNT && ENABLE_DVS_FILTER)
u32 event_cnt_sum[2] = {0,0};
u32 event_cnt_min[2] = {~(u32)0,~(u32)0};
u32 event_cnt_max[2] = {0,0};
u32 event_cnt_sample_num[2] = {0,0};

#define EVENT_COUNT_WINDOW_SIZE 16
u32 event_cnt_win_fifo[EVENT_COUNT_WINDOW_SIZE] = {0,}; // do not need reset
// event_cnt_win_fifo_idx = event_cnt_sample_num % EVENT_COUNT_WINDOW_SIZE
u32 event_cnt_win_sum[2] = {0,0};  // do not need reset
u32 event_cnt_win_sum_min[2] = {0,0};  // do not need reset

u8 event_cnt_double_buf_idx = 0;
void ReadEventCountAndRecord(void)
{
	u32 cur_event_cnt = DVS_ADAPTIVE_FILTER_mReadReg(XPAR_DVS_ADAPTIVE_FILTER_0_AXI_LITE_BASEADDR, DVS_ADAPTIVE_FILTER_AXI_Lite_SLV_REG2_OFFSET);

	event_cnt_sum[event_cnt_double_buf_idx] += cur_event_cnt;
	if(cur_event_cnt < event_cnt_min[event_cnt_double_buf_idx]) event_cnt_min[event_cnt_double_buf_idx] = cur_event_cnt;
	else if(cur_event_cnt > event_cnt_max[event_cnt_double_buf_idx]) event_cnt_max[event_cnt_double_buf_idx] = cur_event_cnt;

	if(event_cnt_sample_num[event_cnt_double_buf_idx] == (EVENT_COUNT_WINDOW_SIZE-1)){
		event_cnt_win_sum[event_cnt_double_buf_idx] = event_cnt_sum[event_cnt_double_buf_idx];
		event_cnt_win_sum_min[event_cnt_double_buf_idx] = event_cnt_sum[event_cnt_double_buf_idx];
	}
	else if(event_cnt_sample_num[event_cnt_double_buf_idx] > (EVENT_COUNT_WINDOW_SIZE-1)){
		event_cnt_win_sum[event_cnt_double_buf_idx] -= event_cnt_win_fifo[event_cnt_sample_num[event_cnt_double_buf_idx]%EVENT_COUNT_WINDOW_SIZE];
		event_cnt_win_sum[event_cnt_double_buf_idx] += cur_event_cnt;
		if(event_cnt_win_sum[event_cnt_double_buf_idx] < event_cnt_win_sum_min[event_cnt_double_buf_idx])
		{
			event_cnt_win_sum_min[event_cnt_double_buf_idx] = event_cnt_win_sum[event_cnt_double_buf_idx];
		}
	}
	event_cnt_win_fifo[event_cnt_sample_num[event_cnt_double_buf_idx]%EVENT_COUNT_WINDOW_SIZE] = cur_event_cnt;

	event_cnt_sample_num[event_cnt_double_buf_idx]++;
}



void ShowEventCountAndReset(void)
{
	event_cnt_double_buf_idx = !event_cnt_double_buf_idx;
	xil_printf("------sample num: %d-------\r\n", event_cnt_sample_num[!event_cnt_double_buf_idx]);
	xil_printf("event count mean: %d\r\n", event_cnt_sum[!event_cnt_double_buf_idx] / event_cnt_sample_num[!event_cnt_double_buf_idx] );
	xil_printf("event count min: %d\r\n", event_cnt_min[!event_cnt_double_buf_idx]);
	xil_printf("event count max: %d\r\n", event_cnt_max[!event_cnt_double_buf_idx]);
	xil_printf("event count %d sum min: %d\r\n", EVENT_COUNT_WINDOW_SIZE, event_cnt_win_sum_min[!event_cnt_double_buf_idx]);

	event_cnt_sum[!event_cnt_double_buf_idx] = 0;
	event_cnt_sample_num[!event_cnt_double_buf_idx] = 0;
	event_cnt_min[!event_cnt_double_buf_idx] = ~(u32)0;
	event_cnt_max[!event_cnt_double_buf_idx] = 0;
}
#endif


void DMARxIntrHandler(void *Callback)
{
//	xil_printf("DMA Interrupt Handler!!");
	XAxiDma_BdRing *RxRingPtr = (XAxiDma_BdRing *)Callback;

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
		XAxiDma_Reset(&DVSDma);

		TimeOut = RESET_TIMEOUT_COUNTER;

		while (TimeOut)
		{
			if (XAxiDma_ResetIsDone(&DVSDma))
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
int I2cMux(void)
{
	u8 Buffer;
	int Status;

	/* Select SI5324 clock generator */
	Buffer = 0x80;
	Status = XIic_Send((XPAR_IIC_0_BASEADDR), (I2C_MUX_ADDR),
					   (u8 *)&Buffer, 1, (XIIC_STOP));

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
int I2cClk(u32 InFreq, u32 OutFreq)
{
	int Status;

	/* Free running mode */
	if (InFreq == 0)
	{

		Status = Si5324_SetClock((XPAR_IIC_0_BASEADDR),
						(I2C_CLK_ADDR),
						(SI5324_CLKSRC_XTAL),
						(SI5324_XTAL_FREQ),
						OutFreq);

		if (Status != (SI5324_SUCCESS))
		{
			print("Error programming free mode SI5324\n\r");
			return 0;
		}
	}

	/* Locked mode */
	else
	{
		Status = Si5324_SetClock((XPAR_IIC_0_BASEADDR),
						(I2C_CLK_ADDR),
						(SI5324_CLKSRC_CLK1),
						InFreq,
						OutFreq);

		if (Status != (SI5324_SUCCESS))
		{
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
int SetupInterruptSystem(void)
{

	XAxiDma_BdRing *RxRingPtr = XAxiDma_GetRxRing(&DVSDma);
	int Status;

	XScuGic *IntcInstPtr = &Intc;

	/*
	 * Initialize the interrupt controller driver so that it's ready to
	 * use, specify the device ID that was generated in xparameters.h
	 */

	XScuGic_Config *IntcCfgPtr;
	IntcCfgPtr = XScuGic_LookupConfig(PSU_INTR_DEVICE_ID);
	if (IntcCfgPtr == NULL)
	{
		print("ERR:: Interrupt Controller not found");
		return (XST_DEVICE_NOT_FOUND);
	}
	Status = XScuGic_CfgInitialize(IntcInstPtr, IntcCfgPtr,
			IntcCfgPtr->CpuBaseAddress);

	if (Status != XST_SUCCESS)
	{
		xil_printf("Intc initialization failed!\r\n");
		return XST_FAILURE;
	}

	XScuGic_SetPriorityTriggerType(IntcInstPtr, XPAR_FABRIC_AXIDMA_0_VEC_ID, 0xA8, 0x3);

#if(ENABLE_DVS_FILTER)
	XScuGic_SetPriorityTriggerType(IntcInstPtr, DMA_FIL_TX_IRPT_INTR, 0xA8, 0x3);
	XScuGic_SetPriorityTriggerType(IntcInstPtr, DMA_FIL_RX_IRPT_INTR, 0xA8, 0x3);
#endif
	XScuGic_SetPriorityTriggerType(IntcInstPtr, XPAR_INTC_0_V_FRMBUF_WR_0_VEC_ID, 0xA8, 0x3);

	/*
	 * Start the interrupt controller such that interrupts are recognized
	 * and handled by the processor
	 */

	Status = XScuGic_Connect(IntcInstPtr, IIC_SENSOR_INTR_ID,
							 (XInterruptHandler)XIic_InterruptHandler,
							 (void *)&IicSensor);
	if (Status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	Status = XScuGic_Connect(IntcInstPtr, XPAR_INTC_0_V_FRMBUF_WR_0_VEC_ID,
							 (XInterruptHandler)XVFrmbufWr_InterruptHandler,
							 (void *)&frmbufwr);
	if (Status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

#if(ENABLE_DVS_FILTER)
	Status = XScuGic_Connect(IntcInstPtr, DMA_FIL_TX_IRPT_INTR,
						(Xil_InterruptHandler) DmaFILTxIntrHandler, FilTxPtr);
	if (Status != XST_SUCCESS) return Status;
	Status = XScuGic_Connect(IntcInstPtr, DMA_FIL_RX_IRPT_INTR,
						(Xil_InterruptHandler) DmaFILRxIntrHandler, FilRxPtr);
	if (Status != XST_SUCCESS) return Status;
#endif

	/*
	 * Connect the device driver handler that will be called when an
	 * interrupt for the device occurs, the handler defined above performs
	 * the specific interrupt processing for the device.
	 */
	Status = XScuGic_Connect(IntcInstPtr, XPAR_XIICPS_1_INTR,
			(Xil_InterruptHandler)XIicPs_MasterInterruptHandler,
			(void *)&IicPsInstance);
	if (Status != XST_SUCCESS)
	{
		return Status;
	}
	Status = XScuGic_Connect(IntcInstPtr, XPAR_FABRIC_AXIDMA_0_VEC_ID,
							 (Xil_InterruptHandler)DMARxIntrHandler,
							 RxRingPtr);
	if (Status != XST_SUCCESS)
	{
		return Status;
	}


	/* Enable IO expander and sensor IIC interrupts */
	XScuGic_Enable(IntcInstPtr, IIC_SENSOR_INTR_ID);
	XScuGic_Enable(IntcInstPtr, XPAR_INTC_0_V_FRMBUF_WR_0_VEC_ID);
#if(ENABLE_DVS_FILTER)
	XScuGic_Enable(IntcInstPtr, DMA_FIL_TX_IRPT_INTR);
	XScuGic_Enable(IntcInstPtr, DMA_FIL_RX_IRPT_INTR);
#endif
	XScuGic_Enable(IntcInstPtr, XPAR_XIICPS_1_INTR);
	XScuGic_Enable(IntcInstPtr, XPAR_FABRIC_AXIDMA_0_VEC_ID);

	Xil_ExceptionInit();

	/*Register the interrupt controller handler with the exception table.*/
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
								 (Xil_ExceptionHandler)XScuGic_InterruptHandler,
								 (XScuGic *)IntcInstPtr);

	//Xil_ExceptionEnable();

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
void Xil_AssertCallbackRoutine(u8 *File, s32 Line)
{
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

	Xil_Out32(GPIO_IP_RESET1, 0x00);
	Xil_Out32(GPIO_IP_RESET2, 0x00);
	usleep(1000);
	Xil_Out32(GPIO_IP_RESET1, 0x01);
	Xil_Out32(GPIO_IP_RESET2, 0x01);
}


#if(CHECK_FILTER_REG_VALUES)
u32 popcount(u32 x) {
    x = x - ((x >> 1) & 0x55555555);
    x = (x & 0x33333333) + ((x >> 2) & 0x33333333);
    x = (x + (x >> 4)) & 0x0F0F0F0F;
    x = x + (x >> 8);
    x = x + (x >> 16);
    return x & 0x3F;
}
#endif

int EditSensorConfig(void)
{
	int Status;

	regval_list sensor_cfg[SENSOR_CFG_WORKBUF_CAP];
	size_t sensor_cfg_len = 0;

	Status = sensor_cfg_input(DVS_regs, length_DVS_regs,
							  sensor_cfg, ARRAY_LEN(sensor_cfg), &sensor_cfg_len);
	if (Status == SENSOR_CFG_OK)
	{
		// Now you have a finalized array (sensor_cfg[0..sensor_cfg_len))
		// Decide elsewhere whether to call your programming function.
		// e.g., enqueue for another tool:
		// enqueue_cfg_for_programming(sensor_cfg, sensor_cfg_len);
		xil_printf("Programming sensor with the updated configuration.\r\n");
		Status = ProgramDVSSensor(sensor_cfg, sensor_cfg_len);
	}
	else if (Status == SENSOR_CFG_ABORTED)
	{
		// User canceled; keep previous config or do nothing.
		xil_printf("User aborted; programming sensor with the default configuration.\r\n");
		Status = ProgramDVSSensor(DVS_regs, length_DVS_regs);
	}
	else if (Status == SENSOR_CFG_ERR_CAP)
	{
		// Buffer too small; consider increasing SENSOR_CFG_WORKBUF_CAP.
		xil_printf("Overrides exceed capacity; programming sensor with the default configuration.\r\n");
		Status = ProgramDVSSensor(DVS_regs, length_DVS_regs);
	}
	else
	{
		// Parse/other error; handle as needed.
		xil_printf("Unexpected error; programming sensor with the default configuration.\r\n");
		Status = ProgramDVSSensor(DVS_regs, length_DVS_regs);
	}

	return Status;
}
int SelectSensorConfigPreset(void)
{
	int Status=999;
	u8 Response;
	do{
		xil_printf("Select a preset.\r\n");
		xil_printf("[1] DVS_regs_1958fps \r\n");
		xil_printf("[2] DVS_regs_3287fps \r\n");
		xil_printf("[C] Cancel and Return \r\n");

		Response = XUartPs_RecvByte(UART_BASEADDR);
		XUartPs_SendByte(UART_BASEADDR, Response);
		xil_printf("\r\n");
		if (Response == '1')
		{
			xil_printf("Programming sensor with 1958fps configuration.\r\n");
			Status = ProgramDVSSensor(DVS_regs_1958fps, length_DVS_regs_1958fps);
			return Status;
		}
		else if (Response == '2')
		{
			xil_printf("Programming sensor with 3287fps configuration.\r\n");
			Status = ProgramDVSSensor(DVS_regs_3287fps, length_DVS_regs_3287fps);
			return Status;
		}
		else if ((Response == 'C')||(Response == 'c'))
		{
			xil_printf("Canceled. \r\n");
			return Status;
		}
		else
		{
			xil_printf("Unknown commend.\r\n");
			continue;
		}
	}while(1);

	xil_printf("Canceled. \r\n");
	return Status;
}

int setup_all(void)
{
	u8 Response;
	u32 Status;
	static int cfg_num = 0;

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
	//Pipeline_Cfg.CameraPresent = TRUE; // set later

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

#if(AUTO_TEST_MODE==0)
	do
	{
		xil_printf("Is the camera sensor connected? (Y/N)\r\n");

		Response = XUartPs_RecvByte(UART_BASEADDR);
		XUartPs_SendByte(UART_BASEADDR, Response);

		if ((Response == 'Y') || (Response == 'y'))
		{
			Pipeline_Cfg.CameraPresent = TRUE;
			break;
		}
		else if ((Response == 'N') || (Response == 'n'))
		{
			Pipeline_Cfg.CameraPresent = FALSE;
			break;
		}
	} while (1);\

#else
	Pipeline_Cfg.CameraPresent = TRUE;
#endif


	if (Pipeline_Cfg.CameraPresent)
		print(TXT_GREEN);
	else
		print(TXT_RED);

	xil_printf("\r\nCamera sensor is set as %s\r\n",
			(Pipeline_Cfg.CameraPresent) ? "Connected" : "Disconnected");
	print(TXT_RST);

	if (!Pipeline_Cfg.CameraPresent)
	{
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
	if (Status != XST_SUCCESS)
	{
		xil_printf(TXT_RED "\n\rIIC Init Failed \n\r" TXT_RST);
		return XST_FAILURE;
	}

	Status = InitDVSIIC();
	if (Status != XST_SUCCESS)
	{
		xil_printf(TXT_RED "\n\rDVS IIC Init Failed \n\r" TXT_RST);
		return XST_FAILURE;
	}
	else
	{
		print(TXT_GREEN "IIC init SUCCEEDED.\n\r" TXT_RST);
	}


#if(ENABLE_DVS_FILTER)
		//Filter Setup & Run
		// Reg1: Set Threshold  ==>> [31:16]OffTh,  [15:0]OnTh
		// Reg0: Run Filter ==>> 1: Run
		//u32 Threshold = 0b00000100000010;
	u32 Threshold = 0b0;
		//u32 Threshold = 0b11111111111111;
	DVS_ADAPTIVE_FILTER_mWriteReg(XPAR_DVS_ADAPTIVE_FILTER_0_AXI_LITE_BASEADDR, DVS_ADAPTIVE_FILTER_AXI_Lite_SLV_REG1_OFFSET, Threshold);
	DVS_ADAPTIVE_FILTER_mWriteReg(XPAR_DVS_ADAPTIVE_FILTER_0_AXI_LITE_BASEADDR, DVS_ADAPTIVE_FILTER_AXI_Lite_SLV_REG0_OFFSET, 1);

	FrameBufferPointerInit();
	FilRxInit();
	FilTxInit();
	g_FILTxRunning = 0;
	g_FILRxRunning = 1;

	FilRxSetAndRun();

	fil_frm_cnt = 0;
#endif
	dvs_frm_cnt = 0;
	dvs_wr_ptr = 0;

	frm_cnt = 0; // cis
	wr_ptr = 0; // cis


	/* Turn on DVS Sensor */
	Status = StartDVSSensor();
	if (Status != XST_SUCCESS)
	{
		print(TXT_RED "START DVS error.\n\r" TXT_RST);
		return XST_FAILURE;
	}
	else
	{
		print(TXT_GREEN "START DVS SUCCEEDED.\n\r" TXT_RST);
	}

	/* Initialize IRQ */
	Status = SetupInterruptSystem();
	if (Status == XST_FAILURE)
	{
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
	if (Status != XST_SUCCESS)
	{
		xil_printf(TXT_RED "CSI Rx Ss Init failed status = %x.\r\n" TXT_RST, Status);
		return XST_FAILURE;
	}

	Status = InitializeDphy();
	if (Status != XST_SUCCESS)
	{
		xil_printf(TXT_RED "DVS DPHY Init failed status = %x.\r\n" TXT_RST, Status);
		return XST_FAILURE;
	}
	else
	{
		xil_printf(TXT_GREEN "DVS DPHY Init Success\r\n" TXT_RST);
	}
	config_csi_cap_path();

	/* MIPI colour depth in bits per clock */
	SetColorDepth();



	Status = config_dvs_cap_path();
	if(Status != XST_SUCCESS)
	{
		xil_printf("config_dvs_cap_path() falied\r\n");
		return XST_FAILURE;
	}

	print("---------------------------------\r\n");


	/* Enable exceptions. */
	Xil_AssertSetCallback((Xil_AssertCallback)Xil_AssertCallbackRoutine);
	Xil_ExceptionEnable();

	/* Reset Camera Sensor module through GPIO */
	xil_printf("Disable CAM_RST of Sensor through GPIO\r\n");
	CamReset();
	xil_printf("Sensor is  Enabled\r\n");

	/* Program Camera sensor */
	Status = SetupCameraSensor();
	if (Status != XST_SUCCESS)
	{
		xil_printf("Failed to setup Camera sensor\r\n");
		return XST_FAILURE;
	}

	/*************** Program DVS sensor **************/
	// frame header setting
	MIPI_CONTROLLER_mWriteReg(MIPI_CONTROLLER_BASEADDR,
				MIPI_CONTROLLER_S00_AXI_SLV_REG0_OFFSET,
				1);
	MIPI_CONTROLLER_mWriteReg(MIPI_CONTROLLER_BASEADDR,
				MIPI_CONTROLLER_S00_AXI_SLV_REG1_OFFSET,
				1);

	xil_printf(TXT_CYAN"[RESET] start_dvs_cap_pipe() & waiting for pipeline to drain...\r\n"TXT_RST);
	start_dvs_cap_pipe();
	usleep(1500); // 1.5 ms
	xil_printf(TXT_CYAN"===Done===\r\n"TXT_RST);

#if(CHECK_DVS_FRAME_DROP)
	frame_drop_check_init=1;
	#if(!DVS_FRAME_DROP_LOG_IMMEDIATE)
	frame_drop_count_dvs=0;
		#if(CHECK_DVS_MULTIPLE_FRAME_DROP)
	multiple_frame_drop_count_dvs=0;
		#endif
	#endif
#endif

#if(AUTO_TEST_MODE==0)
	do
	{
	    xil_printf("\r\nChange sensor settings now?\r\n");
	    xil_printf("  [E]dit via UART tool (build UPDATED configuration)\r\n");
	    xil_printf("  [D]efaults only (skip editing, use BASE configuration)\r\n");
	    xil_printf("  [P]reset (use predefined PRESET configuration)\r\n");
	    xil_printf("Select (E/D/P): ");

		Response = XUartPs_RecvByte(UART_BASEADDR);
		XUartPs_SendByte(UART_BASEADDR, Response);
		xil_printf("\r\n");
		if ((Response == 'E') || (Response == 'e'))
		{
			Status = EditSensorConfig();
			break;
		}
		else if ((Response == 'D') || (Response == 'd'))
		{
			xil_printf("Programming sensor with the default configuration.\r\n");
			Status = ProgramDVSSensor(DVS_regs, length_DVS_regs);
			break;
		}
		else if ((Response == 'P') || (Response == 'p'))
		{
			Status = SelectSensorConfigPreset();
			if(Status==999) continue;
			else	break;

		}
		else
		{
			xil_printf("Unknown commend.\r\n");
			continue;
		}
	} while (1);
#else
	// TODO:
	// After completing the handshake with the host, program DVS with the new configuration values.
	switch(cfg_num){
		case 0: {
			xil_printf("Programming sensor with default configuration.\r\n");
			Status = ProgramDVSSensor(DVS_regs, length_DVS_regs);
			break;
		}
		case 1: {
			xil_printf("Programming sensor with 1958fps configuration.\r\n");
			Status = ProgramDVSSensor(DVS_regs_1958fps, length_DVS_regs_1958fps);
			break;
		}
	}

#endif

	if (Status != XST_SUCCESS)
	{
		print(TXT_RED "PROGRAM DVS error.\n\r" TXT_RST);
		return XST_FAILURE;
	}
	else
	{
		print(TXT_GREEN "PROGRAM DVS SUCCEEDED.\n\r" TXT_RST);
		// TODO: Update next config flag
		cfg_num = (cfg_num+1) % 2;
	}
	/*************************************************/


	start_csi_cap_pipe(Pipeline_Cfg.VideoMode);

	InitImageProcessingPipe();

	/* Start Camera Sensor to capture video */
	StartSensor();

#if(ENABLE_MENU && AUTO_TEST_MODE==0)
	XMipi_MenuInitialize(&XmipiMenu, UART_BASEADDR);
#endif

	New_Cfg = Pipeline_Cfg;
	/* Print the Pipe line configuration */
	PrintPipeConfig();


	DEBUG_PRINT(INFO, "DMA setup is done and will be run.\r\n");

	//slv_reg2_old = slv_reg2;
	slv_reg3_old = slv_reg3;
	//slv_reg4_old = slv_reg4;
	//slv_reg5_old = slv_reg5;


	return XST_SUCCESS;
}

int reset_dvs(void)
{
	u8 Response;
	u32 Status;

	static int cfg_num = 0;


	/*Status = InitDVSIIC();
	if (Status != XST_SUCCESS)
	{
		xil_printf(TXT_RED "\n\rDVS IIC Init Failed \n\r" TXT_RST);
		return XST_FAILURE;
	}
	else
	{
		print(TXT_GREEN "IIC init SUCCEEDED.\n\r" TXT_RST);
	}*/
#if(RESET_DMA_BEFORE_PROGRAM)

	#if(ENABLE_DVS_FILTER)
		//Filter Setup & Run
		// Reg1: Set Threshold  ==>> [31:16]OffTh,  [15:0]OnTh
		// Reg0: Run Filter ==>> 1: Run
		//u32 Threshold = 0b00000100000010;
	u32 Threshold = 0b0;
		//u32 Threshold = 0b11111111111111;
	DVS_ADAPTIVE_FILTER_mWriteReg(XPAR_DVS_ADAPTIVE_FILTER_0_AXI_LITE_BASEADDR, DVS_ADAPTIVE_FILTER_AXI_Lite_SLV_REG1_OFFSET, Threshold);
	DVS_ADAPTIVE_FILTER_mWriteReg(XPAR_DVS_ADAPTIVE_FILTER_0_AXI_LITE_BASEADDR, DVS_ADAPTIVE_FILTER_AXI_Lite_SLV_REG0_OFFSET, 1);

	FrameBufferPointerInit();
	FilRxInit();
	FilTxInit();
	g_FILTxRunning = 0;
	g_FILRxRunning = 1;

	FilRxSetAndRun();
	#endif


	// reset buffer
	//reset_dvs_buffer_rdy();
	//dvs_frm_cnt = 0;

	dvs_wr_ptr = 0; // **********

	Status = config_dvs_cap_path();
	if(Status != XST_SUCCESS)
	{
		xil_printf("config_dvs_cap_path() falied\r\n");
		return XST_FAILURE;
	}

	print("---------------------------------\r\n");

	xil_printf(TXT_CYAN"[RESET] start_dvs_cap_pipe() & waiting for pipeline to drain...\r\n"TXT_RST);
	start_dvs_cap_pipe();
	usleep(1500); // 1.5 ms
	xil_printf(TXT_CYAN"===Done===\r\n"TXT_RST);
#endif



#if(AUTO_TEST_MODE==0)
	do
	{
	    xil_printf("\r\nChange sensor settings now?\r\n");
	    xil_printf("  [E]dit via UART tool (build UPDATED configuration)\r\n");
	    xil_printf("  [D]efaults only (skip editing, use BASE configuration)\r\n");
	    xil_printf("  [P]reset (use predefined PRESET configuration)\r\n");
	    xil_printf("Select (E/D/P): ");

		Response = XUartPs_RecvByte(UART_BASEADDR);
		XUartPs_SendByte(UART_BASEADDR, Response);
		xil_printf("\r\n");
		if ((Response == 'E') || (Response == 'e'))
		{
			Status = EditSensorConfig();
			break;
		}
		else if ((Response == 'D') || (Response == 'd'))
		{
			xil_printf("Programming sensor with the default configuration.\r\n");
			Status = ProgramDVSSensor(DVS_regs, length_DVS_regs);
			break;
		}
		else if ((Response == 'P') || (Response == 'p'))
		{
			Status = SelectSensorConfigPreset();
			if(Status==999) continue;
			else	break;

		}
		else
		{
			xil_printf("Unknown commend.\r\n");
			continue;
		}
	} while (1);
#else
	// TODO:
	// After completing the handshake with the host, program DVS with the new configuration values.
	switch(cfg_num){
		case 0: {
			xil_printf("Programming sensor with default configuration.\r\n");
			Status = ProgramDVSSensor(DVS_regs, length_DVS_regs);
			break;
		}
		case 1: {
			xil_printf("Programming sensor with 1958fps configuration.\r\n");
			Status = ProgramDVSSensor(DVS_regs_1958fps, length_DVS_regs_1958fps);
			break;
		}
	}
#endif

	if (Status != XST_SUCCESS)
	{
		print(TXT_RED "PROGRAM DVS error.\n\r" TXT_RST);
		return XST_FAILURE;
	}
	else
	{
		print(TXT_GREEN "PROGRAM DVS SUCCEEDED.\n\r" TXT_RST);
		// TODO: Update next config flag
		cfg_num = (cfg_num+1) % 2;
	}

#if(CHECK_DVS_FRAME_DROP)
	frame_drop_check_init=1;
	#if(!DVS_FRAME_DROP_LOG_IMMEDIATE)
	frame_drop_count_dvs=0;
		#if(CHECK_DVS_MULTIPLE_FRAME_DROP)
	multiple_frame_drop_count_dvs=0;
		#endif
	#endif
#endif

#if(RESET_DMA_BEFORE_PROGRAM)
	// reset buffer
	reset_dvs_buffer_rdy();
	#if(ENABLE_DVS_FILTER)
	fil_frm_cnt = 0;
	#endif
	dvs_frm_cnt = 0;
	dvs_wr_ptr = 0;

	DEBUG_PRINT(INFO, "DMA setup is done and will be run.\r\n");
#endif

	New_Cfg = Pipeline_Cfg;

	test_header++;

	//slv_reg2_old = slv_reg2;
	slv_reg3_old = slv_reg3;
	//slv_reg4_old = slv_reg4;
	//slv_reg5_old = slv_reg5;

	return XST_SUCCESS;
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

	Status = setup_all();

	/* Main loop */
	u16 error_count;
	u16 old_error_count = 0;
	u32 sensor_config_idx = 0;
	u8 design_space_exploration_done = 0;

#if(ENABLE_DVS_RESET==1)
	XTime reset_timeout_start_t,  reset_timeout_cur_t;
#endif


#if(ENABLE_MAIN_DEBUG_OUTPUT && AUTO_TEST_MODE==0)
	XTime start_t, end_t;
	XTime_GetTime(&start_t);
	u8 cnt_ptr=0;
#endif

#if(CHECK_FRAME_COUNT)
	#if(ENABLE_DVS_FILTER)
	int fil_cnts[2]={0,0};
	#endif
	int dvs_cnts[2]={0,0};
#endif


#if(CHECK_DVS_BUFFER_HOST_DRAIN)
	int dvs_miss_cnts[2]={0,0};
#endif
#if(CHECK_FIL_BUFFER_HOST_DRAIN && ENABLE_DVS_FILTER)
	int fil_miss_cnts[2]={0,0};
#endif

#if(CHECK_FILTER_REG_VALUES && ENABLE_DVS_FILTER)
	int frm_event_cnt = 0;
	u32 kern_size = 0;
	u32 debug1, debug2, debug3, debug4, debug5, debug6;
#endif


	do{

#if(ENABLE_MAIN_DEBUG_OUTPUT && AUTO_TEST_MODE==0)
		XTime_GetTime(&end_t);
		if( !is_user_input_active && (end_t-start_t)>(COUNTS_PER_SECOND * MAIN_DEBUG_OUTPUT_INTERVAL_SEC))
		{
			XTime_GetTime(&start_t);

#if(CHECK_DVS_BUFFER_HOST_DRAIN)
			dvs_miss_cnts[cnt_ptr] = host_delay_count_dvs;
#endif
#if(CHECK_FIL_BUFFER_HOST_DRAIN && ENABLE_DVS_FILTER)
			fil_miss_cnts[cnt_ptr] = host_delay_count_filter;
#endif
			xil_printf("==DEBUG MESSAGE BEGIN==\r\n");
#if(CHECK_FRAME_COUNT)
			dvs_cnts[cnt_ptr] = dvs_frm_cnt;
	#if(ENABLE_DVS_FILTER)
			fil_cnts[cnt_ptr] = fil_frm_cnt;
			xil_printf("CIS frame count: %d, DVS frame count: %d, fil_frm_cnt: %d\r\n",frm_cnt, dvs_cnts[cnt_ptr],fil_cnts[cnt_ptr]);
			xil_printf("DVS frame count-fil_frm_cnt: %d, dvs_fps: %d, fil_fps: %d\r\n",dvs_cnts[cnt_ptr]-fil_cnts[cnt_ptr], dvs_cnts[cnt_ptr]-dvs_cnts[!cnt_ptr] ,fil_cnts[cnt_ptr]-fil_cnts[!cnt_ptr]);
	#else
			xil_printf("CIS frame count: %d, DVS frame count: %d, DVS FPS: %d\r\n",frm_cnt, dvs_cnts[cnt_ptr], dvs_cnts[cnt_ptr]-dvs_cnts[!cnt_ptr]);
	#endif

#endif
#if(CHECK_DVS_BUFFER_HOST_DRAIN)
			xil_printf("host_read_miss_count(dvs): %d\r\n",dvs_miss_cnts[cnt_ptr]);
#endif
#if(CHECK_FIL_BUFFER_HOST_DRAIN && ENABLE_DVS_FILTER)
			xil_printf("host_read_miss_count/2(filter) %d\r\n",fil_miss_cnts[cnt_ptr]);
#endif
#if(CHECK_DVS_FRAME_DROP && !DVS_FRAME_DROP_LOG_IMMEDIATE)
			xil_printf("dvs_frame_skip_count= %d\r\n", frame_drop_count_dvs);
	#if(CHECK_DVS_MULTIPLE_FRAME_DROP)
			xil_printf("dvs_multiple_frame_skip_count= %d\r\n", multiple_frame_drop_count_dvs);
	#endif
#endif
#if (CHECK_FIL_EVENT_COUNT && ENABLE_DVS_FILTER)
			ShowEventCountAndReset();
#endif
			cnt_ptr = !cnt_ptr;

#if(CHECK_ALL_ISR_INTERVAL)
			SwapIntervalBuffers();
			PrintAndResetAllIntervals();
#elif(CHECK_DVS_ISR_INTERVAL)
			SwapIntervalBuffers();
			PrintAndResetInterval(INTERVAL_DVS_DMA_DONE_ISR);
#elif(CHECK_FIL_ISR_INTERVAL && ENABLE_DVS_FILTER)
			SwapIntervalBuffers();
			PrintAndResetInterval(INTERVAL_FIL_DMA_DONE_ISR);
#endif


#if(CHECK_FILTER_REG_VALUES && ENABLE_DVS_FILTER)
			frm_event_cnt = DVS_ADAPTIVE_FILTER_mReadReg(XPAR_DVS_ADAPTIVE_FILTER_0_AXI_LITE_BASEADDR, DVS_ADAPTIVE_FILTER_AXI_Lite_SLV_REG2_OFFSET);
			kern_size = popcount(DVS_ADAPTIVE_FILTER_mReadReg(XPAR_DVS_ADAPTIVE_FILTER_0_AXI_LITE_BASEADDR, DVS_ADAPTIVE_FILTER_AXI_Lite_SLV_REG3_OFFSET));
			xil_printf("Event Count: %d / %d \r\n", frm_event_cnt, MAX_EVENTS_2FRAMES);
			xil_printf("Kernel Size: %dx%d\r\n", (2*kern_size + 1), (2*kern_size + 1));

			/*debug1 = DVS_ADAPTIVE_FILTER_mReadReg(XPAR_DVS_ADAPTIVE_FILTER_0_AXI_LITE_BASEADDR, DVS_ADAPTIVE_FILTER_AXI_Lite_SLV_REG4_OFFSET);
			debug2 = DVS_ADAPTIVE_FILTER_mReadReg(XPAR_DVS_ADAPTIVE_FILTER_0_AXI_LITE_BASEADDR, DVS_ADAPTIVE_FILTER_AXI_Lite_SLV_REG5_OFFSET);
			xil_printf("pipestall: %d,  next_dataready: %d\r\n", debug1, debug2);

			debug3 = AXI4_WRITE_DMA_mReadReg(XPAR_AXI4_WRITE_DMA_0_HWDMA_AXI_LITE_BASEADDR, AXI4_WRITE_DMA_HWDMA_AXI_LITE_SLV_REG7_OFFSET);
			debug4 = AXI4_WRITE_DMA_mReadReg(XPAR_AXI4_WRITE_DMA_0_HWDMA_AXI_LITE_BASEADDR, AXI4_WRITE_DMA_HWDMA_AXI_LITE_SLV_REG8_OFFSET);
			debug5 = AXI4_WRITE_DMA_mReadReg(XPAR_AXI4_WRITE_DMA_0_HWDMA_AXI_LITE_BASEADDR, AXI4_WRITE_DMA_HWDMA_AXI_LITE_SLV_REG9_OFFSET);
			debug6 = AXI4_WRITE_DMA_mReadReg(XPAR_AXI4_WRITE_DMA_0_HWDMA_AXI_LITE_BASEADDR, AXI4_WRITE_DMA_HWDMA_AXI_LITE_SLV_REG10_OFFSET);
			xil_printf("w_pending_count: %d,  n_w_addr: %d, w_enough_stream_data: %d, aw_stall: %d\r\n", debug3, debug4, debug5, debug6);
			*/
#endif

			//CsiRxPrintRegStatus();
			slv_reg2= MIPI_CONTROLLER_mReadReg(MIPI_CONTROLLER_BASEADDR, MIPI_CONTROLLER_S00_AXI_SLV_REG2_OFFSET);

			xil_printf("slv_reg2 = %x\r\n", slv_reg2); // "mipi_fifo_out_frame[15:8] = %d, frame_counter[7:0] = %d\r\n"
			slv_reg3= MIPI_CONTROLLER_mReadReg(MIPI_CONTROLLER_BASEADDR, MIPI_CONTROLLER_S00_AXI_SLV_REG3_OFFSET);
			//xil_printf("slv_reg3 = %x\r\n", slv_reg3); //"col_err[31:24] = %d, group_err[23:16] = %d, tstamp_err[15:8] = %d, frame_end_err[7:0] = %d\r\n"
			//slv_reg3  [31:24] = column packet 끼리 연속
			//[23:16] = group packet 역전
			//[15:8] = timestamp 누락
			//[7:0] 이 column index 역전 or frame end packet 생략됨
			// u8 e1, e2, e3, e4;

			xil_printf("slv_reg3 = %x\r\n", slv_reg3-slv_reg3_old);
			slv_reg4 = MIPI_CONTROLLER_mReadReg(MIPI_CONTROLLER_BASEADDR, MIPI_CONTROLLER_S00_AXI_SLV_REG4_OFFSET);
			xil_printf("slv_reg4 = %x\r\n", slv_reg4);
			slv_reg5= MIPI_CONTROLLER_mReadReg(MIPI_CONTROLLER_BASEADDR, MIPI_CONTROLLER_S00_AXI_SLV_REG5_OFFSET);
			xil_printf("slv_reg5 = %x\r\n", slv_reg5); // "mipi_fifo_out_frame[15:8] = %d, frame_counter[7:0] = %d\r\n"
			xil_printf("DVS frame_error_count = %d\r\n", error_count);
			xil_printf("CIS frame count: %d, DVS frame count:  %d\r\n", frm_cnt, dvs_frm_cnt);
			//xil_printf("==DEBUG MESSAGE END==\r\n");
		}


#endif //( ENABLE_MAIN_DEBUG_OUTPUT )

#if(AUTO_TEST_MODE)
		usleep(AUTO_RESET_ITV_US);
		dvs_reset=1;

#elif(ENABLE_MENU)
	XMipi_MenuProcess(&XmipiMenu);
#endif

#if(ENABLE_DVS_FILTER)
	#if (FILTER_IGNORE_DVS_READY)
		if(g_FILTxRunning==0)
	#elif(ENABLE_DVS_RESET==1)
		if(dvs_reset==0 && g_FILTxRunning==0 && (NumOfEmptyDVSBuf<=(DVS_BUFFER_NUM-FIL_RUN_MARGIN)))
	#else
		if(g_FILTxRunning==0 && (NumOfEmptyDVSBuf<=(DVS_BUFFER_NUM-FIL_RUN_MARGIN)))
	#endif
		{
			g_FILTxRunning = 1;
			FilTxSetAndRun();

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
#endif
			
#if(ENABLE_DVS_RESET==1)
		if(dvs_reset==1)
		{

			xil_printf(TXT_GREEN "\n\r DVS RESET TEST \n\r" TXT_RST);

	#if(RESET_DMA_BEFORE_PROGRAM)
			XTime_GetTime(&reset_timeout_start_t);
			while(dvs_dma_reset()==XST_FAILURE){
				XTime_GetTime(&reset_timeout_cur_t);
				if(reset_timeout_cur_t-reset_timeout_start_t>COUNTS_PER_SECOND)
				{
					xil_printf(TXT_RED"\n\r+++++ERROR: DVS DMA Reset timeout+++++\n\r"TXT_RST);
					while(1){
						xil_printf(TXT_RED"\n\r+++++ERROR: DVS DMA Reset timeout+++++\n\r"TXT_RST);
						usleep(500000);
					}
				}
			}

		#if(ENABLE_DVS_FILTER)
			XTime_GetTime(&reset_timeout_start_t);
			while(g_FILTxRunning)
			{
				XTime_GetTime(&reset_timeout_cur_t);
				if(reset_timeout_cur_t-reset_timeout_start_t>COUNTS_PER_SECOND)
				{
					xil_printf("\n\r FILTxRun timeout\n\r");
					break;
				}
			}
			XTime_GetTime(&reset_timeout_start_t);
			while(g_FILRxRunning)
			{
				XTime_GetTime(&reset_timeout_cur_t);
				if(reset_timeout_cur_t-reset_timeout_start_t>COUNTS_PER_SECOND)
				{
					// TODO: FILTxRun() 1회 실행해보는 작업 추가
					// (dvs_reset==1)인 동안 모든 filter dma는 ISR 안에서 재실행되지 않음
					xil_printf("\n\r FILRxRun() timeout or FILTx was not run or DVS stream not received.\n\r");
					break;
				}
			}
		#endif // ENABLE_DVS_FILTER
	#endif // RESET_DMA_BEFORE_PROGRAM

			dvs_reset=0;
			//while(setup_all()==XST_FAILURE);
			while(reset_dvs()==XST_FAILURE);
		} // if dvs_reset

#endif // ENABLE_DVS_RESET

	}while(design_space_exploration_done == 0);
	return 0;
}

#if(ENABLE_DVS_FILTER)

int FilRxInit(){

	// This DMA module is designed to automatically calculate and
	//access sequential addresses starting from an initial address.
	// The module accesses a defined range of addresses within a circular FIFO buffer.

	// When the address reaches the buffer's end address, it needs to wrap around
	//to the start address of the circular buffer. To handle this, the DMA module
	//requires two parameters: the buffer's end address and the buffer's total size.
	// If the current address exceeds the buffer's end address, the DMA subtracts
	//the buffer size from the current address to wrap around correctly.

	// The following section writes the buffer's end address and total size to the DMA module registers.

	// the address must be split into two separate registers: the upper 32 bits and the lower 32 bits.
	// When using addresses larger than 32 bits, the value should be written to both the upper and
	//lower 32-bit registers.
	// if the target address is within the 32-bit address range, only the lower 32-bit register needs to be written.

	AXI4_WRITE_DMA_mWriteReg(XPAR_AXI4_WRITE_DMA_0_HWDMA_AXI_LITE_BASEADDR, AXI4_WRITE_DMA_HWDMA_AXI_LITE_SLV_REG3_OFFSET, 0);//((DVS_BUFFER_HIGH)>>32) & 0xFFFFFFFF);
	AXI4_WRITE_DMA_mWriteReg(XPAR_AXI4_WRITE_DMA_0_HWDMA_AXI_LITE_BASEADDR, AXI4_WRITE_DMA_HWDMA_AXI_LITE_SLV_REG4_OFFSET, (FIL_BUF_END_ADDR) & 0xFFFFFFFF );

	AXI4_WRITE_DMA_mWriteReg(XPAR_AXI4_WRITE_DMA_0_HWDMA_AXI_LITE_BASEADDR, AXI4_WRITE_DMA_HWDMA_AXI_LITE_SLV_REG5_OFFSET, 0);//((DVS_BUFFER_TOTAL_SIZE)>>32) & 0xFFFFFFFF);
	AXI4_WRITE_DMA_mWriteReg(XPAR_AXI4_WRITE_DMA_0_HWDMA_AXI_LITE_BASEADDR, AXI4_WRITE_DMA_HWDMA_AXI_LITE_SLV_REG6_OFFSET, (FIL_BUF_TOTAL_SIZE) & 0xFFFFFFFF );

}
int FilRxSetAndRun(){

	DEBUG_PRINT(DEBUG, "Fil Rx Set: %d\r\n", g_dma_fil_wr_work_ptr%FIL_BUFFER_NUM);
	// TODO: Separate the process of setting the starting address
	//      and triggering the DMA execution to minimize the delay
	//      caused by the address configuration.
	//       - The starting address should be configured in advance.
	//       - The execution flag should be set only after all necessary
	//        parameters are written to the DMA module.

	// This section writes the starting address and the execution flag to the DMA module.

	// 1. The starting address defines where the DMA transfer will begin.
	//    This address must be aligned correctly based on the DMA's data requirements.
	AXI4_WRITE_DMA_mWriteReg(XPAR_AXI4_WRITE_DMA_0_HWDMA_AXI_LITE_BASEADDR, AXI4_WRITE_DMA_HWDMA_AXI_LITE_SLV_REG1_OFFSET, (fil_frame_array[g_dma_fil_wr_work_ptr%FIL_BUFFER_NUM]>>32) & 0xFFFFFFFF);
	AXI4_WRITE_DMA_mWriteReg(XPAR_AXI4_WRITE_DMA_0_HWDMA_AXI_LITE_BASEADDR, AXI4_WRITE_DMA_HWDMA_AXI_LITE_SLV_REG2_OFFSET, fil_frame_array[g_dma_fil_wr_work_ptr%FIL_BUFFER_NUM] & 0xFFFFFFFF );

	// 2. The execution flag is used to trigger the DMA operation.
	//    Once the starting address and other necessary parameters are set,
	//   writing "1" to this flag initiates the DMA transfer process.
	AXI4_WRITE_DMA_mWriteReg(XPAR_AXI4_WRITE_DMA_0_HWDMA_AXI_LITE_BASEADDR, AXI4_WRITE_DMA_HWDMA_AXI_LITE_SLV_REG0_OFFSET, 0x1);

}


int FilTxSetAndRun()
{
	DEBUG_PRINT(DEBUG, "Fil Tx Set: %d\r\n", g_dma_fil_rd_work_ptr%DVS_BUFFER_NUM);

	// TODO: Separate the process of setting the starting address
	//      and triggering the DMA execution to minimize the delay
	//      caused by the address configuration.
	//       - The starting address should be configured in advance.
	//       - The execution flag should be set only after all necessary
	//        parameters are written to the DMA module.

	// This section writes the starting address and the execution flag to the DMA module.

	// 1. The starting address defines where the DMA transfer will begin.
	//    This address must be aligned correctly based on the DMA's data requirements.
	u64 dvs_frame_addr = dvs_frame_array[g_dma_fil_rd_work_ptr%DVS_BUFFER_NUM] - DVS_FRAME_HEADER_BYTES_EXT; // 확장 헤더라면 확장된 부분까지 읽기
	AXI4_READ_DMA_mWriteReg(XPAR_AXI4_READ_DMA_0_HWDMA_AXI_LITE_BASEADDR, AXI4_READ_DMA_HWDMA_AXI_LITE_SLV_REG1_OFFSET, (dvs_frame_addr>>32) & 0xFFFFFFFF);
	AXI4_READ_DMA_mWriteReg(XPAR_AXI4_READ_DMA_0_HWDMA_AXI_LITE_BASEADDR, AXI4_READ_DMA_HWDMA_AXI_LITE_SLV_REG2_OFFSET, dvs_frame_addr & 0xFFFFFFFF );

	// 2. The execution flag is used to trigger the DMA operation.
	//    Once the starting address and other necessary parameters are set,
	//   writing "1" to this flag initiates the DMA transfer process.
	AXI4_READ_DMA_mWriteReg(XPAR_AXI4_READ_DMA_0_HWDMA_AXI_LITE_BASEADDR, AXI4_READ_DMA_HWDMA_AXI_LITE_SLV_REG0_OFFSET, 0x1);


	//u64 dvs_header_addr = dvs_frame_array[g_dma_fil_rd_work_ptr%DVS_BUFFER_NUM];
	//u64 dvs_header_addr2 = dvs_frame_array[(g_dma_fil_rd_work_ptr%DVS_BUFFER_NUM)+1];
	//******************* dvs buf num  must be divisible by fil buf num ***********//
	u64 dvs_header_addr = dvs_frame_array[(g_dma_fil_rd_work_ptr+2)%DVS_BUFFER_NUM] - DVS_FRAME_HEADER_BYTES_EXT; // 확장 헤더라면 확장된 부분까지 읽기;
	u64 dvs_header_addr2 = dvs_frame_array[((g_dma_fil_rd_work_ptr+2)%DVS_BUFFER_NUM)+1] - DVS_FRAME_HEADER_BYTES_EXT; // 확장 헤더라면 확장된 부분까지 읽기

	u64 fil_header_addr = fil_frame_array[g_dma_fil_rd_work_ptr%FIL_BUFFER_NUM];
	u64 fil_header_addr2 = fil_frame_array[(g_dma_fil_rd_work_ptr%FIL_BUFFER_NUM)+1];
	//*****************************************************************************//

	memcpy((void*)fil_header_addr,(void*)dvs_header_addr, DVS_FRAME_HEADER_SIZE_IN_BYTE );
	memcpy((void*)fil_header_addr2,(void*)dvs_header_addr2, DVS_FRAME_HEADER_SIZE_IN_BYTE );
	//Xil_Out64((UINTPTR)fil_header_addr, Xil_In64((UINTPTR)dvs_header_addr));
	//Xil_Out64((UINTPTR)fil_header_addr2, Xil_In64((UINTPTR)dvs_header_addr2));


}

int FilTxInit()
{
	// This DMA module is designed to automatically calculate and
	//access sequential addresses starting from an initial address.
	// The module accesses a defined range of addresses within a circular FIFO buffer.

	// When the address reaches the buffer's end address, it needs to wrap around
	//to the start address of the circular buffer. To handle this, the DMA module
	//requires two parameters: the buffer's end address and the buffer's total size.
	// If the current address exceeds the buffer's end address, the DMA subtracts
	//the buffer size from the current address to wrap around correctly.

	// The following section writes the buffer's end address and total size to the DMA module registers.


	// the address must be split into two separate registers: the upper 32 bits and the lower 32 bits.
	// When using addresses larger than 32 bits, the value should be written to both the upper and
	//lower 32-bit registers.
	// if the target address is within the 32-bit address range, only the lower 32-bit register needs to be written.

	AXI4_READ_DMA_mWriteReg(XPAR_AXI4_READ_DMA_0_HWDMA_AXI_LITE_BASEADDR, AXI4_READ_DMA_HWDMA_AXI_LITE_SLV_REG3_OFFSET, 0);//((DVS_BUFFER_HIGH)>>32) & 0xFFFFFFFF);
	AXI4_READ_DMA_mWriteReg(XPAR_AXI4_READ_DMA_0_HWDMA_AXI_LITE_BASEADDR, AXI4_READ_DMA_HWDMA_AXI_LITE_SLV_REG4_OFFSET, (DVS_BUFFER_HIGH) & 0xFFFFFFFF );

	AXI4_READ_DMA_mWriteReg(XPAR_AXI4_READ_DMA_0_HWDMA_AXI_LITE_BASEADDR, AXI4_READ_DMA_HWDMA_AXI_LITE_SLV_REG5_OFFSET, 0);//((DVS_BUFFER_TOTAL_SIZE)>>32) & 0xFFFFFFFF);
	AXI4_READ_DMA_mWriteReg(XPAR_AXI4_READ_DMA_0_HWDMA_AXI_LITE_BASEADDR, AXI4_READ_DMA_HWDMA_AXI_LITE_SLV_REG6_OFFSET, (DVS_BUFFER_TOTAL_SIZE) & 0xFFFFFFFF );

}


void DmaFILTxIntrHandler(int *FilTxPtr) {


#if (CHECK_FILTER_DMA_READ_WRITE_SYNC)
		if(prev_isr==0) DEBUG_PRINT(ERROR, "Tx->Tx error\r\n");
		prev_isr = 0;
#endif

		DEBUG_PRINT(DEBUG, "DmaFilTxDone is called\r\n");

		UpdateFilRDBufPtr(&g_dma_fil_rd_work_ptr,2);
		if((dvs_wr_ptr>=DVS_BUFFER_NUM) == (g_dma_fil_rd_work_ptr>=DVS_BUFFER_NUM))
			NumOfEmptyDVSBuf = ( DVS_BUFFER_NUM - (dvs_wr_ptr - g_dma_fil_rd_work_ptr) % DVS_BUFFER_NUM ) ;
		else
			NumOfEmptyDVSBuf = ( g_dma_fil_rd_work_ptr - dvs_wr_ptr) % DVS_BUFFER_NUM;

		DEBUG_PRINT(DEBUG, "NumOfEmptyDVSBuf: %d \r\n",NumOfEmptyDVSBuf);

		//g_FILTxRunning = 1;
		//FilTxSetAndRun();

#if(ENABLE_DVS_RESET==1)
		if(dvs_reset==0 && (NumOfEmptyDVSBuf <= (DVS_BUFFER_NUM-FIL_RUN_MARGIN)))
#else
		if((NumOfEmptyDVSBuf <= (DVS_BUFFER_NUM-FIL_RUN_MARGIN)))
#endif
		{
			g_FILTxRunning = 1;

			FilTxSetAndRun();
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
			g_FILTxRunning = 0;
			DEBUG_PRINT(DEBUG, "There is not enough data accumulated in the DVS Buffer, FIL Tx Stopped (fil read work pointer: 0x%x)\r\n", g_dma_fil_rd_work_ptr%DVS_BUFFER_NUM);
		}

	return ;
}


void DmaFILRxIntrHandler(int *FilRxPtr) {

#if(CHECK_FIL_ISR_INTERVAL)
	IntervalRecordStart(INTERVAL_FIL_DMA_DONE_ISR);
#endif

#if (CHECK_FILTER_DMA_READ_WRITE_SYNC)
		if(prev_isr==1) DEBUG_PRINT(ERROR, "Rx->Rx error\r\n");
		prev_isr = 1;
#endif

		DEBUG_PRINT(DEBUG, "DmaFILRxDone is called\r\n");

		UpdateFilWRBufPtr(&g_dma_fil_wr_work_ptr,2);

#if(ENABLE_DVS_RESET==1)
		if(!dvs_reset) FilRxSetAndRun();
		else g_FILRxRunning = 0;
#else
		FilRxSetAndRun();
#endif

		/*if((g_dma_fil_wr_work_ptr>=FIL_BUFFER_NUM) == (g_dma_fil_send_work_ptr>=FIL_BUFFER_NUM))
			NumOfEmptyFILBuf = ( FIL_BUFFER_NUM - (g_dma_fil_wr_work_ptr - g_dma_fil_send_work_ptr) % FIL_BUFFER_NUM ) ;
		else
			NumOfEmptyFILBuf = ( g_dma_fil_send_work_ptr - g_dma_fil_wr_work_ptr) % FIL_BUFFER_NUM;
	*/
		//DEBUG_PRINT(DEBUG, "NumOfEmptyFILBuf: %d \r\n",NumOfEmptyFILBuf);

		//if(NumOfEmptyFILBuf < 2){
		//	g_FILRxRunning = 0;
		//	DEBUG_PRINT(INFO, "There is no empty space in the FIL Buffer, FIL Rx Stopped (fil write work pointer: 0x%x)\r\n", g_dma_fil_wr_work_ptr%FIL_BUFFER_NUM);
		//}
		//else{
		//	g_FILRxRunning = 1;
		//	Status = RxBDRun(RxRingPtr, DMA_FIL_RXBD_NUM);
		//	if (Status != XST_SUCCESS) DEBUG_PRINT(ERROR, "RxBDRun failed.\n\r");
		//}

#if (CHECK_FIL_BUFFER_HOST_DRAIN)
		if(Xil_In8(fil_frame_rdy[ g_dma_fil_wr_setup_ptr % FIL_BUFFER_NUM ]) == 0x1){
			//DEBUG_PRINT(WARNING, "Host is too slow to retrieve Filtered DVS data via PCIe in time.\r\n");
			host_delay_count_filter++;
		}
#endif

		u32 wptr = g_dma_fil_wr_setup_ptr % FIL_BUFFER_NUM ;
		Xil_Out8(fil_frame_rdy[ wptr ], 0x1);
		Xil_Out8(fil_frame_rdy[ (wptr+1) ], 0x1);
		Xil_Out32(FIL_BUFFER_COMMIT_IDX, (wptr+1));


		UpdateFilWRBufPtr(&g_dma_fil_wr_setup_ptr,2);

		fil_frm_cnt+=2;

#if (CHECK_FIL_EVENT_COUNT)
	ReadEventCountAndRecord();
#endif

}


void FrameBufferPointerInit()
{
	g_dma_fil_rd_setup_ptr = 0;
	g_dma_fil_rd_work_ptr = 0;
	g_dma_fil_wr_setup_ptr = 0;
	g_dma_fil_wr_work_ptr = 0;

	NumOfEmptyDVSBuf = DVS_BUFFER_NUM;
	NumOfEmptyFILBuf = FIL_BUFFER_NUM;
}

void UpdateFilWRBufPtr(u32 *pCurBufPtr, u32 uiStep)
{
	u32 uiNextBufPtrCand;
	static u32 uiBufPtrMaxPlus1 = 2*FIL_BUFFER_NUM;
	uiNextBufPtrCand = *pCurBufPtr + uiStep;


	if(uiNextBufPtrCand > (uiBufPtrMaxPlus1 - 1)) *pCurBufPtr = uiNextBufPtrCand - uiBufPtrMaxPlus1;
	else *pCurBufPtr = uiNextBufPtrCand;
}
void UpdateFilRDBufPtr(u32 *pCurBufPtr, u32 uiStep)
{
	u32 uiNextBufPtrCand;
	static u32 uiBufPtrMaxPlus1 = 2*DVS_BUFFER_NUM;
	uiNextBufPtrCand = *pCurBufPtr + uiStep;


	if(uiNextBufPtrCand > (uiBufPtrMaxPlus1 - 1)) *pCurBufPtr = uiNextBufPtrCand - uiBufPtrMaxPlus1;
	else *pCurBufPtr = uiNextBufPtrCand;
}
#endif


