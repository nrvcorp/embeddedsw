/******************************************************************************
* Copyright (C) 2017 - 2020 Xilinx, Inc.  All rights reserved.
* SPDX-License-Identifier: MIT
 *****************************************************************************/

/*****************************************************************************/
/**
 *
 * @file pipeline_program.c
 *
 * This file contains the video pipe line configuration, Sensor configuration
 * and its programming as per the resolution selected by user.
 * Please see pipeline_program.h for more details.
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

#include "stdlib.h"
#include "xparameters.h"
#include "sleep.h"
#include "xiic.h"
#include "xil_exception.h"
#include "xil_cache.h"
#include "sensor_cfgs.h"
#include "xscugic.h"

#include "xiicps.h"

#include "xgpio.h"
#include "xcsiss.h"
#include <xvidc.h>
#include <xvprocss.h>
#include "xv_frmbufwr_l2.h"
#include "xaxidma.h"

#define IIC_DVS_DEVICE_ID	XPAR_XIICPS_1_DEVICE_ID
#define GPIO_DEVICE_ID		XPAR_SENSOR_CTRL_AXI_GPIO_0_DEVICE_ID
#define DVS_RSTN 0x01
#define DVS_ADDR		(0x20>>1)	// DVS
#define DVS_FRAME_HORIZONTAL_LEN 	0xF0 	/* 960 pixels(2bit) = 240 bytes*/
#define DVS_FRAME_VERTICAL_LEN 		0x2D0 	/* 720 pixels*/


#define IIC_SENSOR_DEV_ID	XPAR_SENSOR_CTRL_AXI_IIC_0_DEVICE_ID

#define VPROCSSCSC_BASE	XPAR_XVPROCSS_0_BASEADDR
#define DEMOSAIC_BASE	XPAR_XV_DEMOSAIC_0_S_AXI_CTRL_BASEADDR
#define VGAMMALUT_BASE	XPAR_XV_GAMMA_LUT_0_S_AXI_CTRL_BASEADDR

#define PAGE_SIZE	16

#define EEPROM_TEST_START_ADDRESS	128

#define FRAME_BASE	0x10000000
#define CSIFrame	0x20000000
#define ScalerFrame	0x30000000

#define XVPROCSS_DEVICE_ID	XPAR_XVPROCSS_0_DEVICE_ID

#define ACTIVE_LANES_1	1
#define ACTIVE_LANES_2	2
#define ACTIVE_LANES_3	3
#define ACTIVE_LANES_4	4


#define XCSIRXSS_DEVICE_ID	XPAR_CSISS_0_DEVICE_ID
#define XDPHY_DEVICE_ID		XPAR_DVS_STREAM_MIPI_DPHY_0_DEVICE_ID
XCsiSs CsiRxSs;
XDphy DphyRx;

XVprocSs scaler_new_inst;

XVidC_VideoMode    VideoMode;
XVidC_VideoStream  VidStream;
XVidC_ColorFormat  Cfmt;
XVidC_VideoStream  StreamOut;


XV_FrmbufWr_l2     	frmbufwr;
XAxiDma				DVSDma;

/**************************** Type Definitions *******************************/
typedef u8 AddressType;

u8 SensorIicAddr; /* Variable for storing Eeprom IIC address */

#define SENSOR_ADDR         (0x34>>1)	/* for IMX274 Vision */

#define IIC_MUX_ADDRESS 		0x75
#define IIC_EEPROM_CHANNEL		0x01	/* 0x08 */

XIic IicSensor; /* The instance of the IIC device. */

// DVS IIC define
#define DVS_IIC_DEVICE_ID	XPAR_XIICPS_1_DEVICE_ID
#define INTC_DEVICE_ID		XPAR_SCUGIC_SINGLE_DEVICE_ID
#define SLV_MON_LOOP_COUNT 	0x000FFFFF	/**< Slave Monitor Loop Count*/
#define MAX_CHANNELS 0xf
#define DVS_MUX_ADDRESS			0x75
#define DVS_MUX_CHANNEL			0x02

/* DVS variable */
XGpio DVS_rstn_Gpio;
XIicPs IicPsInstance;		/* The instance of the IIC device. */
XScuGic InterruptController;	/* The instance of the Interrupt Controller. */
u8 DVSIicAddr; 			/* Variable for storing DVS IIC address */


volatile u8 TransmitComplete; 		/* Flag to check completion of Transmission */
volatile u8 ReceiveComplete; 		/* Flag to check completion of Reception */
volatile u8 DVSIICTransmitComplete; 	/* Flag to check completion of DVS IIC Transmission */
volatile u8 DVSIICReceiveComplete; 		/* Flag to check completion of Reception */
volatile u32 DVSIICTotalErrorCount;	/**< Total Error Count Flag */
volatile u32 DVSIICSlaveResponse;	/**< Slave Response Flag */

u8 WriteBuffer[sizeof(AddressType) + PAGE_SIZE];
u8 ReadBuffer[PAGE_SIZE]; /* Read buffer for reading a page. */
u8 DVSIICWriteBuf[sizeof(u8) + 16];
u8 DVSIICReadBuf[16];

extern XPipeline_Cfg Pipeline_Cfg;
//extern XAxiVdma_DmaSetup DVSVdma_WriteCfg;


void ConfigDemosaicResolution(XVidC_VideoMode videomode);
void DisableDemosaicResolution();

#define DDR_BASEADDR 0x10000000

#define CIS_BUFFER_RDY 		(DDR_BASEADDR + 0x1000000)
#define CIS_BUFFER_BASEADDR (DDR_BASEADDR + (0x10000000))
#define CIS_BUFFER_NUM 		5
#define CIS_BUFFER_SIZE		0x5EEC00 // 1920*1080*3
#define CHROMA_ADDR_OFFSET  (0x01000000U)

u8 RxStatusFlag;
u64 frame_array[CIS_BUFFER_NUM];
u64 frame_rdy[CIS_BUFFER_NUM];
u32 rd_ptr = CIS_BUFFER_NUM - 1;
u32 wr_ptr = 0;
u64 XVFRMBUFWR_BUFFER_BASEADDR;
u32 frmrd_start = 0;
u32 frm_cnt = 0;
u32 frm_cnt1 = 0;

#define DVS_BUFFER_RDY 		(DDR_BASEADDR + 0x2000000)
#define DVS_BUFFER_BASEADDR (DDR_BASEADDR + (0x30000000))
#define DVS_BUFFER_NUM		100
#define DVS_BUFFER_SIZE		0x2a308 // 960 * 720 (2bit) / 8bit + 8


#define BD_LEN				DVS_BUFFER_SIZE
#define COALESCING_COUNT	1
#define DELAY_TIMER_COUNT 	0
#define RX_BD_SPACE_BASE	(DDR_BASEADDR + 0x3000000)
#define RX_BD_SPACE_HIGH	(DDR_BASEADDR + 0x3000000 + DVS_BUFFER_NUM * 0x40 - 1)

u64 dvs_frame_array[DVS_BUFFER_NUM];
u64 dvs_frame_rdy[DVS_BUFFER_NUM];
u32 dvs_wr_ptr = 0;
u32 dvs_frm_cnt = 0;

void configure_buffer_system() {
	/* CIS Buffer Setting */
	u64 cis_frame_addr = CIS_BUFFER_BASEADDR;
	u64 cis_frame_rdy_addr = CIS_BUFFER_RDY;
	for (int i = 0; i < CIS_BUFFER_NUM; ++i) {
		frame_array[i] = cis_frame_addr;
		frame_rdy[i] = cis_frame_rdy_addr;
		cis_frame_addr += CIS_BUFFER_SIZE;
		cis_frame_rdy_addr += 1;
	}
	Xil_DCacheInvalidateRange((UINTPTR)CIS_BUFFER_BASEADDR, CIS_BUFFER_NUM*CIS_BUFFER_SIZE);
	Xil_DCacheInvalidateRange((UINTPTR)CIS_BUFFER_RDY, CIS_BUFFER_NUM);
	/* DVS Buffer Setting */
	u64 dvs_frame_addr = DVS_BUFFER_BASEADDR;
	u64 dvs_frame_rdy_addr = DVS_BUFFER_RDY;
	for (int i=0; i < DVS_BUFFER_NUM; ++i) {
		dvs_frame_array[i] = dvs_frame_addr;
		dvs_frame_rdy[i] = dvs_frame_rdy_addr;
		dvs_frame_addr += DVS_BUFFER_SIZE;
		dvs_frame_rdy_addr += 1;
	}
	Xil_DCacheInvalidateRange((UINTPTR)DVS_BUFFER_BASEADDR, DVS_BUFFER_NUM*DVS_BUFFER_SIZE);
	Xil_DCacheInvalidateRange((UINTPTR)DVS_BUFFER_RDY, DVS_BUFFER_NUM);
}

void DmaWriteDoneCallback (XAxiDma_BdRing * RxRingPtr) {
//	xil_printf("DVS DMA Wr Done %d\r\n", dvs_frm_cnt);
	Xil_Out8(dvs_frame_rdy[dvs_wr_ptr], 0x1);

	if (dvs_wr_ptr == DVS_BUFFER_NUM -1) {
		dvs_wr_ptr = 0;
	} else {
		dvs_wr_ptr = dvs_wr_ptr + 1;
	}

	dvs_frm_cnt++;
}

/*****************************************************************************/
/**
 * This Send handler is called asynchronously from an interrupt
 * context and indicates that data in the specified buffer has been sent.
 *
 * @param	InstancePtr is not used, but contains a pointer to the IIC
 *		device driver instance which the handler is being called for.
 *
 * @return	None.
 *
 * @note	None.
 *
 *****************************************************************************/
static void SendHandler(XIic *InstancePtr) {
	TransmitComplete = 0;
}
/*****************************************************************************/
/**
 * This function configures Frame Buffer for defined mode
 *
 * @return XST_SUCCESS if init is OK else XST_FAILURE
 *
 *****************************************************************************/
static int ConfigFrmbuf(u32 StrideInBytes,
                        XVidC_ColorFormat Cfmt,
                        XVidC_VideoStream *StreamPtr
						)
{
	int Status;

	XVFrmbufWr_WaitForIdle(&frmbufwr);

	XVFRMBUFWR_BUFFER_BASEADDR = frame_array[wr_ptr];

    /* Configure Frame Buffers */
    Status = XVFrmbufWr_SetMemFormat(&frmbufwr, StrideInBytes, Cfmt, StreamPtr);
    if(Status != XST_SUCCESS) {
      xil_printf("ERROR:: Unable to configure Frame Buffer Write\r\n");
      return(XST_FAILURE);
    }
    Status = XVFrmbufWr_SetBufferAddr(&frmbufwr, XVFRMBUFWR_BUFFER_BASEADDR);
    if(Status != XST_SUCCESS) {
      xil_printf("ERROR:: Unable to configure Frame \
                                        Buffer Write buffer address\r\n");
      return(XST_FAILURE);
    }

    /* Set Chroma Buffer Address for semi-planar color formats */
    if ((Cfmt == XVIDC_CSF_MEM_Y_UV8) || (Cfmt == XVIDC_CSF_MEM_Y_UV8_420) ||
        (Cfmt == XVIDC_CSF_MEM_Y_UV10) || (Cfmt == XVIDC_CSF_MEM_Y_UV10_420)) {

      Status = XVFrmbufWr_SetChromaBufferAddr(&frmbufwr,
                               XVFRMBUFWR_BUFFER_BASEADDR+CHROMA_ADDR_OFFSET);
      if(Status != XST_SUCCESS) {
        xil_printf("ERROR::Unable to configure Frame Buffer \
                                           Write chroma buffer address\r\n");
        return(XST_FAILURE);
      }
    }
    /* Enable Interrupt */
    XVFrmbufWr_InterruptEnable(&frmbufwr, XVFRMBUFWR_IRQ_DONE_MASK);

    return(Status);
}





/*****************************************************************************/
/**
 * This Receive handler is called asynchronously from an interrupt
 * context and indicates that data in the specified buffer has been Received.
 *
 * @param	InstancePtr is not used, but contains a pointer to the IIC
 *		device driver instance which the handler is being called for.
 *
 * @return	None.
 *
 * @note	None.
 *
 *****************************************************************************/
static void ReceiveHandler(XIic *InstancePtr) {
	ReceiveComplete = 0;
}

/*****************************************************************************/
/**
 * This Status handler is called asynchronously from an interrupt
 * context and indicates the events that have occurred.
 *
 * @param	InstancePtr is a pointer to the IIC driver instance for which
 *		the handler is being called for.
 * @param	Event indicates the condition that has occurred.
 *
 * @return	None.
 *
 * @note	None.
 *
 *****************************************************************************/
static void StatusHandler(XIic *InstancePtr, int Event) {

}

int MuxInitChannel(u16 MuxIicAddr, u8 WriteBuf)
{
	u8 Buffer = 0;

	DVSIICTotalErrorCount = 0;
	DVSIICTransmitComplete = FALSE;

	XIicPs_MasterSend(&IicPsInstance, &WriteBuf,1,MuxIicAddr);
	while (DVSIICTransmitComplete == FALSE) {
		if (0 != DVSIICTotalErrorCount) {
			return XST_FAILURE;
		}
	}
	/*
	 * Wait until bus is idle to start another transfer.
	 */

	while (XIicPs_BusIsBusy(&IicPsInstance));

	DVSIICReceiveComplete = FALSE;
	/*
	 * Receive the Data.
	 */
	XIicPs_MasterRecv(&IicPsInstance, &Buffer,1, MuxIicAddr);

	while (DVSIICReceiveComplete == FALSE) {
		if (0 != DVSIICTotalErrorCount) {
			return XST_FAILURE;
		}
	}

	/*
	 * Wait until bus is idle to start another transfer.
	 */
	while (XIicPs_BusIsBusy(&IicPsInstance));

	return XST_SUCCESS;
}

void DVSIICStatusHandler(void *CallBackRef, u32 Event) {
	/*
	 * All of the data transfer has been finished.
	 */

	if (0 != (Event & XIICPS_EVENT_COMPLETE_SEND)) {
		DVSIICTransmitComplete = TRUE;
	} else if (0 != (Event & XIICPS_EVENT_COMPLETE_RECV)){
		DVSIICReceiveComplete = TRUE;
	} else if (0 != (Event & XIICPS_EVENT_SLAVE_RDY)) {
		DVSIICSlaveResponse = TRUE;
	} else if (0 != (Event & XIICPS_EVENT_ERROR)){
		DVSIICTotalErrorCount++;
	}
}

/*****************************************************************************/
/**
 * This function writes a buffer of data to the IIC serial sensor.
 *
 * @param	ByteCount is the number of bytes in the buffer to be written.
 *
 * @return	XST_SUCCESS if successful else XST_FAILURE.
 *
 * @note	None.
 *
 *****************************************************************************/

int SensorWriteData(u16 ByteCount) {
	int Status;

	/* Set the defaults. */
	TransmitComplete = 1;

	IicSensor.Stats.TxErrors = 0;

	/* Start the IIC device. */

	Status = XIic_Start(&IicSensor);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/* Send the Data. */
	Status = XIic_MasterSend(&IicSensor, WriteBuffer, ByteCount);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/* Wait till the transmission is completed. */
	while ((TransmitComplete) || (XIic_IsIicBusy(&IicSensor) == TRUE)) {

		if (IicSensor.Stats.TxErrors != 0) {

			/* Enable the IIC device. */
			Status = XIic_Start(&IicSensor);
			if (Status != XST_SUCCESS) {
				return XST_FAILURE;
			}

			if (!XIic_IsIicBusy(&IicSensor)) {
				/* Send the Data. */
				Status = XIic_MasterSend(&IicSensor,
								WriteBuffer,
								ByteCount);

				if (Status == XST_SUCCESS) {
					IicSensor.Stats.TxErrors = 0;
				}
			}
		}
	}

	/* Stop the IIC device. */
	Status = XIic_Stop(&IicSensor);

	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
 * This function reads data from the IIC serial Camera Sensor into a specified
 * buffer.
 *
 * @param	BufferPtr contains the address of the data buffer to be filled.
 * @param	ByteCount contains the number of bytes in the buffer to be read.
 *
 * @return	XST_SUCCESS if successful else XST_FAILURE.
 *
 * @note	None.
 *
 *****************************************************************************/

int SensorReadData(u8 *BufferPtr, u16 ByteCount) {
	int Status;

	/* Set the Defaults. */
	ReceiveComplete = 1;

	Status = SensorWriteData(2);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/* Start the IIC device. */
	Status = XIic_Start(&IicSensor);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/* Receive the Data. */
	Status = XIic_MasterRecv(&IicSensor, BufferPtr, ByteCount);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/* Wait till all the data is received. */
	while ((ReceiveComplete) || (XIic_IsIicBusy(&IicSensor) == TRUE)) {

	}

	/* Stop the IIC device. */
	Status = XIic_Stop(&IicSensor);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
 * This function setup Camera sensor programming wrt resolution selected
 *
 * @return	XST_SUCCESS if successful else XST_FAILURE.
 *
 * @note	None.
 *
 *****************************************************************************/
int SetupCameraSensor(void) {
	int Status;
	u32 Index, MaxIndex;
	SensorIicAddr = SENSOR_ADDR;
	struct regval_list *sensor_cfg = NULL;

	/* If no camera present then return */
	if (Pipeline_Cfg.CameraPresent == FALSE) {
		xil_printf("%s - No camera present\r\n", __func__);
		return XST_SUCCESS;
	}

	/* Validate Pipeline Configuration */
	if ((Pipeline_Cfg.VideoMode == XVIDC_VM_3840x2160_30_P)
			&& (Pipeline_Cfg.ActiveLanes != 4)) {
		xil_printf("4K supports only 4 Lane configuration\r\n");
		return XST_FAILURE;
	}

	if ((Pipeline_Cfg.VideoMode == XVIDC_VM_1920x1080_60_P)
			&& (Pipeline_Cfg.ActiveLanes == 1)) {
		xil_printf("1080p doesn't support 1 Lane configuration\r\n");
		return XST_FAILURE;
	}

	Status = XIic_SetAddress(&IicSensor, XII_ADDR_TO_SEND_TYPE,
					SensorIicAddr);

	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/* Select the sensor configuration based on resolution and lane */
	switch (Pipeline_Cfg.VideoMode) {

		case XVIDC_VM_1280x720_60_P:
			MaxIndex = length_imx274_config_720p_60fps_regs;
			sensor_cfg = imx274_config_720p_60fps_regs;
			break;

		case XVIDC_VM_1920x1080_30_P:
			MaxIndex = length_imx274_config_1080p_60fps_regs;
			sensor_cfg = imx274_config_1080p_60fps_regs;
			break;

		case XVIDC_VM_1920x1080_60_P:
			MaxIndex = length_imx274_config_1080p_60fps_regs;
			sensor_cfg = imx274_config_1080p_60fps_regs;
			break;

		case XVIDC_VM_3840x2160_30_P:
			MaxIndex = length_imx274_config_4K_30fps_regs;
			sensor_cfg = imx274_config_4K_30fps_regs;
			break;

		case XVIDC_VM_3840x2160_60_P:
			MaxIndex = length_imx274_config_4K_30fps_regs;
			sensor_cfg = imx274_config_4K_30fps_regs;
			break;


		default:
			return XST_FAILURE;
			break;

	}

	/* Program sensor */
	for (Index = 0; Index < (MaxIndex - 1); Index++) {

		WriteBuffer[0] = sensor_cfg[Index].Address >> 8;
		WriteBuffer[1] = sensor_cfg[Index].Address;
		WriteBuffer[2] = sensor_cfg[Index].Data;

		Status = SensorWriteData(3);

		if (Status == XST_SUCCESS) {
			ReadBuffer[0] = 0;
			Status = SensorReadData(ReadBuffer, 1);

		} else {
			xil_printf("Error in Writing entry status = %x \r\n",
					Status);
			break;
		}
	}

	if (Index != (MaxIndex - 1)) {
		/* all registers are written into */
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
 * This function initializes IIC controller and gets config parameters.
 *
 * @return	XST_SUCCESS if successful else XST_FAILURE.
 *
 * @note	None.
 *
 *****************************************************************************/
int InitIIC(void) {
	int Status;
	XIic_Config *ConfigPtr; /* Pointer to configuration data */

	memset(&IicSensor, 0, sizeof(XIic));

	/*
	 * Initialize the IIC driver so that it is ready to use.
	 */
	ConfigPtr = XIic_LookupConfig(IIC_SENSOR_DEV_ID);
	if (ConfigPtr == NULL) {
		return XST_FAILURE;
	}

	Status = XIic_CfgInitialize(&IicSensor, ConfigPtr,
					ConfigPtr->BaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	return Status;
}


int InitDVSIIC(void) {
	int Status;
	XIicPs_Config * ConfigPtr;
	memset(&IicPsInstance, 0, sizeof(IicPsInstance));

	/*
	 * Initialize the IIC driver so that it is ready to use.
	 */
	ConfigPtr = XIicPs_LookupConfig(DVS_IIC_DEVICE_ID);
	if (ConfigPtr == NULL) {
		return XST_FAILURE;
	}

	Status = XIicPs_CfgInitialize(&IicPsInstance, ConfigPtr,
					ConfigPtr->BaseAddress);
	if(Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	return Status;
}

int DVSWriteData(u16 ByteCount) {
	DVSIICTotalErrorCount = 0;
	DVSIICTransmitComplete = FALSE;
	DVSIICTotalErrorCount = 0;

	while (XIicPs_BusIsBusy(&IicPsInstance));

	/* Send the data */
	XIicPs_MasterSend(&IicPsInstance, DVSIICWriteBuf, ByteCount, DVS_ADDR);
	/* Wait till the transmission is completed */
	while (DVSIICTransmitComplete == FALSE) {
		if (0 != DVSIICTotalErrorCount) {
			return XST_FAILURE;
		}
	}

	return XST_SUCCESS;
}

int ProgramDVSSensor(void) {
	int Status;
	u16 DeviceId;
	u32 Index;
	u32 MaxIndex = length_DVS_regs;
	DVSIicAddr = DVS_ADDR;
	struct regval_list * sensor_cfg = DVS_regs;


	/*
	 * 1. Setup the handlers for the IIC that will be called from the
	 * interrupt context when data has been sent and received, specify a
	 * pointer to the IIC driver instance as the callback reference so
	 * the handlers are able to access the instance data.
	 */
	XIicPs_SetStatusHandler(&IicPsInstance, (void *) &IicPsInstance, DVSIICStatusHandler);

	/*
	 * 2. Set the IIC serial clock rate.
	 */
	XIicPs_SetSClk(&IicPsInstance, 400000);

	Status = MuxInitChannel(DVS_MUX_ADDRESS, DVS_MUX_CHANNEL);
	if (Status != XST_SUCCESS){
		xil_printf("IIC MUX Channel set error at 0X%x\r\n", DVS_MUX_CHANNEL);
		return XST_FAILURE;
	}

	// 3. program dvs sensor
	for (Index = 0; Index < (MaxIndex); Index ++) {
		DVSIICWriteBuf[0] = sensor_cfg[Index].Address >> 8;
		DVSIICWriteBuf[1] = sensor_cfg[Index].Address;
		DVSIICWriteBuf[2] = sensor_cfg[Index].Data;

		Status = DVSWriteData(3);

		if (Status != XST_SUCCESS) {
			xil_printf("Error in Writing line: %d,\r\n register address: %x,\r\n status = %x \r\n", Index, sensor_cfg[Index].Address, Status);
			break;
		}
	}

	if (Index != (MaxIndex)) {
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}


int SetupIicDVSInterruptSystem(void)
{
	int Status;
	XScuGic_Config *IntcConfig;

	Xil_ExceptionInit();

	IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
	if (NULL == IntcConfig){
		return XST_FAILURE;
	}

	Status = XScuGic_CfgInitialize(&InterruptController, IntcConfig,
					IntcConfig->CpuBaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Connect the interrupt controller interrupt handler to the hardware
	 * interrupt handling logic in the processor.
	 */
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_IRQ_INT,
				(Xil_ExceptionHandler)XScuGic_InterruptHandler,
				&InterruptController);

	/*
	 * Connect the device driver handler that will be called when an
	 * interrupt for the device occurs, the handler defined above performs
	 * the specific interrupt processing for the device.
	 */
	Status = XScuGic_Connect(&InterruptController, XPAR_XIICPS_1_INTR,
			(Xil_InterruptHandler)XIicPs_MasterInterruptHandler,
			(void *)&IicPsInstance);
	if (Status != XST_SUCCESS) {
		return Status;
	}

	/*
	 * Enable the interrupt for the Iic device.
	 */
	XScuGic_Enable(&InterruptController, XPAR_XIICPS_1_INTR);


	/*
	 * Enable interrupts in the Processor.
	 */
	Xil_ExceptionEnable();

	return XST_SUCCESS;
}

int StartDVSSensor(void) {
	int Status;
	int Delay;

	// Start the rstn of DVS
	Status = XGpio_Initialize(&DVS_rstn_Gpio, GPIO_DEVICE_ID);
	if (Status != XST_SUCCESS) {
		xil_printf("Gpio Initialization Failed\r\n");
		return XST_FAILURE;
	}
	XGpio_SetDataDirection(&DVS_rstn_Gpio, 2, ~DVS_RSTN); // => sometimes not working??
	for (Delay = 0; Delay < 10000000; Delay++);
	XGpio_DiscreteWrite(&DVS_rstn_Gpio, 2, DVS_RSTN);

	return XST_SUCCESS;
}

u32 InitializeDphy(void)
{
	u32 Status = 0;
	XDphy_Config *DphyCfgPtr = NULL;
	DphyCfgPtr = XDphy_LookupConfig(XDPHY_DEVICE_ID);
	if (!DphyCfgPtr) {
		xil_printf("Dphy Lookup Cfg failed\r\n");
		return XST_FAILURE;
	}
	Status = XDphy_CfgInitialize(&DphyRx, DphyCfgPtr,
			DphyCfgPtr->BaseAddr);
	if (Status != XST_SUCCESS) {
		xil_printf("Dphy Cfg init failed - %x\r\n", Status);
		return Status;
	}

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
 * This function sets send, receive and error handlers for IIC interrupts.
 *
 * @return	None.
 *
 * @note	None.
 *
 *****************************************************************************/
void SetupIICIntrHandlers(void) {
	/*
	 * Set the Handlers for transmit and reception.
	 */
	XIic_SetSendHandler(&IicSensor, &IicSensor,
				(XIic_Handler) SendHandler);
	XIic_SetRecvHandler(&IicSensor, &IicSensor,
				(XIic_Handler) ReceiveHandler);
	XIic_SetStatusHandler(&IicSensor, &IicSensor,
				(XIic_StatusHandler) StatusHandler);

}

/*****************************************************************************/
/**
 * This function programs colour space converter with the given width and height
 *
 * @param	width is Hsize of a packet in pixels.
 * @param	height is number of lines of a packet.
 *
 * @return	None.
 *
 * @note	None.
 *
 *****************************************************************************/
void ConfigCSC(u32 width , u32 height)
{
//	Xil_Out32((VPROCSSCSC_BASE + 0x0010), 0x0   );
//	Xil_Out32((VPROCSSCSC_BASE + 0x0018), 0x0   );
//	Xil_Out32((VPROCSSCSC_BASE + 0x0050), 0x1000);
//	Xil_Out32((VPROCSSCSC_BASE + 0x0058), 0x0   );
//	Xil_Out32((VPROCSSCSC_BASE + 0x0060), 0x0   );
//	Xil_Out32((VPROCSSCSC_BASE + 0x0068), 0x0   );
//	Xil_Out32((VPROCSSCSC_BASE + 0x0070), 0x1000);
//	Xil_Out32((VPROCSSCSC_BASE + 0x0078), 0x0   );
//	Xil_Out32((VPROCSSCSC_BASE + 0x0080), 0x0   );
//	Xil_Out32((VPROCSSCSC_BASE + 0x0088), 0x0   );
//	Xil_Out32((VPROCSSCSC_BASE + 0x0090), 0x1000);
//	Xil_Out32((VPROCSSCSC_BASE + 0x0098), 0x0   );
//	Xil_Out32((VPROCSSCSC_BASE + 0x00a0), 0x0   );
//	Xil_Out32((VPROCSSCSC_BASE + 0x00a8), 0x0   );
//	Xil_Out32((VPROCSSCSC_BASE + 0x00b0), 0x0   );
//	Xil_Out32((VPROCSSCSC_BASE + 0x00b8), 0x0   );
//	Xil_Out32((VPROCSSCSC_BASE + 0x0020), width );
//	Xil_Out32((VPROCSSCSC_BASE + 0x0028), height );
//	Xil_Out32((VPROCSSCSC_BASE + 0x0000), 0x81  );

//	Xil_Out32((VPROCSSCSC_BASE + 0x0010), 0x0   );
//	Xil_Out32((VPROCSSCSC_BASE + 0x0018), 0x0   );
//	Xil_Out32((VPROCSSCSC_BASE + 0x0050), 0x24cc);
//	Xil_Out32((VPROCSSCSC_BASE + 0x0058), 0xf9c3);
//	Xil_Out32((VPROCSSCSC_BASE + 0x0060), 0xf91f);
//	Xil_Out32((VPROCSSCSC_BASE + 0x0068), 0xf75c);
//	Xil_Out32((VPROCSSCSC_BASE + 0x0070), 0x1429);
//	Xil_Out32((VPROCSSCSC_BASE + 0x0078), 0xfae1);
//	Xil_Out32((VPROCSSCSC_BASE + 0x0080), 0xfdc3);
//	Xil_Out32((VPROCSSCSC_BASE + 0x0088), 0xfa66);
//	Xil_Out32((VPROCSSCSC_BASE + 0x0090), 0x23ae);
//	Xil_Out32((VPROCSSCSC_BASE + 0x0098), 0x26  );
//	Xil_Out32((VPROCSSCSC_BASE + 0x00a0), 0x3f  );
//	Xil_Out32((VPROCSSCSC_BASE + 0x00a8), 0x2b  );
//	Xil_Out32((VPROCSSCSC_BASE + 0x00b0), 0x0   );
//	Xil_Out32((VPROCSSCSC_BASE + 0x00b8), 0xb2  );
//	Xil_Out32((VPROCSSCSC_BASE + 0x0020), width );
//	Xil_Out32((VPROCSSCSC_BASE + 0x0028), height );
//	Xil_Out32((VPROCSSCSC_BASE + 0x0000), 0x81  );

	Xil_Out32((VPROCSSCSC_BASE + 0x0010), 0x0   );
	Xil_Out32((VPROCSSCSC_BASE + 0x0018), 0x0   );
	Xil_Out32((VPROCSSCSC_BASE + 0x0050), 0x1010);
	Xil_Out32((VPROCSSCSC_BASE + 0x0058), 0x0   );
	Xil_Out32((VPROCSSCSC_BASE + 0x0060), 0x0   );
	Xil_Out32((VPROCSSCSC_BASE + 0x0068), 0x1F4 );
	Xil_Out32((VPROCSSCSC_BASE + 0x0070), 0x7D0 );
	Xil_Out32((VPROCSSCSC_BASE + 0x0078), 0x1F4 );
	Xil_Out32((VPROCSSCSC_BASE + 0x0080), 0x0   );
	Xil_Out32((VPROCSSCSC_BASE + 0x0088), 0x160 );
	Xil_Out32((VPROCSSCSC_BASE + 0x0090), 0xED8 );
	Xil_Out32((VPROCSSCSC_BASE + 0x0098), 0x10  );
	Xil_Out32((VPROCSSCSC_BASE + 0x00a0), 0x10  );
	Xil_Out32((VPROCSSCSC_BASE + 0x00a8), 0x10  );
	Xil_Out32((VPROCSSCSC_BASE + 0x00b0), 0x0   );
	Xil_Out32((VPROCSSCSC_BASE + 0x00b8), 0xff  );
	Xil_Out32((VPROCSSCSC_BASE + 0x0020), width );
	Xil_Out32((VPROCSSCSC_BASE + 0x0028), height );
	Xil_Out32((VPROCSSCSC_BASE + 0x0000), 0x81  );



//	xil_printf("address 0x0010: %x\r\n", Xil_In32((VPROCSSCSC_BASE + 0x0010)));
//	xil_printf("address 0x0010: %x\r\n", Xil_In32((VPROCSSCSC_BASE + 0x0018)));
//	xil_printf("address 0x0010: %x\r\n", Xil_In32((VPROCSSCSC_BASE + 0x0050)));
//	xil_printf("address 0x0010: %x\r\n", Xil_In32((VPROCSSCSC_BASE + 0x0058)));
//	xil_printf("address 0x0010: %x\r\n", Xil_In32((VPROCSSCSC_BASE + 0x0060)));
//	xil_printf("address 0x0010: %x\r\n", Xil_In32((VPROCSSCSC_BASE + 0x0068)));
//	xil_printf("address 0x0010: %x\r\n", Xil_In32((VPROCSSCSC_BASE + 0x0070)));
//	xil_printf("address 0x0010: %x\r\n", Xil_In32((VPROCSSCSC_BASE + 0x0078)));
//	xil_printf("address 0x0010: %x\r\n", Xil_In32((VPROCSSCSC_BASE + 0x0080)));
//	xil_printf("address 0x0010: %x\r\n", Xil_In32((VPROCSSCSC_BASE + 0x0088)));
//	xil_printf("address 0x0010: %x\r\n", Xil_In32((VPROCSSCSC_BASE + 0x0090)));
//	xil_printf("address 0x0010: %x\r\n", Xil_In32((VPROCSSCSC_BASE + 0x0098)));
//	xil_printf("address 0x0010: %x\r\n", Xil_In32((VPROCSSCSC_BASE + 0x00a0)));
//	xil_printf("address 0x0010: %x\r\n", Xil_In32((VPROCSSCSC_BASE + 0x00a8)));
//	xil_printf("address 0x0010: %x\r\n", Xil_In32((VPROCSSCSC_BASE + 0x00b0)));
//	xil_printf("address 0x0010: %x\r\n", Xil_In32((VPROCSSCSC_BASE + 0x00b8)));
//	xil_printf("address 0x0010: %x\r\n", Xil_In32((VPROCSSCSC_BASE + 0x0020)));
//	xil_printf("address 0x0010: %x\r\n", Xil_In32((VPROCSSCSC_BASE + 0x0028)));
//	xil_printf("address 0x0010: %x\r\n", Xil_In32((VPROCSSCSC_BASE + 0x0000)));
}

/*****************************************************************************/
/**
 * This function programs colour space converter with the given width and height
 *
 * @param	width is Hsize of a packet in pixels.
 * @param	height is number of lines of a packet.
 *
 * @return	None.
 *
 * @note	None.
 *
 *****************************************************************************/
void ConfigGammaLut(u32 width , u32 height)
{
	u32 count;
	Xil_Out32((VGAMMALUT_BASE + 0x10), width );
	Xil_Out32((VGAMMALUT_BASE + 0x18), height );
	Xil_Out32((VGAMMALUT_BASE + 0x20), 0x0   );

	for(count=0; count < 0x200; count += 2)
	{
		Xil_Out16((VGAMMALUT_BASE + 0x800 + count), count/2 );
	}

	for(count=0; count < 0x200; count += 2)
	{
		Xil_Out16((VGAMMALUT_BASE + 0x1000 + count), count/2 );
	}

	for(count=0; count < 0x200; count += 2)
	{
		Xil_Out16((VGAMMALUT_BASE + 0x1800 + count), count/2 );
	}

	Xil_Out32((VGAMMALUT_BASE + 0x00), 0x81   );
}

/*****************************************************************************/
/**
 * This function programs colour space converter with the given width and height
 *
 * @param	width is Hsize of a packet in pixels.
 * @param	height is number of lines of a packet.
 *
 * @return	None.
 *
 * @note	None.
 *
 *****************************************************************************/
void ConfigDemosaic(u32 width , u32 height)
{
	Xil_Out32((DEMOSAIC_BASE + 0x10), width );
	Xil_Out32((DEMOSAIC_BASE + 0x18), height );
	Xil_Out32((DEMOSAIC_BASE + 0x20), 0x0   );
	Xil_Out32((DEMOSAIC_BASE + 0x28), 0x0   );
	Xil_Out32((DEMOSAIC_BASE + 0x00), 0x81   );

}

/*****************************************************************************/
/**
 *
 * This function is called when a Frame Buffer Write Done has occurred.
 *
 * @param	CallbackRef is a callback function reference.
 *
 * @return	None.
 *
 * @note	None.
 *
 *****************************************************************************/
void FrmbufwrDoneCallback(void *CallbackRef) {
//	 xil_printf("  Wr Done  \r\n");
	int Status;

	Xil_Out8(frame_rdy[wr_ptr], 1);

	 if(wr_ptr == CIS_BUFFER_NUM - 1) {
		  wr_ptr = 0;
	 }
	 else{
		 wr_ptr = wr_ptr + 1;
	 }

	 XVFRMBUFWR_BUFFER_BASEADDR = frame_array[wr_ptr];

	 Status = XVFrmbufWr_SetBufferAddr(&frmbufwr,
                                               XVFRMBUFWR_BUFFER_BASEADDR);
	   if(Status != XST_SUCCESS) {
	     xil_printf("ERROR:: Unable to configure Frame Buffer \
                                                 Write buffer address\r\n");
	   }

	   /* Set Chroma Buffer Address for semi-planar color formats */
	   if ((Cfmt == XVIDC_CSF_MEM_Y_UV8) ||
               (Cfmt == XVIDC_CSF_MEM_Y_UV8_420) ||
	       (Cfmt == XVIDC_CSF_MEM_Y_UV10) ||
               (Cfmt == XVIDC_CSF_MEM_Y_UV10_420)) {
	     Status = XVFrmbufWr_SetChromaBufferAddr(&frmbufwr,
                             XVFRMBUFWR_BUFFER_BASEADDR+CHROMA_ADDR_OFFSET);
	     if(Status != XST_SUCCESS) {
	       xil_printf("ERROR:: Unable to configure Frame Buffer \
                                           Write chroma buffer address\r\n");
	     }
	   }

	   frm_cnt++;
}

/*****************************************************************************/
/**
 *
 * This function is called when a Frame Buffer Read Done has occurred.
 *
 * @param	CallbackRef is a callback function reference.
 *
 * @return	None.
 *
 * @note	None.
 *
 *****************************************************************************/
void FrmbufrdDoneCallback(void *CallbackRef) {
      frm_cnt1++;
}

/*****************************************************************************/
/**
 * This function disables Demosaic, GammaLut and VProcSS IPs
 *
 * @return	None.
 *
 * @note	None.
 *
 *****************************************************************************/
void DisableImageProcessingPipe(void)
{
	Xil_Out32((DEMOSAIC_BASE + 0x00), 0x0   );
	Xil_Out32((VGAMMALUT_BASE + 0x00), 0x0   );
	Xil_Out32((VPROCSSCSC_BASE + 0x00), 0x0  );

}

/*****************************************************************************/
/**
 * This function calculates the stride
 *
 * @returns stride in bytes
 *
 *****************************************************************************/
static u32 CalcStride(XVidC_ColorFormat Cfmt,
                      u16 AXIMMDataWidth,
                      XVidC_VideoStream *StreamPtr)
{
  u32 stride;
  int width = StreamPtr->Timing.HActive;
  u16 MMWidthBytes = AXIMMDataWidth/8;

  if ((Cfmt == XVIDC_CSF_MEM_Y_UV10) || (Cfmt == XVIDC_CSF_MEM_Y_UV10_420)
      || (Cfmt == XVIDC_CSF_MEM_Y10)) {
    // 4 bytes per 3 pixels (Y_UV10, Y_UV10_420, Y10)
    stride = ((((width*4)/3)+MMWidthBytes-1)/MMWidthBytes)*MMWidthBytes;
  }
  else if ((Cfmt == XVIDC_CSF_MEM_Y_UV8) || (Cfmt == XVIDC_CSF_MEM_Y_UV8_420)
           || (Cfmt == XVIDC_CSF_MEM_Y8)) {
    // 1 byte per pixel (Y_UV8, Y_UV8_420, Y8)
    stride = ((width+MMWidthBytes-1)/MMWidthBytes)*MMWidthBytes;
  }
  else if ((Cfmt == XVIDC_CSF_MEM_RGB8) || (Cfmt == XVIDC_CSF_MEM_YUV8)
           || (Cfmt == XVIDC_CSF_MEM_BGR8)) {
    // 3 bytes per pixel (RGB8, YUV8, BGR8)
     stride = (((width*3)+MMWidthBytes-1)/MMWidthBytes)*MMWidthBytes;
  }
  else {
    // 4 bytes per pixel
    stride = (((width*4)+MMWidthBytes-1)/MMWidthBytes)*MMWidthBytes;
  }

  return(stride);
}


int start_csi_cap_pipe(XVidC_VideoMode VideoMode)
{

    int stride;
    XVidC_VideoTiming const *TimingPtr;
	int  widthIn, heightIn;
	/* Local variables */
	XVidC_VideoMode  resIdOut;


	/* Default Resolution that to be displayed */
	Pipeline_Cfg.VideoMode = VideoMode ;

	/* Select the sensor configuration based on resolution and lane */
	switch (Pipeline_Cfg.VideoMode) {

		case XVIDC_VM_1920x1080_30_P:
		case XVIDC_VM_1920x1080_60_P:
	        widthIn  = 1920;
	        heightIn = 1080;
			break;

		case XVIDC_VM_3840x2160_30_P:
		case XVIDC_VM_3840x2160_60_P:
	        widthIn  = 3840;
	        heightIn = 2160;
			break;

		case XVIDC_VM_1280x720_60_P:
			widthIn  = 1280;
			heightIn = 720;
			break;

		default:
		    xil_printf("Invalid Input Selection ");
			return XST_FAILURE;
			break;

	}


    usleep(1000);

	resIdOut = XVidC_GetVideoModeId(widthIn, heightIn, XVIDC_FR_60HZ,
					FALSE);

	StreamOut.VmId = resIdOut;
	StreamOut.Timing.HActive = widthIn;
	StreamOut.Timing.VActive = heightIn;
	StreamOut.ColorFormatId = XVIDC_CSF_RGB;
	StreamOut.FrameRate = XVIDC_FR_60HZ;
	StreamOut.IsInterlaced = 0;


	/* Setup a default stream */
	StreamOut.ColorDepth = (XVidC_ColorDepth)frmbufwr.FrmbufWr.Config.MaxDataWidth;
	StreamOut.PixPerClk = (XVidC_PixelsPerClock)frmbufwr.FrmbufWr.Config.PixPerClk;

	VidStream.PixPerClk =(XVidC_PixelsPerClock)frmbufwr.FrmbufWr.Config.PixPerClk;
	VidStream.ColorDepth = (XVidC_ColorDepth)frmbufwr.FrmbufWr.Config.MaxDataWidth;
	Cfmt = XVIDC_CSF_MEM_RGB8 ;
	VidStream.ColorFormatId = StreamOut.ColorFormatId;
	VidStream.VmId = StreamOut.VmId;

	/* Get mode timing parameters */
	TimingPtr = XVidC_GetTimingInfo(VidStream.VmId);
	VidStream.Timing = *TimingPtr;
	VidStream.FrameRate = XVidC_GetFrameRate(VidStream.VmId);
	xil_printf("\r\n********************************************\r\n");
	xil_printf("Test Input Stream: %s (%s)\r\n",
	           XVidC_GetVideoModeStr(VidStream.VmId),
	           XVidC_GetColorFormatStr(Cfmt));
	xil_printf("********************************************\r\n");
	stride = CalcStride(Cfmt ,
	                         frmbufwr.FrmbufWr.Config.AXIMMDataWidth,
	                               &StreamOut);
	 xil_printf(" Stride is calculated %d \r\n",stride);
	ConfigFrmbuf(stride, Cfmt, &StreamOut);
	xil_printf(" Frame Buffer Setup is Done\r\n");


	XV_frmbufwr_EnableAutoRestart(&frmbufwr.FrmbufWr);
	XVFrmbufWr_Start(&frmbufwr);


	xil_printf(TXT_RST);

      return 0;

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

int config_csi_cap_path(){
	u32 Status;
	/* Initialize Frame Buffer Write */
	 Status =  XVFrmbufWr_Initialize(&frmbufwr, XPAR_XV_FRMBUFWR_0_DEVICE_ID);
	if (Status != XST_SUCCESS) {
		xil_printf(TXT_RED "Frame Buffer Write Init failed status = %x.\r\n"
				 TXT_RST, Status);
		return XST_FAILURE;
	}


	Status = XVFrmbufWr_SetCallback(&frmbufwr,
	                                    XVFRMBUFWR_HANDLER_DONE,
	                                    (void *)FrmbufwrDoneCallback,
	                                    (void *) &frmbufwr);
	if (Status != XST_SUCCESS) {
		xil_printf(TXT_RED "Frame Buffer Write Call back  failed status = %x.\r\n"
					TXT_RST, Status);
		return XST_FAILURE;
	}

	print("\r\n\r\n--------------------------------\r\n");

	return 0;
}

int DVSDma_WriteSetup(XAxiDma * AxiDmaInstPtr) {
	XAxiDma_BdRing * RxRingPtr;
	int Status;
	XAxiDma_Bd BdTemplate;
	XAxiDma_Bd *BdPtr;
	XAxiDma_Bd *BdCurPtr;
	int BdCount;
	UINTPTR RxBufferPtr;
	int Index;

	RxRingPtr = XAxiDma_GetRxRing(AxiDmaInstPtr);

	XAxiDma_BdRingIntDisable(RxRingPtr, XAXIDMA_IRQ_ALL_MASK);

	BdCount = XAxiDma_BdRingCntCalc(XAXIDMA_BD_MINIMUM_ALIGNMENT,
				RX_BD_SPACE_HIGH - RX_BD_SPACE_BASE + 1);

	Status = XAxiDma_BdRingCreate(RxRingPtr, RX_BD_SPACE_BASE,
				RX_BD_SPACE_BASE,
				XAXIDMA_BD_MINIMUM_ALIGNMENT, BdCount);
	if (Status != XST_SUCCESS) {
		xil_printf("Rx bd create failed with %d\r\n", Status);
		return XST_FAILURE;
	}

	/*
	 * Setup a BD template for the Rx channel. Then copy it to every Rx Bd
	 */
	XAxiDma_BdClear(&BdTemplate);
	Status = XAxiDma_BdRingClone(RxRingPtr, &BdTemplate);
	if (Status != XST_SUCCESS) {
		xil_printf("Rx bd clone failed with %d\r\n", Status);
		return XST_FAILURE;
	}

	Status = XAxiDma_BdRingAlloc(RxRingPtr, DVS_BUFFER_NUM, &BdPtr);
	if (Status != XST_SUCCESS) {
		xil_printf("Rx bd alloc failed with %d\r\n", Status);
		return XST_FAILURE;
	}

	BdCurPtr = BdPtr;
	RxBufferPtr = dvs_frame_array[dvs_wr_ptr];

	for (Index = 0; Index < DVS_BUFFER_NUM; Index++) {
		Status = XAxiDma_BdSetBufAddr(BdCurPtr, RxBufferPtr);
		if (Status != XST_SUCCESS) {
			xil_printf("Rx set buffer addr %x on BD %x failed %d \r\n",
			(unsigned int)RxBufferPtr,
			(UINTPTR)BdCurPtr, Status);

			return XST_FAILURE;
		}
		Status = XAxiDma_BdSetLength(BdCurPtr, BD_LEN,
				RxRingPtr->MaxTransferLen);
		if (Status != XST_SUCCESS) {
			xil_printf("Rx set length %d on BD %x failed %d\r\n",
					BD_LEN, (UINTPTR) BdCurPtr, Status);
			return XST_FAILURE;
		}

		XAxiDma_BdSetCtrl(BdCurPtr, 0);

		XAxiDma_BdSetId(BdCurPtr, RxBufferPtr);

		RxBufferPtr += BD_LEN;
		BdCurPtr = (XAxiDma_Bd *)XAxiDma_BdRingNext(RxRingPtr, BdCurPtr);
	}
	/*
	 * Set the coalescing threshold
	 */
	Status = XAxiDma_BdRingSetCoalesce(RxRingPtr, COALESCING_COUNT,
			DELAY_TIMER_COUNT);
	if (Status != XST_SUCCESS) {
		xil_printf("Rx set coalesce failed with %d\r\n", Status);
		return XST_FAILURE;
	}

	Status = XAxiDma_BdRingToHw(RxRingPtr, DVS_BUFFER_NUM, BdPtr);
	if (Status != XST_SUCCESS) {
		xil_printf("Rx ToHw failed with %d\r\n", Status);
		return XST_FAILURE;
	}
}

int start_dvs_cap_pipe() {
	int Status;

	XAxiDma_BdRing* RxRingPtr = XAxiDma_GetRxRing(&DVSDma);
	XAxiDma_BdRingIntEnable(RxRingPtr, XAXIDMA_IRQ_ALL_MASK);
	// Enable Cyclic DMA mode
	XAxiDma_BdRingEnableCyclicDMA(RxRingPtr);
	XAxiDma_SelectCyclicMode(&DVSDma, XAXIDMA_DEVICE_TO_DMA, 1);

	/* Start RX DMA channel */
	Status = XAxiDma_BdRingStart(RxRingPtr);
	if (Status != XST_SUCCESS) {
		xil_printf("Rx start BD ring failed with %d\r\n", Status);
		return XST_FAILURE;
	}

	xil_printf("Started DVS CAP PIPE\r\n");
	return 0;
}

void config_dvs_cap_path(){
	int Status;
	XAxiDma_Config *Dma_Config;

	Dma_Config = XAxiDma_LookupConfig(XPAR_DVS_STREAM_AXI_DMA_0_DEVICE_ID);
	if (!Dma_Config) {
		xil_printf("No config found for %d\r\n", XPAR_DVS_STREAM_AXI_DMA_0_DEVICE_ID);
		return XST_FAILURE;
	}
	Status = XAxiDma_CfgInitialize(&DVSDma, Dma_Config);
	if (Status != XST_SUCCESS) {
		xil_printf("Initialization failed %d\r\n", Status);
		return XST_FAILURE;
	}

	/* Setup the write channel
	 */
	Status = DVSDma_WriteSetup(&DVSDma);
	if (Status != XST_SUCCESS) {
		xil_printf("Write channel setup failed %d\r\n", Status);
		return XST_FAILURE;
	}

	xil_printf("DVS Stream DMA cap path setup finished \r\n");
	return 0;
}




/*****************************************************************************/
/**
 * This function Initializes Image Processing blocks wrt to selected resolution
 *
 * @return	None.
 *
 * @note	None.
 *
 *****************************************************************************/
void InitImageProcessingPipe(void)
{
	u32 width, height;

	switch (Pipeline_Cfg.VideoMode) {

		case XVIDC_VM_3840x2160_30_P:
		case XVIDC_VM_3840x2160_60_P:
			width = 3840;
			height = 2160;
			break;
		case XVIDC_VM_1920x1080_30_P:
		case XVIDC_VM_1920x1080_60_P:
			width = 1920;
			height = 1080;
			break;
		case XVIDC_VM_1280x720_60_P:
			width = 1280;
			height = 720;
			break;

		default:
			xil_printf("InitDemosaicGammaCSC - Invalid Video Mode");
			xil_printf("\n\r");
			return;
	}
	ConfigCSC(width, height);
	ConfigGammaLut(width, height);
	ConfigDemosaic(width, height);

}

/*****************************************************************************/
/**
 * This function enables MIPI CSI IP
 *
 * @return	None.
 *
 * @note	None.
 *
 *****************************************************************************/
void EnableCSI(void)
{
	XCsiSs_Reset(&CsiRxSs);
	XCsiSs_Configure(&CsiRxSs, (Pipeline_Cfg.ActiveLanes), 0);
	XCsiSs_Activate(&CsiRxSs, XCSI_ENABLE);

	usleep(1000000);
}

/*****************************************************************************/
/**
 * This function disables MIPI CSI IP
 *
 * @return	None.
 *
 * @note	None.
 *
 *****************************************************************************/
void DisableCSI(void)
{
	usleep(1000000);
	XCsiSs_Reset(&CsiRxSs);
}

/*****************************************************************************/
/**
 * This function starts camera sensor to transmit captured video
 *
 * @return	XST_SUCCESS if successful else XST_FAILURE.
 *
 * @note		None.
 *
 *****************************************************************************/
int StartSensor(void)
{
	int Status;

	if (Pipeline_Cfg.CameraPresent == FALSE) {
		xil_printf("%s - No camera present\r\n", __func__);
		return XST_SUCCESS;
	}

	usleep(1000000);
	WriteBuffer[0] = 0x30;
	WriteBuffer[1] = 0x00;
	WriteBuffer[2] = 0x00;
	Status = SensorWriteData(3);
	usleep(1000000);
	WriteBuffer[0] = 0x30;
	WriteBuffer[1] = 0x3E;
	WriteBuffer[2] = 0x02;
	Status = SensorWriteData(3);
	usleep(1000000);
	WriteBuffer[0] = 0x30;
	WriteBuffer[1] = 0xF4;
	WriteBuffer[2] = 0x00;
	Status = SensorWriteData(3);
	usleep(1000000);
	WriteBuffer[0] = 0x30;
	WriteBuffer[1] = 0x18;
	WriteBuffer[2] = 0xA2;
	Status = SensorWriteData(3);

	if (Status != XST_SUCCESS) {
		xil_printf("Error: in Writing entry status = %x \r\n", Status);
		xil_printf("%s - Failed\r\n", __func__);
		return XST_FAILURE;
	}

	return Status;
}


/*****************************************************************************/
/**
 * This function initializes and configures VProcSS IP for scalar mode with the
 * given input and output width and height values.
 *
 * @param	count is a flag value to initialize IP only once.
 *
 * @return	None.
 *
 * @note	None.
 *
 *****************************************************************************/
void InitVprocSs_Scaler(int count)
{
	XVprocSs_Config* p_vpss_cfg;
	int status;
	int widthIn, heightIn, widthOut, heightOut;

	/* Fixed output to DSI */
	widthOut = 1920;
	heightOut = 1080;

	/* Local variables */
	XVidC_VideoMode resIdIn, resIdOut;
	XVidC_VideoStream StreamIn, StreamOut;

	if (Pipeline_Cfg.VideoMode == XVIDC_VM_1280x720_60_P) {
		widthIn = 1280;
		heightIn = 720;
		StreamIn.FrameRate = XVIDC_FR_60HZ;
	}

	if (Pipeline_Cfg.VideoMode == XVIDC_VM_1920x1080_30_P) {
		widthIn = 1920;
		heightIn = 1080;
		StreamIn.FrameRate = XVIDC_FR_60HZ;
	}

	if (Pipeline_Cfg.VideoMode == XVIDC_VM_1920x1080_60_P) {
		widthIn = 1920;
		heightIn = 1080;
		StreamIn.FrameRate = XVIDC_FR_60HZ;
	}

	if (Pipeline_Cfg.VideoMode == XVIDC_VM_3840x2160_30_P) {
		widthIn = 3840;
		heightIn = 2160;
		StreamIn.FrameRate = XVIDC_FR_60HZ;
	}

	if (Pipeline_Cfg.VideoMode == XVIDC_VM_3840x2160_60_P) {
		widthIn = 3840;
		heightIn = 2160;
		StreamIn.FrameRate = XVIDC_FR_60HZ;
	}

	if (count) {
		p_vpss_cfg = XVprocSs_LookupConfig(XVPROCSS_DEVICE_ID);
		if (p_vpss_cfg == NULL) {
			xil_printf("ERROR! Failed to find VPSS-based scaler.");
			xil_printf("\n\r");
			return;
		}
		status = XVprocSs_CfgInitialize(&scaler_new_inst, p_vpss_cfg,
				p_vpss_cfg->BaseAddress);
		if (status != XST_SUCCESS) {
			xil_printf("ERROR! Failed to initialize VPSS-based ");
			xil_printf("scaler.\n\r");
			return;
		}
	}

	XVprocSs_Stop(&scaler_new_inst);

	/* Get resolution ID from frame size */
	resIdIn = XVidC_GetVideoModeId(widthIn, heightIn, StreamIn.FrameRate,
			FALSE);

	/* Setup Video Processing Subsystem */
	StreamIn.VmId = resIdIn;
	StreamIn.Timing.HActive = widthIn;
	StreamIn.Timing.VActive = heightIn;
	StreamIn.ColorFormatId = XVIDC_CSF_RGB;
	StreamIn.ColorDepth = scaler_new_inst.Config.ColorDepth;
	StreamIn.PixPerClk = scaler_new_inst.Config.PixPerClock;
	StreamIn.IsInterlaced = 0;

	status = XVprocSs_SetVidStreamIn(&scaler_new_inst, &StreamIn);
	if (status != XST_SUCCESS) {
		xil_printf("Unable to set input video stream parameters \
				correctly\r\n");
		return;
	}

	/* Get resolution ID from frame size */
	resIdOut = XVidC_GetVideoModeId(widthOut, heightOut, XVIDC_FR_60HZ,
					FALSE);

	if (resIdOut != XVIDC_VM_1920x1200_60_P) {
	xil_printf("resIdOut %d doesn't match XVIDC_VM_1920x1200_60_P \r\n", resIdOut);
	}

	StreamOut.VmId = resIdOut;
	StreamOut.Timing.HActive = widthOut;
	StreamOut.Timing.VActive = heightOut;
	StreamOut.ColorFormatId = XVIDC_CSF_RGB;
	StreamOut.ColorDepth = scaler_new_inst.Config.ColorDepth;
	StreamOut.PixPerClk = scaler_new_inst.Config.PixPerClock;
	StreamOut.FrameRate = XVIDC_FR_60HZ;
	StreamOut.IsInterlaced = 0;

	XVprocSs_SetVidStreamOut(&scaler_new_inst, &StreamOut);
	if (status != XST_SUCCESS) {
		xil_printf("Unable to set output video stream parameters correctly\r\n");
		return;
	}

	status = XVprocSs_SetSubsystemConfig(&scaler_new_inst);
	if (status != XST_SUCCESS) {
		xil_printf("XVprocSs_SetSubsystemConfig failed %d\r\n", status);
		return;
	}

}

/*****************************************************************************/
/**
 * This function resets VProcSS_scalar IP.
 *
 * @return	None.
 *
 * @note	None.
 *
 *****************************************************************************/
void ResetVprocSs_Scaler(void)
{
	XVprocSs_Reset(&scaler_new_inst);
}

/*****************************************************************************/
/**
 * This function stops VProc_SS scalar IP.
 *
 * @return	None.
 *
 * @note	None.
 *
 *****************************************************************************/
void DisableScaler(void)
{
	XVprocSs_Stop(&scaler_new_inst);
}

/*****************************************************************************/
/**
 * This function initializes MIPI CSI2 RX SS and gets config parameters.
 *
 * @return	XST_SUCCESS if successful or else XST_FAILURE.
 *
 * @note	None.
 *
 *****************************************************************************/
u32 InitializeCsiRxSs(void)
{
	u32 Status = 0;
	XCsiSs_Config *CsiRxSsCfgPtr = NULL;

	CsiRxSsCfgPtr = XCsiSs_LookupConfig(XCSIRXSS_DEVICE_ID);
	if (!CsiRxSsCfgPtr) {
		xil_printf("CSI2RxSs LookupCfg failed\r\n");
		return XST_FAILURE;
	}

	Status = XCsiSs_CfgInitialize(&CsiRxSs, CsiRxSsCfgPtr,
			CsiRxSsCfgPtr->BaseAddr);

	if (Status != XST_SUCCESS) {
		xil_printf("CsiRxSs Cfg init failed - %x\r\n", Status);
		return Status;
	}

	return XST_SUCCESS;
}

/*****************************************************************************/
/**
 * This function returns selected colour depth of MIPI CSI2 RX SS.
 *
 * @param	CsiDataFormat is video data format
 *
 * @return	ColorDepth returns colour depth value.
 *
 * @note	None.
 *
 *****************************************************************************/
XVidC_ColorDepth GetColorDepth(u32 CsiDataFormat)
{
	XVidC_ColorDepth ColorDepth;

	switch (CsiDataFormat) {
		case XCSI_PXLFMT_RAW8:
			xil_printf("Color Depth = RAW8\r\n");
			ColorDepth = XVIDC_BPC_8;
			break;
		case XCSI_PXLFMT_RAW10:
			xil_printf("Color Depth = RAW10\r\n");
			ColorDepth = XVIDC_BPC_10;
			break;
		case XCSI_PXLFMT_RAW12:
			xil_printf("Color Depth = RAW12\r\n");
			ColorDepth = XVIDC_BPC_12;
			break;
		default:
			ColorDepth = XVIDC_BPC_UNKNOWN;
			break;
	}

	return ColorDepth;
}

/*****************************************************************************/
/**
 * This function sets colour depth value getting from MIPI CSI2 RX SS
 *
 * @return	None.
 *
 * @note	None.
 *
 *****************************************************************************/
void SetColorDepth(void)
{
	Pipeline_Cfg.ColorDepth = GetColorDepth(CsiRxSs.Config.PixelFormat);
	print(TXT_GREEN);
	xil_printf("Setting Color Depth = %d bpc\r\n", Pipeline_Cfg.ColorDepth);
	print(TXT_RST);
	return;
}




/*****************************************************************************/
/**
 * This function prints the video pipeline information.
 *
 * @return	None.
 *
 * @note	None.
 *
 *****************************************************************************/
void PrintPipeConfig(void)
{
	xil_printf(TXT_YELLOW);
	xil_printf("-------------Current Pipe Configuration-------------\r\n");

	xil_printf("Color Depth	: ");
	switch (Pipeline_Cfg.ColorDepth) {
		case XVIDC_BPC_8:
			xil_printf("RAW8");
			break;
		case XVIDC_BPC_10:
			xil_printf("RAW10");
			break;
		case XVIDC_BPC_12:
			xil_printf("RAW12");
			break;
		default:
			xil_printf("Invalid");
			break;
	}
	xil_printf("\r\n");

	xil_printf("Source 		: %s\r\n",
			(Pipeline_Cfg.VideoSrc == XVIDSRC_SENSOR) ?
			"Sensor" : "Test Pattern Generator");
	xil_printf("Destination 	: %s\r\n",
			(Pipeline_Cfg.VideoDestn == XVIDDES_HDMI) ?
			"HDMI" : "DSI");

	xil_printf("Resolution	: ");
	switch (Pipeline_Cfg.VideoMode) {
		case XVIDC_VM_1280x720_60_P:
			xil_printf("1280x720@60");
			break;
		case XVIDC_VM_1920x1080_30_P:
			xil_printf("1920x1080@30");
			break;
		case XVIDC_VM_1920x1080_60_P:
			xil_printf("1920x1080@60");
			break;
		case XVIDC_VM_3840x2160_30_P:
			xil_printf("3840x2160@30");
			break;
		case XVIDC_VM_3840x2160_60_P:
			xil_printf("3840x2160@60");
			break;

		default:
			xil_printf("Invalid");
			break;
	}
	xil_printf("\r\n");

	xil_printf("Lanes		: ");
	switch (Pipeline_Cfg.ActiveLanes) {
		case 1:
			xil_printf("1");
			break;
		case 2:
			xil_printf("2");
			break;
		case 4:
			xil_printf("4");
			break;
		default:
			xil_printf("Invalid");
			break;
	}
	xil_printf(" Lanes\r\n");

	xil_printf("-----------------------------------------------------\r\n");

	xil_printf(TXT_RST);
	return;
}

void CsiRxPrintRegStatus(void) {
	u32 Status = 0;

	xil_printf(TXT_RST);
	XDphy_Config *DphyCfgPtr = NULL;

	DphyCfgPtr = XDphy_LookupConfig(XDPHY_DEVICE_ID);
	if (!DphyCfgPtr) {
		xil_printf("Dphy LookupCfg failed\r\n");
		return XST_FAILURE;
	}

	xil_printf("\r\n\r\n");
	xil_printf(TXT_GREEN);
	xil_printf("-----------Dphy Register value-------------\r\n");
	u32 dphy_offset_list[] = {
			XDPHY_CTRL_REG_OFFSET,
			XDPHY_HSEXIT_IDELAY_REG_OFFSET,
			XDPHY_INIT_REG_OFFSET,
			XDPHY_WAKEUP_REG_OFFSET,
			XDPHY_HSTIMEOUT_REG_OFFSET,
			XDPHY_ESCTIMEOUT_REG_OFFSET,
			XDPHY_CLSTATUS_REG_OFFSET,
			XDPHY_DL0STATUS_REG_OFFSET,
			XDPHY_DL1STATUS_REG_OFFSET,
			XDPHY_DL2STATUS_REG_OFFSET,
			XDPHY_DL3STATUS_REG_OFFSET,
			XDPHY_HSSETTLE_REG_OFFSET,
			XDPHY_IDELAY58_REG_OFFSET,
			XDPHY_HSSETTLE1_REG_OFFSET,
			XDPHY_HSSETTLE2_REG_OFFSET,
			XDPHY_HSSETTLE3_REG_OFFSET,
			XDPHY_HSSETTLE4_REG_OFFSET,
			XDPHY_HSSETTLE5_REG_OFFSET,
			XDPHY_HSSETTLE6_REG_OFFSET,
			XDPHY_HSSETTLE7_REG_OFFSET,
			XDPHY_DL4STATUS_REG_OFFSET,
			XDPHY_DL5STATUS_REG_OFFSET,
			XDPHY_DL6STATUS_REG_OFFSET,
			XDPHY_DL7STATUS_REG_OFFSET
	};

	for (int i= 0; i < sizeof(dphy_offset_list)/ sizeof(dphy_offset_list[0]); i++){
		u32 value = XDphy_ReadReg(XPAR_DVS_STREAM_MIPI_DPHY_0_BASEADDR, dphy_offset_list[i]);
		printf("Value of register offset 0X%x : 0X%x \r\n", dphy_offset_list[i], value);
	}
	xil_printf(TXT_RST);

}
