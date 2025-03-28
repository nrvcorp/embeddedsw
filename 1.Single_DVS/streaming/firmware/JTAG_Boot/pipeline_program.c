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
//#include "xiic.h"
#include "xil_exception.h"
#include "xil_cache.h"
#include "sensor_cfgs.h"
#include "xscugic.h"
#include "pipeline_program.h"

#include "xgpio.h"
//#include "xcsiss.h"
#include "xdphy.h"
#include <xvidc.h>

#define IIC_SENSOR_DEV_ID	XPAR_AXI_IIC_0_DEVICE_ID

#define PAGE_SIZE	16

#define EEPROM_TEST_START_ADDRESS	128

#define FRAME_BASE	0x10000000
#define CSIFrame	0x20000000
#define ScalerFrame	0x30000000

#define ACTIVE_LANES_1	1
#define ACTIVE_LANES_2	2
#define ACTIVE_LANES_3	3
#define ACTIVE_LANES_4	4

#define XCSIRXSS_DEVICE_ID	XPAR_CSISS_0_DEVICE_ID
#define XDPHY_DEVICE_ID		XPAR_MIPI_DPHY_0_DEVICE_ID
//XCsiSs CsiRxSs;
XDphy DphyRx;

XVidC_VideoMode VideoMode;
XVidC_VideoStream  VidStream;
XVidC_ColorFormat  Cfmt;
XVidC_VideoStream  StreamOut;

/**************************** Type Definitions *******************************/
typedef u8 AddressType;

u8 SensorIicAddr; /* Variable for storing Eeprom IIC address */

#define SENSOR_ADDR         (0x34>>1)	/* for IMX274 Vision */

#define IIC_MUX_ADDRESS 		0x75
#define IIC_EEPROM_CHANNEL		0x01	/* 0x08 */

//XIic IicSensor; /* The instance of the IIC device. */

u8 WriteBuffer[sizeof(AddressType) + PAGE_SIZE];
u8 ReadBuffer[PAGE_SIZE]; /* Read buffer for reading a page. */

extern XPipeline_Cfg Pipeline_Cfg;

void ConfigDemosaicResolution(XVidC_VideoMode videomode);
void DisableDemosaicResolution();



#define DDR_BASEADDR 0x10000000

#define BUFFER_BASEADDR0 (DDR_BASEADDR + (0x10000000))
#define BUFFER_BASEADDR1 (DDR_BASEADDR + (0x20000000))
#define BUFFER_BASEADDR2 (DDR_BASEADDR + (0x30000000))
#define BUFFER_BASEADDR3 (DDR_BASEADDR + (0x40000000))
#define BUFFER_BASEADDR4 (DDR_BASEADDR + (0x50000000))
#define CHROMA_ADDR_OFFSET   (0x01000000U)

u32 frame_array[5] = {BUFFER_BASEADDR0, BUFFER_BASEADDR1, BUFFER_BASEADDR2,
		              BUFFER_BASEADDR3, BUFFER_BASEADDR4};
u32 rd_ptr = 4 ;
u32 wr_ptr = 0 ;
u64 XVFRMBUFRD_BUFFER_BASEADDR;
u64 XVFRMBUFWR_BUFFER_BASEADDR;
u32 frmrd_start  = 0;
u32 frm_cnt = 0;
u32 frm_cnt1 = 0;

/**********************************************************************/
/**********************************************************************/

XGpio DVS_rstn_Gpio;
XIicPs IicPsInstance;		/* The instance of the IIC device. */
XScuGic InterruptController;	/* The instance of the Interrupt Controller. */
u8 DVSIicAddr; 			/* Variable for storing DVS IIC address */

volatile u8 TransmitComplete;	/**< Flag to check completion of Transmission */
volatile u8 ReceiveComplete;	/**< Flag to check completion of Reception */
volatile u32 TotalErrorCount;	/**< Total Error Count Flag */
volatile u32 SlaveResponse;		/**< Slave Response Flag */

u8 WriteBuf[sizeof(u8) + 16];
u8 ReadBuf[16];


int StartSetupDVSSensor(void) {
	int Status;
	Status = StartDVSSensor();
	if (Status != XST_SUCCESS){
		xil_printf("starting DVS sensor error\r\n");
		return XST_FAILURE;
	}
	Status = SetupDVSSensor();
	if (Status != XST_SUCCESS){
		xil_printf("setting DVS sensor error\r\n");
		return XST_FAILURE;
	}
	return XST_SUCCESS;
}


// setup GPIO so that DVS is programmable
int StartDVSSensor(void) {
	int Status;
	int Delay;

	// Start the rstn of DVS
	Status = XGpio_Initialize(&DVS_rstn_Gpio, GPIO_DEVICE_ID);
	if (Status != XST_SUCCESS) {
		xil_printf("Gpio Initialization Failed\r\n");
		return XST_FAILURE;
	}
	XGpio_SetDataDirection(&DVS_rstn_Gpio, DVS_CHANNEL, ~DVS_RSTN); // => sometimes not working??
	for (Delay = 0; Delay < DVS_DELAY; Delay++);
	XGpio_DiscreteWrite(&DVS_rstn_Gpio, DVS_CHANNEL, DVS_RSTN);

	return XST_SUCCESS;
}

//setup the DVS Sensor
int SetupDVSSensor(void) {
	int Status;
	u16 DeviceId;
	u32 Index;
	u32 MaxIndex = length_DVS_regs;
	DVSIicAddr = DVS_ADDR;
	struct regval_list * sensor_cfg = DVS_regs;

	// 1. init iic
	Status = InitIIC();

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

	Status = MuxInitChannel(MUX_ADDRESS, MUX_CHANNEL);
	if (Status != XST_SUCCESS){
		xil_printf("IIC MUX Channel set error at 0X%x\r\n", MUX_CHANNEL);
		return XST_FAILURE;
	}

	// 5. program dvs sensor
	for (Index = 0; Index < (MaxIndex); Index ++) {
		WriteBuf[0] = sensor_cfg[Index].Address >> 8;
		WriteBuf[1] = sensor_cfg[Index].Address;
		WriteBuf[2] = sensor_cfg[Index].Data;

		Status = SensorWriteData(3);

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
	XIicPs_SetStatusHandler(&IicPsInstance, (void *) &IicPsInstance, Handler);

	/*
	 * 2. Set the IIC serial clock rate.
	 */
	XIicPs_SetSClk(&IicPsInstance, IIC_SCLK_RATE);

	Status = MuxInitChannel(MUX_ADDRESS, MUX_CHANNEL);
	if (Status != XST_SUCCESS){
		xil_printf("IIC MUX Channel set error at 0X%x\r\n", MUX_CHANNEL);
		return XST_FAILURE;
	}
	xil_printf("3\r\n");

	// 3. program dvs sensor
	for (Index = 0; Index < (MaxIndex); Index ++) {
		WriteBuf[0] = sensor_cfg[Index].Address >> 8;
		WriteBuf[1] = sensor_cfg[Index].Address;
		WriteBuf[2] = sensor_cfg[Index].Data;

		Status = SensorWriteData(3);

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


int SetupIicPsInterruptSystem(void)
{
	int Status;
	XScuGic_Config *IntcConfig; /* Instance of the interrupt controller */

	Xil_ExceptionInit();

	/*
	 * Initialize the interrupt controller driver so that it is ready to
	 * use.
	 */
	IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
	if (NULL == IntcConfig) {
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
	XIicPs_Config * ConfigPtr;

	memset(&IicPsInstance, 0, sizeof(IicPsInstance));

	/*
	 * Initialize the IIC driver so that it is ready to use.
	 */
	ConfigPtr = XIicPs_LookupConfig(IIC_DEVICE_ID);
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

int MuxInitChannel(u16 MuxIicAddr, u8 WriteBuffer)
{
	u8 Buffer = 0;

	TotalErrorCount = 0;
	TransmitComplete = FALSE;
	TotalErrorCount = 0;

	XIicPs_MasterSend(&IicPsInstance, &WriteBuffer,1,MuxIicAddr);
	while (TransmitComplete == FALSE) {
		if (0 != TotalErrorCount) {
			return XST_FAILURE;
		}
	}
	/*
	 * Wait until bus is idle to start another transfer.
	 */

	while (XIicPs_BusIsBusy(&IicPsInstance));

	ReceiveComplete = FALSE;
	/*
	 * Receive the Data.
	 */
	XIicPs_MasterRecv(&IicPsInstance, &Buffer,1, MuxIicAddr);

	while (ReceiveComplete == FALSE) {
		if (0 != TotalErrorCount) {
			return XST_FAILURE;
		}
	}

	/*
	 * Wait until bus is idle to start another transfer.
	 */
	while (XIicPs_BusIsBusy(&IicPsInstance));

	return XST_SUCCESS;
}

void Handler(void *CallBackRef, u32 Event)
{
	/*
	 * All of the data transfer has been finished.
	 */

	if (0 != (Event & XIICPS_EVENT_COMPLETE_SEND)) {
		TransmitComplete = TRUE;
	} else if (0 != (Event & XIICPS_EVENT_COMPLETE_RECV)){
		ReceiveComplete = TRUE;
	} else if (0 != (Event & XIICPS_EVENT_SLAVE_RDY)) {
		SlaveResponse = TRUE;
	} else if (0 != (Event & XIICPS_EVENT_ERROR)){
		TotalErrorCount++;
	}
}

int SensorWriteData(u16 ByteCount) {
	TotalErrorCount = 0;
	TransmitComplete = FALSE;
	TotalErrorCount = 0;

	while (XIicPs_BusIsBusy(&IicPsInstance));

	/* Send the data */
	XIicPs_MasterSend(&IicPsInstance, WriteBuf, ByteCount, DVS_ADDR);
	/* Wait till the transmission is completed */
	while (TransmitComplete == FALSE) {
		if (0 != TotalErrorCount) {
			return XST_FAILURE;
		}
	}

	return XST_SUCCESS;
}

int SensorReadData(u8 *BufferPtr, u16 ByteCount) {
	int Status;
	ReceiveComplete = FALSE;

	/*
	 * Wait until bus is idle to start another transfer.
	 */
	while (XIicPs_BusIsBusy(&IicPsInstance));

	/*
	 * Receive the Data.
	 */
	XIicPs_MasterRecv(&IicPsInstance, BufferPtr, ByteCount, DVS_ADDR);

	while (ReceiveComplete == FALSE) {
		if (0 != TotalErrorCount) {
			return XST_FAILURE;
		}
	}
	xil_printf("SendReceiveData: 0X%x\r\n", BufferPtr[0]);

	return XST_SUCCESS;
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
//static void SendHandler(XIic *InstancePtr) {
//	TransmitComplete = 0;
//}

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
//static void ReceiveHandler(XIic *InstancePtr) {
//	ReceiveComplete = 0;
//}

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
//static void StatusHandler(XIic *InstancePtr, int Event) {
//
//}



/*****************************************************************************/
/**
 * This function setup Camera sensor programming wrt resolution selected
 *
 * @return	XST_SUCCESS if successful else XST_FAILURE.
 *
 * @note	None.
 *
 *****************************************************************************/
//int SetupCameraSensor(void) {
//	int Status;
//	u32 Index, MaxIndex;
//	SensorIicAddr = SENSOR_ADDR;
//	struct regval_list *sensor_cfg = NULL;
//
//	/* If no camera present then return */
//	if (Pipeline_Cfg.CameraPresent == FALSE) {
//		xil_printf("%s - No camera present\r\n", __func__);
//		return XST_SUCCESS;
//	}
//
//	/* Validate Pipeline Configuration */
//	if ((Pipeline_Cfg.VideoMode == XVIDC_VM_3840x2160_30_P)
//			&& (Pipeline_Cfg.ActiveLanes != 4)) {
//		xil_printf("4K supports only 4 Lane configuration\r\n");
//		return XST_FAILURE;
//	}
//
//	if ((Pipeline_Cfg.VideoMode == XVIDC_VM_1920x1080_60_P)
//			&& (Pipeline_Cfg.ActiveLanes == 1)) {
//		xil_printf("1080p doesn't support 1 Lane configuration\r\n");
//		return XST_FAILURE;
//	}
//
//	Status = XIic_SetAddress(&IicSensor, XII_ADDR_TO_SEND_TYPE,
//					SensorIicAddr);
//
//	if (Status != XST_SUCCESS) {
//		return XST_FAILURE;
//	}
//
//	/* Select the sensor configuration based on resolution and lane */
//	switch (Pipeline_Cfg.VideoMode) {
//
//		case XVIDC_VM_1280x720_60_P:
//			MaxIndex = length_imx274_config_720p_60fps_regs;
//			sensor_cfg = imx274_config_720p_60fps_regs;
//			break;
//
//		case XVIDC_VM_1920x1080_30_P:
//			MaxIndex = length_imx274_config_1080p_60fps_regs;
//			sensor_cfg = imx274_config_1080p_60fps_regs;
//			break;
//
//		case XVIDC_VM_1920x1080_60_P:
//			MaxIndex = length_imx274_config_1080p_60fps_regs;
//			sensor_cfg = imx274_config_1080p_60fps_regs;
//			break;
//
//		case XVIDC_VM_3840x2160_30_P:
//			MaxIndex = length_imx274_config_4K_30fps_regs;
//			sensor_cfg = imx274_config_4K_30fps_regs;
//			break;
//
//		case XVIDC_VM_3840x2160_60_P:
//			MaxIndex = length_imx274_config_4K_30fps_regs;
//			sensor_cfg = imx274_config_4K_30fps_regs;
//			break;
//
//
//		default:
//			return XST_FAILURE;
//			break;
//
//	}
//
//	/* Program sensor */
//	for (Index = 0; Index < (MaxIndex - 1); Index++) {
//
//		WriteBuffer[0] = sensor_cfg[Index].Address >> 8;
//		WriteBuffer[1] = sensor_cfg[Index].Address;
//		WriteBuffer[2] = sensor_cfg[Index].Data;
//
//		Status = SensorWriteData(3);
//
//		if (Status == XST_SUCCESS) {
//			ReadBuffer[0] = 0;
//			Status = SensorReadData(ReadBuffer, 1);
//
//		} else {
//			xil_printf("Error in Writing entry status = %x \r\n",
//					Status);
//			break;
//		}
//	}
//
//	if (Index != (MaxIndex - 1)) {
//		/* all registers are written into */
//		return XST_FAILURE;
//	}
//
//	return XST_SUCCESS;
//}

/*****************************************************************************/
/**
 * This function sets send, receive and error handlers for IIC interrupts.
 *
 * @return	None.
 *
 * @note	None.
 *
 *****************************************************************************/
//void SetupIICIntrHandlers(void) {
//	/*
//	 * Set the Handlers for transmit and reception.
//	 */
//	XIic_SetSendHandler(&IicSensor, &IicSensor,
//				(XIic_Handler) SendHandler);
//	XIic_SetRecvHandler(&IicSensor, &IicSensor,
//				(XIic_Handler) ReceiveHandler);
//	XIic_SetStatusHandler(&IicSensor, &IicSensor,
//				(XIic_StatusHandler) StatusHandler);
//
//}

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

	//DMA set Here
	print("\r\n\r\n--------------------------------\r\n");

	return 0;

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
//void EnableCSI(void)
//{
//	XCsiSs_Reset(&CsiRxSs);
////	XCsiSs_Configure(&CsiRxSs, (Pipeline_Cfg.ActiveLanes), 0);
//	XCsiSs_Activate(&CsiRxSs, XCSI_ENABLE);
//
//	usleep(1000000);
//
//}

/*****************************************************************************/
/**
 * This function disables MIPI CSI IP
 *
 * @return	None.
 *
 * @note	None.
 *
 *****************************************************************************/
//void DisableCSI(void)
//{
//	usleep(1000000);
//	XCsiSs_Reset(&CsiRxSs);
//}

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

	usleep(1000000);
	WriteBuffer[0] = 0x30;
	WriteBuffer[1] = 0x91;
	WriteBuffer[2] = 0x6c;
	Status = SensorWriteData(3);
	usleep(1000000);
	WriteBuffer[0] = 0x01;
	WriteBuffer[1] = 0x00;
	WriteBuffer[2] = 0x01;
	Status = SensorWriteData(3);


//	usleep(1000000);
//	WriteBuffer[0] = 0x30;
//	WriteBuffer[1] = 0x00;
//	WriteBuffer[2] = 0x00;
//	Status = SensorWriteData(3);
//	usleep(1000000);
//	WriteBuffer[0] = 0x30;
//	WriteBuffer[1] = 0x3E;
//	WriteBuffer[2] = 0x02;
//	Status = SensorWriteData(3);
//	usleep(1000000);
//	WriteBuffer[0] = 0x30;
//	WriteBuffer[1] = 0xF4;
//	WriteBuffer[2] = 0x00;
//	Status = SensorWriteData(3);
//	usleep(1000000);
//	WriteBuffer[0] = 0x30;
//	WriteBuffer[1] = 0x18;
//	WriteBuffer[2] = 0xA2;
//	Status = SensorWriteData(3);

	if (Status != XST_SUCCESS) {
		xil_printf("Error: in Writing entry status = %x \r\n", Status);
		xil_printf("%s - Failed\r\n", __func__);
		return XST_FAILURE;
	}

	return Status;
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

//	XDphy_WriteReg(XPAR_MIPI_DPHY_0_BASEADDR,0x30, 0xB0);
//	XDphy_WriteReg(XPAR_MIPI_DPHY_0_BASEADDR,0x48, 0xB0);
//	XDphy_WriteReg(XPAR_MIPI_DPHY_0_BASEADDR,0x4c, 0xB0);
//	XDphy_WriteReg(XPAR_MIPI_DPHY_0_BASEADDR,0x50, 0xB0);
//	XDphy_WriteReg(XPAR_MIPI_DPHY_0_BASEADDR,0x54, 0xB0);
//	XDphy_WriteReg(XPAR_MIPI_DPHY_0_BASEADDR,0x58, 0xB0);
//	XDphy_WriteReg(XPAR_MIPI_DPHY_0_BASEADDR,0x5c, 0xB0);
//	XDphy_WriteReg(XPAR_MIPI_DPHY_0_BASEADDR,0x60, 0xB0);

	return XST_SUCCESS;
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
//u32 InitializeCsiRxSs(void)
//{
//	u32 Status = 0;
//
//	XCsiSs_Config *CsiRxSsCfgPtr = NULL;
//
//	CsiRxSsCfgPtr = XCsiSs_LookupConfig(XCSIRXSS_DEVICE_ID);
//	if (!CsiRxSsCfgPtr) {
//		xil_printf("CSI2RxSs LookupCfg failed\r\n");
//		return XST_FAILURE;
//	}
//
//	Status = XCsiSs_CfgInitialize(&CsiRxSs, CsiRxSsCfgPtr,
//			CsiRxSsCfgPtr->BaseAddr);
//	if (Status != XST_SUCCESS) {
//		xil_printf("CsiRxSs Cfg init failed - %x\r\n", Status);
//		return Status;
//	}
////	XCsi_IntrEnable(&CsiRxSs, XCSI_ISR_SKEWCALCHS_MASK);
////	XCsi_IntrEnable(&CsiRxSs, XCSI_ISR_WC_MASK);
////	XCsi_IntrEnable(&CsiRxSs, XCSI_ISR_ILC_MASK);
////	XCsi_IntrEnable(&CsiRxSs, XCSI_ISR_FR_MASK);
////	XCsi_IntrEnable(&CsiRxSs, XCSI_ISR_ECC2BERR_MASK);
////	XCsi_IntrEnable(&CsiRxSs, XCSI_ISR_ECC1BERR_MASK);
//
//	printf("dphy addroffset: %d\r\n", CsiRxSs.Config.DphyInfo.AddrOffset);
//	printf("dphy deviceid: %d\r\n", CsiRxSs.Config.DphyInfo.DeviceId);
//	printf("dphy ispresent: %d\r\n", CsiRxSs.Config.DphyInfo.IsPresent);
//	printf("lanes present: %d\r\n", CsiRxSs.Config.LanesPresent);
//	printf("dphy line rate: %d\r\n", CsiRxSs.Config.DphyLineRate);
//	printf("color depth: %d\r\n", CsiRxSs.Config.PixelFormat);
//
//	/***************   dphy   ****************/
//	XDphy_Config *DphyCfgPtr = NULL;
//	DphyCfgPtr = XDphy_LookupConfig(XPAR_MIPI_CSI2_RX_SUBSYST_0_PHY_DEVICE_ID);
//	if (!DphyCfgPtr) {
//		xil_printf("Dphy Lookup Cfg failed\r\n");
//		return XST_FAILURE;
//	}
//	Status = XDphy_CfgInitialize(&DphyRx, DphyCfgPtr,
//			DphyCfgPtr->BaseAddr);
//	if (Status != XST_SUCCESS) {
//		xil_printf("Dphy Cfg init failed - %x\r\n", Status);
//		return Status;
//	}
////	XDphy_WriteReg(CsiRxSsCfgPtr->BaseAddr + DphyCfgPtr->BaseAddr,
////				0x08, 0xf4240);
//
////	XDphy_WriteReg(CsiRxSsCfgPtr->BaseAddr + DphyCfgPtr->BaseAddr,
////			0x30, 0x8d);
////	XDphy_WriteReg(CsiRxSsCfgPtr->BaseAddr + DphyCfgPtr->BaseAddr,
////				0x48, 0x8d);
////	XDphy_WriteReg(CsiRxSsCfgPtr->BaseAddr + DphyCfgPtr->BaseAddr,
////				0x4c, 0x8d);
////	XDphy_WriteReg(CsiRxSsCfgPtr->BaseAddr + DphyCfgPtr->BaseAddr,
////				0x50, 0x8d);
//
////	XDphy_Reset(&DphyRx);
////	Status = XDphy_Configure(&DphyRx, XDPHY_HANDLE_INIT_TIMER, 100000);
////	if (Status != XST_SUCCESS) {
////		xil_printf("Dphy init timer set failed - %x\r\n", Status);
////		return Status;
////	}
////	Status = XDphy_Configure(&DphyRx, XDPHY_HANDLE_HSTIMEOUT, CsiRxSs.DphyPtr->Config.HSTimeOut);
////	if (Status != XST_SUCCESS) {
////		xil_printf("Dphy init timer set failed - %x\r\n", Status);
////		return Status;
////	}
////	Status = XDphy_Configure(&DphyRx, XDPHY_HANDLE_ESCTIMEOUT, CsiRxSs.DphyPtr->Config.EscTimeout);
////	if (Status != XST_SUCCESS) {
////		xil_printf("Dphy init timer set failed - %x\r\n", Status);
////		return Status;
////	}
////	u32 clock_lane_info = XDphy_GetInfo(&DphyRx, XDPHY_HANDLE_CLKLANE);
////	printf("clock_lane_info: %x\r\n", clock_lane_info);
////	Status = XDphy_SelfTest(&DphyRx);
////	if (Status != XST_SUCCESS) {
////		xil_printf("Dphy selftest failed - %x\r\n", Status);
////		return Status;
////	}
////	XDphy_Activate(&DphyRx, XDPHY_ENABLE_FLAG);
////
////	printf("isRx: %d\r\n", CsiRxSs.DphyPtr->Config.IsRx);
////	printf("HSLineRate: %d\r\n", CsiRxSs.DphyPtr->Config.HSLineRate);
////	printf("HSTimeOut: %d\r\n", CsiRxSs.DphyPtr->Config.HSTimeOut);
////	printf("LPXPeriod: %d\r\n", CsiRxSs.DphyPtr->Config.LPXPeriod);
////	printf("StableClkPeriod: %d\r\n", CsiRxSs.DphyPtr->Config.StableClkPeriod);
////	printf("TxPllClkinPeriod: %d\r\n", CsiRxSs.DphyPtr->Config.TxPllClkinPeriod);
////	printf("Wakeup: %d\r\n", CsiRxSs.DphyPtr->Config.Wakeup);
////	printf("EnableTimeOutRegs: %d\r\n", CsiRxSs.DphyPtr->Config.EnableTimeOutRegs);
////	printf("HSSettle: %d\r\n", CsiRxSs.DphyPtr->Config.HSSettle);
////	printf("csirx ready?: %x\r\n", CsiRxSs.IsReady);
////	printf("dphy ready?: %x\r\n", CsiRxSs.DphyPtr->IsReady);
//	/***************   dphy   ****************/
//
//
//	return XST_SUCCESS;
//}

void CsiRxPrintRegStatus(void) {
	u32 Status = 0;
//	XCsiSs_Config *CsiRxSsCfgPtr = NULL;
//
//	CsiRxSsCfgPtr = XCsiSs_LookupConfig(XCSIRXSS_DEVICE_ID);
//	if (!CsiRxSsCfgPtr) {
//		xil_printf("CSI2RxSs LookupCfg failed\r\n");
//		return XST_FAILURE;
//	}
//	xil_printf("\r\n\r\n");
//	xil_printf(TXT_RED);
//	xil_printf("-----------CSIRXSS Register value-------------\r\n");
//	u32 offset_list[] = {
//			XCSI_CCR_OFFSET, 		// core configuration
//			XCSI_PCR_OFFSET, 		// Protocol configuration
//			XCSI_CSR_OFFSET, 		// core status register
//			XCSI_GIER_OFFSET, 		// global interrupt
//			XCSI_ISR_OFFSET, 		// interrupt status
//			XCSI_IER_OFFSET,		// interrupt enable register
//			XCSI_SPKTR_OFFSET, 		// generic short packet
//			XCSI_VCX_FE_OFFSET, 	// VCx Frame Error
//			XCSI_CLKINFR_OFFSET, 	// Clock lane info
//			XCSI_L0INFR_OFFSET,
//			XCSI_L1INFR_OFFSET,
//			XCSI_L2INFR_OFFSET,
//			XCSI_L3INFR_OFFSET,
//			XCSI_VC0INF1R_OFFSET,
//			XCSI_VC0INF2R_OFFSET,
//			XCSI_VC1INF2R_OFFSET
//	};
//
//	for (int i= 0; i < sizeof(offset_list)/ sizeof(offset_list[0]); i++){
//		u32 value = XCsiSs_ReadReg(CsiRxSsCfgPtr->BaseAddr, offset_list[i]);
//		printf("Value of register offset 0X%x : 0X%x \r\n", offset_list[i], value);
//	}
//	u32 stop_bit = XCsi_GetBitField(CsiRxSsCfgPtr->BaseAddr, XCSI_ISR_OFFSET,
//			XCSI_ISR_STOP_MASK, XCSI_ISR_STOP_SHIFT);
//	if(stop_bit) {
//		XCsi_SetBitField(CsiRxSsCfgPtr->BaseAddr, XCSI_ISR_OFFSET,
//			XCSI_ISR_STOP_MASK, XCSI_ISR_STOP_SHIFT, 1);
//	}


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
		u32 value = XDphy_ReadReg(XPAR_MIPI_DPHY_0_BASEADDR, dphy_offset_list[i]);
		printf("Value of register offset 0X%x : 0X%x \r\n", dphy_offset_list[i], value);
	}
	xil_printf(TXT_RST);

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
//void SetColorDepth(void)
//{
//	Pipeline_Cfg.ColorDepth = GetColorDepth(CsiRxSs.Config.PixelFormat);
//	print(TXT_GREEN);
//	xil_printf("Setting Color Depth = %d bpc\r\n", Pipeline_Cfg.ColorDepth);
//	print(TXT_RST);
//	return;
//}




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
