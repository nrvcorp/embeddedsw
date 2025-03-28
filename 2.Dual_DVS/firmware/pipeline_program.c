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

#include "pipeline_program.h"

/**********************************************************************/
// External Variables
XIicPs IicPsInstance;                   // PS IIC Device Instance
XScuGic InterruptController;     // Interrupt Controller

// I2C Buffers
u8 DVSIICWriteBuf[sizeof(u8) + 16];
u8 DVSIICReadBuf[16];
DVSCamera dvs_cameras[DVS_NUM];

/**********************************************************************/
volatile u8 TransmitComplete;	/**< Flag to check completion of Transmission */
volatile u8 ReceiveComplete;	/**< Flag to check completion of Reception */
volatile u8 DVSIICTransmitComplete;
volatile u8 DVSIICReceiveComplete;
volatile u32 DVSIICTotalErrorCount;
volatile u32 DVSIICSlaveResponse;


void printDVSCameraSetting(DVSCamera DVSCamera_t){

	xil_printf("Camera ID: %d\r\n", DVSCamera_t.id);
	xil_printf("IIC MUX Addr: %x\r\n", DVSCamera_t.IICMuxAddr);
	xil_printf("IIC MUX Addr: %d\r\n", DVSCamera_t.IICMuxChannel);
	xil_printf("dphy_device_id: %d\r\n", DVSCamera_t.DphyDeviceId);
	xil_printf("dma_device_id: %d\r\n", DVSCamera_t.axi_dma_device_id);
	xil_printf("dma_intr_id: %d\r\n", DVSCamera_t.axi_dma_intr_id);
	xil_printf("BD_Base: %x\r\n", DVSCamera_t.axi_dma_rx_bd_space_base);
	xil_printf("BD_High: %x\r\n", DVSCamera_t.axi_dma_rx_bd_space_high);
	xil_printf("frame_rdy[0]: %x\r\n", DVSCamera_t.frame_rdy[0]);
	xil_printf("frame_array[0]: %x\r\n", DVSCamera_t.frame_array[0]);
	xil_printf("frame_array[99]: %x\r\n", DVSCamera_t.frame_array[99]);
	xil_printf("Initialize DVS Camera(%d) Success \r\n\r\n", DVSCamera_t.id);
}

int initializeDVSCameras(void) {
	u32 Status = 0;
	XDphy_Config *DphyCfgPtr = NULL;
	for (int cam_id = 0; cam_id < DVS_NUM; ++cam_id) {
		dvs_cameras[cam_id].id = cam_id;

		if (cam_id == 0) {
			dvs_cameras[cam_id].rstn_baseaddr = DVS_GPIO_BASEADDR + XGPIO_DATA_OFFSET;
			dvs_cameras[cam_id].DphyDeviceId = XPAR_DVS_STREAM0_MIPI_DPHY_0_DEVICE_ID;
			dvs_cameras[cam_id].mipirx_subsystem_baseaddr = XPAR_DVS_STREAM0_MIPI_RX_SUBSYSTEM_TOP_0_BASEADDR;
			dvs_cameras[cam_id].IICMuxChannel = DVS_MUX_CHANNEL_FMC0;
			dvs_cameras[cam_id].axi_dma_device_id = XPAR_DVS_STREAM0_AXI_DMA_0_DEVICE_ID;
			dvs_cameras[cam_id].axi_dma_intr_id = XPAR_FABRIC_DVS_STREAM0_AXI_DMA_0_S2MM_INTROUT_INTR;
		} else {
			dvs_cameras[cam_id].rstn_baseaddr = DVS_GPIO_BASEADDR + XGPIO_DATA2_OFFSET;
			dvs_cameras[cam_id].DphyDeviceId = XPAR_DVS_STREAM1_MIPI_DPHY_0_DEVICE_ID;
			dvs_cameras[cam_id].mipirx_subsystem_baseaddr = XPAR_DVS_STREAM1_MIPI_RX_SUBSYSTEM_TOP_0_BASEADDR;
			dvs_cameras[cam_id].IICMuxChannel = DVS_MUX_CHANNEL_FMC1;
			dvs_cameras[cam_id].axi_dma_device_id = XPAR_DVS_STREAM1_AXI_DMA_0_DEVICE_ID;
			dvs_cameras[cam_id].axi_dma_intr_id = XPAR_FABRIC_DVS_STREAM1_AXI_DMA_0_S2MM_INTROUT_INTR;
		}
		dvs_cameras[cam_id].IICMuxAddr = 0x75;
		dvs_cameras[cam_id].DVSIicAddr = DVS_ADDR;


		// DPhy Initialize
		DphyCfgPtr = XDphy_LookupConfig(dvs_cameras[cam_id].DphyDeviceId);
		if (!DphyCfgPtr) {
			xil_printf("Dphy Lookup Cfg failed\r\n");
			return XST_FAILURE;
		}
		Status = XDphy_CfgInitialize(&dvs_cameras[cam_id].DphyRx, DphyCfgPtr,
				DphyCfgPtr->BaseAddr);
		if (Status != XST_SUCCESS) {
			xil_printf("Dphy Cfg init failed - %x\r\n", Status);
			return Status;
		}


		/* Program Camera sensor */
	//	MIPI_CONTROLLER_mWriteReg(dvs_cameras[cam_id].mipirx_subsystem_baseaddr,
	//			MIPI_CONTROLLER_S00_AXI_SLV_REG0_OFFSET,
	//			0);
	//	sleep(1);
	//	MIPI_CONTROLLER_mWriteReg(dvs_cameras[cam_id].mipirx_subsystem_baseaddr,
	//			MIPI_CONTROLLER_S00_AXI_SLV_REG0_OFFSET,
	//			2);
		// frame header setting
		MIPI_CONTROLLER_mWriteReg(dvs_cameras[cam_id].mipirx_subsystem_baseaddr,
					MIPI_CONTROLLER_S00_AXI_SLV_REG0_OFFSET,
					1);
		MIPI_CONTROLLER_mWriteReg(dvs_cameras[cam_id].mipirx_subsystem_baseaddr,
					MIPI_CONTROLLER_S00_AXI_SLV_REG1_OFFSET,
					1);
		// 1. control fifo with drop rate-----------------------------
	//	MIPI_CONTROLLER_mWriteReg(dvs_cameras[cam_id].mipirx_subsystem_baseaddr,
	//			MIPI_CONTROLLER_S00_AXI_SLV_REG3_OFFSET,
	//			10);

		// 2. control fifo with fifo control -------------------------
	//	MIPI_CONTROLLER_mWriteReg(dvs_cameras[cam_id].mipirx_subsystem_baseaddr,
	//			MIPI_CONTROLLER_S00_AXI_SLV_REG3_OFFSET,
	//			0);
	//	MIPI_CONTROLLER_mWriteReg(dvs_cameras[cam_id].mipirx_subsystem_baseaddr,
	//			MIPI_CONTROLLER_S00_AXI_SLV_REG4_OFFSET,
	//			1);

		// AXI Dma Configuration
		dvs_cameras[cam_id].axi_dma_bd_len = DVS_BUFFER_SIZE;
		dvs_cameras[cam_id].axi_dma_rx_bd_space_base = RX_BD_SPACE_BASE(cam_id);
		dvs_cameras[cam_id].axi_dma_rx_bd_space_high = RX_BD_SPACE_HIGH(cam_id);


		// Base addresses for current camera
		u64 dvs_frame_addr = DVS_BUFFER_BASEADDR(cam_id);
		u64 dvs_frame_rdy_addr = DVS_BUFFER_RDY_BASE(cam_id);

		for (int i = 0; i < DVS_BUFFER_NUM; ++i) {
			dvs_cameras[cam_id].frame_array[i] = dvs_frame_addr;
			dvs_cameras[cam_id].frame_rdy[i] = dvs_frame_rdy_addr;

			dvs_frame_addr += DVS_BUFFER_SIZE;  // Increment frame address
			dvs_frame_rdy_addr += 1;           // Increment ready address
		}


		// Invalidate the cache for the camera's buffer range
		Xil_DCacheInvalidateRange((UINTPTR)DVS_BUFFER_RDY_BASE(cam_id), DVS_BUFFER_NUM);
		Xil_DCacheInvalidateRange((UINTPTR)DVS_BUFFER_BASEADDR(cam_id), DVS_BUFFER_NUM * DVS_BUFFER_SIZE);

		printDVSCameraSetting(dvs_cameras[cam_id]);
	}

	return XST_SUCCESS;
}

void DVSDmaWriteDoneCallback(DVSCamera * DVSCamera_t) {
	Xil_Out8(DVSCamera_t->frame_rdy[DVSCamera_t->wr_ptr], 0x1);
	if (DVSCamera_t->wr_ptr == DVS_BUFFER_NUM-1) {
		DVSCamera_t->wr_ptr = 0;
	} else {
		DVSCamera_t->wr_ptr = DVSCamera_t->wr_ptr + 1;
	}
	DVSCamera_t->frm_cnt++;
}

int StartDVSSensor(DVSCamera DVSCamera_t) {
	Xil_Out32(DVSCamera_t.rstn_baseaddr, 0x00);
	usleep(1000);
	Xil_Out32(DVSCamera_t.rstn_baseaddr, 0x01);
	return XST_SUCCESS;
}


int ProgramDVSSensor(DVSCamera DVSCamera_t) {
	int Status;
	u16 DeviceId;
	u32 Index;
	u32 MaxIndex = length_DVS_regs;
	struct regval_list *sensor_cfg = DVS_regs;

	/*
	 * 1. Setup the handlers for the IIC that will be called from the
	 * interrupt context when data has been sent and received, specify a
	 * pointer to the IIC driver instance as the callback reference so
	 * the handlers are able to access the instance data.
	 */
	XIicPs_SetStatusHandler(&IicPsInstance, (void *) &IicPsInstance, PsIICStatusHandler);

	/*
	 * 2. Set the IIC serial clock rate.
	 */
	XIicPs_SetSClk(&IicPsInstance, IIC_SCLK_RATE);

	Status = IICMuxInitChannel(DVSCamera_t.IICMuxAddr, DVSCamera_t.IICMuxChannel);
	if (Status != XST_SUCCESS){
		xil_printf("IIC MUX Channel set error at 0X%x\r\n", DVSCamera_t.IICMuxChannel);
		return XST_FAILURE;
	}

	// 3. program dvs sensor
	for (Index = 0; Index < (MaxIndex); Index ++) {
		DVSIICWriteBuf[0] = sensor_cfg[Index].Address >> 8;
		DVSIICWriteBuf[1] = sensor_cfg[Index].Address;
		DVSIICWriteBuf[2] = sensor_cfg[Index].Data;

		Status = DVSIICWriteData(3, DVSCamera_t.DVSIicAddr);

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

/*****************************************************************************/
/**
 * This function initializes IIC controller and gets config parameters.
 *
 * @return	XST_SUCCESS if successful else XST_FAILURE.
 *
 * @note	None.
 *
 *****************************************************************************/
int InitPsIIC(void) {
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

int IICMuxInitChannel(u16 MuxIicAddr, u8 WriteBuf)
{
	u8 Buffer = 0;

	DVSIICTotalErrorCount = 0;
	DVSIICTransmitComplete = FALSE;

	XIicPs_MasterSend(&IicPsInstance, &WriteBuf, 1, MuxIicAddr);
	while (DVSIICTransmitComplete == FALSE) {
		if (0 != DVSIICTotalErrorCount) {
			return XST_FAILURE;
		}
	}
	/* Wait until bus is idle to start another transfer. */
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

	/* Wait until bus is idle to start another transfer. */
	while (XIicPs_BusIsBusy(&IicPsInstance));

	return XST_SUCCESS;
}

void PsIICStatusHandler(void *CallBackRef, u32 Event)
{
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


int DVSIICWriteData(u16 ByteCount, u8 DVSIicAddr) {
	DVSIICTotalErrorCount = 0;
	DVSIICTransmitComplete = FALSE;
	DVSIICTotalErrorCount = 0;

	while (XIicPs_BusIsBusy(&IicPsInstance));

	/* Send the data */
	XIicPs_MasterSend(&IicPsInstance, DVSIICWriteBuf, ByteCount, DVSIicAddr);
	/* Wait till the transmission is completed */
	while (DVSIICTransmitComplete == FALSE) {
		if (0 != DVSIICTotalErrorCount) {
			return XST_FAILURE;
		}
	}

	return XST_SUCCESS;
}

int DVSIICReadData(u16 ByteCount, u8 DVSIicAddr) {
	DVSIICReceiveComplete = FALSE;

	/*
	 * Wait until bus is idle to start another transfer.
	 */
	while (XIicPs_BusIsBusy(&IicPsInstance));

	/*
	 * Receive the Data.
	 */
	XIicPs_MasterRecv(&IicPsInstance, DVSIICReadBuf, ByteCount, DVSIicAddr);

	while (ReceiveComplete == FALSE) {
		if (0 != DVSIICTotalErrorCount) {
			return XST_FAILURE;
		}
	}
	xil_printf("SendReceiveData: 0X%x\r\n", DVSIICReadBuf[0]);

	return XST_SUCCESS;
}


int DVSDma_WriteSetup(DVSCamera * DVSCamera_t) {

	XAxiDma_BdRing * RxRingPtr;
	int Status;
	XAxiDma_Bd BdTemplate;
	XAxiDma_Bd *BdPtr;
	XAxiDma_Bd *BdCurPtr;
	int BdCount;
	UINTPTR RxBufferPtr;
	int Index;

	RxRingPtr = XAxiDma_GetRxRing(&DVSCamera_t->axi_dma);
	XAxiDma_BdRingIntDisable(RxRingPtr, XAXIDMA_IRQ_ALL_MASK);

	BdCount = XAxiDma_BdRingCntCalc(XAXIDMA_BD_MINIMUM_ALIGNMENT,
				DVSCamera_t->axi_dma_rx_bd_space_high
				- DVSCamera_t->axi_dma_rx_bd_space_base + 1);

	Status = XAxiDma_BdRingCreate(RxRingPtr, DVSCamera_t->axi_dma_rx_bd_space_base,
				DVSCamera_t->axi_dma_rx_bd_space_base,
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
	RxBufferPtr = DVSCamera_t->frame_array[DVSCamera_t->wr_ptr];

	for (Index = 0; Index < DVS_BUFFER_NUM; Index++) {
		Status = XAxiDma_BdSetBufAddr(BdCurPtr, RxBufferPtr);
		if (Status != XST_SUCCESS) {
			xil_printf("Rx set buffer addr %x on BD %x failed %d \r\n",
			(unsigned int)RxBufferPtr,
			(UINTPTR)BdCurPtr, Status);

			return XST_FAILURE;
		}
		Status = XAxiDma_BdSetLength(BdCurPtr, DVSCamera_t->axi_dma_bd_len,
				RxRingPtr->MaxTransferLen);
		if (Status != XST_SUCCESS) {
			xil_printf("Rx set length %d on BD %x failed %d\r\n",
					DVSCamera_t->axi_dma_bd_len, (UINTPTR) BdCurPtr, Status);
			return XST_FAILURE;
		}

		XAxiDma_BdSetCtrl(BdCurPtr, 0);

		XAxiDma_BdSetId(BdCurPtr, RxBufferPtr);

		RxBufferPtr += DVSCamera_t->axi_dma_bd_len;
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


int start_dvs_cap_pipe(DVSCamera* DVSCamera_t) {
	int Status;

	XAxiDma_BdRing* RxRingPtr = XAxiDma_GetRxRing(&DVSCamera_t->axi_dma);
	XAxiDma_BdRingIntEnable(RxRingPtr, XAXIDMA_IRQ_ALL_MASK);
	// Enable Cyclic DMA mode
	XAxiDma_BdRingEnableCyclicDMA(RxRingPtr);
	XAxiDma_SelectCyclicMode(&DVSCamera_t->axi_dma, XAXIDMA_DEVICE_TO_DMA, 1);

	/* Start RX DMA channel */
	Status = XAxiDma_BdRingStart(RxRingPtr);
	if (Status != XST_SUCCESS) {
		xil_printf("Rx start BD ring failed with %d\r\n", Status);
		return XST_FAILURE;
	}

	xil_printf("Started DVS CAP PIPE\r\n");
	return 0;
}

void config_dvs_cap_path(DVSCamera* DVSCamera_t){
	int Status;
	XAxiDma_Config *Dma_Config;

	Dma_Config = XAxiDma_LookupConfig(DVSCamera_t->axi_dma_device_id);
	if (!Dma_Config) {
		xil_printf("No config found for %d\r\n", DVSCamera_t->axi_dma_device_id);
		return XST_FAILURE;
	}
	Status = XAxiDma_CfgInitialize(&DVSCamera_t->axi_dma, Dma_Config);
	if (Status != XST_SUCCESS) {
		xil_printf("Initialization failed %d\r\n", Status);
		return XST_FAILURE;
	}

	/* Setup the write channel
	 */
	Status = DVSDma_WriteSetup(DVSCamera_t);
	if (Status != XST_SUCCESS) {
		xil_printf("Write channel setup failed %d\r\n", Status);
		return XST_FAILURE;
	}

	xil_printf("DVS Stream DMA cap path setup finished \r\n");
	return 0;
}


