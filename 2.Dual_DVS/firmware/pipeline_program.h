/******************************************************************************
* Copyright (C) 2017 - 2020 Xilinx, Inc.  All rights reserved.
* SPDX-License-Identifier: MIT
 ******************************************************************************/

/*****************************************************************************/
/**
 *
 * @file pipeline_program.h
 *
 * This header file contains the definitions for structures for video pipeline
 * and extern declarations.
 *
 * <pre>
 * MODIFICATION HISTORY:
 *
 * Ver   Who    Date     Changes
 * ----- ------ -------- --------------------------------------------------
 * 1.00  pg    12/07/17 Initial release.
 * </pre>
 *
 ******************************************************************************/

#ifndef PIPELINE_PROGRAM_H_
#define PIPELINE_PROGRAM_H_

#include "stdlib.h"
#include "xparameters.h"
#include "sleep.h"
#include "xiicps.h"
#include "xaxidma.h"
#include "sensor_cfgs.h"
#include "xil_exception.h"
#include "xil_cache.h"
#include "xscugic.h"
#include "xgpio.h"
#include "xdphy.h"


// Camera and Buffer configuration
#define DVS_NUM 			2
#define DDR_BASEADDR 		0x10000000

// DVS Buffer Offsets
#define RX_BD_SPACE_BASEADDR_OFFSET	((DDR_BASEADDR) + 0x8000000)
#define DVS_BUFFER_RDY_OFFSET     	((DDR_BASEADDR) + 0x2000000)
#define DVS_BUFFER_BASEADDR_OFFSET 	((DDR_BASEADDR) + 0x20000000)
#define DVS_BUFFER_NUM            	100
#define DVS_BUFFER_SIZE           	0x2a308 // (960 * 720 (2-bit)) / 8 bits + 8 bytes
// DVS Frame Dimensions
#define DVS_FRAME_HORIZONTAL_LEN  0xF0 // 960 pixels (2-bit) = 240 bytes
#define DVS_FRAME_VERTICAL_LEN    0x2D0 // 720 pixels

// GPIO Configuration
#define GPIO_DEVICE_ID            XPAR_SENSOR_CTRL_AXI_GPIO_0_DEVICE_ID
#define DVS_GPIO_BASEADDR         XPAR_SENSOR_CTRL_AXI_GPIO_0_BASEADDR
#define DVS_RSTN                  0x01

// I2C Configuration
#define IIC_DEVICE_ID             XPAR_XIICPS_1_DEVICE_ID
#define INTC_DEVICE_ID            XPAR_SCUGIC_SINGLE_DEVICE_ID
#define IIC_SCLK_RATE             100000
#define SLV_MON_LOOP_COUNT        0x000FFFFF
#define DVS_MUX_ADDRESS           0x75
#define DVS_MUX_CHANNEL_FMC0      0x01
#define DVS_MUX_CHANNEL_FMC1      0x02

// DMA Configuration
#define COALESCING_COUNT          		1
#define DELAY_TIMER_COUNT         		0
#define RX_BD_SPACE_BASE(cameraId)		(RX_BD_SPACE_BASEADDR_OFFSET + ((cameraId) * (0x1000000)))
#define RX_BD_SPACE_HIGH(cameraId)    	(RX_BD_SPACE_BASE(cameraId) + DVS_BUFFER_NUM * 0x40 - 1)

// Camera-Specific Macros
#define DVS_BUFFER_RDY_BASE(cameraId)   (DVS_BUFFER_RDY_OFFSET + ((cameraId) * (0x1000000)))
#define DVS_BUFFER_BASEADDR(cameraId)   (DVS_BUFFER_BASEADDR_OFFSET + ((cameraId) * (0x10000000)))

//DVS define
//#define DVS_RSTN 			0x01
//#define DVS_DELAY			10000000
//#define DVS_CHANNEL		1
#define DVS_ADDR			(0x20>>1)	// DVS

// External Variables
extern XIicPs IicPsInstance;                   // PS IIC Device Instance
extern XScuGic InterruptController;     // Interrupt Controller

// I2C Buffers
extern u8 DVSIICWriteBuf[sizeof(u8) + 16];
extern u8 DVSIICReadBuf[16];

// Camera Buffers and States
typedef struct {
	u8 id;
	u32 rstn_baseaddr;
	u32 DphyDeviceId;
    XDphy DphyRx;						// Dphy for DVS Camera
    u64 mipirx_subsystem_baseaddr;		// Baseaddr for MIPI subsystem
	u8 IICMuxAddr;
	u8 IICMuxChannel;
    u8 DVSIicAddr;						// DVS IIC Address
    u32 axi_dma_device_id;
    XAxiDma axi_dma;					// DMA for DVS Camera
    u32 axi_dma_intr_id;				// DMA intr id
    u32 axi_dma_bd_len;					// DMA BD length (Frame size)
    u64 axi_dma_rx_bd_space_base;
    u64 axi_dma_rx_bd_space_high;
    u64 frame_rdy[DVS_BUFFER_NUM];    	// Frame Ready States
    u64 frame_array[DVS_BUFFER_NUM];    // Frame Buffer
    u32 wr_ptr;                  		// Write Pointer
    u32 frm_cnt;               			// Frame Count
} DVSCamera;

// Buffers for all cameras
extern DVSCamera dvs_cameras[DVS_NUM];

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


/**************************** Type Definitions *******************************/
typedef u8 AddressType;

/**************************** Function Definitions *******************************/
extern int initializeDVSCameras(void);
extern void DVSDmaWriteDoneCallback(DVSCamera* DVSCamera_t);
extern int StartDVSSensor(DVSCamera DVSCamera_t);
extern int ProgramDVSSensor(DVSCamera DVSCamera_t);

extern int InitPsIIC(void);
extern int IICMuxInitChannel(u16 MuxIicAddr, u8 WriteBuf);
extern void PsIICStatusHandler(void *CallBackRef, u32 Event);
extern int DVSIICWriteData(u16 ByteCount, u8 DVSIicAddr);
extern int DVSIICReadData(u16 ByteCount, u8 DVSIicAddr);

extern int start_dvs_cap_pipe(DVSCamera* DVSCamera_t);
extern void config_dvs_cap_path(DVSCamera* DVSCamera_t);

#endif /* PIPELINE_PROGRAM_H_ */
