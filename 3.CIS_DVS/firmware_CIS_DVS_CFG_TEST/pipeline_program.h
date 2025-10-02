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

#ifdef __cplusplus
extern "C" {
#endif
#include "xiicps.h"
#include "xaxidma.h"
#include "xtime_l.h"

extern void DmaWriteDoneCallback (XAxiDma_BdRing * RxRingPtr);

extern void configure_buffer_system();
extern XIicPs	IicPsInstance; /* The instance of the PS IIC device */
extern int InitDVSIIC(void);
extern int SetupIicDVSInterruptSystem(void);
extern void DVSIICStatusHandler(void *CallBackRef, u32 Event);
extern int StartDVSSensor(void);
extern int ProgramDVSSensor(struct regval_list DVS_reg_cfg[], const int length);
extern int DVSWriteData(u16 ByteCount);
extern int DVSReadData(u16 ByteCount);
extern int start_dvs_cap_pipe();
extern int config_dvs_cap_path();
extern int dvs_dma_reset();
//----------------------------------------------------------------

extern u32 InitializeCsiRxSs(void);

extern void SetColorDepth(void);

extern int InitIIC(void);
extern void SetupIICIntrHandlers(void);
extern int SetupCameraSensor(void);
extern void InitDemosaicGammaCSC(void);
extern int StartSensor(void);
extern int StopSensor(void);

extern void PrintPipeConfig(void);

extern void DisableScaler(void);
extern void InitVprocSs_Scaler(int count);
extern void EnableCSI(void);




#define DDR_BASEADDR 0x10000000

//DVS frame info: pixel and bit counts
#define DVS_FRAME_WIDTH		960
#define DVS_FRAME_HEIGHT	720
#define BIT_PER_PEL			2
#define DVS_LINE_SIZE_IN_BYTE 	DVS_FRAME_WIDTH * BIT_PER_PEL / 8	//240
#define DVS_FRAME_SIZE_IN_BYTE	DVS_FRAME_WIDTH * DVS_FRAME_HEIGHT * BIT_PER_PEL / 8	//172800
#define DVS_FRAME_HEADER_SIZE_IN_BYTE 8

#define MAX_EVENTS_2FRAMES ((DVS_FRAME_WIDTH) * (DVS_FRAME_HEIGHT) * 2)

#define CIS_BUFFER_RDY 		(DDR_BASEADDR + 0x1000000)
#define CIS_BUFFER_BASEADDR (DDR_BASEADDR + (0x10000000))
#define CIS_BUFFER_NUM 		5
#define CIS_BUFFER_SIZE		0x5EEC00 // 1920*1080*3
#define CHROMA_ADDR_OFFSET  (0x01000000U)

#define DVS_BUFFER_RDY 		(DDR_BASEADDR + 0x2000000)
#define DVS_BUFFER_COMMIT_IDX 		(DDR_BASEADDR + 0x1500000)
#define DVS_BUFFER_BASEADDR (DDR_BASEADDR + (0x30000000))
#define DVS_BUFFER_NUM		256
#define DVS_BUFFER_SIZE		(DVS_FRAME_SIZE_IN_BYTE +DVS_FRAME_HEADER_SIZE_IN_BYTE)  //include header: 0x2a308 = 960 * 720 * (2bit) / 8bit + 8
#define DVS_BUFFER_TOTAL_SIZE 	DVS_BUFFER_NUM * DVS_BUFFER_SIZE
#define DVS_BUFFER_HIGH  DVS_BUFFER_BASEADDR + DVS_BUFFER_TOTAL_SIZE - 1

#define BD_LEN				    DVS_BUFFER_SIZE
#define COALESCING_COUNT	1
#define DELAY_TIMER_COUNT 	0
#define RX_BD_SPACE_BASE	(DDR_BASEADDR + 0x3000000)
#define RX_BD_SPACE_HIGH	(RX_BD_SPACE_BASE + DVS_BUFFER_NUM * 0x40 - 1)


#define FIL_BUFFER_RDY 			(DDR_BASEADDR + (0x2500000))
#define FIL_BUFFER_COMMIT_IDX 	(DVS_BUFFER_COMMIT_IDX + 0x10)
#define FIL_BUF_BASE_ADDR		(DDR_BASEADDR + (0x40000000))
#define FIL_RUN_MARGIN		6//6 **The number of frames must exceed the minimum required for the filter to function properly.
#define	FIL_BUFFER_NUM	 256
#define FIL_BUF_SIZE		(DVS_FRAME_SIZE_IN_BYTE + DVS_FRAME_HEADER_SIZE_IN_BYTE)	//(960*720*2)/8 byte
#define FIL_BUF_TOTAL_SIZE	(FIL_BUF_SIZE * FIL_BUFFER_NUM)
#define FIL_BUF_END_ADDR 	(FIL_BUF_BASE_ADDR+FIL_BUF_TOTAL_SIZE-1)


#ifdef __cplusplus
}
#endif

#endif /* PIPELINE_PROGRAM_H_ */
