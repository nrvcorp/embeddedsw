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

#include "xiicps.h"
#include "sensor_cfgs.h"

// GPIO define
#define GPIO_DEVICE_ID	XPAR_GPIO_0_DEVICE_ID

// IIC define
#define IIC_DEVICE_ID		XPAR_XIICPS_1_DEVICE_ID
#define INTC_DEVICE_ID		XPAR_SCUGIC_SINGLE_DEVICE_ID
#define IIC_SCLK_RATE		100000
#define SLV_MON_LOOP_COUNT 0x000FFFFF	/**< Slave Monitor Loop Count*/
#define MAX_CHANNELS 0xf
#define MUX_ADDRESS			0x75
#define MUX_CHANNEL			0x01

//DVS define
#define DVS_RSTN 0x01
#define DVS_DELAY		10000000
#define DVS_CHANNEL		1
#define DVS_ADDR		(0x20>>1)	// DVS

#ifdef __cplusplus
extern "C" {
#endif

extern u32 SetupDSI(void);
extern u32 InitStreamMuxGpio(void);
extern u32 InitializeCFA(void);
//extern u32 InitializeCsiRxSs(void);
// dphy initialize
extern u32 InitializeDphy(void);
extern u32 InitializeVdma(void);

extern void SetColorDepth(void);
extern void SelectDSIOuptut(void);
extern void SelectHDMIOutput(void);

//extern int InitIIC(void);
extern void SetupIICIntrHandlers(void);
extern int MuxInit(void);
extern int SetupIOExpander(void);
extern int SetupCameraSensor(void);
extern void InitDemosaicGammaCSC(void);
extern int StartSensor(void);
extern int StopSensor(void);
extern void InitCSI2RxSS2CFA_Vdma(void);
extern void InitCSC2TPG_Vdma(void);

extern int ToggleSensorPattern(void);
extern void PrintPipeConfig(void);

//extern void DisableCSI(void);
extern void DisableCFAVdma(void);
extern void DisableCFA(void);
extern void DisableTPGVdma(void);
extern void DisableScaler(void);
extern void DisableDSI(void);
extern void InitDSI(void);
extern void InitVprocSs_Scaler(int count);
//extern void EnableCSI(void);




/************************** Function Prototypes *****************************/
//extern void CsiRxPrintRegStatus(void);

extern int StartSetupDVSSensor(void);
extern int StartDVSSensor(void);
extern int SetupDVSSensor(void);
extern int ProgramDVSSensor(void);
extern int SetupIicPsInterruptSystem(void);
extern int InitIIC(void);
extern int MuxInitChannel(u16 MuxIicAddr, u8 WriteBuffer);
extern void Handler(void *CallBackRef, u32 Event);
extern int SensorWriteData(u16 ByteCount);
extern int SensorReadData(u8 *BufferPtr, u16 ByteCount);
extern void Handler(void *CallBackRef, u32 Event);

extern XIicPs	IicPsInstance; /* The instance of the PS IIC device */

#ifdef __cplusplus
}
#endif

#endif /* PIPELINE_PROGRAM_H_ */
