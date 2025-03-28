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

extern void DmaWriteDoneCallback (XAxiDma_BdRing * RxRingPtr);

extern void configure_buffer_system();
extern XIicPs	IicPsInstance; /* The instance of the PS IIC device */
extern int InitDVSIIC(void);
extern int SetupIicDVSInterruptSystem(void);
extern void DVSIICStatusHandler(void *CallBackRef, u32 Event);
extern int StartDVSSensor(void);
extern int ProgramDVSSensor(void);
extern int DVSWriteData(u16 ByteCount);
extern int start_dvs_cap_pipe();
extern void config_dvs_cap_path();
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

#ifdef __cplusplus
}
#endif

#endif /* PIPELINE_PROGRAM_H_ */
