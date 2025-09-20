#ifndef DMA_EXAMPLE_H   /* prevent circular inclusions */
#define DMA_EXAMPLE_H   /* by using protection macros */


static int RxBDRun(XAxiDma_BdRing *RxRingPtr, u32 NumBDsPerSingleRx);
static int TxBDRun(XAxiDma_BdRing *TxRingPtr, u32 NumBDsPerSingleTx);
static int RxBDRetrieve(XAxiDma_BdRing* RxRingPtr, u32 NumBDsPerSingleRx);
static int TxBDRetrieve(XAxiDma_BdRing* TxRingPtr, u32 NumBDsPerSingleTx);

static int GenerateAllBDTable(void);
static int GenerateBDTable(XAxiDma_BdRing* RingPtr, u8 TrxType, u8 *BaseAddr, u32 BDLengthPerFrm, u32 SingleBDLength, u32 NumBDsPerSingleTrx, u32 NumOfBDPerFrm, u8 NumOfAlternatingFrm);

static int TrxSetup_v2(u8 Mode);

static int RxBDRingInit(XAxiDma_BdRing *RxRingPtr, char* RxBDBufBase, u32 NumBDsPerSingleRx, u32 NumOfBDGroup);
static int TxBDRingInit(XAxiDma_BdRing *TxRingPtr, char* TxBDBufBase, u32 NumBDsPerSingleTx, u32 NumOfBDGroup);
static int GetDMAHardwareSpecificationAndPutIntoInstance(XAxiDma * AxiDmaInstPtr, u32 DeviceId);


static void DmaFILTxIntrHandler(XAxiDma_BdRing *TxRingPtr);
static void DmaFILTxDoneCallBack(XAxiDma_BdRing *TxRingPtr);
static void DmaFILRxIntrHandler(XAxiDma_BdRing *RxRingPtr);
static void DmaFILRxDoneCallBack(XAxiDma_BdRing *RxRingPtr);



static void FrameBufferPointerInit();


static void UpdateFilWRBufPtr(u32 *pCurBufPtr, u32 uiStep);
static void UpdateFilRDBufPtr(u32 *pCurBufPtr, u32 uiStep);

static int XAxiDma_CustomBdRingToHw(XAxiDma_BdRing * RingPtr, int NumBd, XAxiDma_Bd * BdSetPtr);



#endif
