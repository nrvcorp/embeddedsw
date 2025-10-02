#ifndef DMA_EXAMPLE_H   /* prevent circular inclusions */
#define DMA_EXAMPLE_H   /* by using protection macros */

int FilRxInit();
int FilRxSetAndRun();

int FilTxInit();
int FilTxSetAndRun();

void DmaFILTxIntrHandler(int *FilTxPtr);

void DmaFILRxIntrHandler(int *FilTxPtr);


void FrameBufferPointerInit();


void UpdateFilWRBufPtr(u32 *pCurBufPtr, u32 uiStep);
void UpdateFilRDBufPtr(u32 *pCurBufPtr, u32 uiStep);

#endif
