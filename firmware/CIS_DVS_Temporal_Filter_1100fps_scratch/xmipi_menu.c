/******************************************************************************
* Copyright (C) 2017 - 2020 Xilinx, Inc.  All rights reserved.
* SPDX-License-Identifier: MIT
 ******************************************************************************/

/*****************************************************************************/
/**
 *
 * @file xmipi_menu.c
 *
 * This file contains the Xilinx Menu implementation as used
 * in the MIPI example design. Please see xmipi_menu.h for more details.
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

/***************************** Include Files *********************************/
#include "xil_cache.h"
#include "sleep.h"
#include "xmipi_menu.h"

#include "xvprocss.h"
#include "sensor_cfgs.h"
#include "pipeline_program.h"

#include "xv_frmbufwr_l2.h"

#include "DVS_5x5x5_Filter.h"


/************************** Constant Definitions *****************************/
extern u8 Edid[];
extern u8 TxRestartColorbar;

/***************** Macros (Inline Functions) Definitions *********************/

/**************************** Type Definitions *******************************/

/* Pointer to the menu handling functions */
typedef XMipi_MenuType XMipi_MenuFuncType(XMipi_Menu *InstancePtr, u16 Input);

/************************** Function Prototypes ******************************/
static XMipi_MenuType XMipi_MainMenu(XMipi_Menu *InstancePtr, u16 Input);
static XMipi_MenuType XMipi_OnThreshMenu(XMipi_Menu *InstancePtr, u16 Input);
static XMipi_MenuType XMipi_OffThreshMenu(XMipi_Menu *InstancePtr, u16 Input);
static XMipi_MenuType XMipi_ThreshMenu(XMipi_Menu *InstancePtr, u16 Input);

static void XMipi_DisplayMainMenu(void);
static void XMipi_DisplayCurrentThreshold(void);
static void XMipi_DisplayThresholdMenu(u16 option);


/************************* Variable Definitions *****************************/

extern XVprocSs scaler_new_inst;
extern XPipeline_Cfg Pipeline_Cfg;
extern XPipeline_Cfg New_Cfg;

extern XV_FrmbufWr_l2     frmbufwr;

extern u32 rd_ptr;
extern u32 wr_ptr ;
extern u32 frmrd_start;
extern u32 frm_cnt ;
extern u32 frm_cnt1;

/**
 * This table contains the function pointers for all possible states.
 * The order of elements must match the XMipi_MenuType enumerator definitions.
 */
static XMipi_MenuFuncType* const XMipi_MenuTable[XMIPI_NUM_MENUS] = {
				XMipi_MainMenu, XMipi_OnThreshMenu, XMipi_OffThreshMenu, XMipi_ThreshMenu};

extern u8 IsPassThrough; /**< Demo mode 0-colorbar 1-pass through */
extern u8 TxBusy;  /* TX busy flag is set while the TX is initialized */

/************************** Function Definitions *****************************/

extern void Reset_IP_Pipe(void);
extern void CamReset(void);

/*****************************************************************************/
/**
 *
 * This function takes care of the MIPI menu initialization.
 *
 * @param	InstancePtr is a pointer to the XMipi_Menu instance.
 * @param	UartBaseAddress points to the base address of PS uart.
 *
 * @return	None
 *
 * @note	None
 *
 ******************************************************************************/
void XMipi_MenuInitialize(XMipi_Menu *InstancePtr, u32 UartBaseAddress) {

	/* Verify argument. */
	Xil_AssertVoid(InstancePtr != NULL);

	/* copy configuration settings */
	InstancePtr->CurrentMenu = XMIPI_MAIN_MENU;
	InstancePtr->UartBaseAddress = UartBaseAddress;
	InstancePtr->Value = 0;

	/* Show main menu */
	XMipi_DisplayMainMenu();
}

/*****************************************************************************/
/**
 *
 * This function resets the menu to the main menu.
 *
 * @param	InstancePtr is a pointer to the XMipi_Menu instance.
 *
 * @return	None
 *
 * @note	None
 *
 ******************************************************************************/
void XMipi_MenuReset(XMipi_Menu *InstancePtr) {
	InstancePtr->CurrentMenu = XMIPI_MAIN_MENU;
}

/*****************************************************************************/
/**
 *
 * This function displays the MIPI main menu.
 *
 * @return	None
 *
 * @note	None
 *
 ******************************************************************************/
void XMipi_DisplayMainMenu(void) {
	xil_printf("\r\n");
	xil_printf(TXT_CYAN);
	xil_printf("---------------------\r\n");
	xil_printf("---   MAIN MENU   ---\r\n");
	xil_printf("---------------------\r\n");
	xil_printf("b - Set DVS Filter Threshold : ON, OFF. \n\r");
	xil_printf("p - Set DVS Filter Threshold : ON. \n\r");
	xil_printf("n - Set DVS Filter Threshold : OFF. \n\r");
	xil_printf("t - Show DVS Filter Current Threshold. \n\r");
	xil_printf("\n\r\n\r");
	xil_printf(TXT_RST);
}

/*****************************************************************************/
/**
 *
 * This function implements the MIPI main menu state.
 *
 * @param	input is the value used for the next menu state decoder.
 *
 * @return	The next menu state.
 *
 * @note	None
 *
 ******************************************************************************/
static XMipi_MenuType XMipi_MainMenu(XMipi_Menu *InstancePtr, u16 Input) {

	XMipi_MenuType Menu;

	/* Default */
	Menu = XMIPI_MAIN_MENU;

	switch (Input) {
		case ('b'):
		case ('B'):
			Menu = XMIPI_THRESH_MENU;
			XMipi_DisplayThresholdMenu('b');
			break;
		case ('n'):
		case ('N'):
			Menu = XMIPI_OFF_THRESH_MENU;
			XMipi_DisplayThresholdMenu('n');
			break;
		case ('p'):
		case ('P'):
			Menu = XMIPI_ON_THRESH_MENU;
			XMipi_DisplayThresholdMenu('p');
			break;
		case ('t'):
		case ('T'):
			XMipi_DisplayCurrentThreshold();
			break;
		default:
			XMipi_DisplayMainMenu();
			Menu = XMIPI_MAIN_MENU;
			Xil_DCacheDisable();
			break;
	}
	return Menu;
}

/*****************************************************************************/
/**
 *
 * This function displays the DVS Filter threshold menu.
 *
 * @return	None
 *
 * @note	None
 *
 ******************************************************************************/
void XMipi_DisplayThresholdMenu(u16 option) {
	xil_printf("\r\n");
	xil_printf(TXT_CYAN);
	xil_printf("---------------------------\r\n");
	xil_printf("---   THRESHOLD  MENU   ---\r\n");
	switch(option){
		case('b'):
	xil_printf("---   (ON & OFF th)     ---\r\n");
			break;
		case('n'):
	xil_printf("---   (OFF th)          ---\r\n");
			break;
		case('p'):
	xil_printf("---   (ON th)           ---\r\n");
			break;
	}
	xil_printf("---------------------------\r\n");
	xil_printf("0 ~ 125 is available \r\n");

	xil_printf("999 - Exit\n\r");
	xil_printf("Enter Selection -> ");
	xil_printf(TXT_RST);
}

void XMipi_DisplayCurrentThreshold(void){
	// Reg1: Set Threshold  ==>> [13:7]OffTh,  [6:0]OnTh
	// Reg0: Run Filter ==>> 1: Run
	u32 Threshold = DVS_5X5X5_FILTER_mReadReg(XPAR_DVS_5X5X5_FILTER_0_AXI_LITE_BASEADDR, DVS_5X5X5_FILTER_AXI_Lite_SLV_REG1_OFFSET);
	u32 OffTh = (Threshold & 0b11111110000000)>>7;
	u32 OnTh  = Threshold & 0b00000001111111;
	xil_printf("\r\n");
	xil_printf(TXT_CYAN);
	xil_printf("-------------------------------\r\n");
	xil_printf("---   CURRENT THRESHOLD     ---\r\n");
	xil_printf("      OFF THRESHOLD : %d\r\n",OffTh);
	xil_printf("      ON THRESHOLD : %d\r\n",OnTh);
	xil_printf("-------------------------------\r\n");

	xil_printf(TXT_RST);
}

/*****************************************************************************/
/**
 *
 * This function %%
 *
 * @param	input is the value used for the next menu state decoder.
 *
 * @return	The next menu state.
 *
 * @note	None
 *
 ******************************************************************************/
static XMipi_MenuType XMipi_OnThreshMenu(XMipi_Menu *InstancePtr, u16 Input) {

	/* Variables */
	XMipi_MenuType Menu;

	/* Default */
	Menu = XMIPI_ON_THRESH_MENU;
	if ((Input >= 0) && (Input <= 125)){
		// Reg1: Set Threshold  ==>> [13:7]OffTh,  [6:0]OnTh
		// Reg0: Run Filter ==>> 1: Run
		u32 Threshold = DVS_5X5X5_FILTER_mReadReg(XPAR_DVS_5X5X5_FILTER_0_AXI_LITE_BASEADDR, DVS_5X5X5_FILTER_AXI_Lite_SLV_REG1_OFFSET);
		Threshold &= 0b11111110000000;
		Threshold |= Input;
		DVS_5X5X5_FILTER_mWriteReg(XPAR_DVS_5X5X5_FILTER_0_AXI_LITE_BASEADDR, DVS_5X5X5_FILTER_AXI_Lite_SLV_REG1_OFFSET, Threshold);
		xil_printf("\n\rDone\n\r");
		XMipi_DisplayMainMenu();
		Menu = XMIPI_MAIN_MENU;
	}
	else if (Input == 999){
		xil_printf("\n\rReturning to main menu.\n\r");
		Menu = XMIPI_MAIN_MENU;
		return Menu;
	}
	else{
		xil_printf(TXT_RED "Unknown option\n\r" TXT_RST);
		XMipi_DisplayThresholdMenu('p');
		return Menu;
	}

	return Menu;
}
static XMipi_MenuType XMipi_OffThreshMenu(XMipi_Menu *InstancePtr, u16 Input) {

	/* Variables */
	XMipi_MenuType Menu;

	/* Default */
	Menu = XMIPI_OFF_THRESH_MENU;
	if ((Input >= 0) && (Input <= 125)){
		// Reg1: Set Threshold  ==>> [13:7]OffTh,  [6:0]OnTh
		// Reg0: Run Filter ==>> 1: Run
		u32 Threshold = DVS_5X5X5_FILTER_mReadReg(XPAR_DVS_5X5X5_FILTER_0_AXI_LITE_BASEADDR, DVS_5X5X5_FILTER_AXI_Lite_SLV_REG1_OFFSET);
		Threshold &= 0b00000001111111;
		Threshold |= Input<<7;
		DVS_5X5X5_FILTER_mWriteReg(XPAR_DVS_5X5X5_FILTER_0_AXI_LITE_BASEADDR, DVS_5X5X5_FILTER_AXI_Lite_SLV_REG1_OFFSET, Threshold);
		xil_printf("\n\rDone\n\r");
		XMipi_DisplayMainMenu();
		Menu = XMIPI_MAIN_MENU;
	}
	else if (Input == 999){
		xil_printf("\n\rReturning to main menu.\n\r");
		XMipi_DisplayMainMenu();
		Menu = XMIPI_MAIN_MENU;
		return Menu;
	}
	else{
		xil_printf(TXT_RED "Unknown option\n\r" TXT_RST);
		XMipi_DisplayThresholdMenu('n');
		return Menu;
	}

	return Menu;
}
static XMipi_MenuType XMipi_ThreshMenu(XMipi_Menu *InstancePtr, u16 Input) {

	/* Variables */
	XMipi_MenuType Menu;

	/* Default */
	Menu = XMIPI_THRESH_MENU;
	if ((Input >= 0) && (Input <= 125)){
		// Reg1: Set Threshold  ==>> [13:7]OffTh,  [6:0]OnTh
		// Reg0: Run Filter ==>> 1: Run
		u32 Threshold = Input<<7 | Input;
		DVS_5X5X5_FILTER_mWriteReg(XPAR_DVS_5X5X5_FILTER_0_AXI_LITE_BASEADDR, DVS_5X5X5_FILTER_AXI_Lite_SLV_REG1_OFFSET, Threshold);
		xil_printf("\n\rDone\n\r");
		XMipi_DisplayMainMenu();
		Menu = XMIPI_MAIN_MENU;
	}
	else if (Input == 999){
		xil_printf("\n\rReturning to main menu.\n\r");
		Menu = XMIPI_MAIN_MENU;
		XMipi_DisplayMainMenu();
		return Menu;
	}
	else{
		xil_printf(TXT_RED "Unknown option\n\r" TXT_RST);
		XMipi_DisplayThresholdMenu('b');
		return Menu;
	}

	return Menu;
}
/*****************************************************************************/
/**
 *
 * This function is called to trigger the MIPI menu state machine.
 *
 * @param	InstancePtr is a pointer to the XMipi_Menu instance.
 *
 * @return	None
 *
 * @note	None
 *
 ******************************************************************************/
void XMipi_MenuProcess(XMipi_Menu *InstancePtr) {
	u8 Data;

	/* Verify argument. */
	Xil_AssertVoid(InstancePtr != NULL);



	/* Check if the uart has any data */
	if (XUartPs_IsReceiveData(InstancePtr->UartBaseAddress)) {

		/* Read data from uart */
		Data = XUartPs_RecvByte(InstancePtr->UartBaseAddress);

		/* Main menu */
		if (InstancePtr->CurrentMenu == XMIPI_MAIN_MENU) {
			InstancePtr->CurrentMenu =
				XMipi_MenuTable[InstancePtr->CurrentMenu](InstancePtr, Data);
			InstancePtr->Value = 0;
		}

		/* Sub menu */
		else {

			/* Send response to user */
			XUartPs_SendByte(InstancePtr->UartBaseAddress, Data);

			/* Alpha numeric data */
			if (isalpha(Data)) {
xil_printf(TXT_RED "\r\nInvalid input."TXT_RST);
xil_printf(TXT_RED "Valid entry is only digits 0-9. \r\n\r\n"TXT_RST);
xil_printf(TXT_RED " Try again\r\n\r\n"TXT_RST);
xil_printf(TXT_CYAN "Enter Selection -> " TXT_RST);
				InstancePtr->Value = 0;
			}

			/* Numeric data */
			else if ((Data >= '0') && (Data <= '9')) {
				InstancePtr->Value = InstancePtr->Value * 10 + (Data - '0');
			}

			/* Backspace */
			else if (Data == '\b') {
				InstancePtr->Value = InstancePtr->Value / 10; /*discard previous input */
			}

			/* Execute */
			else if ((Data == '\n') || (Data == '\r')) {
				InstancePtr->CurrentMenu =
					XMipi_MenuTable[InstancePtr->CurrentMenu](InstancePtr, InstancePtr->Value);
				InstancePtr->Value = 0;
			}
		}
	}
}
