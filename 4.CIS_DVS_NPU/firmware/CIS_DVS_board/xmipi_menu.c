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


/************************** Constant Definitions *****************************/
extern u8 Edid[];
extern u8 TxRestartColorbar;

/***************** Macros (Inline Functions) Definitions *********************/

/**************************** Type Definitions *******************************/

/* Pointer to the menu handling functions */
typedef XMipi_MenuType XMipi_MenuFuncType(XMipi_Menu *InstancePtr, u8 Input);

/************************** Function Prototypes ******************************/
static XMipi_MenuType XMipi_MainMenu(XMipi_Menu *InstancePtr, u8 Input);
static XMipi_MenuType XMipi_ResolutionMenu(XMipi_Menu *InstancePtr, u8 Input);
static XMipi_MenuType XMipi_LanesMenu(XMipi_Menu *InstancePtr, u8 Input);

static void XMipi_DisplayMainMenu(void);
static void XMipi_DisplayResolutionMenu(void);
static void XMipi_DisplayLanesMenu(void);

extern void Info(void);

extern void CloneTxEdid(void);

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
				XMipi_MainMenu, XMipi_ResolutionMenu, XMipi_LanesMenu };

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
	InstancePtr->WaitForColorbar = (FALSE);

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
	xil_printf("s - Select Video Source : Sensor. \n\r");
	xil_printf("t - Select Video Source : TPG. \n\r");
	xil_printf("h - Select Display Device : HDMI\n\r");
	xil_printf("d - Select Display Device : DSI\n\r");
	xil_printf("r - Change the video resolution 720p/1080p/4K.\n\r");
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
static XMipi_MenuType XMipi_MainMenu(XMipi_Menu *InstancePtr, u8 Input) {

	XMipi_MenuType Menu;

	/* Default */
	Menu = XMIPI_MAIN_MENU;

	switch (Input) {

		case ('s'):
		case ('S'):
			/* xil_printf("%s : Sensor input\r\n", __func__); */
			/* Show sensor output */
			Menu = XMIPI_MAIN_MENU;

			if (Pipeline_Cfg.CameraPresent == TRUE)
				New_Cfg.VideoSrc = XVIDSRC_SENSOR;
			else
xil_printf(TXT_RED "Switch failed as no Camera Sensor connected!\r\n" TXT_RST);
			break;

		case ('t'):
		case ('T'):
			/* xil_printf("%s : TPG input\r\n", __func__); */
			/* Show TPG output */
			Menu = XMIPI_MAIN_MENU;
			New_Cfg.VideoSrc = XVIDSRC_TPG;
			break;

		case ('h'):
		case ('H'):
			/* xil_printf("%s : HDMI output\r\n", __func__); */
			/* Use HDMI as output */
			/* This should drive TREADY of Video scaler high */
			Menu = XMIPI_MAIN_MENU;
			break;

		case ('d'):
		case ('D'):
			/* xil_printf("%s : DSI output\r\n", __func__); */
			/* Use DSI as output */
			/* This should drive TREADY of HDMI Tx SS as high */
			Menu = XMIPI_MAIN_MENU;

			if (Pipeline_Cfg.DSIDisplayPresent == TRUE) {
				New_Cfg.VideoDestn = XVIDDES_DSI;
			} else {
xil_printf(TXT_RED "Invalid selection -" TXT_RST);
xil_printf(TXT_RED "The DSI Display panel is not connected\r\n" TXT_RST);
			}
			break;

			/* Resolution */
		case ('r'):
		case ('R'):

			/* Default */
			Menu = XMIPI_MAIN_MENU;
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
 * This function displays the MIPI resolution menu.
 *
 * @return	None
 *
 * @note	None
 *
 ******************************************************************************/
void XMipi_DisplayResolutionMenu(void) {
	xil_printf("\r\n");
	xil_printf(TXT_CYAN);
	xil_printf("---------------------------\r\n");
	xil_printf("---   RESOLUTION MENU   ---\r\n");
	xil_printf("---------------------------\r\n");
	xil_printf(" 1 - 1280 x  720p@60fps\r\n");
	xil_printf(" 2 - 1920 x 1080p@30fps\r\n");
	xil_printf(" 3 - 1920 x 1080p@60fps\r\n");
	xil_printf(" 4 - 3840 x 2160p@30fps\r\n");
	xil_printf(" 5 - 3840 x 2160p@60fps\r\n");

	xil_printf("99 - Exit\n\r");
	xil_printf("Enter Selection -> ");
	xil_printf(TXT_RST);
}

/*****************************************************************************/
/**
 *
 * This function implements the MIPI resolution menu state.
 *
 * @param	input is the value used for the next menu state decoder.
 *
 * @return	The next menu state.
 *
 * @note	None
 *
 ******************************************************************************/
static XMipi_MenuType XMipi_ResolutionMenu(XMipi_Menu *InstancePtr, u8 Input) {

	/* Variables */
	XMipi_MenuType Menu;
	XVidC_VideoMode VideoMode;

	/* Default */
	Menu = XMIPI_RESOLUTION_MENU;
	VideoMode = XVIDC_VM_NO_INPUT;

	switch (Input) {

		/* 1280 x 720p @ 60fps */
		case 1:
			New_Cfg.VideoMode = XVIDC_VM_1280x720_60_P;
			break;

			/* 1920 x 1080p @ 30fps */
		case 2:
			New_Cfg.VideoMode = XVIDC_VM_1920x1080_30_P;
			break;

			/* 1920 x 1080p @ 60fps */
		case 3:
			New_Cfg.VideoMode = XVIDC_VM_1920x1080_60_P;
			break;

			/* 3840 x 2160p @ 30fps */
		case 4:
			New_Cfg.VideoMode = XVIDC_VM_3840x2160_30_P;
			break;

			/* 3840 x 2160p @ 60fps */
		case 5:

			if (Pipeline_Cfg.VideoDestn == XVIDDES_HDMI)
			{
xil_printf(TXT_RED "\n\rCurrent Resolution: XVIDC_VM_3840x2160_60_P\n\r");
xil_printf("Current HDMI configuration doesn't support it\n\r");
xil_printf("Switching to supported maximum resolution:" TXT_RST);
xil_printf(" XVIDC_VM_3840x2160_30_P\n\r" TXT_RST);
				New_Cfg.VideoMode = XVIDC_VM_3840x2160_30_P;
			} else
			{
				New_Cfg.VideoMode = XVIDC_VM_3840x2160_60_P;
			}
			break;
			/* Exit */
		case 99:
			xil_printf("\n\rReturning to main menu.\n\r");
			Menu = XMIPI_MAIN_MENU;
			return Menu;

		default:
			xil_printf(TXT_RED "Unknown option\n\r" TXT_RST);
			PrintPipeConfig();
			XMipi_DisplayResolutionMenu();
			return Menu;
	}

	if (New_Cfg.VideoMode == Pipeline_Cfg.VideoMode) {
		/* Nothing needs to be done */
		xil_printf(TXT_RED "Selected and Current resolution are same!\r\n" );
		xil_printf(TXT_RST);
		PrintPipeConfig();
		XMipi_DisplayResolutionMenu();
		return Menu;
	} else {
		Pipeline_Cfg.VideoMode = New_Cfg.VideoMode;
		Pipeline_Cfg.ActiveLanes = New_Cfg.ActiveLanes;
	}

	/* shutdown pipeline */
	CamReset();
	DisableImageProcessingPipe();
	Reset_IP_Pipe();
	DisableScaler();
	rd_ptr = 4 ; wr_ptr = 0; frmrd_start  = 0;
	frm_cnt = 0; frm_cnt1 = 0;
	/* Enable pipeline */
	start_csi_cap_pipe(Pipeline_Cfg.VideoMode);

	InitVprocSs_Scaler(0);
	InitImageProcessingPipe();
	EnableCSI();
	usleep(20000);
	SetupCameraSensor();

	usleep(200000);
	StartSensor();

	usleep(200000);
	TxRestartColorbar = 1;
	PrintPipeConfig();

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

	if ((InstancePtr->WaitForColorbar) && (!TxBusy)) {
		InstancePtr->WaitForColorbar = (FALSE);
		xil_printf(TXT_CYAN "Enter Selection -> " TXT_RST);
	}

	/* Check if the uart has any data */
	else if (XUartPs_IsReceiveData(InstancePtr->UartBaseAddress)) {

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

/*****************************************************************************/
/**
 *
 * This function implements the MIPI Lanes menu state.
 *
 * @param	input is the value used for the next menu state decoder.
 *
 * @return	The next menu state.
 *
 * @note	None
 *
 ******************************************************************************/
static XMipi_MenuType XMipi_LanesMenu(XMipi_Menu *InstancePtr, u8 Input) {

	/* Variables */
	XMipi_MenuType Menu;

	/* Default */
	Menu = XMIPI_CSI_LANES_MENU;

	switch (Input) {

		/* Single Lane */
		case 1:
			New_Cfg.ActiveLanes = 1;
			break;

			/* Dual Lane */
		case 2:
			New_Cfg.ActiveLanes = 2;
			break;

			/* Quad Lane */
		case 4:
			New_Cfg.ActiveLanes = 4;
			break;

			/* Exit */
		case 99:
			xil_printf("Returning to main menu.\n\r");
			Menu = XMIPI_MAIN_MENU;
			return Menu;

		default:
			xil_printf(TXT_RED "Unknown option\n\r" TXT_RST);
			PrintPipeConfig();
			return Menu;
	}

	if (New_Cfg.ActiveLanes == Pipeline_Cfg.ActiveLanes) {
		/* Nothing needs to be done */
		xil_printf(TXT_RED "Selected and Current lanes are same!\r\n" TXT_RST);
		PrintPipeConfig();
		return Menu;
	} else {
		Pipeline_Cfg.VideoMode = New_Cfg.VideoMode;
		Pipeline_Cfg.ActiveLanes = New_Cfg.ActiveLanes;
	}

	/* shutdown and enable pipeline */
	DisableImageProcessingPipe();
	Reset_IP_Pipe();
	CamReset();
	rd_ptr = 4 ; wr_ptr = 0; frmrd_start  = 0;
	frm_cnt = 0; frm_cnt1 = 0;
	usleep(20000);
	InitImageProcessingPipe();
	EnableCSI();
	usleep(20000);
	SetupCameraSensor();
	StartSensor();

	usleep(200000);


	TxRestartColorbar = 1;

	PrintPipeConfig();

	return Menu;
}
