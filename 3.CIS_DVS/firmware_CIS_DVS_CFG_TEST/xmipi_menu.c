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
#include "debug_config.h"

#include "xv_frmbufwr_l2.h"

#include "DVS_Adaptive_Filter.h"

#include "debug_config.h"


/************************** Constant Definitions *****************************/
extern u8 Edid[];
extern u8 TxRestartColorbar;

/***************** Macros (Inline Functions) Definitions *********************/

/**************************** Type Definitions *******************************/

/* Pointer to the menu handling functions */
typedef XMipi_MenuType XMipi_MenuFuncType(XMipi_Menu *InstancePtr, u16 Input);

/************************** Function Prototypes ******************************/
static XMipi_MenuType XMipi_MainMenu(XMipi_Menu *InstancePtr, u16 Input);
#if(ENABLE_DVS_FILTER)
static XMipi_MenuType XMipi_OnThreshMenu(XMipi_Menu *InstancePtr, u16 Input);
static XMipi_MenuType XMipi_OffThreshMenu(XMipi_Menu *InstancePtr, u16 Input);
static XMipi_MenuType XMipi_ThreshMenu(XMipi_Menu *InstancePtr, u16 Input);
#endif
#if(ENABLE_DVS_RESET)
static XMipi_MenuType XMipi_DVSResetMenu(XMipi_Menu *InstancePtr, u16 Input);
#endif
static XMipi_MenuType XMipi_ResetDebugMenu(XMipi_Menu *InstancePtr, u16 Input);

static void XMipi_DisplayMainMenu(void);
#if(ENABLE_DVS_FILTER)
static void XMipi_DisplayCurrentThreshold(void);
static void XMipi_DisplayThresholdMenu(u16 option);
#endif
#if(ENABLE_DVS_RESET)
static void XMipi_DisplayDVSResetMenu();
#endif
static void XMipi_DisplayResetDebugMenu(void);

/************************* Variable Definitions *****************************/

extern XVprocSs scaler_new_inst;
extern XPipeline_Cfg Pipeline_Cfg;
extern XPipeline_Cfg New_Cfg;

extern XV_FrmbufWr_l2     frmbufwr;

u8 is_user_input_active = 0;

#if(CHECK_FRAME_COUNT)
extern u32 frm_cnt; // cis
extern u32 dvs_frm_cnt; // dvs
	#if(ENABLE_DVS_FILTER)
	extern u32 fil_frm_cnt; // fil
	#endif
#endif

#if(CHECK_DVS_FRAME_DROP)
extern int frame_drop_count_dvs;
	#if(CHECK_DVS_MULTIPLE_FRAME_DROP)
extern int multiple_frame_drop_count_dvs;
	#endif
#endif

#if(CHECK_DVS_BUFFER_HOST_DRAIN && ENABLE_HOST_DIRECT_DVS_ACCESS)
extern int host_delay_count_dvs;
#endif

#if(CHECK_FIL_BUFFER_HOST_DRAIN && ENABLE_DVS_FILTER)
extern int host_delay_count_filter;
#endif
#if(ENABLE_DVS_RESET)
extern u8 dvs_reset;
#endif


/**
 * This table contains the function pointers for all possible states.
 * The order of elements must match the XMipi_MenuType enumerator definitions.
 */
static XMipi_MenuFuncType* const XMipi_MenuTable[XMIPI_NUM_MENUS] = {
				XMipi_MainMenu,
#if(ENABLE_DVS_FILTER)
				XMipi_OnThreshMenu,
				XMipi_OffThreshMenu,
				XMipi_ThreshMenu,
#endif
#if(ENABLE_DVS_RESET)
				XMipi_DVSResetMenu,
#endif
				XMipi_ResetDebugMenu
			};

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
#if(ENABLE_DVS_FILTER)
	xil_printf("b - Set DVS Filter Threshold : ON, OFF. \n\r");
	xil_printf("p - Set DVS Filter Threshold : ON. \n\r");
	xil_printf("n - Set DVS Filter Threshold : OFF. \n\r");
	xil_printf("t - Show DVS Filter Current Threshold. \n\r");
#endif
#if(ENABLE_DVS_RESET)
	xil_printf("q - Reset system. \n\r");
#endif
	xil_printf("r - Reset Debug Counters. \n\r");
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
#if(ENABLE_DVS_FILTER)
		case ('b'):
		case ('B'):
			Menu = XMIPI_THRESH_MENU;
			is_user_input_active = 1;
			XMipi_DisplayThresholdMenu('b');
			break;
		case ('n'):
		case ('N'):
			Menu = XMIPI_OFF_THRESH_MENU;
			is_user_input_active = 1;
			XMipi_DisplayThresholdMenu('n');
			break;
		case ('p'):
		case ('P'):
			Menu = XMIPI_ON_THRESH_MENU;
			is_user_input_active = 1;
			XMipi_DisplayThresholdMenu('p');
			break;
		case ('t'):
		case ('T'):
			XMipi_DisplayCurrentThreshold();
			break;
#endif
#if(ENABLE_DVS_RESET)
		case ('q'):
		case ('Q'):
			Menu = XMIPI_DVS_RESET_MENU;
			is_user_input_active = 1;
			XMipi_DisplayDVSResetMenu();
			break;
#endif
		case ('r'):
		case ('R'):
			Menu = XMIPI_RESET_DEBUG_MENU;
			is_user_input_active = 1;
			XMipi_DisplayResetDebugMenu();
			break;
		default:
			XMipi_DisplayMainMenu();
			Menu = XMIPI_MAIN_MENU;
			Xil_DCacheDisable();
			break;
	}
	return Menu;
}


#if(ENABLE_DVS_FILTER)

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
	xil_printf("0 ~ %d is available \r\n", MAX_THRESHOLD);

	xil_printf(">%d - Exit\n\r", MAX_THRESHOLD);
	xil_printf("Enter Selection -> ");
	xil_printf(TXT_RST);
}

static u32 popcount(u32 x) {
    x = x - ((x >> 1) & 0x55555555);
    x = (x & 0x33333333) + ((x >> 2) & 0x33333333);
    x = (x + (x >> 4)) & 0x0F0F0F0F;
    x = x + (x >> 8);
    x = x + (x >> 16);
    return x & 0x3F;
}
void XMipi_DisplayCurrentThreshold(void){
	// Reg1: Set Threshold  ==>> [13:7]OffTh,  [6:0]OnTh
	// Reg0: Run Filter ==>> 1: Run
	u32 Threshold = DVS_ADAPTIVE_FILTER_mReadReg(XPAR_DVS_ADAPTIVE_FILTER_0_AXI_LITE_BASEADDR, DVS_ADAPTIVE_FILTER_AXI_Lite_SLV_REG1_OFFSET);
	u32 OffTh = (Threshold & 0xFFFF0000)>>16;
	u32 OnTh  = Threshold & 0x0000FFFF;
	u32 frm_event_cnt = DVS_ADAPTIVE_FILTER_mReadReg(XPAR_DVS_ADAPTIVE_FILTER_0_AXI_LITE_BASEADDR, DVS_ADAPTIVE_FILTER_AXI_Lite_SLV_REG2_OFFSET);
	u32 kern_size = popcount(DVS_ADAPTIVE_FILTER_mReadReg(XPAR_DVS_ADAPTIVE_FILTER_0_AXI_LITE_BASEADDR, DVS_ADAPTIVE_FILTER_AXI_Lite_SLV_REG3_OFFSET));
	xil_printf("\r\n");
	xil_printf(TXT_CYAN);
	xil_printf("-------------------------------\r\n");
	xil_printf("---  CURRENT FILTER STATE   ---\r\n");
	xil_printf("     ON  THRESHOLD : %d\r\n", OnTh);
	xil_printf("     OFF THRESHOLD : %d\r\n", OffTh);
	xil_printf("     2 Frame Event Count : %d / %d\r\n", frm_event_cnt, MAX_EVENTS_2FRAMES);
	xil_printf("     Kernel Size: %dx%d\r\n", (2*kern_size + 1), (2*kern_size + 1));
	xil_printf("-------------------------------\r\n");

	xil_printf(TXT_RST);
}

static XMipi_MenuType XMipi_OnThreshMenu(XMipi_Menu *InstancePtr, u16 Input) {

	/* Variables */
	XMipi_MenuType Menu;

	/* Default */
	Menu = XMIPI_ON_THRESH_MENU;
	if ((Input >= 0) && (Input <= MAX_THRESHOLD)){
		// Reg1: Set Threshold  ==>> [31:16]OffTh,  [15:0]OnTh
		// Reg0: Run Filter ==>> 1: Run
		u32 Threshold = DVS_ADAPTIVE_FILTER_mReadReg(XPAR_DVS_ADAPTIVE_FILTER_0_AXI_LITE_BASEADDR, DVS_ADAPTIVE_FILTER_AXI_Lite_SLV_REG1_OFFSET);
		Threshold &= 0xFFFF0000;
		Threshold |= Input;
		DVS_ADAPTIVE_FILTER_mWriteReg(XPAR_DVS_ADAPTIVE_FILTER_0_AXI_LITE_BASEADDR, DVS_ADAPTIVE_FILTER_AXI_Lite_SLV_REG1_OFFSET, Threshold);
		xil_printf("\n\rDone\n\r");
		is_user_input_active = 0;
		XMipi_DisplayMainMenu();
		Menu = XMIPI_MAIN_MENU;
	}
	else if (Input > MAX_THRESHOLD){
		xil_printf("\n\rReturning to main menu.\n\r");
		is_user_input_active = 0;
		XMipi_DisplayMainMenu();
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
	if ((Input >= 0) && (Input <= MAX_THRESHOLD)){
		// Reg1: Set Threshold  ==>> [31:16]OffTh,  [15:0]OnTh
		// Reg0: Run Filter ==>> 1: Run
		u32 Threshold = DVS_ADAPTIVE_FILTER_mReadReg(XPAR_DVS_ADAPTIVE_FILTER_0_AXI_LITE_BASEADDR, DVS_ADAPTIVE_FILTER_AXI_Lite_SLV_REG1_OFFSET);
		Threshold &= 0x0000FFFF;
		Threshold |= Input<<16;
		DVS_ADAPTIVE_FILTER_mWriteReg(XPAR_DVS_ADAPTIVE_FILTER_0_AXI_LITE_BASEADDR, DVS_ADAPTIVE_FILTER_AXI_Lite_SLV_REG1_OFFSET, Threshold);
		xil_printf("\n\rDone\n\r");
		is_user_input_active = 0;
		XMipi_DisplayMainMenu();
		Menu = XMIPI_MAIN_MENU;
	}
	else if (Input > MAX_THRESHOLD){
		xil_printf("\n\rReturning to main menu.\n\r");
		is_user_input_active = 0;
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
	if ((Input >= 0) && (Input <= MAX_THRESHOLD)){
		// Reg1: Set Threshold  ==>> [31:16]OffTh,  [15:0]OnTh
		// Reg0: Run Filter ==>> 1: Run
		u32 Threshold = Input<<16 | Input;
		DVS_ADAPTIVE_FILTER_mWriteReg(XPAR_DVS_ADAPTIVE_FILTER_0_AXI_LITE_BASEADDR, DVS_ADAPTIVE_FILTER_AXI_Lite_SLV_REG1_OFFSET, Threshold);
		xil_printf("\n\rDone\n\r");
		is_user_input_active = 0;
		XMipi_DisplayMainMenu();
		Menu = XMIPI_MAIN_MENU;
	}
	else if (Input > MAX_THRESHOLD){
		xil_printf("\n\rReturning to main menu.\n\r");
		is_user_input_active = 0;
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

#endif // (ENABLE_DVS_FILTER)

#if(ENABLE_DVS_RESET)
static void XMipi_DisplayDVSResetMenu()
{
	xil_printf("\r\n");
	xil_printf(TXT_CYAN);
	xil_printf("-------------------------------\r\n");
	xil_printf("---      SYSTEM RESET       ---\r\n");
	xil_printf("---   Input 1 to reset DVS  ---\r\n");
	xil_printf("-------------------------------\r\n");
	xil_printf(TXT_RST);
}
static XMipi_MenuType XMipi_DVSResetMenu(XMipi_Menu *InstancePtr, u16 Input)
{
	/* Variables */
	XMipi_MenuType Menu;

	/* Default */
	Menu = XMIPI_DVS_RESET_MENU;
	if (Input==1){
		xil_printf("\n\rSystem will be reset.\n\r");
		dvs_reset=1;
		is_user_input_active = 0;
		XMipi_DisplayMainMenu();
		Menu = XMIPI_MAIN_MENU;
	}
	else{
		xil_printf("\n\rCancelled. Returning to main menu.\n\r");
		is_user_input_active = 0;
		XMipi_DisplayMainMenu();
		Menu = XMIPI_MAIN_MENU;
		return Menu;
	}

	return Menu;
}
#endif // (ENABLE_DVS_RESET)

void XMipi_DisplayResetDebugMenu(void)
{
	xil_printf("\r\n");
	xil_printf(TXT_CYAN);
	xil_printf("---------------------------\r\n");
	xil_printf("---  RESET DEBUG  MENU  ---\r\n");
#if(CHECK_FRAME_COUNT)
	xil_printf("1: Reset Frame Counts\r\n");
#endif
#if(CHECK_DVS_FRAME_DROP)
	xil_printf("2: Reset DVS Frame Drop Count\r\n");
#endif
#if(ENABLE_HOST_DIRECT_DVS_ACCESS && CHECK_DVS_BUFFER_HOST_DRAIN)
	xil_printf("3: Reset Host Delay Count (DVS)\r\n");
#endif
#if(CHECK_FIL_BUFFER_HOST_DRAIN && ENABLE_DVS_FILTER)
	xil_printf("4: Reset Host Delay Count (Filter)\r\n");
#endif
	xil_printf("---------------------------\r\n");

	xil_printf("Any other inputs - Exit\n\r");
	xil_printf("Enter Selection -> ");
	xil_printf(TXT_RST);
}

static XMipi_MenuType XMipi_ResetDebugMenu(XMipi_Menu *InstancePtr, u16 Input) {

	/* Variables */
	XMipi_MenuType Menu;

	/* Default */
	Menu = XMIPI_RESET_DEBUG_MENU;
	if (Input== 0){
		xil_printf("\n\rCancelled. Returning to main menu.\n\r");
		is_user_input_active = 0;
		XMipi_DisplayMainMenu();
		Menu = XMIPI_MAIN_MENU;
		return Menu;
	}
#if(CHECK_FRAME_COUNT)
	else if (Input==1){
		frm_cnt = 0;
		dvs_frm_cnt = 0;
#if(ENABLE_DVS_FILTER)
		fil_frm_cnt = 0;
#endif
		xil_printf("\n\rFrame counts have been reset.\n\r");
		is_user_input_active = 0;
		XMipi_DisplayMainMenu();
		Menu = XMIPI_MAIN_MENU;
	}
#endif
#if(CHECK_DVS_FRAME_DROP)
	else if (Input==2){
		frame_drop_count_dvs = 0;
	#if(CHECK_DVS_MULTIPLE_FRAME_DROP)
		multiple_frame_drop_count_dvs = 0;
	#endif
		xil_printf("\n\rDVS Frame Drop Count has been reset.\n\r");
		is_user_input_active = 0;
		XMipi_DisplayMainMenu();
		Menu = XMIPI_MAIN_MENU;
	}
#endif
#if(ENABLE_HOST_DIRECT_DVS_ACCESS && CHECK_DVS_BUFFER_HOST_DRAIN)
	else if (Input==3){
		host_delay_count_dvs = 0;
		xil_printf("\n\rHost Delay Count(DVS) has been reset.\n\r");
		is_user_input_active = 0;
		XMipi_DisplayMainMenu();
		Menu = XMIPI_MAIN_MENU;
	}
#endif
#if(CHECK_FIL_BUFFER_HOST_DRAIN && ENABLE_DVS_FILTER)
	else if (Input==4){
		host_delay_count_filter = 0;
		xil_printf("\n\rHost Delay Count(Filter) has been reset.\n\r");
		is_user_input_active = 0;
		XMipi_DisplayMainMenu();
		Menu = XMIPI_MAIN_MENU;
	}
#endif
	else{
		xil_printf("\n\rCancelled. Returning to main menu.\n\r");
		is_user_input_active = 0;
		XMipi_DisplayMainMenu();
		Menu = XMIPI_MAIN_MENU;
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
