/*
** $Id: //Department/DaVinci/BRANCHES/MT6620_WIFI_DRIVER_V2_3/os/linux/include/gl_rst.h#1 $
*/

/*! \file   gl_rst.h
    \brief  Declaration of functions and finite state machine for 
            MT6620 Whole-Chip Reset Mechanism
*/




#ifndef _GL_RST_H
#define _GL_RST_H

/*******************************************************************************
*                         C O M P I L E R   F L A G S
********************************************************************************
*/

/*******************************************************************************
*                    E X T E R N A L   R E F E R E N C E S
********************************************************************************
*/
#include "gl_typedef.h"

/*******************************************************************************
*                              C O N S T A N T S
********************************************************************************
*/

/*******************************************************************************
*                             D A T A   T Y P E S
********************************************************************************
*/
#if 1
typedef INT_32(*wmt_wlan_probe_cb)(VOID);
typedef INT_32(*wmt_wlan_remove_cb)(VOID);
typedef INT_32(*wmt_wlan_bus_cnt_get_cb)(VOID);
typedef INT_32(*wmt_wlan_bus_cnt_clr_cb)(VOID);

typedef struct _MTK_WCN_WMT_WLAN_CB_INFO{
	wmt_wlan_probe_cb wlan_probe_cb;
	wmt_wlan_remove_cb wlan_remove_cb;
	wmt_wlan_bus_cnt_get_cb wlan_bus_cnt_get_cb;
	wmt_wlan_bus_cnt_clr_cb wlan_bus_cnt_clr_cb;
}MTK_WCN_WMT_WLAN_CB_INFO,*P_MTK_WCN_WMT_WLAN_CB_INFO;

extern INT_32 mtk_wcn_wmt_wlan_reg(P_MTK_WCN_WMT_WLAN_CB_INFO pWmtWlanCbInfo);
extern INT_32 mtk_wcn_wmt_wlan_unreg(VOID);
#endif

typedef enum _ENUM_RESET_STATUS_T {
    RESET_FAIL,
    RESET_SUCCESS
} ENUM_RESET_STATUS_T;

typedef struct _RESET_STRUCT_T {
    ENUM_RESET_STATUS_T rst_data;
    struct work_struct rst_work;
} RESET_STRUCT_T;

/* duplicated from wmt_exp.h for better driver isolation */
typedef enum _ENUM_WMTDRV_TYPE_T {
    WMTDRV_TYPE_BT = 0,
    WMTDRV_TYPE_FM = 1,
    WMTDRV_TYPE_GPS = 2,
    WMTDRV_TYPE_WIFI = 3,
    WMTDRV_TYPE_WMT = 4,
    WMTDRV_TYPE_STP = 5,
    WMTDRV_TYPE_SDIO1 = 6,
    WMTDRV_TYPE_SDIO2 = 7,
    WMTDRV_TYPE_LPBK = 8,
    WMTDRV_TYPE_MAX
} ENUM_WMTDRV_TYPE_T, *P_ENUM_WMTDRV_TYPE_T;

typedef enum _ENUM_WMTMSG_TYPE_T {
    WMTMSG_TYPE_POWER_ON = 0,
    WMTMSG_TYPE_POWER_OFF = 1,
    WMTMSG_TYPE_RESET = 2,
    WMTMSG_TYPE_STP_RDY= 3,
    WMTMSG_TYPE_HW_FUNC_ON= 4,
    WMTMSG_TYPE_MAX
} ENUM_WMTMSG_TYPE_T, *P_ENUM_WMTMSG_TYPE_T;

typedef enum _ENUM_WMTRSTMSG_TYPE_T{
    WMTRSTMSG_RESET_START = 0x0,
    WMTRSTMSG_RESET_END = 0x1,
    WMTRSTMSG_RESET_END_FAIL = 0x2,
    WMTRSTMSG_RESET_MAX,
    WMTRSTMSG_RESET_INVALID = 0xff
} ENUM_WMTRSTMSG_TYPE_T, *P_ENUM_WMTRSTMSG_TYPE_T;

typedef void (*PF_WMT_CB)(
    ENUM_WMTDRV_TYPE_T, /* Source driver type */
    ENUM_WMTDRV_TYPE_T, /* Destination driver type */
    ENUM_WMTMSG_TYPE_T, /* Message type */
    void *, /* READ-ONLY buffer. Buffer is allocated and freed by WMT_drv. Client
               can't touch this buffer after this function return. */
    unsigned int /* Buffer size in unit of byte */
    );

/*******************************************************************************
*                    E X T E R N A L   F U N C T I O N S
********************************************************************************
*/
extern MTK_WCN_BOOL mtk_wcn_wmt_assert(ENUM_WMTDRV_TYPE_T type, UINT32 reason);

#define glDoChipReset() \
do { \
	if (!kalStrnCmp(current->comm, "mtk_wmtd", 8)) { \
		DBGLOG(INIT, ERROR, ("forbid core dump or chip reset in mtk_wmtd %s line %d\n", __func__, __LINE__)); \
		break; \
	} \
	DBGLOG(INIT, ERROR, ("Do core dump and chip reset in %s line %d\n", __func__, __LINE__)); \
	mtk_wcn_wmt_assert(WMTDRV_TYPE_WIFI, 40); \
} while (0)

#if CFG_CHIP_RESET_SUPPORT
extern int mtk_wcn_wmt_msgcb_reg(ENUM_WMTDRV_TYPE_T eType, PF_WMT_CB pCb);
extern int mtk_wcn_wmt_msgcb_unreg(ENUM_WMTDRV_TYPE_T eType);
extern int wifi_reset_start(void);
extern int wifi_reset_end(ENUM_RESET_STATUS_T);
extern MTK_WCN_BOOL mtk_wcn_wmt_assert(ENUM_WMTDRV_TYPE_T type, UINT32 reason);

#define glDoCoreDump() \
	do { \
		if (!kalStrnCmp(current->comm, "mtk_wmtd", 8)) { \
			printk("will do core dump after wlanProbe/wlanRemove, now in %s\n", __func__); \
			break; \
		} \
		printk("do core dump in %s\n", __func__); \
		mtk_wcn_wmt_assert(WMTDRV_TYPE_WIFI, 40); \
	} while(0)

#endif

/*******************************************************************************
*                            P U B L I C   D A T A
********************************************************************************
*/

/*******************************************************************************
*                           P R I V A T E   D A T A
********************************************************************************
*/

/*******************************************************************************
*                                 M A C R O S
********************************************************************************
*/

/*******************************************************************************
*                  F U N C T I O N   D E C L A R A T I O N S
********************************************************************************
*/

/*******************************************************************************
*                              F U N C T I O N S
********************************************************************************
*/

VOID
glResetInit(
    VOID
    );

VOID
glResetUninit(
    VOID
    );

VOID
glSendResetRequest(
    VOID
    );

BOOLEAN
kalIsResetting(
    VOID
    );


#endif /* _GL_RST_H */
