/*******************************************************************************
    CANopen Object Dictionary definition for CANopenNode V4

    This file was automatically generated by CANopenEditor v4.1-2-gff637a7

    https://github.com/CANopenNode/CANopenNode
    https://github.com/CANopenNode/CANopenEditor

    DON'T EDIT THIS FILE MANUALLY !!!!
********************************************************************************

    File info:
        File Names:   OD.h; OD.c
        Project File: c2000_bootloader.xdd
        File Version: 1

        Created:      17/6/2023 7:20:50 pm
        Created By:   Sicris Rey Embay
        Modified:     17/6/2023 9:06:09 pm
        Modified By:  

    Device Info:
        Vendor Name:  
        Vendor ID:    
        Product Name: C2000 Bootloader
        Product ID:   

        Description:  
*******************************************************************************/

#ifndef OD_H
#define OD_H
/*******************************************************************************
    Counters of OD objects
*******************************************************************************/
#define OD_CNT_NMT 1
#define OD_CNT_EM 1
#define OD_CNT_EM_PROD 1
#define OD_CNT_HB_CONS 1
#define OD_CNT_HB_PROD 1


/*******************************************************************************
    Sizes of OD arrays
*******************************************************************************/
#define OD_CNT_ARR_1003 16
#define OD_CNT_ARR_1016 8
#define OD_CNT_ARR_1F50 1
#define OD_CNT_ARR_1F51 1
#define OD_CNT_ARR_1F52 2


/*******************************************************************************
    OD data declaration of all groups
*******************************************************************************/
typedef struct {
    uint32_t x1000_deviceType;
    uint32_t x1014_COB_ID_EMCY;
    uint16_t x1015_inhibitTimeEMCY;
    uint8_t x1016_consumerHeartbeatTime_sub0;
    uint32_t x1016_consumerHeartbeatTime[OD_CNT_ARR_1016];
    uint16_t x1017_producerHeartbeatTime;
} OD_PERSIST_COMM_t;

typedef struct {
    uint8_t x1001_errorRegister;
    uint8_t x1F50_downloadProgramData_sub0;
    uint8_t x1F51_programControl_sub0;
    uint32_t x1F51_programControl[OD_CNT_ARR_1F51];
    uint8_t x1F52_verifyApplicationSoftware_sub0;
    uint32_t x1F52_verifyApplicationSoftware[OD_CNT_ARR_1F52];
} OD_RAM_t;

#ifndef OD_ATTR_PERSIST_COMM
#define OD_ATTR_PERSIST_COMM
#endif
extern OD_ATTR_PERSIST_COMM OD_PERSIST_COMM_t OD_PERSIST_COMM;

#ifndef OD_ATTR_RAM
#define OD_ATTR_RAM
#endif
extern OD_ATTR_RAM OD_RAM_t OD_RAM;

#ifndef OD_ATTR_OD
#define OD_ATTR_OD
#endif
extern OD_ATTR_OD OD_t *OD;


/*******************************************************************************
    Object dictionary entries - shortcuts
*******************************************************************************/
#define OD_ENTRY_H1000 &OD->list[0]
#define OD_ENTRY_H1001 &OD->list[1]
#define OD_ENTRY_H1003 &OD->list[2]
#define OD_ENTRY_H1014 &OD->list[3]
#define OD_ENTRY_H1015 &OD->list[4]
#define OD_ENTRY_H1016 &OD->list[5]
#define OD_ENTRY_H1017 &OD->list[6]
#define OD_ENTRY_H1F50 &OD->list[7]
#define OD_ENTRY_H1F51 &OD->list[8]
#define OD_ENTRY_H1F52 &OD->list[9]


/*******************************************************************************
    Object dictionary entries - shortcuts with names
*******************************************************************************/
#define OD_ENTRY_H1000_deviceType &OD->list[0]
#define OD_ENTRY_H1001_errorRegister &OD->list[1]
#define OD_ENTRY_H1003_pre_definedErrorField &OD->list[2]
#define OD_ENTRY_H1014_COB_ID_EMCY &OD->list[3]
#define OD_ENTRY_H1015_inhibitTimeEMCY &OD->list[4]
#define OD_ENTRY_H1016_consumerHeartbeatTime &OD->list[5]
#define OD_ENTRY_H1017_producerHeartbeatTime &OD->list[6]
#define OD_ENTRY_H1F50_downloadProgramData &OD->list[7]
#define OD_ENTRY_H1F51_programControl &OD->list[8]
#define OD_ENTRY_H1F52_verifyApplicationSoftware &OD->list[9]


/*******************************************************************************
    OD config structure
*******************************************************************************/
#ifdef CO_MULTIPLE_OD
#define OD_INIT_CONFIG(config) {\
    (config).CNT_NMT = OD_CNT_NMT;\
    (config).ENTRY_H1017 = OD_ENTRY_H1017;\
    (config).CNT_HB_CONS = OD_CNT_HB_CONS;\
    (config).CNT_ARR_1016 = OD_CNT_ARR_1016;\
    (config).ENTRY_H1016 = OD_ENTRY_H1016;\
    (config).CNT_EM = OD_CNT_EM;\
    (config).ENTRY_H1001 = OD_ENTRY_H1001;\
    (config).ENTRY_H1014 = OD_ENTRY_H1014;\
    (config).ENTRY_H1015 = OD_ENTRY_H1015;\
    (config).CNT_ARR_1003 = OD_CNT_ARR_1003;\
    (config).ENTRY_H1003 = OD_ENTRY_H1003;\
    (config).CNT_SDO_SRV = 0;\
    (config).ENTRY_H1200 = NULL;\
    (config).CNT_SDO_CLI = 0;\
    (config).ENTRY_H1280 = NULL;\
    (config).CNT_TIME = 0;\
    (config).ENTRY_H1012 = NULL;\
    (config).CNT_SYNC = 0;\
    (config).ENTRY_H1005 = NULL;\
    (config).ENTRY_H1006 = NULL;\
    (config).ENTRY_H1007 = NULL;\
    (config).ENTRY_H1019 = NULL;\
    (config).CNT_RPDO = 0;\
    (config).ENTRY_H1400 = NULL;\
    (config).ENTRY_H1600 = NULL;\
    (config).CNT_TPDO = 0;\
    (config).ENTRY_H1800 = NULL;\
    (config).ENTRY_H1A00 = NULL;\
    (config).CNT_LEDS = 0;\
    (config).CNT_GFC = 0;\
    (config).ENTRY_H1300 = NULL;\
    (config).CNT_SRDO = 0;\
    (config).ENTRY_H1301 = NULL;\
    (config).ENTRY_H1381 = NULL;\
    (config).ENTRY_H13FE = NULL;\
    (config).ENTRY_H13FF = NULL;\
    (config).CNT_LSS_SLV = 0;\
    (config).CNT_LSS_MST = 0;\
    (config).CNT_GTWA = 0;\
    (config).CNT_TRACE = 0;\
}
#endif

#endif /* OD_H */
