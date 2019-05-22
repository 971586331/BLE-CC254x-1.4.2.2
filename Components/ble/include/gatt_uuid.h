/******************************************************************************

 @file  gatt_uuid.h

 @brief This file contains Generic Attribute Profile (GATT)
        UUID types.

 Group: WCS, BTS
 Target Device: CC2540, CC2541

 ******************************************************************************
 
 Copyright (c) 2010-2016, Texas Instruments Incorporated
 All rights reserved.

 IMPORTANT: Your use of this Software is limited to those specific rights
 granted under the terms of a software license agreement between the user
 who downloaded the software, his/her employer (which must be your employer)
 and Texas Instruments Incorporated (the "License"). You may not use this
 Software unless you agree to abide by the terms of the License. The License
 limits your use, and you acknowledge, that the Software may not be modified,
 copied or distributed unless embedded on a Texas Instruments microcontroller
 or used solely and exclusively in conjunction with a Texas Instruments radio
 frequency transceiver, which is integrated into your product. Other than for
 the foregoing purpose, you may not use, reproduce, copy, prepare derivative
 works of, modify, distribute, perform, display or sell this Software and/or
 its documentation for any purpose.

 YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
 PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
 INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
 NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
 TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
 NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
 LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
 INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
 OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
 OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
 (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

 Should you have any questions regarding your right to use this Software,
 contact Texas Instruments Incorporated at www.TI.com.

 ******************************************************************************
 Release Name: ble_sdk_1.4.2.2
 Release Date: 2016-06-09 06:57:09
 *****************************************************************************/

#ifndef GATT_UUID_H
#define GATT_UUID_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

/*
 * WARNING: The 16-bit UUIDs are assigned by the Bluetooth SIG and published
 *          in the Bluetooth Assigned Numbers page. Do not change these values.
 *          Changing them will cause Bluetooth interoperability issues.
 	警告：这些16位的UUID已经由蓝牙技术联盟发布在蓝牙编号分配页中，不要改变这些值，改变这些值会影响蓝牙协仪的通用性
 */

/**
 * GATT Services	GATT服务
 */
#define GAP_SERVICE_UUID                           0x1800 // Generic Access Profile		通用接入规范
#define GATT_SERVICE_UUID                          0x1801 // Generic Attribute Profile	通用属性规范

/**
 * GATT Declarations	GATT声明
 */
#define GATT_PRIMARY_SERVICE_UUID                  0x2800 // Primary Service	主要服务
#define GATT_SECONDARY_SERVICE_UUID                0x2801 // Secondary Service	辅助服务
#define GATT_INCLUDE_UUID                          0x2802 // Include			包含
#define GATT_CHARACTER_UUID                        0x2803 // Characteristic		特征

/**
 * GATT Descriptors	GATT描述符
 */
#define GATT_CHAR_EXT_PROPS_UUID                   0x2900 // Characteristic Extended Properties	扩展属性特征
#define GATT_CHAR_USER_DESC_UUID                   0x2901 // Characteristic User Description	特征使用描述
#define GATT_CLIENT_CHAR_CFG_UUID                  0x2902 // Client Characteristic Configuration	客户端特征配置
#define GATT_SERV_CHAR_CFG_UUID                    0x2903 // Server Characteristic Configuration	服务端特征配置
#define GATT_CHAR_FORMAT_UUID                      0x2904 // Characteristic Presentation Format		特征表示格式
#define GATT_CHAR_AGG_FORMAT_UUID                  0x2905 // Characteristic Aggregate Format		特征聚合格式
#define GATT_VALID_RANGE_UUID                      0x2906 // Valid Range							有资值范围
#define GATT_EXT_REPORT_REF_UUID                   0x2907 // External Report Reference Descriptor	外部报告引用描述符
#define GATT_REPORT_REF_UUID                       0x2908 // Report Reference Descriptor			报告参考描述符

/**
 * GATT Characteristics	GATT特征
 */
#define DEVICE_NAME_UUID                           0x2A00 // Device Name	设备名称
#define APPEARANCE_UUID                            0x2A01 // Appearance		外观
#define PERI_PRIVACY_FLAG_UUID                     0x2A02 // Peripheral Privacy Flag	周边隐私标志
#define RECONNECT_ADDR_UUID                        0x2A03 // Reconnection Address		重连地址
#define PERI_CONN_PARAM_UUID                       0x2A04 // Peripheral Preferred Connection Parameters	外围首选连接参数
#define SERVICE_CHANGED_UUID                       0x2A05 // Service Changed	服务改变

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * VARIABLES
 */

/**
 * GATT Services
 */
extern CONST uint8 gapServiceUUID[];
extern CONST uint8 gattServiceUUID[];

/**
 * GATT Attribute Types
 */
extern CONST uint8 primaryServiceUUID[];
extern CONST uint8 secondaryServiceUUID[];
extern CONST uint8 includeUUID[];
extern CONST uint8 characterUUID[];

/**
 * GATT Characteristic Descriptors
 */
extern CONST uint8 charExtPropsUUID[];
extern CONST uint8 charUserDescUUID[];
extern CONST uint8 clientCharCfgUUID[];
extern CONST uint8 servCharCfgUUID[];
extern CONST uint8 charFormatUUID[];
extern CONST uint8 charAggFormatUUID[];
extern CONST uint8 validRangeUUID[];
extern CONST uint8 extReportRefUUID[];
extern CONST uint8 reportRefUUID[];

/**
 * GATT Characteristic Types
 */
extern CONST uint8 deviceNameUUID[];
extern CONST uint8 appearanceUUID[];
extern CONST uint8 periPrivacyFlagUUID[];
extern CONST uint8 reconnectAddrUUID[];
extern CONST uint8 periConnParamUUID[];
extern CONST uint8 serviceChangedUUID[];
extern CONST uint8 manuNameUUID[];
extern CONST uint8 serialNumUUID[];
extern CONST uint8 manuAddrUUID[];

/*********************************************************************
 * FUNCTIONS
 */
extern const uint8 *GATT_FindUUIDRec( const uint8 *pUUID, uint8 len );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* GATT_UUID_H */
