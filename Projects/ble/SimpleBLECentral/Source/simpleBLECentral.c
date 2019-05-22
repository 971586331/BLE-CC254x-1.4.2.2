/******************************************************************************

 @file  simpleBLECentral.c

 @brief This file contains the Simple BLE Central sample application for use
        with the CC2540 Bluetooth Low Energy Protocol Stack.

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
 PROVIDED �AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
 Release Date: 2016-06-09 06:57:10
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OnBoard.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"
#include "gatt.h"
#include "ll.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "simpleGATTprofile.h"
#include "simpleBLECentral.h"

#include "hal_uart.h"
#include "stdio.h"
#include "string.h"

/*********************************************************************
 * MACROS
 */

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

/*********************************************************************
 * CONSTANTS
 */

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  8

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 4000

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// TRUE to use high scan duty cycle when creating link	�ڴ�������ʱʹ�ø�ɨ��ռ�ձ�Ϊ
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE
	
// TRUE to use white list when creating link	��������ʱʹ�ð�����
#define DEFAULT_LINK_WHITE_LIST               FALSE

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                   1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      400

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           600

// Default passcode
#define DEFAULT_PASSCODE                      19655

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

// Application states
enum
{
  BLE_STATE_IDLE,
  BLE_STATE_CONNECTING,
  BLE_STATE_CONNECTED,
  BLE_STATE_DISCONNECTING
};

// Discovery states
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
};

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Task ID for internal task/event processing
static uint8 simpleBLETaskId;

// GAP GATT Attributes
static const uint8 simpleBLEDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Central";

// Number of scan results and scan result index
static uint8 simpleBLEScanRes;
static uint8 simpleBLEScanIdx;

// Scan result list
static gapDevRec_t simpleBLEDevList[DEFAULT_MAX_SCAN_RES];

// Scanning state
static uint8 simpleBLEScanning = FALSE;

// RSSI polling state
static uint8 simpleBLERssi = FALSE;

// Connection handle of current connection 
static uint16 simpleBLEConnHandle = GAP_CONNHANDLE_INIT;

// Application state
static uint8 simpleBLEState = BLE_STATE_IDLE;

// Discovery state
static uint8 simpleBLEDiscState = BLE_DISC_STATE_IDLE;

// Discovered service start and end handle
static uint16 simpleBLESvcStartHdl = 0;
static uint16 simpleBLESvcEndHdl = 0;

// Discovered characteristic handle
static uint16 simpleBLECharHdl = 0;

// Value to write
static uint8 simpleBLECharVal = 0;

// Value read/write toggle
static bool simpleBLEDoWrite = FALSE;

// GATT read/write procedure state
static bool simpleBLEProcedureInProgress = FALSE;

uint8 gStatus;

uint8 target_addr[6] = {6,5,4,3,2,1};
uint8 target_addrType = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLECentralProcessGATTMsg( gattMsgEvent_t *pMsg );
static void simpleBLECentralRssiCB( uint16 connHandle, int8  rssi );
static uint8 simpleBLECentralEventCB( gapCentralRoleEvent_t *pEvent );
static void simpleBLECentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs );
static void simpleBLECentralPairStateCB( uint16 connHandle, uint8 state, uint8 status );
static void simpleBLECentral_HandleKeys( uint8 shift, uint8 keys );
static void simpleBLECentral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg );
static void simpleBLECentralStartDiscovery( void );
static bool simpleBLEFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen );
static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType );
char *bdAddr2Str ( uint8 *pAddr );

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static const gapCentralRoleCB_t simpleBLERoleCB =
{
  simpleBLECentralRssiCB,       // RSSI callback
  simpleBLECentralEventCB       // Event callback
};

// Bond Manager Callbacks
static const gapBondCBs_t simpleBLEBondCB =
{
  simpleBLECentralPasscodeCB,	//GAP�󶨹�����Կ�ص�����
  simpleBLECentralPairStateCB	//GAP�󶨹���״̬�ص�����
};

void my_printf(uint8 * str)
{
	uint8 len;

	len = strlen(str);
	HalUARTWrite(0, str, len);
}

void my_printf_num(uint8 * str, uint32 num)
{
	uint8 buff[50];

	sprintf(buff, "%s %d\n", str, num);
	my_printf(buff);
}

uint8 uartbuff;
static void uart_rxCB(uint8 port, uint8 event)
{
	HalUARTRead(0, &uartbuff, 1);

	if( uartbuff == 0x11 )	//UP
	{
		// Start or stop discovery ��ʼ����״̬ʱ
		if ( simpleBLEState != BLE_STATE_CONNECTED )
		{
		  if ( !simpleBLEScanning ) 	//ִ���豸����
		  {
			simpleBLEScanning = TRUE;
			simpleBLEScanRes = 0;
			
			GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
										   DEFAULT_DISCOVERY_ACTIVE_SCAN,
										   DEFAULT_DISCOVERY_WHITE_LIST );		
		  }
		  else
		  {
			GAPCentralRole_CancelDiscovery();	//ȡ���豸����ɨ��
		  }
		}
			//���������״̬�����ҵ����豸������Ϊ0��������һ�׶δ���������
		else if ( simpleBLEState == BLE_STATE_CONNECTED &&
				  simpleBLECharHdl != 0 &&
				  simpleBLEProcedureInProgress == FALSE )
		{
		  uint8 status;
		  
		  // Do a read or write as long as no other read or write is in progress
		  // ֻҪû����������д���ڽ��У��ͽ��ж���д
		  if ( simpleBLEDoWrite )
		  {
			// Do a write
			attWriteReq_t req;
			
			req.pValue = GATT_bm_alloc( simpleBLEConnHandle, ATT_WRITE_REQ, 1, NULL );
			if ( req.pValue != NULL )
			{
			  req.handle = simpleBLECharHdl;
			  req.len = 1;
			  req.pValue[0] = simpleBLECharVal;
			  req.sig = 0;
			  req.cmd = 0;
			  status = GATT_WriteCharValue( simpleBLEConnHandle, &req, simpleBLETaskId );	//��ָ���ľ����д������
			  if ( status != SUCCESS )
			  {
				GATT_bm_free( (gattMsg_t *)&req, ATT_WRITE_REQ );
			  }
			}
			else
			{
			  status = bleMemAllocError;
			}
		  }
		  else
		  {
			// Do a read
			attReadReq_t req;
			
			req.handle = simpleBLECharHdl;
			status = GATT_ReadCharValue( simpleBLEConnHandle, &req, simpleBLETaskId );	//��ָ������ж�ȡ����
		  }
		  
		  if ( status == SUCCESS )
		  {
			simpleBLEProcedureInProgress = TRUE;
			simpleBLEDoWrite = !simpleBLEDoWrite;	//������ζ��ɹ������´ΰ�UP��Ϊд���������д�ɹ������´ν�UP��Ϊ��
		  }
		}	 
	}
	if(uartbuff == 0x22)	//ִ�����Ӳ���
	{
		// Connect or disconnect	���ӻ�Ͽ�
	    if ( simpleBLEState == BLE_STATE_IDLE )	//����ǿ���״̬
	    {
	    	my_printf("key 22 -> BLE_STATE_IDLE\n");
	      // if there is a scan result	�����ɨ����
	      if ( simpleBLEScanRes > 0 )
	      {
	      
	        simpleBLEState = BLE_STATE_CONNECTING;	//״̬��Ϊ������
			
			//�������� ����ֵΪ0��ʾ�ɹ�
	        uint8 ret = GAPCentralRole_EstablishLink( DEFAULT_LINK_HIGH_DUTY_CYCLE,DEFAULT_LINK_WHITE_LIST,target_addrType, target_addr );
	  		my_printf_num("GAPCentralRole_EstablishLink() > ", ret);
	      }
	    }
		//����������л�����״̬
	    else if ( simpleBLEState == BLE_STATE_CONNECTING ||
	              simpleBLEState == BLE_STATE_CONNECTED )
	    {
	    	my_printf("key 22 -> BLE_STATE_CONNECTING BLE_STATE_CONNECTED\n");
	      // disconnect	״̬��Ϊ�Ͽ�״̬
	      simpleBLEState = BLE_STATE_DISCONNECTING;

			//��ֹ����
	      gStatus = GAPCentralRole_TerminateLink( simpleBLEConnHandle );
	      
	      LCD_WRITE_STRING( "Disconnecting", HAL_LCD_LINE_1 ); 
		}
	}
	if(uartbuff == 0x33)	//�������Ӳ���
	{
		    // Connection update
	    if ( simpleBLEState == BLE_STATE_CONNECTED )
	    {
	    	//�������Ӳ���
	      GAPCentralRole_UpdateLink( simpleBLEConnHandle,
	                                 DEFAULT_UPDATE_MIN_CONN_INTERVAL,
	                                 DEFAULT_UPDATE_MAX_CONN_INTERVAL,
	                                 DEFAULT_UPDATE_SLAVE_LATENCY,
	                                 DEFAULT_UPDATE_CONN_TIMEOUT );
	    }
	}
	
	uartbuff = 0;
}

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLECentral_Init
 *
 * @brief   Initialization function for the Simple BLE Central App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *			�������ܳ�ʼ��
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLECentral_Init( uint8 task_id )
{
  simpleBLETaskId = task_id;

  // Setup Central Profile	���������������ļ�
  {
    uint8 scanRes = DEFAULT_MAX_SCAN_RES;	//���ɷ����豸����
    GAPCentralRole_SetParameter ( GAPCENTRALROLE_MAX_SCAN_RES, sizeof( uint8 ), &scanRes );
  }
  
  // Setup GAP
  GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION );	//������ͨɨ��ģʽ��С����ʱ��
  GAP_SetParamValue( TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION );	//��������ɨ��ģʽ��С����ʱ��
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (uint8 *) simpleBLEDeviceName );

  // Setup the GAP Bond Manager
  {
    uint32 passkey = DEFAULT_PASSCODE;		//Ĭ�ϵ�����
    uint8 pairMode = DEFAULT_PAIRING_MODE;	//Ĭ�ϵ����ģʽ
    uint8 mitm = DEFAULT_MITM_MODE;			//Ĭ��MITMģʽ��TRUE�����ʱ��Ҫ�����OOB��
    uint8 ioCap = DEFAULT_IO_CAPABILITIES;	//Ĭ��GAP��I / O����
    uint8 bonding = DEFAULT_BONDING_MODE;	//Ĭ�ϰ�ģʽ
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof( uint8 ), &bonding );
  }  

  // Initialize GATT Client	��ʼ��GATT�ͻ���
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications	ע����մ����ATTָʾ/֪ͨ
  GATT_RegisterForInd( simpleBLETaskId );

  // Initialize GATT attributes	��ʼ��GATT����
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes

  // Register for all key events - This app will handle all key events	ע������¼�
  RegisterForKeys( simpleBLETaskId );
  
  // makes sure LEDs are off	�ر�LED
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );
  
  // Setup a delayed profile startup	�����ӳ������ļ�����
  osal_set_event( simpleBLETaskId, START_DEVICE_EVT );

  	//���ڳ�ʼ��
	halUARTCfg_t uartConfig;
	uartConfig.configured = TRUE;
	uartConfig.baudRate = HAL_UART_BR_115200;
	uartConfig.flowControl = FALSE;
	uartConfig.callBackFunc = uart_rxCB;
	HalUARTOpen(0, &uartConfig);

	//HalUARTWrite(0, "central start!\n", 20);
	my_printf("central start!\n");
	
}

/*********************************************************************
 * @fn      SimpleBLECentral_ProcessEvent
 *
 * @brief   Simple BLE Central Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLECentral_ProcessEvent( uint8 task_id, uint16 events )
{
	my_printf_num("SimpleBLECentral_ProcessEvent() >", events);
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  
  if ( events & SYS_EVENT_MSG )	//OASL���з��񣬴���OSAL��Ϣ
  {
    uint8 *pMsg;
    if ( (pMsg = osal_msg_receive( simpleBLETaskId )) != NULL )
    {
      simpleBLECentral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );	//����һ����Ϣ

      // Release the OSAL message	ɾ�����������Ϣ
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & START_DEVICE_EVT )		//�����豸
  {
    // Start the Device	�����豸��ע��ص�
    VOID GAPCentralRole_StartDevice( (gapCentralRoleCB_t *) &simpleBLERoleCB );

    // Register with bond manager after starting device
    GAPBondMgr_Register( (gapBondCBs_t *) &simpleBLEBondCB );

    return ( events ^ START_DEVICE_EVT );
  }

  if ( events & START_DISCOVERY_EVT )	//�����豸������
  {
    simpleBLECentralStartDiscovery( );	//��ʼ������
    
    return ( events ^ START_DISCOVERY_EVT );
  }
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      simpleBLECentral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLECentral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{

	my_printf_num("simpleBLECentral_ProcessOSALMsg() > ", pMsg->event);
  switch ( pMsg->event )
  {
  	//����ǰ����¼������ð����¼�������
    case KEY_CHANGE:
      simpleBLECentral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;

	//�����GATT�¼�������GATT������
    case GATT_MSG_EVENT:
      simpleBLECentralProcessGATTMsg( (gattMsgEvent_t *) pMsg );
      break;
  }
}

/*********************************************************************
 * @fn      simpleBLECentral_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */

static void simpleBLECentral_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter

	//��һ��д���ٰ�һ�¶�
  if ( keys & HAL_KEY_UP )
  {
    // Start or stop discovery ��ʼ����״̬ʱ
    if ( simpleBLEState != BLE_STATE_CONNECTED )
    {
      if ( !simpleBLEScanning )		//����ɨ��״̬
      {
        simpleBLEScanning = TRUE;
        simpleBLEScanRes = 0;
        
        LCD_WRITE_STRING( "Discovering...", HAL_LCD_LINE_1 );
        LCD_WRITE_STRING( "", HAL_LCD_LINE_2 );
        
        GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                       DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                       DEFAULT_DISCOVERY_WHITE_LIST );      
      }
      else
      {
        GAPCentralRole_CancelDiscovery();	//ȡ���豸����ɨ��
      }
    }
		//���������״̬�����ҵ����豸������Ϊ0��������һ�׶δ���������
    else if ( simpleBLEState == BLE_STATE_CONNECTED &&
              simpleBLECharHdl != 0 &&
              simpleBLEProcedureInProgress == FALSE )
    {
      uint8 status;
      
      // Do a read or write as long as no other read or write is in progress
      // ֻҪû����������д���ڽ��У��ͽ��ж���д
      if ( simpleBLEDoWrite )
      {
        // Do a write
        attWriteReq_t req;
        
        req.pValue = GATT_bm_alloc( simpleBLEConnHandle, ATT_WRITE_REQ, 1, NULL );
        if ( req.pValue != NULL )
        {
          req.handle = simpleBLECharHdl;
          req.len = 1;
          req.pValue[0] = simpleBLECharVal;
          req.sig = 0;
          req.cmd = 0;
          status = GATT_WriteCharValue( simpleBLEConnHandle, &req, simpleBLETaskId );	//��ָ���ľ����д������
          if ( status != SUCCESS )
          {
            GATT_bm_free( (gattMsg_t *)&req, ATT_WRITE_REQ );
          }
        }
        else
        {
          status = bleMemAllocError;
        }
      }
      else
      {
        // Do a read
        attReadReq_t req;
        
        req.handle = simpleBLECharHdl;
        status = GATT_ReadCharValue( simpleBLEConnHandle, &req, simpleBLETaskId );	//��ָ������ж�ȡ����
      }
      
      if ( status == SUCCESS )
      {
        simpleBLEProcedureInProgress = TRUE;
        simpleBLEDoWrite = !simpleBLEDoWrite;	//������ζ��ɹ������´ΰ�UP��Ϊд���������д�ɹ������´ν�UP��Ϊ��
      }
    }    
  }

	//��ʾ���ֵĽ��
  if ( keys & HAL_KEY_LEFT )
  {
    // Display discovery results	//��ʾ���ֵĽ��
    if ( !simpleBLEScanning && simpleBLEScanRes > 0 )
    {
        // Increment index of current result (with wraparound)	��ǰ���ֵ��豸������һ
        simpleBLEScanIdx++;
        if ( simpleBLEScanIdx >= simpleBLEScanRes )
        {
          simpleBLEScanIdx = 0;
        }
        
        LCD_WRITE_STRING_VALUE( "Device", simpleBLEScanIdx + 1,
                                10, HAL_LCD_LINE_1 );
        LCD_WRITE_STRING( bdAddr2Str( simpleBLEDevList[simpleBLEScanIdx].addr ),
                          HAL_LCD_LINE_2 );
    }
  }

	//�������Ӳ���
  if ( keys & HAL_KEY_RIGHT )
  {
    // Connection update
    if ( simpleBLEState == BLE_STATE_CONNECTED )
    {
    	//�������Ӳ���
      GAPCentralRole_UpdateLink( simpleBLEConnHandle,
                                 DEFAULT_UPDATE_MIN_CONN_INTERVAL,
                                 DEFAULT_UPDATE_MAX_CONN_INTERVAL,
                                 DEFAULT_UPDATE_SLAVE_LATENCY,
                                 DEFAULT_UPDATE_CONN_TIMEOUT );
    }
  }

  //ִ�����Ӳ���
  if ( keys & HAL_KEY_CENTER )
  {
    uint8 addrType;
    uint8 *peerAddr;
    
    // Connect or disconnect	���ӻ�Ͽ�
    if ( simpleBLEState == BLE_STATE_IDLE )	//����ǿ���״̬
    {
      // if there is a scan result	�����ɨ����
      if ( simpleBLEScanRes > 0 )
      {
        // connect to current device in scan result	��ɨ���������ӵ���ǰ�豸
        peerAddr = simpleBLEDevList[simpleBLEScanIdx].addr;
        addrType = simpleBLEDevList[simpleBLEScanIdx].addrType;
      
        simpleBLEState = BLE_STATE_CONNECTING;	//״̬��Ϊ������

		//��������
        GAPCentralRole_EstablishLink( DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                      DEFAULT_LINK_WHITE_LIST,
                                      addrType, peerAddr );
  
        LCD_WRITE_STRING( "Connecting", HAL_LCD_LINE_1 );
        LCD_WRITE_STRING( bdAddr2Str( peerAddr ), HAL_LCD_LINE_2 ); 
      }
    }
	//����������л�����״̬
    else if ( simpleBLEState == BLE_STATE_CONNECTING ||
              simpleBLEState == BLE_STATE_CONNECTED )
    {
      // disconnect	״̬��Ϊ�Ͽ�״̬
      simpleBLEState = BLE_STATE_DISCONNECTING;

		//��ֹ����
      gStatus = GAPCentralRole_TerminateLink( simpleBLEConnHandle );
      
      LCD_WRITE_STRING( "Disconnecting", HAL_LCD_LINE_1 ); 
    }
  }

  //��ʼ������źŲ�ѯ
  if ( keys & HAL_KEY_DOWN )
  {
    // Start or cancel RSSI polling	��ʼ��ȡ���ź���ѯ
    if ( simpleBLEState == BLE_STATE_CONNECTED )
    {
      if ( !simpleBLERssi )	//�����ǰû������ѯ
      {
        simpleBLERssi = TRUE;
		//��ʼ�ź���ѯ
        GAPCentralRole_StartRssi( simpleBLEConnHandle, DEFAULT_RSSI_PERIOD );
      }
      else
      {
        simpleBLERssi = FALSE;
		//�����ź���ѯ
        GAPCentralRole_CancelRssi( simpleBLEConnHandle );
        
        LCD_WRITE_STRING( "RSSI Cancelled", HAL_LCD_LINE_1 );
      }
    }
  }
}

/*********************************************************************
 * @fn      simpleBLECentralProcessGATTMsg
 *
 * @brief   Process GATT messages	����GATT��Ϣ
 *
 * @return  none
 */
static void simpleBLECentralProcessGATTMsg( gattMsgEvent_t *pMsg )
{
	
	my_printf_num("simpleBLECentralProcessGATTMsg() > ", pMsg->method);
  if ( simpleBLEState != BLE_STATE_CONNECTED )	//���BLE��������״̬
  {
    // In case a GATT message came after a connection has dropped,
    // ignore the message	��������ӶϿ������GATT��Ϣ������Ը���Ϣ
    return;
  }

  //����Ƕ���Ӧ�����д�����Ӧ���ж�����
  if ( ( pMsg->method == ATT_READ_RSP ) ||
       ( ( pMsg->method == ATT_ERROR_RSP ) &&
         ( pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ ) ) )
  {
    if ( pMsg->method == ATT_ERROR_RSP )	//����Ǵ�����Ӧ
    {
      uint8 status = pMsg->msg.errorRsp.errCode;	//��ʾ�������
      
      LCD_WRITE_STRING_VALUE( "Read Error", status, 10, HAL_LCD_LINE_1 );
    }
    else
    {
      // After a successful read, display the read value	�ڶ��ɹ�����ʾֵ
      uint8 valueRead = pMsg->msg.readRsp.pValue[0];

      LCD_WRITE_STRING_VALUE( "Read rsp:", valueRead, 10, HAL_LCD_LINE_1 );
	  my_printf_num("Read rsp:", valueRead);
    }
    
    simpleBLEProcedureInProgress = FALSE;
  }
		 //д
  else if ( ( pMsg->method == ATT_WRITE_RSP ) ||
       ( ( pMsg->method == ATT_ERROR_RSP ) &&
         ( pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ ) ) )
  {
    
    if ( pMsg->method == ATT_ERROR_RSP == ATT_ERROR_RSP )
    {
      uint8 status = pMsg->msg.errorRsp.errCode;
      
      LCD_WRITE_STRING_VALUE( "Write Error", status, 10, HAL_LCD_LINE_1 );
    }
    else
    {
      // After a succesful write, display the value that was written and increment value
      LCD_WRITE_STRING_VALUE( "Write sent:", simpleBLECharVal++, 10, HAL_LCD_LINE_1 );    
	  my_printf_num("Write sent:", simpleBLECharVal);
    }
    
    simpleBLEProcedureInProgress = FALSE;    

  }
  else if ( simpleBLEDiscState != BLE_DISC_STATE_IDLE )	//�������״̬���ǿ���״̬
  {
    simpleBLEGATTDiscoveryEvent( pMsg );	//����Ӧ�����ݷ���
  }
  
  GATT_bm_free( &pMsg->msg, pMsg->method );
}

/*********************************************************************
 * @fn      simpleBLECentralRssiCB
 *
 * @brief   RSSI callback.
 *
 * @param   connHandle - connection handle
 * @param   rssi - RSSI
 *
 * @return  none
 */
static void simpleBLECentralRssiCB( uint16 connHandle, int8 rssi )
{
    LCD_WRITE_STRING_VALUE( "RSSI -dB:", (uint8) (-rssi), 10, HAL_LCD_LINE_1 );
}

/*********************************************************************
 * @fn      simpleBLECentralEventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  TRUE if safe to deallocate event message, FALSE otherwise.
 */
static uint8 simpleBLECentralEventCB( gapCentralRoleEvent_t *pEvent )
{
	
	my_printf_num("simpleBLECentralEventCB > ", pEvent->gap.opcode);
  switch ( pEvent->gap.opcode )
  {
    case GAP_DEVICE_INIT_DONE_EVENT:  
      {
        LCD_WRITE_STRING( "BLE Central", HAL_LCD_LINE_1 );
        LCD_WRITE_STRING( bdAddr2Str( pEvent->initDone.devAddr ),  HAL_LCD_LINE_2 );
      }
      break;

	//������豸���͹����з����豸
    case GAP_DEVICE_INFO_EVENT:
      {
        // if filtering device discovery results based on service UUID �Ƿ���˷����豸��UUID
        if ( DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE )
        {
          //if ( simpleBLEFindSvcUuid( SIMPLEPROFILE_SERV_UUID,pEvent->deviceInfo.pEvtData,pEvent->deviceInfo.dataLen ) )
          {
            simpleBLEAddDeviceInfo( pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType );
          }
        }
      }
      break;

	  //�豸���ֹ��������
    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        // discovery complete
        simpleBLEScanning = FALSE;

        // if not filtering device discovery results based on service UUID
        if ( DEFAULT_DEV_DISC_BY_SVC_UUID == FALSE )
        {
          // Copy results	ɨ�赽���豸����
          simpleBLEScanRes = pEvent->discCmpl.numDevs;
          osal_memcpy( simpleBLEDevList, pEvent->discCmpl.pDevList,
                       (sizeof( gapDevRec_t ) * pEvent->discCmpl.numDevs) );
        }
        
        LCD_WRITE_STRING_VALUE( "Devices Found", simpleBLEScanRes,
                                10, HAL_LCD_LINE_1 );
        if ( simpleBLEScanRes > 0 )		
        {
          LCD_WRITE_STRING( "<- To Select", HAL_LCD_LINE_2 );
        }

        // initialize scan index to last device ��ʼ��ɨ�������Ϊ��һ���豸
        simpleBLEScanIdx = simpleBLEScanRes;

      }
      break;

	//���������������
    case GAP_LINK_ESTABLISHED_EVENT:
      {
        if ( pEvent->gap.hdr.status == SUCCESS )
        {          
          simpleBLEState = BLE_STATE_CONNECTED;	//��BLE״̬��Ϊ������
          simpleBLEConnHandle = pEvent->linkCmpl.connectionHandle;
          simpleBLEProcedureInProgress = TRUE;   //��ʾGATT��д���ڽ��� 

          // If service discovery not performed initiate service discovery	Ϊ0��ʾ��û�н���GATT������
          if ( simpleBLECharHdl == 0 )
          {
            osal_start_timerEx( simpleBLETaskId, START_DISCOVERY_EVT, DEFAULT_SVC_DISCOVERY_DELAY );
          }
                    
          LCD_WRITE_STRING( "Connected", HAL_LCD_LINE_1 );
          LCD_WRITE_STRING( bdAddr2Str( pEvent->linkCmpl.devAddr ), HAL_LCD_LINE_2 );   
        }
        else
        {
          simpleBLEState = BLE_STATE_IDLE;
          simpleBLEConnHandle = GAP_CONNHANDLE_INIT;
          simpleBLERssi = FALSE;
          simpleBLEDiscState = BLE_DISC_STATE_IDLE;
          
          LCD_WRITE_STRING( "Connect Failed", HAL_LCD_LINE_1 );
          LCD_WRITE_STRING_VALUE( "Reason:", pEvent->gap.hdr.status, 10, HAL_LCD_LINE_2 );
        }
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {
        simpleBLEState = BLE_STATE_IDLE;
        simpleBLEConnHandle = GAP_CONNHANDLE_INIT;
        simpleBLERssi = FALSE;
        simpleBLEDiscState = BLE_DISC_STATE_IDLE;
        simpleBLECharHdl = 0;
        simpleBLEProcedureInProgress = FALSE;
          
        LCD_WRITE_STRING( "Disconnected", HAL_LCD_LINE_1 );
        LCD_WRITE_STRING_VALUE( "Reason:", pEvent->linkTerminate.reason,
                                10, HAL_LCD_LINE_2 );
      }
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
        LCD_WRITE_STRING( "Param Update", HAL_LCD_LINE_1 );
      }
      break;
      
    default:
      break;
  }
  
  return ( TRUE );
}

/*********************************************************************
 * @fn      pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void simpleBLECentralPairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
  if ( state == GAPBOND_PAIRING_STATE_STARTED )
  {
    LCD_WRITE_STRING( "Pairing started", HAL_LCD_LINE_1 );
  }
  else if ( state == GAPBOND_PAIRING_STATE_COMPLETE )
  {
    if ( status == SUCCESS )
    {
      LCD_WRITE_STRING( "Pairing success", HAL_LCD_LINE_1 );
    }
    else
    {
      LCD_WRITE_STRING_VALUE( "Pairing fail", status, 10, HAL_LCD_LINE_1 );
    }
  }
  else if ( state == GAPBOND_PAIRING_STATE_BONDED )
  {
    if ( status == SUCCESS )
    {
      LCD_WRITE_STRING( "Bonding success", HAL_LCD_LINE_1 );
    }
  }
}

/*********************************************************************
 * @fn      simpleBLECentralPasscodeCB
 *
 * @brief   Passcode callback.	����ص�
 *
 * @return  none
 */
static void simpleBLECentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs )
{
#if (HAL_LCD == TRUE)

  uint32  passcode;
  uint8   str[7];

  // Create random passcode	�����������
  LL_Rand( ((uint8 *) &passcode), sizeof( uint32 ));
  passcode %= 1000000;
  
  // Display passcode to user	���û���ʾ��Կ
  if ( uiOutputs != 0 )
  {
    LCD_WRITE_STRING( "Passcode:",  HAL_LCD_LINE_1 );
    LCD_WRITE_STRING( (char *) _ltoa(passcode, str, 10),  HAL_LCD_LINE_2 );
  }
  
  // Send passcode response ����һ����������
  GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, passcode );
#endif
}

/*********************************************************************
 * @fn      simpleBLECentralStartDiscovery
 *
 * @brief   Start service discovery.	��ʼ������
 *
 * @return  none
 */
static void simpleBLECentralStartDiscovery( void )
{
  uint8 uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(SIMPLEPROFILE_SERV_UUID),
                                   HI_UINT16(SIMPLEPROFILE_SERV_UUID) };
  
  // Initialize cached handles	��ʼ������ľ��
  simpleBLESvcStartHdl = simpleBLESvcEndHdl = simpleBLECharHdl = 0;

  simpleBLEDiscState = BLE_DISC_STATE_SVC;	//������״̬��Ϊ������
  
  // Discovery simple BLE service	����һ���򵥵�BLE����
  GATT_DiscPrimaryServiceByUUID( simpleBLEConnHandle,
                                 uuid,
                                 ATT_BT_UUID_SIZE,
                                 simpleBLETaskId );
}

/*********************************************************************
 * @fn      simpleBLEGATTDiscoveryEvent
 *
 * @brief   Process GATT discovery event	����GATT�����¼�
 *
 * @return  none
 */
static void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg )
{
  attReadByTypeReq_t req;

	my_printf_num("simpleBLEGATTDiscoveryEvent() > ", pMsg->method);
  
  if ( simpleBLEDiscState == BLE_DISC_STATE_SVC )	//�������״̬�Ƿ�����
  {
  	my_printf("simpleBLEGATTDiscoveryEvent() > if BLE_DISC_STATE_SVC\n");
    // Service found, store handles	��Ϣ�����ǰ����Ͳ���ֵ��Ӧ���ҵ��ľ��������Ϣ
    if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
         pMsg->msg.findByTypeValueRsp.numInfo > 0 )
    {
    	my_printf("simpleBLEGATTDiscoveryEvent() > if 1\n");
      simpleBLESvcStartHdl = ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
      simpleBLESvcEndHdl = ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
    }
    
    // If procedure complete	�����Ͳ���ֵ��Ӧ
    if ( ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  && 
           pMsg->hdr.status == bleProcedureComplete ) ||
         ( pMsg->method == ATT_ERROR_RSP ) )
    {
    	my_printf("simpleBLEGATTDiscoveryEvent() > if 2\n");
      if ( simpleBLESvcStartHdl != 0 )
      {
      	my_printf("simpleBLEGATTDiscoveryEvent() > if 3\n");
        // Discover characteristic
        simpleBLEDiscState = BLE_DISC_STATE_CHAR;
        
        req.startHandle = simpleBLESvcStartHdl;
        req.endHandle = simpleBLESvcEndHdl;
        req.type.len = ATT_BT_UUID_SIZE;
        req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR1_UUID);	//Ҫ���������Ե�UUID
        req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR1_UUID);

        GATT_ReadUsingCharUUID( simpleBLEConnHandle, &req, simpleBLETaskId );	//���������Եľ��
      }
    }
  }
  else if ( simpleBLEDiscState == BLE_DISC_STATE_CHAR )	//�������������
  {
  	my_printf("simpleBLEGATTDiscoveryEvent() > if BLE_DISC_STATE_CHAR\n");
    // Characteristic found, store handle	�ҵ�����ֵ
    if ( pMsg->method == ATT_READ_BY_TYPE_RSP && 
         pMsg->msg.readByTypeRsp.numPairs > 0 )
    {
    	my_printf("simpleBLEGATTDiscoveryEvent() > if 4\n");
    	//�ӷ��ص������еõ�һ������ֵ���
      simpleBLECharHdl = BUILD_UINT16(pMsg->msg.readByTypeRsp.pDataList[0],
                                      pMsg->msg.readByTypeRsp.pDataList[1]);
      
      LCD_WRITE_STRING( "Simple Svc Found", HAL_LCD_LINE_1 );
      simpleBLEProcedureInProgress = FALSE;
    }
    
    simpleBLEDiscState = BLE_DISC_STATE_IDLE;	//����״̬��Ϊ����

    
  }    
}


/*********************************************************************
 * @fn      simpleBLEFindSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.	�ڹ���̵ķ���UUID�б��в��Ҹ�����UUID��
 *
 * @return  TRUE if service UUID found
 */
static bool simpleBLEFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen )
{
  uint8 adLen;
  uint8 adType;
  uint8 *pEnd;
  
  pEnd = pData + dataLen - 1;
  
  // While end of data not reached
  while ( pData < pEnd )
  {
    // Get length of next AD item
    adLen = *pData++;
    if ( adLen > 0 )
    {
      adType = *pData;
      
      // If AD type is for 16-bit service UUID
      if ( adType == GAP_ADTYPE_16BIT_MORE || adType == GAP_ADTYPE_16BIT_COMPLETE )
      {
        pData++;
        adLen--;
        
        // For each UUID in list
        while ( adLen >= 2 && pData < pEnd )
        {
          // Check for match
          if ( pData[0] == LO_UINT16(uuid) && pData[1] == HI_UINT16(uuid) )
          {
            // Match found
            return TRUE;
          }
          
          // Go to next
          pData += 2;
          adLen -= 2;
        }
        
        // Handle possible erroneous extra byte in UUID list
        if ( adLen == 1 )
        {
          pData++;
        }
      }
      else
      {
        // Go to next item
        pData += adLen;
      }
    }
  }
  
  // Match not found
  return FALSE;
}

/*********************************************************************
 * @fn      simpleBLEAddDeviceInfo
 *
 * @brief   Add a device to the device discovery result list	���һ���豸���豸�����б�
 *
 * @return  none
 */
static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType )
{
  uint8 i;
  uint8 uart_buff[50];
  
  // If result count not at max	����ܷ���8���豸
  if ( simpleBLEScanRes < DEFAULT_MAX_SCAN_RES )
  {
    // Check if device is already in scan results	����豸�Ƿ��Ѵ���ɨ������
    for ( i = 0; i < simpleBLEScanRes; i++ )
    {
      if ( osal_memcmp( pAddr, simpleBLEDevList[i].addr , B_ADDR_LEN ) )
      {
        return;
      }
    }

	memset(uart_buff, 0x00, 50);
	sprintf(uart_buff, "mac %x:%x:%x:%x:%x:%x %d\n", pAddr[0],pAddr[1],pAddr[2],pAddr[3],pAddr[4],pAddr[5], addrType);
	my_printf(uart_buff);
    // Add addr to scan result list	����豸���豸�����б�
    osal_memcpy( simpleBLEDevList[simpleBLEScanRes].addr, pAddr, B_ADDR_LEN );
    simpleBLEDevList[simpleBLEScanRes].addrType = addrType;
    
    // Increment scan result count
    simpleBLEScanRes++;
  }
}

/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;
  
  *pStr++ = '0';
  *pStr++ = 'x';
  
  // Start from end of addr
  pAddr += B_ADDR_LEN;
  
  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }
  
  *pStr = 0;
  
  return str;
}

/*********************************************************************
*********************************************************************/
