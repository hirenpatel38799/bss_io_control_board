/*******************************************************************************
  Application to Demo HTTP NET Server

  Summary:
    Support for HTTP NET module in Microchip TCP/IP Stack

  Description:
    -Implements the application
 *******************************************************************************/

/*
Copyright (C) 2012-2023, Microchip Technology Inc., and its subsidiaries. All rights reserved.

The software and documentation is provided by microchip and its contributors
"as is" and any express, implied or statutory warranties, including, but not
limited to, the implied warranties of merchantability, fitness for a particular
purpose and non-infringement of third party intellectual property rights are
disclaimed to the fullest extent permitted by law. In no event shall microchip
or its contributors be liable for any direct, indirect, incidental, special,
exemplary, or consequential damages (including, but not limited to, procurement
of substitute goods or services; loss of use, data, or profits; or business
interruption) however caused and on any theory of liability, whether in contract,
strict liability, or tort (including negligence or otherwise) arising in any way
out of the use of the software and documentation, even if advised of the
possibility of such damage.

Except as expressly permitted hereunder and subject to the applicable license terms
for any third-party software incorporated in the software and any applicable open
source software license terms, no license or other rights, whether express or
implied, are granted under any patent or other intellectual property rights of
Microchip or any third party.
*/

#include "system_config.h"
#include "system_definitions.h"
#include "http_net_print.h"
#if defined(TCPIP_STACK_USE_HTTP_NET_SERVER)

#include "crypto/crypto.h"
#include "net_pres/pres/net_pres_socketapi.h"
#include "system/sys_random_h2_adapter.h"
#include "system/sys_time_h2_adapter.h"
#include "tcpip/tcpip.h"
#include "tcpip/src/common/helpers.h"
#include "cJSON.h"
#include "definitions.h"                // SYS function prototypes
/****************************************************************************
  Section:
    Definitions
 ****************************************************************************/
#ifndef APP_SWITCH_1StateGet
#define APP_SWITCH_1StateGet() 0
#endif

#ifndef APP_SWITCH_2StateGet
#define APP_SWITCH_2StateGet() 0
#endif

#ifndef APP_SWITCH_3StateGet
#define APP_SWITCH_3StateGet() 0
#endif

#ifndef APP_LED_1StateGet
#define APP_LED_1StateGet() 0
#endif
#ifndef APP_LED_2StateGet
#define APP_LED_2StateGet() 0
#endif
#ifndef APP_LED_3StateGet
#define APP_LED_3StateGet() 0
#endif

#ifndef APP_LED_1StateSet
#define APP_LED_1StateSet()
#endif
#ifndef APP_LED_2StateSet
#define APP_LED_2StateSet()
#endif
#ifndef APP_LED_3StateSet
#define APP_LED_3StateSet()
#endif

#ifndef APP_LED_1StateClear
#define APP_LED_1StateClear()
#endif
#ifndef APP_LED_2StateClear
#define APP_LED_2StateClear()
#endif
#ifndef APP_LED_3StateClear
#define APP_LED_3StateClear()
#endif

#ifndef APP_LED_1StateToggle
#define APP_LED_1StateToggle()
#endif
#ifndef APP_LED_2StateToggle
#define APP_LED_2StateToggle()
#endif
#ifndef APP_LED_3StateToggle
#define APP_LED_3StateToggle()
#endif

// Use the web page in the Demo App (~2.5kb ROM, ~0b RAM)
#define HTTP_APP_USE_RECONFIG

#if !defined( NO_MD5 )        // no MD5 if crypto_config.h says NO_MD5   
// Use the MD5 Demo web page (~5kb ROM, ~160b RAM)
#define HTTP_APP_USE_MD5
#endif

// Use the e-mail demo web page
#if defined(TCPIP_STACK_USE_SMTPC)
#define HTTP_APP_USE_EMAIL  1
#else
#define HTTP_APP_USE_EMAIL  0
#endif

// Use authentication for MPFS upload
//#define HTTP_MPFS_UPLOAD_REQUIRES_AUTH

/****************************************************************************
Section:
Function Prototypes and Memory Globalizers
 ****************************************************************************/
#if defined(TCPIP_HTTP_NET_USE_POST)
    #if defined(SYS_OUT_ENABLE)
        static TCPIP_HTTP_NET_IO_RESULT HTTPPostLCD(TCPIP_HTTP_NET_CONN_HANDLE connHandle);
    #endif
    #if defined(HTTP_APP_USE_RECONFIG)
        #if defined(TCPIP_STACK_USE_SNMP_SERVER)
        static TCPIP_HTTP_NET_IO_RESULT HTTPPostSNMPCommunity(TCPIP_HTTP_NET_CONN_HANDLE connHandle);
        #endif
    #endif
    #if defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT)
        static TCPIP_HTTP_NET_IO_RESULT HTTPPostDDNSConfig(TCPIP_HTTP_NET_CONN_HANDLE connHandle);
    #endif
#endif

extern const char *const ddnsServiceHosts[];
// RAM allocated for DDNS parameters
#if defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT)
    static uint8_t DDNSData[100];
#endif

// Sticky status message variable.
// This is used to indicated whether or not the previous POST operation was
// successful.  The application uses these to store status messages when a
// POST operation redirects.  This lets the application provide status messages
// after a redirect, when connection instance data has already been lost.
static bool lastSuccess = false;

// Sticky status message variable.  See lastSuccess for details.
static bool lastFailure = false;

flash_data_t writeData;
/****************************************************************************
  Section:
    Customized HTTP NET Functions
 ****************************************************************************/

// processing the HTTP buffer acknowledgment
void TCPIP_HTTP_NET_DynAcknowledge(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const void *buffer, const struct _tag_TCPIP_HTTP_NET_USER_CALLBACK *pCBack)
{
    HTTP_APP_DYNVAR_BUFFER *pDynBuffer = (HTTP_APP_DYNVAR_BUFFER*)((const uint8_t *)buffer - offsetof(struct HTTP_APP_DYNVAR_BUFFER, data));

    pDynBuffer->busy = 0;
}

// processing the HTTP reported events
void TCPIP_HTTP_NET_EventReport(TCPIP_HTTP_NET_CONN_HANDLE connHandle, TCPIP_HTTP_NET_EVENT_TYPE evType, const void *evInfo, const struct _tag_TCPIP_HTTP_NET_USER_CALLBACK *pCBack)
{
    const char *evMsg = (const char *)evInfo;

    if(evType < 0)
    {   // display errors only
        if(evMsg == 0)
        {
            evMsg = "none";
        }
        SYS_CONSOLE_PRINT("HTTP event: %d, info: %s\r\n", evType, evMsg);
    }
}

// example of processing an SSI notification
// return false for standard processing of this SSI command by the HTTP module
// return true if the processing is done by you and HTTP need take no further action
bool TCPIP_HTTP_NET_SSINotification(TCPIP_HTTP_NET_CONN_HANDLE connHandle, TCPIP_HTTP_SSI_NOTIFY_DCPT *pSSINotifyDcpt, const struct _tag_TCPIP_HTTP_NET_USER_CALLBACK *pCBack)
{
    static  int newVarCount = 0;

    char    *cmdStr, *varName;
    char    newVarVal[] = "Page Visits: ";
    char    scratchBuff[100];

    cmdStr = pSSINotifyDcpt->ssiCommand;

    if(strcmp(cmdStr, "include") == 0)
    {   // here a standard SSI include directive is processed
        return false;
    }

    if(strcmp(cmdStr, "set") == 0)
    {   // a SSI variable is set; let the standard processing take place
        return false;
    }

    if(strcmp(cmdStr, "echo") == 0)
    {   // SSI echo command
        // inspect the variable name
        varName = pSSINotifyDcpt->pAttrDcpt->value;
        if(strcmp(varName, "myStrVar") == 0)
        {   // change the value of this variable
            sprintf(scratchBuff, "%s%d", newVarVal, ++newVarCount);

            if(!TCPIP_HTTP_NET_SSIVariableSet(varName, TCPIP_HTTP_DYN_ARG_TYPE_STRING, scratchBuff, 0))
            {
                SYS_CONSOLE_MESSAGE("SSI set myStrVar failed!!!\r\n");
            }
            // else success
            return false;
        }
    }

    return false;
}

/****************************************************************************
  Section:
    GET Form Handlers
 ****************************************************************************/

/*****************************************************************************
  Function:
    TCPIP_HTTP_NET_IO_RESULT TCPIP_HTTP_NET_ConnectionGetExecute(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_NET_USER_CALLBACK *pCBack)

  Internal:
    See documentation in the TCP/IP Stack APIs or http_net.h for details.
 ****************************************************************************/
TCPIP_HTTP_NET_IO_RESULT TCPIP_HTTP_NET_ConnectionGetExecute(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_NET_USER_CALLBACK *pCBack)
{
    const uint8_t *ptr;
    uint8_t *httpDataBuff;
    uint8_t filename[20];

    // Load the file name
    // Make sure uint8_t filename[] above is large enough for your longest name
    filename[0] = 0;
    SYS_FS_FileNameGet(TCPIP_HTTP_NET_ConnectionFileGet(connHandle), filename, 20);

    httpDataBuff = TCPIP_HTTP_NET_ConnectionDataBufferGet(connHandle);

    // If its the forms.htm page
    if(!memcmp(filename, "forms.htm", 9))
    {
        // Seek out each of the four LED strings, and if it exists set the LED states
        ptr = TCPIP_HTTP_NET_ArgGet(httpDataBuff, (const uint8_t *)"led2");
        if(ptr)
        {
            if(*ptr == '1')
            {
                APP_LED_2StateSet();
            }
            else
            {
                APP_LED_2StateClear();
            }
        }

        ptr = TCPIP_HTTP_NET_ArgGet(httpDataBuff, (const uint8_t *)"led1");
        if(ptr)
		{
            if(*ptr == '1')
            {
                APP_LED_1StateSet();
            }
            else
            {
                APP_LED_1StateClear();
            }
        }
    }

    else if(!memcmp(filename, "cookies.htm", 11))
    {
        // This is very simple.  The names and values we want are already in
        // the data array.  We just set the hasArgs value to indicate how many
        // name/value pairs we want stored as cookies.
        // To add the second cookie, just increment this value.
        // remember to also add a dynamic variable callback to control the printout.
        TCPIP_HTTP_NET_ConnectionHasArgsSet(connHandle, 0x01);
    }

    // If it's the LED updater file
    else if(!memcmp(filename, "leds.cgi", 8))
    {
        // Determine which LED to toggle
        ptr = TCPIP_HTTP_NET_ArgGet(httpDataBuff, (const uint8_t *)"led");

        // Toggle the specified LED
        if(ptr)
        {
            switch(*ptr)
            {
                case '0':
                    APP_LED_1StateToggle();
                    break;
                case '1':
                    APP_LED_2StateToggle();
                    break;
                case '2':
                    APP_LED_3StateToggle();
                    break;
            }
        }
    }

    return TCPIP_HTTP_NET_IO_RES_DONE;
}

/****************************************************************************
  Section:
    POST Form Handlers
 ****************************************************************************/
#if defined(TCPIP_HTTP_NET_USE_POST)

/*****************************************************************************
  Function:
    TCPIP_HTTP_NET_IO_RESULT TCPIP_HTTP_NET_ConnectionPostExecute(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_NET_USER_CALLBACK *pCBack)

  Internal:
    See documentation in the TCP/IP Stack APIs or http_net.h for details.
 ****************************************************************************/
TCPIP_HTTP_NET_IO_RESULT TCPIP_HTTP_NET_ConnectionPostExecute(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_NET_USER_CALLBACK *pCBack)
{
    uint8_t filename[20] = {0};
    uint8_t httpDataBuff[512] = {0};

    SYS_FS_FileNameGet(TCPIP_HTTP_NET_ConnectionFileGet(connHandle), filename, sizeof(filename));
    SYS_CONSOLE_PRINT("Post function - Requested File: %s\n", filename);

    TCPIP_HTTP_NET_ConnectionRead(connHandle, httpDataBuff, sizeof(httpDataBuff));
    SYS_CONSOLE_PRINT("Received JSON Data: %s\r\n", httpDataBuff);

    if (httpDataBuff[0] == '\0') {
        SYS_CONSOLE_MESSAGE("Error: No JSON data received\r\n");
        return TCPIP_HTTP_NET_IO_RES_DONE;
    }

    cJSON *json = cJSON_Parse((const char *)httpDataBuff);
    if (!json) {
        SYS_CONSOLE_MESSAGE("Error: JSON Parsing Failed\r\n");
        return TCPIP_HTTP_NET_IO_RES_DONE;
    }

    // ---- MAPPING ----
    if (!memcmp(filename, "mapping.cgi", 11)) {
        SYS_CONSOLE_MESSAGE("Processing Mapping Configuration from mapping.cgi\r\n");

        cJSON *mappingArray = cJSON_GetObjectItem(json, "mapping");
        if (cJSON_IsArray(mappingArray)) {
            for (int i = 0; i < cJSON_GetArraySize(mappingArray); i++) {
                cJSON *item = cJSON_GetArrayItem(mappingArray, i);
                if (!item) continue;

                cJSON *typeItem = cJSON_GetObjectItem(item, "type");
                cJSON *interfaceItem = cJSON_GetObjectItem(item, "interface");
                cJSON *tcpPortItem = cJSON_GetObjectItem(item, "tcpPort");

                if (cJSON_IsString(typeItem) && cJSON_IsString(interfaceItem) && cJSON_IsString(tcpPortItem)) {
                    int tcpPort = atoi(tcpPortItem->valuestring);

                    if (strncmp(typeItem->valuestring, "CAN", 3) == 0) {
                        int index = atoi(&interfaceItem->valuestring[3]) - 1;
                        if (index >= 0 && index < 6) {
                            writeData.canPorts[index] = tcpPort;
                            SYS_CONSOLE_PRINT("Updated %s to port %d\n", interfaceItem->valuestring, tcpPort);
                        }                       
                    } else if (strncmp(typeItem->valuestring, "RS485", 5) == 0) {
                        int index = atoi(&interfaceItem->valuestring[6]) - 1;
                        if (index >= 0 && index < 2) {
                            writeData.rs485Ports[index] = tcpPort;
                            SYS_CONSOLE_PRINT("Updated %s to port %d\n", interfaceItem->valuestring, tcpPort);
                        }
#if HEV_IO_Aggregator                         
                        // TCPIP_TCP_Close(sUart8ServerSocket);
                        // sUart8ServerSocket = INVALID_SOCKET;
                        // TCPIP_TCP_Close(sUart9ServerSocket);
                        // sUart9ServerSocket = INVALID_SOCKET;  
#endif                        
                    }
                }
            }
        }
    }

   // ---- UART CONFIG ----
    else if (!memcmp(filename, "uartconfig.cgi", 14)) {
        SYS_CONSOLE_MESSAGE("Processing UART Configuration from uartconfig.cgi\r\n");

        cJSON *uartArray = cJSON_GetObjectItem(json, "uartConfig");
        if (cJSON_IsArray(uartArray)) {
            for (int i = 0; i < cJSON_GetArraySize(uartArray); i++) {
                cJSON *uartItem = cJSON_GetArrayItem(uartArray, i);
                if (!uartItem) continue;

                cJSON *uart = cJSON_GetObjectItem(uartItem, "uart");
                cJSON *baudRate = cJSON_GetObjectItem(uartItem, "baudRate");
                cJSON *dataBits = cJSON_GetObjectItem(uartItem, "dataBits");
                cJSON *parity = cJSON_GetObjectItem(uartItem, "parity");
                cJSON *stopBits = cJSON_GetObjectItem(uartItem, "stopBits");

                if (cJSON_IsString(uart) && cJSON_IsString(baudRate) &&
                    cJSON_IsString(dataBits) && cJSON_IsString(parity) && cJSON_IsString(stopBits)) {

                    int baud = atoi(baudRate->valuestring);
                    int dataWidthVal = atoi(dataBits->valuestring);
                    int stop = atoi(stopBits->valuestring);
                    char parityChar = parity->valuestring[0];

                    USART_DATA dataWidth;
                    switch (dataWidthVal) {
                        case 5: dataWidth = USART_DATA_5_BIT; break;
                        case 6: dataWidth = USART_DATA_6_BIT; break;
                        case 7: dataWidth = USART_DATA_7_BIT; break;
                        case 8: dataWidth = USART_DATA_8_BIT; break;
                        default:
                            SYS_CONSOLE_PRINT("? Error: Invalid Data Bits: %d\n", dataWidthVal);
                            continue;
                    }

                    USART_PARITY newParity;
                    uint8_t parityEncoded;
                    if (parityChar == 'N') {
                        newParity = USART_PARITY_NONE;
                        parityEncoded = 0;
                    } else if (parityChar == 'O') {
                        newParity = USART_PARITY_ODD;
                        parityEncoded = 1;
                    } else if (parityChar == 'E') {
                        newParity = USART_PARITY_EVEN;
                        parityEncoded = 2;
                    } else {
                        SYS_CONSOLE_PRINT("? Error: Invalid Parity: %c\n", parityChar);
                        continue;
                    }

                    USART_SERIAL_SETUP setup = {
                        .baudRate = baud,
                        .dataWidth = dataWidth,
                        .parity = newParity,
                        .stopBits = (stop == 1) ? USART_STOP_0_BIT : USART_STOP_1_BIT
                    };

                    int uartIndex = -1;
                    if (strcmp(uart->valuestring, "RS485_1") == 0) {
                        uartIndex = 0;
                        SERCOM8_USART_ReadAbort();  // <- abort previous read
                        if (!SERCOM8_USART_SerialSetup(&setup, 0))
                        {
                            SYS_CONSOLE_PRINT("? RS485_1 Setup Failed: Baud=%d, DataBits=%d, Parity=%d, StopBits=%d\n",
                                              baud, dataWidthVal, parityEncoded, stop);
                        }
                        else
                        {
                            SYS_CONSOLE_MESSAGE("RS485_1 Setup Done\n");
                            uint8_t dummy;
                            bool result = SERCOM8_USART_Read(&dummy, 1); // after setup
                            if (result) {
                                SYS_CONSOLE_PRINT("RS485_1: RX resumed successfully\n");
                            } else {
                                SYS_CONSOLE_PRINT("RS485_1: RX resume FAILED\n");
                            }
                        }

                    } else if (strcmp(uart->valuestring, "RS485_2") == 0) {
                        uartIndex = 1;
                        SERCOM9_USART_ReadAbort();  // <- abort previous read
                        if (!SERCOM9_USART_SerialSetup(&setup, 0))
                        {
                            SYS_CONSOLE_PRINT("? RS485_2 Setup Failed: Baud=%d, DataBits=%d, Parity=%d, StopBits=%d\n",
                                              baud, dataWidthVal, parityEncoded, stop);
                        }
                        else
                        {
                            SYS_CONSOLE_MESSAGE("RS485_2 Setup Done\n");
                            uint8_t dummy;
                            bool result = SERCOM9_USART_Read(&dummy, 1); // after setup
                            if (result) {
                                SYS_CONSOLE_PRINT("RS485_2: RX resumed successfully\n");
                            } else {
                                SYS_CONSOLE_PRINT("RS485_2: RX resume FAILED\n");
                            }
                        }                        
                    }

                    // Save to config if UART index is valid
                    if (uartIndex >= 0) {
                        writeData.rs485Config[uartIndex].baudRate = baud;
                        writeData.rs485Config[uartIndex].dataBits = dataWidthVal;
                        writeData.rs485Config[uartIndex].parity = parityEncoded;
                        writeData.rs485Config[uartIndex].stopBits = stop;
                        SYS_CONSOLE_PRINT("Saved UART config for RS485_%d: %d,%d,%d,%d\n",
                            uartIndex + 1, baud, dataWidthVal, parityEncoded, stop);
                    }
                }
            }
        }
    }


    else if (!memcmp(filename, "canconfig.cgi", 13)) {
        SYS_CONSOLE_MESSAGE("Processing CAN Configuration from canconfig.cgi\r\n");

        cJSON *canArray = cJSON_GetObjectItem(json, "canConfig");
        if (cJSON_IsArray(canArray)) {
            for (int i = 0; i < cJSON_GetArraySize(canArray); i++) {
                cJSON *canItem = cJSON_GetArrayItem(canArray, i);
                if (!canItem) continue;

                cJSON *can = cJSON_GetObjectItem(canItem, "can");
                cJSON *canSpeed = cJSON_GetObjectItem(canItem, "canSpeed");

                if (cJSON_IsString(can) && cJSON_IsString(canSpeed)) {
                    uint32_t speed = strtoul(canSpeed->valuestring, NULL, 10);
                    int index = -1;

                    if      (strcmp(can->valuestring, "CAN1") == 0) index = 0;
                    else if (strcmp(can->valuestring, "CAN2") == 0) index = 1;
                    else if (strcmp(can->valuestring, "CAN3") == 0) index = 2;
                    else if (strcmp(can->valuestring, "CAN4") == 0) index = 3;
                    else if (strcmp(can->valuestring, "CAN5") == 0) index = 4;
                    else if (strcmp(can->valuestring, "CAN6") == 0) index = 5;

                    if (index >= 0 && index < 6) {
                        writeData.canBaudRates[index] = speed;
                        SYS_CONSOLE_PRINT("CAN%d Speed Set: %lu bps\r\n", index + 1, speed);
                    } else {
                        SYS_CONSOLE_PRINT("? Error: Unknown CAN Interface: %s\n", can->valuestring);
                    }
                }
            }
        }
    }

    saveOutputsToFlash(doStatus, relayStatus,serialnum);
    SYS_CONSOLE_MESSAGE("UART configuration saved to flash\r\n");

    cJSON_Delete(json);

    
    return TCPIP_HTTP_NET_IO_RES_DONE;
}


/*****************************************************************************
  Function:
    static TCPIP_HTTP_NET_IO_RESULT HTTPPostLCD(TCPIP_HTTP_NET_CONN_HANDLE connHandle)

  Summary:
    Processes the LCD form on forms.htm

  Description:
    Locates the 'lcd' parameter and uses it to update the text displayed
    on the board's LCD display.

    This function has four states.  The first reads a name from the data
    string returned as part of the POST request.  If a name cannot
    be found, it returns, asking for more data.  Otherwise, if the name
    is expected, it reads the associated value and writes it to the LCD.
    If the name is not expected, the value is discarded and the next name
    parameter is read.

    In the case where the expected string is never found, this function
    will eventually return TCPIP_HTTP_NET_IO_RES_NEED_DATA when no data is left.  In that
    case, the HTTP server will automatically trap the error and issue an
    Internal Server Error to the browser.

  Precondition:
    None

  Parameters:
    connHandle  - HTTP connection handle

  Return Values:
    TCPIP_HTTP_NET_IO_RES_DONE - the parameter has been found and saved
    TCPIP_HTTP_NET_IO_RES_WAITING - the function is pausing to continue later
    TCPIP_HTTP_NET_IO_RES_NEED_DATA - data needed by this function has not yet arrived
 ****************************************************************************/
#if defined(SYS_OUT_ENABLE)
static TCPIP_HTTP_NET_IO_RESULT HTTPPostLCD(TCPIP_HTTP_NET_CONN_HANDLE connHandle)
{
    uint8_t *cDest;
    uint8_t *httpDataBuff;
    uint16_t httpBuffSize;

#define SM_POST_LCD_READ_NAME       (0u)
#define SM_POST_LCD_READ_VALUE      (1u)

    httpDataBuff = TCPIP_HTTP_NET_ConnectionDataBufferGet(connHandle);
    httpBuffSize = TCPIP_HTTP_NET_ConnectionDataBufferSizeGet(connHandle);
    switch(TCPIP_HTTP_NET_ConnectionPostSmGet(connHandle))
    {
        // Find the name
        case SM_POST_LCD_READ_NAME:

            // Read a name
            if(TCPIP_HTTP_NET_ConnectionPostNameRead(connHandle, httpDataBuff, httpBuffSize) == TCPIP_HTTP_NET_READ_INCOMPLETE)
                return TCPIP_HTTP_NET_IO_RES_NEED_DATA;

            TCPIP_HTTP_NET_ConnectionPostSmSet(connHandle, SM_POST_LCD_READ_VALUE);
            // No break...continue reading value

        // Found the value, so store the LCD and return
        case SM_POST_LCD_READ_VALUE:

            // If value is expected, read it to data buffer,
            // otherwise ignore it (by reading to NULL)
            if(!strcmp((char *)httpDataBuff, (const char *)"lcd"))
                cDest = httpDataBuff;
            else
                cDest = NULL;

            // Read a value string
            if(TCPIP_HTTP_NET_ConnectionPostValueRead(connHandle, cDest, httpBuffSize) == TCPIP_HTTP_NET_READ_INCOMPLETE)
                return TCPIP_HTTP_NET_IO_RES_NEED_DATA;

            // If this was an unexpected value, look for a new name
            if(!cDest)
            {
                TCPIP_HTTP_NET_ConnectionPostSmSet(connHandle, SM_POST_LCD_READ_NAME);
                break;
            }

            SYS_OUT_MESSAGE((char *)cDest);

            // This is the only expected value, so callback is done
            strcpy((char *)httpDataBuff, "/forms.htm");
            TCPIP_HTTP_NET_ConnectionStatusSet(connHandle, TCPIP_HTTP_NET_STAT_REDIRECT);
            return TCPIP_HTTP_NET_IO_RES_DONE;
    }

    // Default assumes that we're returning for state machine convenience.
    // Function will be called again later.
    return TCPIP_HTTP_NET_IO_RES_WAITING;
}
#endif

/*****************************************************************************
  Function:
    static TCPIP_HTTP_NET_IO_RESULT HTTPPostMD5(TCPIP_HTTP_NET_CONN_HANDLE connHandle)

  Summary:
    Processes the file upload form on upload.htm

  Description:
    This function demonstrates the processing of file uploads.  First, the
    function locates the file data, skipping over any headers that arrive.
    Second, it reads the file 64 bytes at a time and hashes that data.  Once
    all data has been received, the function calculates the MD5 sum and
    stores it in current connection data buffer.

    After the headers, the first line from the form will be the MIME
    separator.  Following that is more headers about the file, which we
    discard.  After another CRLFCRLF, the file data begins, and we read
    it 16 bytes at a time and add that to the MD5 calculation.  The reading
    terminates when the separator string is encountered again on its own
    line.  Notice that the actual file data is trashed in this process,
    allowing us to accept files of arbitrary size, not limited by RAM.
    Also notice that the data buffer is used as an arbitrary storage array
    for the result.  The ~uploadedmd5~ callback reads this data later to
    send back to the client.

  Precondition:
    None

  Parameters:
    connHandle  - HTTP connection handle

  Return Values:
    TCPIP_HTTP_NET_IO_RES_DONE - all parameters have been processed
    TCPIP_HTTP_NET_IO_RES_WAITING - the function is pausing to continue later
    TCPIP_HTTP_NET_IO_RES_NEED_DATA - data needed by this function has not yet arrived
 ****************************************************************************/
#if defined(HTTP_APP_USE_MD5)

#endif // #if defined(HTTP_APP_USE_MD5)

/*****************************************************************************
  Function:
    static TCPIP_HTTP_NET_IO_RESULT HTTPPostConfig(TCPIP_HTTP_NET_CONN_HANDLE connHandle)

  Summary:
    Processes the configuration form on config/index.htm

  Description:
    Accepts configuration parameters from the form, saves them to a
    temporary location in RAM, then eventually saves the data to EEPROM or
    external Flash.

    When complete, this function redirects to config/reboot.htm, which will
    display information on reconnecting to the board.

    This function creates a shadow copy of a network info structure in
    RAM and then overwrites incoming data there as it arrives.  For each
    name/value pair, the name is first read to cur connection data[0:5].  Next, the
    value is read to newNetConfig.  Once all data has been read, the new
    network info structure is saved back to storage and the browser is redirected to
    reboot.htm.  That file includes an AJAX call to reboot.cgi, which
    performs the actual reboot of the machine.

    If an IP address cannot be parsed, too much data is POSTed, or any other
    parsing error occurs, the browser reloads config.htm and displays an error
    message at the top.

  Precondition:
    None

  Parameters:
    connHandle  - HTTP connection handle

  Return Values:
    TCPIP_HTTP_NET_IO_RES_DONE - all parameters have been processed
    TCPIP_HTTP_NET_IO_RES_NEED_DATA - data needed by this function has not yet arrived
 ****************************************************************************/
#if defined(HTTP_APP_USE_RECONFIG)
// network configuration/information storage space


#if defined(TCPIP_STACK_USE_SNMP_SERVER)
static TCPIP_HTTP_NET_IO_RESULT HTTPPostSNMPCommunity(TCPIP_HTTP_NET_CONN_HANDLE connHandle)
{
    uint8_t len = 0;
    uint8_t vCommunityIndex;
    uint8_t *httpDataBuff;
    uint16_t httpBuffSize;

    #define SM_CFG_SNMP_READ_NAME   (0u)
    #define SM_CFG_SNMP_READ_VALUE  (1u)

    httpDataBuff = TCPIP_HTTP_NET_ConnectionDataBufferGet(connHandle);
    httpBuffSize = TCPIP_HTTP_NET_ConnectionDataBufferSizeGet(connHandle);
    switch(TCPIP_HTTP_NET_ConnectionPostSmGet(connHandle))
    {
        case SM_CFG_SNMP_READ_NAME:
            // If all parameters have been read, end
            if(TCPIP_HTTP_NET_ConnectionByteCountGet(connHandle) == 0u)
            {
                return TCPIP_HTTP_NET_IO_RES_DONE;
            }

            // Read a name
            if(TCPIP_HTTP_NET_ConnectionPostNameRead(connHandle, httpDataBuff, httpBuffSize) == TCPIP_HTTP_NET_READ_INCOMPLETE)
                return TCPIP_HTTP_NET_IO_RES_NEED_DATA;

            // Move to reading a value, but no break
            TCPIP_HTTP_NET_ConnectionPostSmSet(connHandle, SM_CFG_SNMP_READ_VALUE);

        case SM_CFG_SNMP_READ_VALUE:
            // Read a value
            if(TCPIP_HTTP_NET_ConnectionPostValueRead(connHandle, httpDataBuff + 6, httpBuffSize) == TCPIP_HTTP_NET_READ_INCOMPLETE)
                return TCPIP_HTTP_NET_IO_RES_NEED_DATA;

            // Default action after this is to read the next name, unless there's an error
            TCPIP_HTTP_NET_ConnectionPostSmSet(connHandle, SM_CFG_SNMP_READ_NAME);

            // See if this is a known parameter and legal (must be null
            // terminator in 4th field name byte, string must no greater than
            // TCPIP_SNMP_COMMUNITY_MAX_LEN bytes long, and TCPIP_SNMP_MAX_COMMUNITY_SUPPORT
            // must not be violated.
            vCommunityIndex = httpDataBuff[3] - '0';
            if(vCommunityIndex >= TCPIP_SNMP_MAX_COMMUNITY_SUPPORT)
                break;
            if(httpDataBuff[4] != 0x00u)
                break;
            len = strlen((char *)httpDataBuff + 6);
            if(len > TCPIP_SNMP_COMMUNITY_MAX_LEN)
            {
                break;
            }
            if(memcmp((void *)httpDataBuff, (const void *)"rcm", 3) == 0)
            {
                if(TCPIP_SNMP_ReadCommunitySet(vCommunityIndex,len,httpDataBuff + 6)!=true)
                    break;
            }
            else if(memcmp((void *)httpDataBuff, (const void *)"wcm", 3) == 0)
            {
                if(TCPIP_SNMP_WriteCommunitySet(vCommunityIndex,len,httpDataBuff + 6) != true)
                    break;
            }
            else
            {
                break;
            }

            break;
    }

    return TCPIP_HTTP_NET_IO_RES_WAITING; // Assume we're waiting to process more data
}
#endif // #if defined(TCPIP_STACK_USE_SNMP_SERVER)
#endif // #if defined(HTTP_APP_USE_RECONFIG)

/*****************************************************************************
  Function:
    static TCPIP_HTTP_NET_IO_RESULT HTTPPostEmail(void)

  Summary:
    Processes the e-mail form on email/index.htm

  Description:
    This function sends an e-mail message using the SMTPC client.
    If encryption is needed it is done by the SMTPC module communicating with the SMTP server.
    (the NET_PRES layer has to be configured for encryption support).
    
    It demonstrates the use of the SMTPC client, waiting for asynchronous
    processes in an HTTP callback.
    
  Precondition:
    None

  Parameters:
    connHandle  - HTTP connection handle

  Return Values:
    TCPIP_HTTP_NET_IO_RES_DONE - the message has been sent
    TCPIP_HTTP_NET_IO_RES_WAITING - the function is waiting for the SMTP process to complete
    TCPIP_HTTP_NET_IO_RES_NEED_DATA - data needed by this function has not yet arrived
 ****************************************************************************/
#if (HTTP_APP_USE_EMAIL != 0) 
// size of an email parameter
#define HTTP_APP_EMAIL_PARAM_SIZE           30 
// maximum size of the mail body
#define HTTP_APP_EMAIL_BODY_SIZE            200 
// maximum size of the mail attachment
#define HTTP_APP_EMAIL_ATTACHMENT_SIZE      200 

// structure describing the post email operation
typedef struct
{
    char*   ptrParam;       // pointer to the current parameter being retrieved
    int     paramSize;      // size of the buffer to retrieve the parameter
    int     attachLen;      // length of the attachment buffer
    bool    mailParamsDone; // flag that signals that all parameters were retrieved
    TCPIP_SMTPC_ATTACH_BUFFER attachBuffer; // descriptor for the attachment
    TCPIP_SMTPC_MESSAGE_RESULT mailRes;     // operation outcome

    // storage area
    char serverName[HTTP_APP_EMAIL_PARAM_SIZE + 1];
    char username[HTTP_APP_EMAIL_PARAM_SIZE + 1];
    char password[HTTP_APP_EMAIL_PARAM_SIZE + 1];
    char mailTo[HTTP_APP_EMAIL_PARAM_SIZE + 1];
    char serverPort[10 + 1];
    char mailBody[HTTP_APP_EMAIL_BODY_SIZE + 1];
    char mailAttachment[HTTP_APP_EMAIL_ATTACHMENT_SIZE];

}HTTP_POST_EMAIL_DCPT;

#endif // (HTTP_APP_USE_EMAIL != 0) 

/****************************************************************************
  Function:
    TCPIP_HTTP_NET_IO_RESULT HTTPPostDDNSConfig(TCPIP_HTTP_NET_CONN_HANDLE connHandle)

  Summary:
    Parsing and collecting http data received from http form

  Description:
    This routine will be excuted every time the Dynamic DNS Client
    configuration form is submitted.  The http data is received
    as a string of the variables seperated by '&' characters in the TCP RX
    buffer.  This data is parsed to read the required configuration values,
    and those values are populated to the global array (DDNSData) reserved
    for this purpose.  As the data is read, DDNSPointers is also populated
    so that the dynamic DNS client can execute with the new parameters.

  Precondition:
    cur HTTP connection is loaded

  Parameters:
    connHandle  - HTTP connection handle

  Return Values:
    TCPIP_HTTP_NET_IO_RES_DONE      -  Finished with procedure
    TCPIP_HTTP_NET_IO_RES_NEED_DATA -  More data needed to continue, call again later
    TCPIP_HTTP_NET_IO_RES_WAITING   -  Waiting for asynchronous process to complete,
                                        call again later
 ****************************************************************************/
#if defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT)
static TCPIP_HTTP_NET_IO_RESULT HTTPPostDDNSConfig(TCPIP_HTTP_NET_CONN_HANDLE connHandle)
{
    static uint8_t *ptrDDNS;
    uint8_t *httpDataBuff;
    uint16_t httpBuffSize;
    uint8_t smPost;

    #define SM_DDNS_START           (0u)
    #define SM_DDNS_READ_NAME       (1u)
    #define SM_DDNS_READ_VALUE      (2u)
    #define SM_DDNS_READ_SERVICE    (3u)
    #define SM_DDNS_DONE            (4u)

    #define DDNS_SPACE_REMAINING    (sizeof(DDNSData) - (ptrDDNS - DDNSData))

    httpDataBuff = TCPIP_HTTP_NET_ConnectionDataBufferGet(connHandle);
    httpBuffSize = TCPIP_HTTP_NET_ConnectionDataBufferSizeGet(connHandle);
    smPost = TCPIP_HTTP_NET_ConnectionPostSmGet(connHandle);
    switch(smPost)
    {
        // Sets defaults for the system
        case SM_DDNS_START:
            ptrDDNS = DDNSData;
            TCPIP_DDNS_ServiceSet(0);
            DDNSClient.Host.szROM = NULL;
            DDNSClient.Username.szROM = NULL;
            DDNSClient.Password.szROM = NULL;
            DDNSClient.ROMPointers.Host = 0;
            DDNSClient.ROMPointers.Username = 0;
            DDNSClient.ROMPointers.Password = 0;
            TCPIP_HTTP_NET_ConnectionPostSmSet(connHandle, ++smPost);

        // Searches out names and handles them as they arrive
        case SM_DDNS_READ_NAME:
            // If all parameters have been read, end
            if(TCPIP_HTTP_NET_ConnectionByteCountGet(connHandle) == 0u)
            {
                TCPIP_HTTP_NET_ConnectionPostSmSet(connHandle, SM_DDNS_DONE);
                break;
            }

            // Read a name
            if(TCPIP_HTTP_NET_ConnectionPostNameRead(connHandle, httpDataBuff, httpBuffSize) == TCPIP_HTTP_NET_READ_INCOMPLETE)
                return TCPIP_HTTP_NET_IO_RES_NEED_DATA;

            if(!strcmp((char *)httpDataBuff, (const char *)"service"))
            {
                // Reading the service (numeric)
                TCPIP_HTTP_NET_ConnectionPostSmSet(connHandle, SM_DDNS_READ_SERVICE);
                break;
            }
            else if(!strcmp((char *)httpDataBuff, (const char *)"user"))
                DDNSClient.Username.szRAM = ptrDDNS;
            else if(!strcmp((char *)httpDataBuff, (const char *)"pass"))
                DDNSClient.Password.szRAM = ptrDDNS;
            else if(!strcmp((char *)httpDataBuff, (const char *)"host"))
                DDNSClient.Host.szRAM = ptrDDNS;

            // Move to reading the value for user/pass/host
            TCPIP_HTTP_NET_ConnectionPostSmSet(connHandle, ++smPost);

        // Reads in values and assigns them to the DDNS RAM
        case SM_DDNS_READ_VALUE:
            // Read a name
            if(TCPIP_HTTP_NET_ConnectionPostValueRead(connHandle, ptrDDNS, DDNS_SPACE_REMAINING) == TCPIP_HTTP_NET_READ_INCOMPLETE)
                return TCPIP_HTTP_NET_IO_RES_NEED_DATA;

            // Move past the data that was just read
            ptrDDNS += strlen((char *)ptrDDNS);
            if(ptrDDNS < DDNSData + sizeof(DDNSData) - 1)
                ptrDDNS += 1;

            // Return to reading names
            TCPIP_HTTP_NET_ConnectionPostSmSet(connHandle, SM_DDNS_READ_NAME);
            break;

        // Reads in a service ID
        case SM_DDNS_READ_SERVICE:
            // Read the integer id
            if(TCPIP_HTTP_NET_ConnectionPostValueRead(connHandle, httpDataBuff, httpBuffSize) == TCPIP_HTTP_NET_READ_INCOMPLETE)
                return TCPIP_HTTP_NET_IO_RES_NEED_DATA;

            // Convert to a service ID
            TCPIP_DDNS_ServiceSet((uint8_t)atol((char *)httpDataBuff));

            // Return to reading names
            TCPIP_HTTP_NET_ConnectionPostSmSet(connHandle, SM_DDNS_READ_NAME);
            break;

        // Sets up the DDNS client for an update
        case SM_DDNS_DONE:
            // Since user name and password changed, force an update immediately
            TCPIP_DDNS_UpdateForce();

            // Redirect to prevent POST errors
            lastSuccess = true;
            strcpy((char *)httpDataBuff, "/dyndns/index.htm");
            TCPIP_HTTP_NET_ConnectionStatusSet(connHandle, TCPIP_HTTP_NET_STAT_REDIRECT);
            return TCPIP_HTTP_NET_IO_RES_DONE;
    }

    return TCPIP_HTTP_NET_IO_RES_WAITING;     // Assume we're waiting to process more data
}
#endif // #if defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT)

#endif // #if defined(TCPIP_HTTP_NET_USE_POST)

/****************************************************************************
  Section:
    Authorization Handlers
 ****************************************************************************/

/*****************************************************************************
  Function:
    uint8_t TCPIP_HTTP_NET_ConnectionFileAuthenticate(TCPIP_HTTP_NET_CONN_HANDLE connHandle, uint8_t *cFile, const TCPIP_HTTP_NET_USER_CALLBACK *pCBack)

  Internal:
    See documentation in the TCP/IP Stack APIs or http_net.h for details.
 ****************************************************************************/
#if defined(TCPIP_HTTP_NET_USE_AUTHENTICATION)
uint8_t TCPIP_HTTP_NET_ConnectionFileAuthenticate(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const char *cFile, const TCPIP_HTTP_NET_USER_CALLBACK *pCBack)
{
    // If the filename begins with the folder "protect", then require auth
    if(memcmp(cFile, "protect", 7) == 0)
        return 0x00;        // Authentication will be needed later

    // If the filename begins with the folder "snmp", then require auth
    if(memcmp(cFile, "snmp", 4) == 0)
        return 0x00;        // Authentication will be needed later

    #if defined(HTTP_MPFS_UPLOAD_REQUIRES_AUTH)
    if(memcmp(cFile, TCPIP_HTTP_NET_FILE_UPLOAD_NAME, sizeof(TCPIP_HTTP_NET_FILE_UPLOAD_NAME)) == 0)
        return 0x00;
    #endif
    // You can match additional strings here to password protect other files.
    // You could switch this and exclude files from authentication.
    // You could also always return 0x00 to require auth for all files.
    // You can return different values (0x00 to 0x79) to track "realms" for below.

    return 0x80; // No authentication required
}
#endif

/*****************************************************************************
  Function:
    uint8_t TCPIP_HTTP_NET_ConnectionUserAuthenticate(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const char *cUser, const char *cPass, const TCPIP_HTTP_NET_USER_CALLBACK *pCBack)

  Internal:
    See documentation in the TCP/IP Stack APIs or http_net.h for details.
 ****************************************************************************/
#if defined(TCPIP_HTTP_NET_USE_AUTHENTICATION)
uint8_t TCPIP_HTTP_NET_ConnectionUserAuthenticate(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const char *cUser, const char *cPass, const TCPIP_HTTP_NET_USER_CALLBACK *pCBack)
{
    if(strcmp(cUser,(const char *)"admin") == 0
        && strcmp(cPass, (const char *)"microchip") == 0)
        return 0x80;        // We accept this combination

    // You can add additional user/pass combos here.
    // If you return specific "realm" values above, you can base this
    //   decision on what specific file or folder is being accessed.
    // You could return different values (0x80 to 0xff) to indicate
    //   various users or groups, and base future processing decisions
    //   in TCPIP_HTTP_NET_ConnectionGetExecute/Post or HTTPPrint callbacks on this value.

    return 0x00;            // Provided user/pass is invalid
}
#endif

/****************************************************************************
  Section:
    Dynamic Variable Callback Functions
 ****************************************************************************/
static void JSON_EscapeXML(char *dest, const char *src)
{
    while (*src) {
        switch (*src) {
            case '&': dest += sprintf(dest, "&amp;"); break;
            case '<': dest += sprintf(dest, "&lt;"); break;
            case '>': dest += sprintf(dest, "&gt;"); break;
            case '"': dest += sprintf(dest, "&quot;"); break;
            default: *dest++ = *src; break;
        }
        src++;
    }
    *dest = '\0';
}

TCPIP_HTTP_DYN_PRINT_RES TCPIP_HTTP_Print_mapping_data(
    TCPIP_HTTP_NET_CONN_HANDLE connHandle,
    const TCPIP_HTTP_DYN_VAR_DCPT *vDcpt)
{
    char rawJSON[1024], escaped[2048];  // bigger buffers
    char *p = rawJSON;
    bool first = true;

    *p++ = '[';

    for (int i = 0; i < 6; i++) {
        if (writeData.canPorts[i] > 0) {
            if (!first) *p++ = ',';
            p += sprintf(p, "{\"type\":\"CAN\",\"interface\":\"CAN%d\",\"tcpPort\":%d}", i+1, writeData.canPorts[i]);
            first = false;
        }
    }

    for (int i = 0; i < 2; i++) {
        if (writeData.rs485Ports[i] > 0) {
            if (!first) *p++ = ',';
            p += sprintf(p, "{\"type\":\"RS485\",\"interface\":\"RS485_%d\",\"tcpPort\":%d}", i+1, writeData.rs485Ports[i]);
            first = false;
        }
    }

    *p++ = ']';
    *p = '\0';

    JSON_EscapeXML(escaped, rawJSON);
    SYS_CONSOLE_PRINT("Escaped Mapping JSON: %s\n", escaped);
    TCPIP_HTTP_NET_DynamicWrite(vDcpt, escaped, strlen(escaped), true);

    return TCPIP_HTTP_DYN_PRINT_RES_DONE;
}

TCPIP_HTTP_DYN_PRINT_RES TCPIP_HTTP_Print_can_data(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_DYN_VAR_DCPT *vDcpt)
{
    HTTP_APP_DYNVAR_BUFFER *pDynBuffer = HTTP_APP_GetDynamicBuffer();
    if (!pDynBuffer) return TCPIP_HTTP_DYN_PRINT_RES_AGAIN;

    char *p = pDynBuffer->data;
    *p++ = '[';

    for (int i = 0; i < 6; i++) {
        if (writeData.canBaudRates[i]) {
            p += sprintf(p, "{\"can\":\"CAN%d\",\"canSpeed\":\"%lu\"},", i + 1, writeData.canBaudRates[i]);
        }
    }

    if (*(p-1) == ',') p--;  // remove trailing comma
    *p++ = ']';
    *p = 0;

    TCPIP_HTTP_NET_DynamicWrite(vDcpt, pDynBuffer->data, strlen(pDynBuffer->data), true);
    SYS_CONSOLE_PRINT("Mapping JSON Sent: %s\n", pDynBuffer->data);

    return TCPIP_HTTP_DYN_PRINT_RES_DONE;
}
TCPIP_HTTP_DYN_PRINT_RES TCPIP_HTTP_Print_rs485_data(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_DYN_VAR_DCPT *vDcpt)
{
    HTTP_APP_DYNVAR_BUFFER *pDynBuffer = HTTP_APP_GetDynamicBuffer();
    if (!pDynBuffer) return TCPIP_HTTP_DYN_PRINT_RES_AGAIN;

    char *p = pDynBuffer->data;
    *p++ = '[';

    for (int i = 0; i < 2; i++) {
        if (writeData.rs485Config[i].baudRate) {
            p += sprintf(p,
                "{\"uart\":\"RS485_%d\",\"baudRate\":\"%ld\",\"dataBits\":\"%d\",\"parity\":\"%s\",\"stopBits\":\"%d\"},",
                i+1,
                writeData.rs485Config[i].baudRate,
                writeData.rs485Config[i].dataBits,
                writeData.rs485Config[i].parity == 0 ? "None" :
                writeData.rs485Config[i].parity == 1 ? "Odd" : "Even",
                writeData.rs485Config[i].stopBits
            );
        }
    }

    if (*(p-1) == ',') p--;  // remove trailing comma
    *p++ = ']';
    *p = 0;

    TCPIP_HTTP_NET_DynamicWrite(vDcpt, pDynBuffer->data, strlen(pDynBuffer->data), true);
    SYS_CONSOLE_PRINT("Mapping JSON Sent: %s\n", pDynBuffer->data);

    return TCPIP_HTTP_DYN_PRINT_RES_DONE;
}


TCPIP_HTTP_DYN_PRINT_RES TCPIP_HTTP_Print_hellomsg(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_DYN_VAR_DCPT *vDcpt)
{
    const uint8_t *ptr;

    ptr = TCPIP_HTTP_NET_ArgGet(TCPIP_HTTP_NET_ConnectionDataBufferGet(connHandle), (const uint8_t *)"name");

    if(ptr != NULL)
    {
        size_t nChars;
        HTTP_APP_DYNVAR_BUFFER *pDynBuffer = HTTP_APP_GetDynamicBuffer();
        if(pDynBuffer == 0)
        {   // failed to get a buffer; retry
            return TCPIP_HTTP_DYN_PRINT_RES_AGAIN;
        }

        nChars = sprintf(pDynBuffer->data, "Hello, %s", ptr);
        TCPIP_HTTP_NET_DynamicWrite(vDcpt, pDynBuffer->data, nChars, true);
    }

    return TCPIP_HTTP_DYN_PRINT_RES_DONE;
}

TCPIP_HTTP_DYN_PRINT_RES TCPIP_HTTP_Print_builddate(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_DYN_VAR_DCPT *vDcpt)
{
    TCPIP_HTTP_NET_DynamicWriteString(vDcpt, __DATE__" "__TIME__, false);
    return TCPIP_HTTP_DYN_PRINT_RES_DONE;
}

TCPIP_HTTP_DYN_PRINT_RES TCPIP_HTTP_Print_version(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_DYN_VAR_DCPT *vDcpt)
{
    TCPIP_HTTP_NET_DynamicWriteString(vDcpt, (const char *)TCPIP_STACK_VERSION_STR, false);
    return TCPIP_HTTP_DYN_PRINT_RES_DONE;
}

TCPIP_HTTP_DYN_PRINT_RES TCPIP_HTTP_Print_cookiename(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_DYN_VAR_DCPT *vDcpt)
{
    const char *ptr;

    ptr = (const char *)TCPIP_HTTP_NET_ArgGet(TCPIP_HTTP_NET_ConnectionDataBufferGet(connHandle), (const uint8_t *)"name");

    if(ptr == 0)
        ptr = "not set";

    size_t nChars;
    HTTP_APP_DYNVAR_BUFFER *pDynBuffer = HTTP_APP_GetDynamicBuffer();
    if(pDynBuffer == 0)
    {   // failed to get a buffer; retry
        return TCPIP_HTTP_DYN_PRINT_RES_AGAIN;
    }

    nChars = sprintf(pDynBuffer->data, "%s", ptr);
    TCPIP_HTTP_NET_DynamicWrite(vDcpt, pDynBuffer->data, nChars, true);
    return TCPIP_HTTP_DYN_PRINT_RES_DONE;
}

TCPIP_HTTP_DYN_PRINT_RES TCPIP_HTTP_Print_cookiefav(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_DYN_VAR_DCPT *vDcpt)
{
    const char *ptr;

    ptr = (const char *)TCPIP_HTTP_NET_ArgGet(TCPIP_HTTP_NET_ConnectionDataBufferGet(connHandle), (const uint8_t *)"fav");

    if(ptr == 0)
        ptr = "not set";

    size_t nChars;
    HTTP_APP_DYNVAR_BUFFER *pDynBuffer = HTTP_APP_GetDynamicBuffer();
    if(pDynBuffer == 0)
    {   // failed to get a buffer; retry
        return TCPIP_HTTP_DYN_PRINT_RES_AGAIN;
    }
    nChars = sprintf(pDynBuffer->data, "%s", ptr);
    TCPIP_HTTP_NET_DynamicWrite(vDcpt, pDynBuffer->data, nChars, true);
    return TCPIP_HTTP_DYN_PRINT_RES_DONE;
}

TCPIP_HTTP_DYN_PRINT_RES TCPIP_HTTP_Print_btn(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_DYN_VAR_DCPT *vDcpt)
{
    // Determine which button
    if(vDcpt->nArgs != 0 && vDcpt->dynArgs->argType == TCPIP_HTTP_DYN_ARG_TYPE_INT32)
    {
        int nBtn = vDcpt->dynArgs->argInt32;
        switch(nBtn)
        {
            case 0:
                nBtn = APP_SWITCH_1StateGet();
                break;
            case 1:
                nBtn = APP_SWITCH_2StateGet();
                break;
            case 2:
                nBtn = APP_SWITCH_3StateGet();
                break;
            default:
                nBtn = 0;
        }

        // Print the output
        TCPIP_HTTP_NET_DynamicWriteString(vDcpt, (nBtn ? "up" : "dn"), false);
    }
    return TCPIP_HTTP_DYN_PRINT_RES_DONE;
}

TCPIP_HTTP_DYN_PRINT_RES TCPIP_HTTP_Print_led(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_DYN_VAR_DCPT *vDcpt)
{
    // Determine which LED
    if(vDcpt->nArgs != 0 && vDcpt->dynArgs->argType == TCPIP_HTTP_DYN_ARG_TYPE_INT32)
    {
        int nLed = vDcpt->dynArgs->argInt32;
        switch(nLed)
        {
            case 0:
                nLed = APP_LED_1StateGet();
                break;

            case 1:
                nLed = APP_LED_2StateGet();
                break;

            case 2:
                nLed = APP_LED_3StateGet();
                break;

            default:
                nLed = 0;
        }

        // Print the output
        const char *ledMsg = nLed ? "1": "0";

        TCPIP_HTTP_NET_DynamicWriteString(vDcpt, ledMsg, false);
    }

    return TCPIP_HTTP_DYN_PRINT_RES_DONE;
}

TCPIP_HTTP_DYN_PRINT_RES TCPIP_HTTP_Print_ledSelected(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_DYN_VAR_DCPT *vDcpt)
{
    // Determine which LED to check
    if(vDcpt->nArgs >= 2 && vDcpt->dynArgs->argType == TCPIP_HTTP_DYN_ARG_TYPE_INT32 && (vDcpt->dynArgs + 1)->argType == TCPIP_HTTP_DYN_ARG_TYPE_STRING)
    {
        int nLed = vDcpt->dynArgs->argInt32;
        int state = 0;
        if(strcmp((vDcpt->dynArgs + 1)->argStr, "true") == 0)
        {
            state = 1;
        }

        switch(nLed)
        {
            case 0:
                nLed = APP_LED_1StateGet();
                break;
            case 1:
                nLed = APP_LED_2StateGet();
                break;
            case 2:
                nLed = APP_LED_3StateGet();
                break;
            default:
                nLed = 0;
        }

        // Print output if true and ON or if false and OFF
        if((state && nLed) || (!state && !nLed))
            TCPIP_HTTP_NET_DynamicWriteString(vDcpt, "SELECTED", false);
    }

    return TCPIP_HTTP_DYN_PRINT_RES_DONE;
}

TCPIP_HTTP_DYN_PRINT_RES TCPIP_HTTP_Print_pot(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_DYN_VAR_DCPT *vDcpt)
{
    uint16_t RandVal;
    size_t nChars;

    HTTP_APP_DYNVAR_BUFFER *pDynBuffer = HTTP_APP_GetDynamicBuffer();
    if(pDynBuffer == 0)
    {   // failed to get a buffer; retry
        return TCPIP_HTTP_DYN_PRINT_RES_AGAIN;
    }

    RandVal = (uint16_t)SYS_RANDOM_PseudoGet();
    nChars = sprintf(pDynBuffer->data, "%d", RandVal);
    TCPIP_HTTP_NET_DynamicWrite(vDcpt, pDynBuffer->data, nChars, true);
    return TCPIP_HTTP_DYN_PRINT_RES_DONE;
}

TCPIP_HTTP_DYN_PRINT_RES TCPIP_HTTP_Print_status_ok(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_DYN_VAR_DCPT *vDcpt)
{
    const char *statMsg = lastSuccess ? "block" : "none";
    TCPIP_HTTP_NET_DynamicWriteString(vDcpt, statMsg, false);
    lastSuccess = false;
    return TCPIP_HTTP_DYN_PRINT_RES_DONE;
}

TCPIP_HTTP_DYN_PRINT_RES TCPIP_HTTP_Print_status_fail(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_DYN_VAR_DCPT *vDcpt)
{
    const char *statMsg = lastFailure ? "block" : "none";
    TCPIP_HTTP_NET_DynamicWriteString(vDcpt, statMsg, false);
    lastFailure = false;
    return TCPIP_HTTP_DYN_PRINT_RES_DONE;
}

TCPIP_HTTP_DYN_PRINT_RES TCPIP_HTTP_Print_config_hostname(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_DYN_VAR_DCPT *vDcpt)
{
    TCPIP_NET_HANDLE hNet;
    HTTP_APP_DYNVAR_BUFFER *pDynBuffer;
    const char *nbnsName;

    hNet = TCPIP_HTTP_NET_ConnectionNetHandle(connHandle);
    nbnsName = TCPIP_STACK_NetBIOSName(hNet);

    if(nbnsName == 0)
    {
        TCPIP_HTTP_NET_DynamicWriteString(vDcpt, "Failed to get a Host name", false);
    }
    else
    {
        pDynBuffer = HTTP_APP_GetDynamicBuffer();
        if(pDynBuffer == 0)
        {   // failed to get a buffer; retry
            return TCPIP_HTTP_DYN_PRINT_RES_AGAIN;
        }
        strncpy(pDynBuffer->data, nbnsName, sizeof(pDynBuffer->data) - 1);
        pDynBuffer->data[sizeof(pDynBuffer->data) - 1] = 0;
        TCPIP_HTTP_NET_DynamicWriteString(vDcpt, pDynBuffer->data, true);
    }

    return TCPIP_HTTP_DYN_PRINT_RES_DONE;
}

TCPIP_HTTP_DYN_PRINT_RES TCPIP_HTTP_Print_config_dhcpchecked(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_DYN_VAR_DCPT *vDcpt)
{

    TCPIP_NET_HANDLE hNet;

    hNet = TCPIP_HTTP_NET_ConnectionNetHandle(connHandle);

    if(TCPIP_DHCP_IsEnabled(hNet))
    {
        TCPIP_HTTP_NET_DynamicWriteString(vDcpt, "checked", false);
    }
    return TCPIP_HTTP_DYN_PRINT_RES_DONE;
}

TCPIP_HTTP_DYN_PRINT_RES TCPIP_HTTP_Print_config_ip(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_DYN_VAR_DCPT *vDcpt)
{
    IPV4_ADDR ipAddress;
    char *ipAddStr;
    HTTP_APP_DYNVAR_BUFFER *pDynBuffer = HTTP_APP_GetDynamicBuffer();
    if(pDynBuffer == 0)
    {   // failed to get a buffer; retry
        return TCPIP_HTTP_DYN_PRINT_RES_AGAIN;
    }

    ipAddStr = pDynBuffer->data;
    TCPIP_NET_HANDLE hNet = TCPIP_HTTP_NET_ConnectionNetHandle(connHandle);
    ipAddress.Val = TCPIP_STACK_NetAddress(hNet);

    TCPIP_Helper_IPAddressToString(&ipAddress, ipAddStr, HTTP_APP_DYNVAR_BUFFER_SIZE);
    TCPIP_HTTP_NET_DynamicWriteString(vDcpt, ipAddStr, true);
    return TCPIP_HTTP_DYN_PRINT_RES_DONE;
}

TCPIP_HTTP_DYN_PRINT_RES TCPIP_HTTP_Print_config_gw(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_DYN_VAR_DCPT *vDcpt)
{
    IPV4_ADDR gwAddress;
    char *ipAddStr;
    HTTP_APP_DYNVAR_BUFFER *pDynBuffer = HTTP_APP_GetDynamicBuffer();
    if(pDynBuffer == 0)
    {   // failed to get a buffer; retry
        return TCPIP_HTTP_DYN_PRINT_RES_AGAIN;
    }

    ipAddStr = pDynBuffer->data;
    TCPIP_NET_HANDLE hNet = TCPIP_HTTP_NET_ConnectionNetHandle(connHandle);
    gwAddress.Val = TCPIP_STACK_NetAddressGateway(hNet);
    TCPIP_Helper_IPAddressToString(&gwAddress, ipAddStr, HTTP_APP_DYNVAR_BUFFER_SIZE);
    TCPIP_HTTP_NET_DynamicWriteString(vDcpt, ipAddStr, true);
    return TCPIP_HTTP_DYN_PRINT_RES_DONE;
}

TCPIP_HTTP_DYN_PRINT_RES TCPIP_HTTP_Print_config_subnet(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_DYN_VAR_DCPT *vDcpt)
{
    IPV4_ADDR ipMask;
    char *ipAddStr;
    HTTP_APP_DYNVAR_BUFFER *pDynBuffer = HTTP_APP_GetDynamicBuffer();
    if(pDynBuffer == 0)
    {   // failed to get a buffer; retry
        return TCPIP_HTTP_DYN_PRINT_RES_AGAIN;
    }

    ipAddStr = pDynBuffer->data;
    TCPIP_NET_HANDLE hNet = TCPIP_HTTP_NET_ConnectionNetHandle(connHandle);
    ipMask.Val = TCPIP_STACK_NetMask(hNet);
    TCPIP_Helper_IPAddressToString(&ipMask, ipAddStr, HTTP_APP_DYNVAR_BUFFER_SIZE);
    TCPIP_HTTP_NET_DynamicWriteString(vDcpt, ipAddStr, true);
    return TCPIP_HTTP_DYN_PRINT_RES_DONE;
}

TCPIP_HTTP_DYN_PRINT_RES TCPIP_HTTP_Print_config_dns1(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_DYN_VAR_DCPT *vDcpt)
{
    IPV4_ADDR priDnsAddr;
    char *ipAddStr;
    HTTP_APP_DYNVAR_BUFFER *pDynBuffer = HTTP_APP_GetDynamicBuffer();
    if(pDynBuffer == 0)
    {   // failed to get a buffer; retry
        return TCPIP_HTTP_DYN_PRINT_RES_AGAIN;
    }

    ipAddStr = pDynBuffer->data;
    TCPIP_NET_HANDLE hNet = TCPIP_HTTP_NET_ConnectionNetHandle(connHandle);
    priDnsAddr.Val = TCPIP_STACK_NetAddressDnsPrimary(hNet);
    TCPIP_Helper_IPAddressToString(&priDnsAddr, ipAddStr, HTTP_APP_DYNVAR_BUFFER_SIZE);
    TCPIP_HTTP_NET_DynamicWriteString(vDcpt, ipAddStr, true);
    return TCPIP_HTTP_DYN_PRINT_RES_DONE;
}

TCPIP_HTTP_DYN_PRINT_RES TCPIP_HTTP_Print_config_dns2(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_DYN_VAR_DCPT *vDcpt)
{
    IPV4_ADDR secondDnsAddr;
    char *ipAddStr;
    HTTP_APP_DYNVAR_BUFFER *pDynBuffer = HTTP_APP_GetDynamicBuffer();
    if(pDynBuffer == 0)
    {   // failed to get a buffer; retry
        return TCPIP_HTTP_DYN_PRINT_RES_AGAIN;
    }

    ipAddStr = pDynBuffer->data;

    TCPIP_NET_HANDLE hNet = TCPIP_HTTP_NET_ConnectionNetHandle(connHandle);
    secondDnsAddr.Val = TCPIP_STACK_NetAddressDnsSecond(hNet);
    TCPIP_Helper_IPAddressToString(&secondDnsAddr, ipAddStr, HTTP_APP_DYNVAR_BUFFER_SIZE);
    TCPIP_HTTP_NET_DynamicWriteString(vDcpt, ipAddStr, true);
    return TCPIP_HTTP_DYN_PRINT_RES_DONE;
}

TCPIP_HTTP_DYN_PRINT_RES TCPIP_HTTP_Print_config_mac(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_DYN_VAR_DCPT *vDcpt)
{
    TCPIP_NET_HANDLE hNet;
    const TCPIP_MAC_ADDR *pMacAdd;
    char macAddStr[20];
    HTTP_APP_DYNVAR_BUFFER *pDynBuffer;

    hNet = TCPIP_HTTP_NET_ConnectionNetHandle(connHandle);
    pMacAdd = (const TCPIP_MAC_ADDR*)TCPIP_STACK_NetAddressMac(hNet);
    if(pMacAdd && sizeof(pDynBuffer->data) > sizeof(macAddStr))
    {
        TCPIP_Helper_MACAddressToString(pMacAdd, macAddStr, sizeof(macAddStr));
        pDynBuffer = HTTP_APP_GetDynamicBuffer();
        if(pDynBuffer == 0)
        {   // failed to get a buffer; retry
            return TCPIP_HTTP_DYN_PRINT_RES_AGAIN;
        }
        strncpy(pDynBuffer->data, macAddStr, sizeof(macAddStr) - 1);
        pDynBuffer->data[sizeof(macAddStr) - 1] = 0;
        TCPIP_HTTP_NET_DynamicWriteString(vDcpt, pDynBuffer->data, true);
    }
    else
    {
        TCPIP_HTTP_NET_DynamicWriteString(vDcpt, "Failed to get a MAC address", false);
    }

    return TCPIP_HTTP_DYN_PRINT_RES_DONE;
}

TCPIP_HTTP_DYN_PRINT_RES TCPIP_HTTP_Print_ddns_user(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_DYN_VAR_DCPT *vDcpt)
{
#if defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT)
    if(DDNSClient.ROMPointers.Username || !DDNSClient.Username.szRAM)
    {
        return TCPIP_HTTP_DYN_PRINT_RES_DONE;
    }

    HTTP_APP_DYNVAR_BUFFER *pDynBuffer = HTTP_APP_GetDynamicBuffer();
    if(pDynBuffer == 0)
    {   // failed to get a buffer; retry
        return TCPIP_HTTP_DYN_PRINT_RES_AGAIN;
    }
    strncpy(pDynBuffer->data, (char *)DDNSClient.Username.szRAM, HTTP_APP_DYNVAR_BUFFER_SIZE);
    TCPIP_HTTP_NET_DynamicWriteString(connHandle, pDynBuffer->data, true);
#endif
    return TCPIP_HTTP_DYN_PRINT_RES_DONE;
}

TCPIP_HTTP_DYN_PRINT_RES TCPIP_HTTP_Print_ddns_pass(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_DYN_VAR_DCPT *vDcpt)
{
#if defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT)
    if(DDNSClient.ROMPointers.Password || !DDNSClient.Password.szRAM)
    {
        return TCPIP_HTTP_DYN_PRINT_RES_DONE;
    }

    HTTP_APP_DYNVAR_BUFFER *pDynBuffer = HTTP_APP_GetDynamicBuffer();
    if(pDynBuffer == 0)
    {   // failed to get a buffer; retry
        return TCPIP_HTTP_DYN_PRINT_RES_AGAIN;
    }
    strncpy(pDynBuffer->data, (char *)DDNSClient.Password.szRAM, HTTP_APP_DYNVAR_BUFFER_SIZE);
    TCPIP_HTTP_NET_DynamicWriteString(connHandle, pDynBuffer->data, true);
#endif
    return TCPIP_HTTP_DYN_PRINT_RES_DONE;
}

TCPIP_HTTP_DYN_PRINT_RES TCPIP_HTTP_Print_ddns_host(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_DYN_VAR_DCPT *vDcpt)
{
#if defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT)
    if(DDNSClient.ROMPointers.Host || !DDNSClient.Host.szRAM)
    {
        return TCPIP_HTTP_DYN_PRINT_RES_DONE;
    }

    HTTP_APP_DYNVAR_BUFFER *pDynBuffer = HTTP_APP_GetDynamicBuffer();
    if(pDynBuffer == 0)
    {   // failed to get a buffer; retry
        return TCPIP_HTTP_DYN_PRINT_RES_AGAIN;
    }
    strncpy(pDynBuffer->data, (char *)DDNSClient.Host.szRAM, HTTP_APP_DYNVAR_BUFFER_SIZE);
    TCPIP_HTTP_NET_DynamicWriteString(connHandle, pDynBuffer->data, true);
#endif
    return TCPIP_HTTP_DYN_PRINT_RES_DONE;
}

TCPIP_HTTP_DYN_PRINT_RES TCPIP_HTTP_Print_ddns_service(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_DYN_VAR_DCPT *vDcpt)
{
#if defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT)
    if(DDNSClient.ROMPointers.UpdateServer && DDNSClient.UpdateServer.szROM)
    {
        if(vDcpt->nArgs != 0 && vDcpt->dynArgs->argType == TCPIP_HTTP_DYN_ARG_TYPE_INT32)
        {
            uint16_t nHost = vDcpt->dynArgs->argInt32;

            if((const char *)DDNSClient.UpdateServer.szROM == ddnsServiceHosts[nHost])
            {
                TCPIP_HTTP_NET_DynamicWriteString(vDcpt, "selected", false);
            }
        }
    }
#endif
    return TCPIP_HTTP_DYN_PRINT_RES_DONE;
}

TCPIP_HTTP_DYN_PRINT_RES TCPIP_HTTP_Print_ddns_status(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_DYN_VAR_DCPT *vDcpt)
{
    const char *ddnsMsg;

#if defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT)
    DDNS_STATUS s;

    s = TCPIP_DDNS_LastStatusGet();
    if(s == DDNS_STATUS_GOOD || s == DDNS_STATUS_UNCHANGED || s == DDNS_STATUS_NOCHG)
    {
        ddnsMsg = "ok";
    }
    else if(s == DDNS_STATUS_UNKNOWN)
    {
        ddnsMsg = "unk";
    }
    else
    {
        ddnsMsg = "fail";
    }
#else
    ddnsMsg = "fail";
#endif

    TCPIP_HTTP_NET_DynamicWriteString(vDcpt, ddnsMsg, false);
    return TCPIP_HTTP_DYN_PRINT_RES_DONE;
}

TCPIP_HTTP_DYN_PRINT_RES TCPIP_HTTP_Print_ddns_status_msg(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_DYN_VAR_DCPT *vDcpt)
{
    const char *ddnsMsg;

#if defined(TCPIP_STACK_USE_DYNAMICDNS_CLIENT)
    switch(TCPIP_DDNS_LastStatusGet())
    {
        case DDNS_STATUS_GOOD:
        case DDNS_STATUS_NOCHG:
            ddnsMsg = "The last update was successful.";
            break;

        case DDNS_STATUS_UNCHANGED:
            ddnsMsg = "The IP has not changed since the last update.";
            break;

        case DDNS_STATUS_UPDATE_ERROR:
        case DDNS_STATUS_CHECKIP_ERROR:
            ddnsMsg = "Could not communicate with DDNS server.";
            break;

        case DDNS_STATUS_INVALID:
            ddnsMsg = "The current configuration is not valid.";
            break;

        case DDNS_STATUS_UNKNOWN:
            ddnsMsg = "The Dynamic DNS client is pending an update.";
            break;

        default:
            ddnsMsg = "An error occurred during the update.<br />The DDNS Client is suspended.";
            break;
    }
#else
    ddnsMsg = "The Dynamic DNS Client is not enabled.";
#endif

    TCPIP_HTTP_NET_DynamicWriteString(vDcpt, ddnsMsg, false);
    return TCPIP_HTTP_DYN_PRINT_RES_DONE;
}

TCPIP_HTTP_DYN_PRINT_RES TCPIP_HTTP_Print_rebootaddr(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_DYN_VAR_DCPT *vDcpt)
{   // This is the expected address of the board upon rebooting
    const char *rebootAddr = (const char *)TCPIP_HTTP_NET_ConnectionDataBufferGet(connHandle);

    HTTP_APP_DYNVAR_BUFFER *pDynBuffer = HTTP_APP_GetDynamicBuffer();
    if(pDynBuffer == 0)
    {   // failed to get a buffer; retry
        return TCPIP_HTTP_DYN_PRINT_RES_AGAIN;
    }
    strncpy(pDynBuffer->data, rebootAddr, sizeof(pDynBuffer->data) - 1);
    pDynBuffer->data[sizeof(pDynBuffer->data) - 1] = 0;
    TCPIP_HTTP_NET_DynamicWriteString(vDcpt, pDynBuffer->data, true);
    return TCPIP_HTTP_DYN_PRINT_RES_DONE;
}

TCPIP_HTTP_DYN_PRINT_RES TCPIP_HTTP_Print_snmp_en(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_DYN_VAR_DCPT *vDcpt)
{
#if defined(TCPIP_STACK_USE_SNMP_SERVER)
    const char *snmpMsg = "none";
#else
    const char *snmpMsg = "block";
#endif
    TCPIP_HTTP_NET_DynamicWriteString(vDcpt, snmpMsg, false);
    return TCPIP_HTTP_DYN_PRINT_RES_DONE;
}

// SNMP Read communities configuration page
TCPIP_HTTP_DYN_PRINT_RES TCPIP_HTTP_Print_read_comm(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_DYN_VAR_DCPT *vDcpt)
{
#if defined(TCPIP_STACK_USE_SNMP_SERVER)
    while(vDcpt->nArgs != 0 && vDcpt->dynArgs->argType == TCPIP_HTTP_DYN_ARG_TYPE_INT32)
    {
        uint8_t *dest;
        HTTP_APP_DYNVAR_BUFFER *pDynBuffer;
        uint16_t num = vDcpt->dynArgs->argInt32;

        // Ensure no one tries to read illegal memory addresses by specifying
        // illegal num values.
        if(num >= TCPIP_SNMP_MAX_COMMUNITY_SUPPORT)
        {
            break;
        }

        if(HTTP_APP_DYNVAR_BUFFER_SIZE < TCPIP_SNMP_COMMUNITY_MAX_LEN + 1)
        {
            TCPIP_HTTP_NET_DynamicWriteString(vDcpt, "<b>Not enough room to output SNMP info!</b>", false);
            break;
        }

        pDynBuffer = HTTP_APP_GetDynamicBuffer();
        if(pDynBuffer == 0)
        {   // failed to get a buffer; retry
            return TCPIP_HTTP_DYN_PRINT_RES_AGAIN;
        }

        dest = (uint8_t *)pDynBuffer->data;
        memset(dest, 0, TCPIP_SNMP_COMMUNITY_MAX_LEN + 1);
        if(TCPIP_SNMP_ReadCommunityGet(num, TCPIP_SNMP_COMMUNITY_MAX_LEN, dest) != true)
        {   // failed; release the buffer
            pDynBuffer->busy = 0;
            break;
        }

        // Send proper string
        TCPIP_HTTP_NET_DynamicWriteString(vDcpt, (const char *)dest, true);

        break;
    }
#endif
    return TCPIP_HTTP_DYN_PRINT_RES_DONE;
}

// SNMP Write communities configuration page
TCPIP_HTTP_DYN_PRINT_RES TCPIP_HTTP_Print_write_comm(TCPIP_HTTP_NET_CONN_HANDLE connHandle, const TCPIP_HTTP_DYN_VAR_DCPT *vDcpt)
{
#if defined(TCPIP_STACK_USE_SNMP_SERVER)
    while(vDcpt->nArgs != 0 && vDcpt->dynArgs->argType == TCPIP_HTTP_DYN_ARG_TYPE_INT32)
    {
        uint8_t *dest;
        HTTP_APP_DYNVAR_BUFFER *pDynBuffer;
        uint16_t num = vDcpt->dynArgs->argInt32;

        // Ensure no one tries to read illegal memory addresses by specifying
        // illegal num values.
        if(num >= TCPIP_SNMP_MAX_COMMUNITY_SUPPORT)
        {
            break;
        }

        if(HTTP_APP_DYNVAR_BUFFER_SIZE < TCPIP_SNMP_COMMUNITY_MAX_LEN + 1)
        {
            TCPIP_HTTP_NET_DynamicWriteString(vDcpt, "<b>Not enough room to output SNMP info!</b>", false);
            break;
        }

        pDynBuffer = HTTP_APP_GetDynamicBuffer();
        if(pDynBuffer == 0)
        {   // failed to get a buffer; retry
            return TCPIP_HTTP_DYN_PRINT_RES_AGAIN;
        }

        dest = (uint8_t *)pDynBuffer->data;
        memset(dest, 0, TCPIP_SNMP_COMMUNITY_MAX_LEN + 1);
        if(TCPIP_SNMP_WriteCommunityGet(num, TCPIP_SNMP_COMMUNITY_MAX_LEN, dest) != true)
        {   // failed; release the buffer
            pDynBuffer->busy = 0;
            break;
        }

        // Send proper string
        TCPIP_HTTP_NET_DynamicWriteString(vDcpt, (const char *)dest, true);

        break;
    }
#endif
    return TCPIP_HTTP_DYN_PRINT_RES_DONE;
}

#endif // #if defined(TCPIP_STACK_USE_HTTP_SERVER)