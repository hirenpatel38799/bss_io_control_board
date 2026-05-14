/*******************************************************************************
  Command Processor System Service Implementation

  Company:
    Microchip Technology Inc.

  File Name:
    sys_command.c

  Summary:
    Command Processor System Service Implementation.

  Description:
    This file contains the source code for the Command Processor System
    Service.  It provides a way to interact with the Command Processor subsystem
    to manage the ASCII command requests from the user supported by the system.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2018 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/
//DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include "configuration.h"
#include "system/command/sys_command.h"
#include "system/console/sys_console.h"
#include "system/debug/sys_debug.h"
#include "system/reset/sys_reset.h"
#include "osal/osal.h"
#include "definitions.h"  

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
#define SERVER0_PORT 8080
#define SERVER1_PORT 8081
#define SERVER2_PORT 8082
#define SERVER3_PORT 8083
#define SERVER4_PORT 8084
#define SERVER5_PORT 8085
#define SERVER6_PORT 8086
#define SERVER7_PORT 8087

typedef struct tagHistCmdNode
{
    struct tagHistCmdNode* next;
    struct tagHistCmdNode* prev;
    char    cmdBuff[SYS_CMD_MAX_LENGTH + 1];  // command itself
}histCmdNode;   // simple command history

typedef struct
{
    histCmdNode*    head;
    histCmdNode*    tail;
}histCmdList;     // doubly linked history command list

// standard VT100 key escape sequences
#define                 VT100_MAX_ESC_SEQ_SIZE    4               // max VT100 escape sequence size that is processed

// currently supported sequences received from the terminal:
// up arrow:    ESC [ A
// down arrow:  ESC [ B
// right arrow: ESC [ C
// left arrow:  ESC [ D
//
// VT100 control commands sent to the terminal:
// erase to the end of the line:    ESC [ K
// move cursor backwards:           ESC [ {COUNT} D
// move cursor forward:             ESC [ {COUNT} C
//


#define         LINE_TERM       "\r\n"          // line terminator
#define         promptStr      ">"             // prompt string

// descriptor of the command I/O node
typedef struct SYS_CMD_IO_DCPT
{
    SYS_CMD_DEVICE_NODE devNode;
    // internally maintained data
    struct SYS_CMD_IO_DCPT* next;   // linked list node
    const struct KEY_SEQ_DCPT_T* pSeqDcpt; // current escape sequence in progress
    int16_t         seqChars;   // # of characters from the escape sequence
    char            seqBuff[VT100_MAX_ESC_SEQ_SIZE + 2];     // 0x1b + escape sequence + \0
    char*           cmdPnt; // current pointer
    char*           cmdEnd; // command end
    char            cmdBuff[SYS_CMD_MAX_LENGTH + 1];   // buffer holding the command
    char            ctrlBuff[SYS_CMD_MAX_LENGTH + 10]; // buffer for terminal control
    // history
    histCmdList     histList;                           // arranged as list
    histCmdNode*    currHistN;      // current history node
    histCmdNode     histArray[COMMAND_HISTORY_DEPTH];   // array of history commands
} SYS_CMD_IO_DCPT;

// Defines the list structure to store a list of command instances.
typedef struct
{
    SYS_CMD_IO_DCPT* head;
    SYS_CMD_IO_DCPT* tail;

} SYS_CMD_DEVICE_LIST;


// Defines the command table structure for the Command Processor System Service.
typedef struct
{
    int                         nCmds;          // number of commands available in the table
    const SYS_CMD_DESCRIPTOR*   pCmd;           // pointer to an array of command descriptors
    const char*                 cmdGroupName;   // name identifying the commands
    const char*                 cmdMenuStr;     // help string
    SYS_CMD_Callback            usrCallback;    // user callback if any
    void*                       usrParam;       // user param
} SYS_CMD_DESCRIPTOR_TABLE;                 // table containing the supported commands
uint8_t TCA9539_ReadRegister(uint8_t reg);
// *****************************************************************************
// *****************************************************************************
// Section: Global Variable Definitions
// *****************************************************************************
// *****************************************************************************

static SYS_CMD_DEVICE_LIST cmdIODevList = {0, 0};

static char printBuff[SYS_CMD_PRINT_BUFFER_SIZE] SYS_CMD_BUFFER_DMA_READY;
static int printBuffPtr = 0;

static SYS_CMD_INIT cmdInitData;       // data the command processor has been initialized with

static SYS_CMD_DESCRIPTOR_TABLE   usrCmdTbl[MAX_CMD_GROUP] = { {0} };    // current command table

static int stopRequested = 0;       // request to stop the command processor 

// function processing the VT100 escape sequence
typedef void (*keySeqProcess)(SYS_CMD_IO_DCPT* pCmdIO, const struct KEY_SEQ_DCPT_T* pSeqDcpt);

typedef struct KEY_SEQ_DCPT_T
{
    const char*     keyCode;    // pointer to the key code sequence
    keySeqProcess  keyFnc;     // key processing functions
    int             keySize;    // # of characters in the sequence
}KEY_SEQ_DCPT;


static void lkeyUpProcess(SYS_CMD_IO_DCPT* pCmdIO, const KEY_SEQ_DCPT* pSeqDcpt);
static void lkeyDownProcess(SYS_CMD_IO_DCPT* pCmdIO, const KEY_SEQ_DCPT* pSeqDcpt);
static void lkeyRightProcess(SYS_CMD_IO_DCPT* pCmdIO, const KEY_SEQ_DCPT* pSeqDcpt);
static void lkeyLeftProcess(SYS_CMD_IO_DCPT* pCmdIO, const KEY_SEQ_DCPT* pSeqDcpt);
static void lkeyHomeProcess(SYS_CMD_IO_DCPT* pCmdIO, const KEY_SEQ_DCPT* pSeqDcpt);
static void lkeyEndProcess(SYS_CMD_IO_DCPT* pCmdIO, const KEY_SEQ_DCPT* pSeqDcpt);


/* MISRA C-2012 Rule 4.1, 17.1 and 21.6 deviated below. Deviation record ID -
   H3_MISRAC_2012_R_4_1_DR_1, H3_MISRAC_2012_R_17_1_DR_1 and H3_MISRAC_2012_R_21_6_DR_1 */
// dummy table holding the escape sequences + expected sequence size
// detection of a sequence is done using only the first 3 characters
#define         VT100_DETECT_SEQ_SIZE    3
static const KEY_SEQ_DCPT keySeqTbl[] =
{
    // keyCode      keyFnc              keySize
    {"\x1b[A",      lkeyUpProcess,      (int32_t)sizeof("\x1b[A") - 1},
    {"\x1b[B",      lkeyDownProcess,    (int32_t)sizeof("\x1b[B") - 1},
    {"\x1b[C",      lkeyRightProcess,   (int32_t)sizeof("\x1b[C") - 1},
    {"\x1b[D",      lkeyLeftProcess,    (int32_t)sizeof("\x1b[D") - 1},
    {"\x1b[1~",     lkeyHomeProcess,    (int32_t)sizeof("\x1b[1~") - 1},
    {"\x1b[4~",     lkeyEndProcess,     (int32_t)sizeof("\x1b[4~") - 1},
};



// prototypes

static void     CommandReset(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void     CommandQuit(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);              // command quit
static void     CommandHelp(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);              // help
static void     CommandToggleOutput(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void     CommandToggleSpecificOutput(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);

static void     CommandToggleRelayOutput(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void     CommandToggleSpecificRelay(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void     CommandToggleSpecificEfuse(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);

static void     CommandReadDigitalInput(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void     CommandReadAnalogInput(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void     CommandToggleLED(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void     CommandEmergencyShutdown(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
//static void     CommandDateTime(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void     CommandCANTx(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void     CommandBootModeTest(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void     CommandTCPIPTest(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void     CommandRS485Test(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void     CommandFullLoadTest(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static void     CommandIdealLoadTest(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);

static int      StringToArgs(char *str, char *argv[], size_t argvSize);
static void     ParseCmdBuffer(SYS_CMD_IO_DCPT* pCmdIO);      // parse the command buffer

static void     DisplayNodeMsg(SYS_CMD_IO_DCPT* pCmdIO, histCmdNode* pNext);

static void     CmdAddHead(histCmdList* pL, histCmdNode* pN);
static histCmdNode* CmdRemoveTail(histCmdList* pL);

static void     CmdAdjustPointers(SYS_CMD_IO_DCPT* pCmdIO);

static void SendCommandMessage(const void* cmdIoParam, const char* message);
static void SendCommandPrint(const void* cmdIoParam, const char* format, ...);
static void SendCommandCharacter(const void* cmdIoParam, char c);
static int IsCommandReady(const void* cmdIoParam);
static char GetCommandCharacter(const void* cmdIoParam);
static void RunCmdTask(SYS_CMD_IO_DCPT* pCmdIO);

static const SYS_CMD_API sysConsoleApi =
{
    .msg = SendCommandMessage,
    .print = SendCommandPrint,
    .putc_t = SendCommandCharacter,
    .isRdy = IsCommandReady,
    .getc_t = GetCommandCharacter,
};

// built-in command table
static const SYS_CMD_DESCRIPTOR    builtinCmdTbl[]=
{
    {"reset",   CommandReset,              ": Reset the Host System"},
    {"q",       CommandQuit,               ": Quit Command Processor"},
    {"0",       CommandHelp,               ": Show Help (List of Commands)"},
    {"1",       CommandToggleOutput,       ": Toggle All Digital Outputs"},
    {"2",       CommandToggleRelayOutput,  ": Toggle All Relay and E-Fuse Outputs"},
    {"3",       CommandReadDigitalInput,   ": Read Status of All Digital Inputs"},
    {"4",       CommandReadAnalogInput,    ": Read Real-Time Values of All Analog Inputs"},
    {"5",       CommandToggleLED,          ": Toggle Status and Error LEDs"},
    {"6",       CommandEmergencyShutdown,  ": Trigger Emergency Shutdown"},
    {"R",       CommandToggleSpecificRelay,": Toggle Specific Relay Output (Usage: R <1-2>)"},
    {"E",       CommandToggleSpecificEfuse,": Toggle Specific E-Fuse Output (Usage: E <1-4>)"},
    {"O",       CommandToggleSpecificOutput,": Toggle Specific Digital Output (Usage: O <1-24>)"},
    {"C",       CommandCANTx,              ": Send CAN Data Frame (Usage: C <0-5>)"},
    {"B",       CommandBootModeTest,       ": Enter Boot Mode for Firmware Update"},
    {"T",       CommandTCPIPTest,          ": Perform TCP/IP Communication Test"},
    {"U",       CommandRS485Test,          ": Send Data via RS485 Communication"},
    {"F",       CommandFullLoadTest,       ": Run Full Load Test (Usage: F <1-10> Minutes)"},
    {"I",       CommandIdealLoadTest,      ": Run Ideal Load Test (No Load Condition)"},   
};

// *****************************************************************************
// *****************************************************************************
// Section: SYS CMD Operation Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* MISRA C-2012 Rule 11.3 deviated : 2, 11.8 deviated :2. Deviation record ID -
   H3_MISRAC_2012_R_11_3_DR_1 & H3_MISRAC_2012_R_11_8_DR_1*/
/* Function:
    bool SYS_CMD_Initialize( const SYS_MODULE_INIT * const init  )

  Summary:
    Initializes data for the instance of the Command Processor module.

  Description:
    This function initializes the Command Processor module.
    It also initializes any internal data structures.

  Precondition:
    None.

  Parameters:
    init            - Pointer to a data structure containing any data necessary
                      to initialize the sys command. This pointer may be null if no
                      data is required because static overrides have been
                      provided.

  Returns:
    If successful, returns true.
    If there is an error, returns false.

  Remarks:
    This routine should only be called once during system initialization.
*/
bool SYS_CMD_Initialize(const SYS_MODULE_INIT * const init )
{
    SYS_CMD_INIT *initConfig = (SYS_CMD_INIT*)init;

    if (initConfig == NULL)
    {
        return false;
    }

    cmdInitData = *initConfig; // save a copy of the initialization data


    cmdIODevList.head = NULL;
    cmdIODevList.tail = NULL;

    cmdInitData.consoleIndex = initConfig->consoleIndex;

    stopRequested = 0;

    return true;
}


bool SYS_CMD_READY_TO_READ(void)
{
    return true;
}

// add new command group
bool  SYS_CMD_ADDGRP(const SYS_CMD_DESCRIPTOR* pCmdTbl, int nCmds, const char* groupName, const char* menuStr)
{
    int i, groupIx = -1, emptyIx = -1;
    int insertIx;
    SYS_CMD_Callback usrCallback = NULL;
    void* usrParam = NULL;

    // Check if there is space for new command group; If this table already added, also simply update.
    for (i = 0; i < MAX_CMD_GROUP; i++)
    {
        if(usrCmdTbl[i].pCmd == NULL)
        {   // empty slot
            emptyIx = i;
        }
        else if(usrCmdTbl[i].pCmd == pCmdTbl)
        {   // already have this group; sanity check against the group name
            if(strcmp(groupName, usrCmdTbl[i].cmdGroupName) != 0)
            {   // name mismatch
                return false;
            }

            groupIx = i;
            usrCallback = usrCmdTbl[i].usrCallback;
            usrParam = usrCmdTbl[i].usrParam;
            break;
        }
        else
        {
            /* Do Nothing */
        }
    }

    // reference the command group
    if (groupIx != -1)
    {
        insertIx = groupIx;
    }
    else if(emptyIx != -1)
    {
        insertIx = emptyIx;
    }
    else
    {
        return false;
    }

    usrCmdTbl[insertIx].pCmd = pCmdTbl;
    usrCmdTbl[insertIx].nCmds = nCmds;
    usrCmdTbl[insertIx].cmdGroupName = groupName;
    usrCmdTbl[insertIx].cmdMenuStr = menuStr;
    usrCmdTbl[insertIx].usrCallback = usrCallback;
    usrCmdTbl[insertIx].usrParam = usrParam;
    return true;

}

// Maintains the Command Processor System Service's internal state machine.
bool SYS_CMD_Tasks(void)
{
    SYS_CMD_IO_DCPT* pCmdIO;
    static bool error_reported = false;

    if(stopRequested != 0)
    {
        return true;
    }

    if (cmdIODevList.head == NULL)
    {
        if(SYS_CMDIO_ADD(&sysConsoleApi, &cmdInitData.consoleCmdIOParam, (int32_t)cmdInitData.consoleCmdIOParam) == NULL)
        {
            if(error_reported == false)
            {
                SYS_ERROR_PRINT(SYS_ERROR_WARNING, "Failed to create the Console API\r\n");
                error_reported = true;
            }
        }
    }

    for(pCmdIO = cmdIODevList.head; pCmdIO != NULL; pCmdIO = pCmdIO->next)
    {
        RunCmdTask(pCmdIO);
    }

    return true;
}

static void RunCmdTask(SYS_CMD_IO_DCPT* pCmdIO)
{
    char newCh;
    uint32_t ix;
    int len;
    const KEY_SEQ_DCPT *pKeyDcpt, *pFoundSeq;
    const SYS_CMD_API* pCmdApi = pCmdIO->devNode.pCmdApi;
    const void* cmdIoParam = pCmdIO->devNode.cmdIoParam;


    // Check if there's characters available
    if((*pCmdApi->isRdy)(cmdIoParam) == 0)
    {
        return;
    }

    // read the character
    newCh = (*pCmdApi->getc_t)(cmdIoParam); /* Read data from console. */

    if(pCmdIO->seqChars != 0)
    {   // in the middle of escape sequence
        pCmdIO->seqBuff[pCmdIO->seqChars] = newCh;
        pCmdIO->seqChars++;

        if(pCmdIO->seqChars == VT100_DETECT_SEQ_SIZE)
        {   // detect the exact escape sequence
            pCmdIO->seqBuff[pCmdIO->seqChars] = '\0';
            pKeyDcpt = keySeqTbl;
            pFoundSeq = NULL;

            ix = 0;
            while(ix < (sizeof(keySeqTbl) / sizeof(*keySeqTbl)))
            {
                if(strncmp(pCmdIO->seqBuff, pKeyDcpt->keyCode, VT100_DETECT_SEQ_SIZE) == 0)
                {   // found it
                    pFoundSeq = pKeyDcpt;
                    break;
                }
                ix++;
                pKeyDcpt++;
            }

            if(pFoundSeq == NULL)
            {   // unknown escape sequence
                pCmdIO->seqChars = 0;
                return;
            }

            pCmdIO->pSeqDcpt = pFoundSeq;
        }

        if((pCmdIO->seqChars >= VT100_DETECT_SEQ_SIZE) && (pCmdIO->seqChars == pCmdIO->pSeqDcpt->keySize))
        {   // check for complete sequence
            if(strcmp(pCmdIO->seqBuff, pCmdIO->pSeqDcpt->keyCode) == 0)
            {   // process sequence
                (*pCmdIO->pSeqDcpt->keyFnc)(pCmdIO, pCmdIO->pSeqDcpt);
            }

            // else unknown sequence ?
            pCmdIO->seqChars = 0;
            return;
        }

        return;

    }
    else if((newCh == '\r') || (newCh == '\n'))
    {   // new command assembled
        if(pCmdIO->cmdEnd ==  pCmdIO->cmdBuff)
        {   // just an extra \n or \r
            (*pCmdApi->msg)(cmdIoParam, LINE_TERM promptStr);
            return;
        }
        (*pCmdApi->msg)(cmdIoParam, LINE_TERM);
        *pCmdIO->cmdEnd = '\0';
        pCmdIO->cmdPnt = pCmdIO->cmdBuff;
        pCmdIO->cmdEnd = pCmdIO->cmdBuff;

        ParseCmdBuffer(pCmdIO);
        (*pCmdApi->msg)(cmdIoParam, promptStr);
    }
    else if(newCh == '\b')
    {
        if(pCmdIO->cmdPnt > pCmdIO->cmdBuff)
        {
            if(pCmdIO->cmdEnd > pCmdIO->cmdPnt)
            {
                char* pSrc = pCmdIO->cmdPnt; // current
                char* pDst = pCmdIO->cmdPnt - 1;
                len = pCmdIO->cmdEnd - pSrc;
                for(ix = 0; ix < (uint32_t)len; ix++)
                {
                    *pDst = *pSrc;
                    pDst++;
                    pSrc++;
                }
                pCmdIO->cmdPnt--; pCmdIO->cmdEnd--;
                // update the display; erase to the end of line(<ESC>[K) and move cursor backwards (<ESC>[{COUNT}D)
                *pCmdIO->cmdEnd = '\0';
                (void) sprintf(pCmdIO->ctrlBuff, "\b\x1b[K%s\x1b[%dD", pCmdIO->cmdPnt, len);
                (*pCmdApi->msg)(cmdIoParam, pCmdIO->ctrlBuff);
            }
            else
            {   // delete char under cursor
                (*pCmdApi->msg)(cmdIoParam, "\b\x1b[K");
                pCmdIO->cmdPnt--; pCmdIO->cmdEnd--;
            }
        }
    }
    else if((int32_t)newCh == 0x7f)
    {   // delete
        if(pCmdIO->cmdEnd > pCmdIO->cmdPnt)
        {
            char* pSrc = pCmdIO->cmdPnt + 1;
            char* pDst = pCmdIO->cmdPnt;
            len = pCmdIO->cmdEnd - pSrc;
            for(ix = 0; ix < (uint32_t)len; ix++)
            {
                *pDst = *pSrc;
                pDst++;
                pSrc++;
            }
            pCmdIO->cmdEnd--;
            // update the display; erase to the end of line(<ESC>[K) and move cursor backwards (<ESC>[{COUNT}D)
            *pCmdIO->cmdEnd = '\0';
            (void) sprintf(pCmdIO->ctrlBuff, "\x1b[K%s\x1b[%dD", pCmdIO->cmdPnt, len);
            (*pCmdApi->msg)(cmdIoParam, pCmdIO->ctrlBuff);
        }
    }
    else if((int32_t)newCh == 0x1b)
    {   // start escape sequence... wait for complete sequence
        pCmdIO->seqBuff[0] = newCh;
        pCmdIO->seqChars = 1;
    }
    else if((pCmdIO->cmdEnd - pCmdIO->cmdBuff) < ((int32_t)sizeof(pCmdIO->cmdBuff) - 1))
    {   // valid char; insert and echo it back
        int n_chars = pCmdIO->cmdEnd - pCmdIO->cmdPnt;  // existent chars
        if(n_chars != 0)
        {   // move the existing chars to the right, for insertion...
            char* pSrc = pCmdIO->cmdEnd - 1;
            char* pDst = pCmdIO->cmdEnd;
            for(ix = 0; ix < (uint32_t)n_chars; ix++)
            {
                *pDst = *pSrc;
                pDst--;
                pSrc--;
            }
            pCmdIO->cmdEnd++;
            *pCmdIO->cmdEnd = '\0';
            (void) sprintf(pCmdIO->ctrlBuff + 1, "%s\x1b[%dD", pCmdIO->cmdPnt + 1, n_chars);
        }
        else
        {
            pCmdIO->ctrlBuff[1] = (char)0;
        }
        pCmdIO->ctrlBuff[0] = newCh;

        (*pCmdApi->msg)(cmdIoParam, pCmdIO->ctrlBuff);
        *pCmdIO->cmdPnt = newCh;
        pCmdIO->cmdPnt++;
        CmdAdjustPointers(pCmdIO);
    }
    else
    {
        (*pCmdApi->msg)(cmdIoParam, " *** Command Processor buffer exceeded. Retry. ***" LINE_TERM);
        pCmdIO->cmdPnt = pCmdIO->cmdBuff;
        pCmdIO->cmdEnd = pCmdIO->cmdBuff;
        (*pCmdApi->msg)(cmdIoParam, promptStr);
    }
}

// *****************************************************************************
/* Function:
    void SYS_CMD_MESSAGE (const char* message)

  Summary:
    Outputs a message to the Command Processor System Service console.

  Description:
    This function outputs a message to the Command Processor System Service
    console.
.
  Precondition:
    SYS_CMD_Initialize was successfully run once.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
*/
void SYS_CMD_MESSAGE(const char* message)
{
    SendCommandMessage(NULL, message);
}

// *****************************************************************************
/* Function:
    void SYS_CMD_PRINT(const char *format, ...)

  Summary:
    Outputs a printout to the Command Processor System Service console.

  Description:
    This function outputs a printout to the Command Processor System Service
    console.
.
  Precondition:
    SYS_CMD_Initialize was successfully run once.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    None.
*/
void SYS_CMD_PRINT(const char* format, ...)
{
    char tmpBuf[SYS_CMD_PRINT_BUFFER_SIZE];
    size_t len = 0;
    size_t padding = 0;
    va_list args ;
    va_start( args, format );

    len = (uint32_t)vsnprintf(tmpBuf, SYS_CMD_PRINT_BUFFER_SIZE, format, args);

    va_end( args );

    if ((len > 0U) && (len < SYS_CMD_PRINT_BUFFER_SIZE))
    {
        tmpBuf[len] = '\0';

        if ((len + (uint32_t)printBuffPtr) >= (SYS_CMD_PRINT_BUFFER_SIZE))
        {
            printBuffPtr = 0;
        }

        (void) strcpy(&printBuff[printBuffPtr], tmpBuf);
        SendCommandMessage(NULL, &printBuff[printBuffPtr]);

        padding = len % 4U;

        if (padding > 0U)
        {
            padding = 4U - padding;
        }

        printBuffPtr += (int32_t)len + (int32_t)padding;
    }
}

SYS_CMD_DEVICE_NODE* SYS_CMDIO_GET_HANDLE(short num)
{
    SYS_CMD_IO_DCPT* pNode = cmdIODevList.head;

     while((num != 0) && (pNode != NULL))
    {
        pNode = pNode->next;
        num--;
    }

    return ((pNode == NULL) ? (NULL) : (&pNode->devNode));
}

SYS_CMD_DEVICE_NODE* SYS_CMDIO_ADD(const SYS_CMD_API* opApi, const void* cmdIoParam, int unused)
{
    uint32_t ix;

    // Create new node
    SYS_CMD_IO_DCPT* pNewIo;

    pNewIo = (SYS_CMD_IO_DCPT*)OSAL_Malloc(sizeof(*pNewIo));
    if (pNewIo == NULL)
    {
        return NULL;
    }
    (void) memset(pNewIo, 0, sizeof(*pNewIo));
    pNewIo->devNode.pCmdApi = opApi;
    pNewIo->devNode.cmdIoParam = cmdIoParam;
    pNewIo->cmdPnt = pNewIo->cmdBuff;
    pNewIo->cmdEnd = pNewIo->cmdBuff;

    // construct the command history list
    for(ix = 0; ix < (sizeof(pNewIo->histArray) / sizeof(*pNewIo->histArray)); ix++)
    {
        CmdAddHead(&pNewIo->histList, pNewIo->histArray + ix);
    }

    // Insert node at end
    pNewIo->next = NULL;
    if(cmdIODevList.head == NULL)
    {
        cmdIODevList.head = pNewIo;
        cmdIODevList.tail = pNewIo;
    }
    else
    {
        cmdIODevList.tail->next = pNewIo;
        cmdIODevList.tail = pNewIo;
    }

    return &pNewIo->devNode;
}


bool SYS_CMD_DELETE(SYS_CMD_DEVICE_NODE* pDeviceNode)
{
    SYS_CMD_IO_DCPT* p_listnode = cmdIODevList.head;
    SYS_CMD_IO_DCPT* pre_listnode;
    SYS_CMD_IO_DCPT* pDevNode = (SYS_CMD_IO_DCPT*)pDeviceNode;

    // root list is empty or null node to be deleted
    if((p_listnode == NULL) || (pDevNode == NULL))
    {
        return false;
    }

    if(p_listnode == pDevNode)
    {   // delete the head
        //Root list has only one node
        if(cmdIODevList.tail == pDevNode)
        {
            cmdIODevList.head = NULL;
            cmdIODevList.tail = NULL;
        }
        else
        {
            cmdIODevList.head = p_listnode->next;
        }
        OSAL_Free(pDevNode);
        return true;
    }

    // delete mid node
    pre_listnode = p_listnode;
    while (p_listnode != NULL)
    {
        if(p_listnode == pDevNode)
        {
            pre_listnode->next = p_listnode->next;
            // Deleted node is tail
            if (cmdIODevList.tail==pDevNode) {
                cmdIODevList.tail = pre_listnode;
            }
            OSAL_Free(pDevNode);
            return true;
        }
        pre_listnode = p_listnode;
        p_listnode   = p_listnode->next;
    }


    return false;
}

SYS_CMD_HANDLE  SYS_CMD_CallbackRegister(const SYS_CMD_DESCRIPTOR* pCmdTbl, SYS_CMD_Callback func, void* hParam)
{
    SYS_CMD_HANDLE handle = NULL;

    if(func != NULL)
    {
        int ix;
        SYS_CMD_DESCRIPTOR_TABLE* pTbl = usrCmdTbl;

        ix = 0;
        while (ix < MAX_CMD_GROUP)
        {
            if(pTbl->pCmd == pCmdTbl)
            {   // requested group
                OSAL_CRITSECT_DATA_TYPE critSect =  OSAL_CRIT_Enter(OSAL_CRIT_TYPE_LOW);
                if(pTbl->usrCallback == NULL)
                {
                    pTbl->usrCallback = func;
                    pTbl->usrParam = hParam;
                    handle = pTbl;
                }
                OSAL_CRIT_Leave(OSAL_CRIT_TYPE_LOW, critSect);
            }
            ix++;
            pTbl++;
        }
    }

    return handle;
}

bool SYS_CMD_CallbackDeregister(SYS_CMD_HANDLE handle)
{
    bool res = false;

    SYS_CMD_DESCRIPTOR_TABLE* xTbl = (SYS_CMD_DESCRIPTOR_TABLE*)handle;

    int nIx = xTbl - usrCmdTbl;

    if( (0 <= nIx) && ((uint32_t)nIx < sizeof(usrCmdTbl) / sizeof(*usrCmdTbl)))
    {
        SYS_CMD_DESCRIPTOR_TABLE* pTbl = usrCmdTbl + nIx;

        if(pTbl == xTbl)
        {   // handle is correct
            OSAL_CRITSECT_DATA_TYPE critSect =  OSAL_CRIT_Enter(OSAL_CRIT_TYPE_LOW);
            if((pTbl->pCmd != NULL) && (pTbl->usrCallback != NULL))
            {   // in use
                pTbl->usrCallback = NULL;
                res = true;
            }
            OSAL_CRIT_Leave(OSAL_CRIT_TYPE_LOW, critSect);
        }
    }

    return res;
}

/* MISRAC 2012 deviation block end */
// ignore the console handle for now, we support a single system console
static void SendCommandMessage(const void* cmdIoParam, const char* message)
{
    (void) SYS_CONSOLE_Write(cmdInitData.consoleIndex, message, strlen(message));
}

static void SendCommandPrint(const void* cmdIoParam, const char* format, ...)
{
    char tmpBuf[SYS_CMD_PRINT_BUFFER_SIZE];
    size_t len = 0;
    size_t padding = 0;
    va_list args;
    va_start( args, format );

    len = (uint32_t)vsnprintf(tmpBuf, SYS_CMD_PRINT_BUFFER_SIZE, format, args);

    va_end( args );


    if ((len > 0U) && (len < SYS_CMD_PRINT_BUFFER_SIZE))
    {
        tmpBuf[len] = '\0';

        if ((len + (uint32_t)printBuffPtr) >= SYS_CMD_PRINT_BUFFER_SIZE)
        {
            printBuffPtr = 0;
        }

        (void) strcpy(&printBuff[printBuffPtr], tmpBuf);
        SendCommandMessage(NULL, &printBuff[printBuffPtr]);

        padding = len % 4U;

        if (padding > 0U)
        {
            padding = 4U - padding;
        }

        printBuffPtr += (int32_t)len + (int32_t)padding;
    }
}

static void SendCommandCharacter(const void* cmdIoParam, char c)
{
    if (SYS_CONSOLE_Status((SYS_MODULE_OBJ)cmdInitData.consoleIndex) == SYS_STATUS_READY)
    {
        (void) SYS_CONSOLE_Write(cmdInitData.consoleIndex, (const char*)&c, 1);
    }
}


static int IsCommandReady(const void* cmdIoParam)
{
    return (int)SYS_CONSOLE_ReadCountGet(cmdInitData.consoleIndex);
}

static char GetCommandCharacter(const void* cmdIoParam)
{
    char new_c;

    (void) SYS_CONSOLE_Read(cmdInitData.consoleIndex, &new_c, 1);

    return new_c;
}

// implementation
static void CommandReset(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    const void* cmdIoParam = pCmdIO->cmdIoParam;
    (*pCmdIO->pCmdApi->msg)(cmdIoParam, LINE_TERM " *** System Reboot ***\r\n" );

    SYS_RESET_SoftwareReset();

}

// quit
static void CommandQuit(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    SYS_CMD_IO_DCPT* pCmdIoNode;
    const void* cmdIoParam = pCmdIO->cmdIoParam;

    (*pCmdIO->pCmdApi->msg)(cmdIoParam, LINE_TERM " *** Quitting the Command Processor. Bye ***\r\n" );

    (void) memset(usrCmdTbl, 0x0, sizeof(usrCmdTbl));

    // delete all the nodes in cmdIODevList
    while((pCmdIoNode = cmdIODevList.head) != NULL)
    {
        if(cmdIODevList.head == cmdIODevList.tail)
        {
            cmdIODevList.head = cmdIODevList.tail = NULL;
        }
        else
        {
            cmdIODevList.head = cmdIODevList.head->next;
        }

        OSAL_Free(pCmdIoNode);
    }

    // no longer run the SYS_CMD_Tasks
    stopRequested = 1;
}

uint8_t counter = 0;
void ToggleOutputPins() 
{
    Digital_Output_1_Toggle();
    Digital_Output_2_Toggle();
    Digital_Output_3_Toggle();
    Digital_Output_4_Toggle();
    Digital_Output_5_Toggle();
    Digital_Output_6_Toggle();
    Digital_Output_7_Toggle();
    //this three pins are connected with IO Expander
    Digital_Output_8_Toggle();
    Digital_Output_9_Toggle();
    Digital_Output_10_Toggle();
    Digital_Output_11_Toggle();
    Digital_Output_12_Toggle();
    Digital_Output_13_Toggle();
    Digital_Output_14_Toggle();
    Digital_Output_15_Toggle();
    Digital_Output_16_Toggle();
    Digital_Output_17_Toggle();
    Digital_Output_18_Toggle();
    Digital_Output_19_Toggle();
    Digital_Output_20_Toggle();
    Digital_Output_21_Toggle();
    Digital_Output_22_Toggle();
    Digital_Output_23_Toggle();
    Digital_Output_24_Toggle();
}

static void CommandToggleOutput(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    SYS_CONSOLE_MESSAGE("Toggle Output Command Received\r\n");
    ToggleOutputPins();
}

static void CommandToggleSpecificOutput(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    if (argc < 2)
    {
        SYS_CONSOLE_MESSAGE("Usage: TO <output_number 1-24>\r\n");
        return;
    }

    int outputNum = atoi(argv[1]);

    if (outputNum < 1 || outputNum > 24)
    {
        SYS_CONSOLE_MESSAGE("Invalid Output Number. Use 1?24.\r\n");
        return;
    }

    SYS_CONSOLE_PRINT("Toggle Output %d Command Received\r\n", outputNum);

    switch (outputNum)
    {
        case 1:  Digital_Output_1_Toggle();  break;
        case 2:  Digital_Output_2_Toggle();  break;
        case 3:  Digital_Output_3_Toggle();  break;
        case 4:  Digital_Output_4_Toggle();  break;
        case 5:  Digital_Output_5_Toggle();  break;
        case 6:  Digital_Output_6_Toggle();  break;
        case 7:  Digital_Output_7_Toggle();  break;
        case 8:  Digital_Output_8_Toggle();  break;
        case 9:  Digital_Output_9_Toggle();  break;
        case 10: Digital_Output_10_Toggle(); break;
        case 11: Digital_Output_11_Toggle(); break;
        case 12: Digital_Output_12_Toggle(); break;
        case 13: Digital_Output_13_Toggle(); break;
        case 14: Digital_Output_14_Toggle(); break;
        case 15: Digital_Output_15_Toggle(); break;
        case 16: Digital_Output_16_Toggle(); break;
        case 17: Digital_Output_17_Toggle(); break;
        case 18: Digital_Output_18_Toggle(); break;
        case 19: Digital_Output_19_Toggle(); break;
        case 20: Digital_Output_20_Toggle(); break;
        case 21: Digital_Output_21_Toggle(); break;
        case 22: Digital_Output_22_Toggle(); break;
        case 23: Digital_Output_23_Toggle(); break;
        case 24: Digital_Output_24_Toggle(); break;
        default:
            SYS_CONSOLE_MESSAGE("Unexpected error.\r\n");
            break;
    }
}

void ToggleRelayOutputPins()
{
    efuse1_in_Toggle();
    efuse2_in_Toggle();
    efuse3_in_Toggle();
    efuse4_in_Toggle();
    Relay_Output_1_Toggle();
    Relay_Output_2_Toggle();
}
static void CommandToggleRelayOutput(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    SYS_CONSOLE_MESSAGE("Toggle Relay/EFuse Output Command Received\r\n");
    ToggleRelayOutputPins();
}

static void CommandToggleSpecificRelay(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    SYS_CONSOLE_MESSAGE("Relay Toggle Command Received\r\n");

    if (argc < 2)
    {
        SYS_CONSOLE_MESSAGE("Usage: T <RelayNum 1-2>\r\n");
        return;
    }

    int relayNum = atoi(argv[1]);

    if (relayNum == 1)
    {
        Relay_Output_1_Toggle();
        SYS_CONSOLE_MESSAGE("Relay 1 TOGGLED\r\n");
    }
    else if (relayNum == 2)
    {
        Relay_Output_2_Toggle();
        SYS_CONSOLE_MESSAGE("Relay 2 TOGGLED\r\n");
    }
    else
    {
        SYS_CONSOLE_MESSAGE("Invalid relay number. Use: T <1-2>\r\n");
    }
}

static void CommandToggleSpecificEfuse(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    SYS_CONSOLE_MESSAGE("EFuse Toggle Command Received\r\n");

    if (argc < 2)
    {
        SYS_CONSOLE_MESSAGE("Usage: E <EfuseNum 1-4>\r\n");
        return;
    }

    int efuseNum = atoi(argv[1]);

    switch (efuseNum)
    {
        case 1:
            efuse1_in_Toggle();
            SYS_CONSOLE_MESSAGE("EFuse 1 TOGGLED\r\n");
            break;

        case 2:
            efuse2_in_Toggle();
            SYS_CONSOLE_MESSAGE("EFuse 2 TOGGLED\r\n");
            break;

        case 3:
            efuse3_in_Toggle();
            SYS_CONSOLE_MESSAGE("EFuse 3 TOGGLED\r\n");
            break;

        case 4:
            efuse4_in_Toggle();
            SYS_CONSOLE_MESSAGE("EFuse 4 TOGGLED\r\n");
            break;

        default:
            SYS_CONSOLE_MESSAGE("Invalid EFuse number. Use: E <1-4>\r\n");
            break;
    }
}

char messageCase3[2048]; 
void ReadDigitalInputPins()
{
    uint8_t expanderData0 = TCA9539_ReadRegister(INPUT_PORT_REG);
    uint8_t expanderData1 = TCA9539_ReadRegister(INPUT_PORT_REG_2);
    
    snprintf(messageCase3,sizeof(messageCase3), 
        "Digital Input1 -%s \r\n "
        "Digital Input2 -%s \r\n "
        "Digital Input3 -%s \r\n "
        "Digital Input4 -%s \r\n "
        "Digital Input5 -%s \r\n "
        "Digital Input6 -%s \r\n "
        "Digital Input7 -%s \r\n "
        "Digital Input8 -%s \r\n "
        "Digital Input9 -%s \r\n "
        "Digital Input10 -%s \r\n "
        "Digital Input11 -%s \r\n "
        "Digital Input12 -%s \r\n "
        "Digital Input13 -%s \r\n "
        "Digital Input14 -%s \r\n "
        "Digital Input15 -%s \r\n "
        "Digital Input16 -%s \r\n "
        "Digital Input17 -%s \r\n "
        "Digital Input18 -%s \r\n "
        "Digital Input19 -%s \r\n "
        "Digital Input20 -%s \r\n "
        "Digital Input21 -%s \r\n "
        "Digital Input22 -%s \r\n "
        "Digital Input23 -%s \r\n "
        "Digital Input24 -%s \r\n "
        "Digital Input25 -%s \r\n "
        "Digital Input26 -%s \r\n "
        "Digital Input27 -%s \r\n "
        "Digital Input28 -%s \r\n "
        "Digital Input29 -%s \r\n "
        "Digital Input30 -%s \r\n "
        "Digital Input31 -%s \r\n "
        "Digital Input32 -%s \r\n "
        "Digital Input33 -%s \r\n "
        "Digital Input34 -%s \r\n "
        "Digital Input35 -%s \r\n "
        "Digital Input36 -%s \r\n "
        "Digital Input37 -%s \r\n "
        "Digital Input38 -%s \r\n "
        "Digital Input39 -%s \r\n "
        "Digital Input40 -%s \r\n "
        "Digital Input41 -%s \r\n "
        "Digital Input42 -%s \r\n "
        "Digital Input43 -%s \r\n "
        "Digital Input44 -%s \r\n "    
        "Digital Input45 -%s \r\n "
        "Digital Input46 -%s \r\n "
        "Digital Input47 -%s \r\n "
        "Digital Input48 -%s \r\n "
        "Digital Input49 -%s \r\n "
        "Digital Input50 -%s \r\n "
        "Digital Input51 -%s \r\n "
        "Digital Input52 -%s \r\n "
        "Digital Input53 -%s \r\n "
        "Digital Input54 -%s \r\n "
        "Digital Input55 -%s \r\n "
        "Digital Input56 -%s \r\n "
        "Digital Input57 -%s \r\n "
        "Digital Input58 -%s \r\n "
        "Digital Input59 -%s \r\n "
        "Digital Input60 -%s \r\n ",            
        (Digital_Input_1_Get()) ? "ON" : "OFF",
        (Digital_Input_2_Get()) ? "ON" : "OFF",
        (Digital_Input_3_Get()) ? "ON" : "OFF",
        (Digital_Input_4_Get()) ? "ON" : "OFF",
        (Digital_Input_5_Get()) ? "ON" : "OFF",
        (Digital_Input_6_Get()) ? "ON" : "OFF",
        (Digital_Input_7_Get()) ? "ON" : "OFF",
        (Digital_Input_8_Get()) ? "ON" : "OFF",
        (Digital_Input_9_Get()) ? "ON" : "OFF",
        (Digital_Input_10_Get()) ? "ON" : "OFF",
        (Digital_Input_11_Get()) ? "ON" : "OFF",
        (Digital_Input_12_Get()) ? "ON" : "OFF",
        (Digital_Input_13_Get()) ? "ON" : "OFF",
        (Digital_Input_14_Get()) ? "ON" : "OFF",
        (Digital_Input_15_Get()) ? "ON" : "OFF",
        (Digital_Input_16_Get()) ? "ON" : "OFF",
        (Digital_Input_17_Get()) ? "ON" : "OFF",
        (Digital_Input_18_Get()) ? "ON" : "OFF",
        (Digital_Input_19_Get()) ? "ON" : "OFF",
        (Digital_Input_20_Get()) ? "ON" : "OFF",
        (Digital_Input_21_Get()) ? "ON" : "OFF",
        (Digital_Input_22_Get()) ? "ON" : "OFF",
        (Digital_Input_23_Get()) ? "ON" : "OFF",
        (Digital_Input_24_Get()) ? "ON" : "OFF",
        (Digital_Input_25_Get()) ? "ON" : "OFF",
        (Digital_Input_26_Get()) ? "ON" : "OFF",
        (Digital_Input_27_Get()) ? "ON" : "OFF",
        (Digital_Input_28_Get()) ? "ON" : "OFF",
        (Digital_Input_29_Get()) ? "ON" : "OFF",
        (Digital_Input_30_Get()) ? "ON" : "OFF",
        (Digital_Input_31_Get()) ? "ON" : "OFF",
        (Digital_Input_32_Get()) ? "ON" : "OFF",
        (Digital_Input_33_Get()) ? "ON" : "OFF",
        (Digital_Input_34_Get()) ? "ON" : "OFF",
        (Digital_Input_35_Get()) ? "ON" : "OFF",
        (Digital_Input_36_Get()) ? "ON" : "OFF",
        (Digital_Input_37_Get()) ? "ON" : "OFF",
        (Digital_Input_38_Get()) ? "ON" : "OFF",
        (Digital_Input_39_Get()) ? "ON" : "OFF",
        (Digital_Input_40_Get()) ? "ON" : "OFF",
        (Digital_Input_41_Get()) ? "ON" : "OFF",
        (Digital_Input_42_Get()) ? "ON" : "OFF",
        (Digital_Input_43_Get()) ? "ON" : "OFF",
        (Digital_Input_44_Get()) ? "ON" : "OFF",
        (Digital_Input_45_Get()) ? "ON" : "OFF",
        (Digital_Input_46_Get()) ? "ON" : "OFF",
        (Digital_Input_47_Get()) ? "ON" : "OFF",
        (Digital_Input_48_Get()) ? "ON" : "OFF",
        (Digital_Input_49_Get()) ? "ON" : "OFF",
        // Expander inputs (DI 50?60)
        (expanderData0 & 0x01) ? "ON" : "OFF", // DI 50
        (expanderData0 & 0x02) ? "ON" : "OFF", // DI 51
        (expanderData0 & 0x04) ? "ON" : "OFF", // DI 52
        (expanderData0 & 0x08) ? "ON" : "OFF", // DI 53
        (expanderData0 & 0x10) ? "ON" : "OFF", // DI 54
        (expanderData0 & 0x20) ? "ON" : "OFF", // DI 55
        (expanderData0 & 0x40) ? "ON" : "OFF", // DI 56
        (expanderData0 & 0x80) ? "ON" : "OFF", // DI 57
        (expanderData1 & 0x01) ? "ON" : "OFF", // DI 58
        (expanderData1 & 0x02) ? "ON" : "OFF", // DI 59
        (expanderData1 & 0x04) ? "ON" : "OFF"  // DI 60
    );
    SERCOM0_USART_Write((uint8_t*)messageCase3, strlen(messageCase3));  
}
static void CommandReadDigitalInput(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    SYS_CONSOLE_MESSAGE("Read DI Command Received\r\n");
    ReadDigitalInputPins();
}

static void CommandReadAnalogInput(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    SYS_CONSOLE_MESSAGE("Read Analog Input Command Received\r\n");
    if (total_length > 0U)
    {
        (void)SERCOM0_USART_Write((uint8_t*)messageAnalog, total_length);
    }
}

static void CommandToggleLED(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    SYS_CONSOLE_MESSAGE("Toggle Status and Error LED Command Received\r\n");
    LED_Status_Toggle();
    LED_Error_Toggle();
}

static void CommandEmergencyShutdown(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    SYS_CONSOLE_MESSAGE("Emergency Shutdown Command Received\r\n");
    LED_Error_Set();
}

//#define YEAR_NOW(year) (year + 1900U)
//#define MONTH_NOW(month) (month + 1U)
//static void CommandDateTime(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
//{
//    SYS_CONSOLE_MESSAGE("Date and Time Command Received\r\n");
//    struct tm sys_time = {0};
//    char DateTime[100]; 
////    RTC_TimeGet(&sys_time);
//    (void) sprintf(DateTime,"%02d:%02ld:%04ld %02d:%02d:%02d", sys_time.tm_mday, 
//            MONTH_NOW((uint32_t)sys_time.tm_mon), 
//            YEAR_NOW((uint32_t)sys_time.tm_year), 
//            sys_time.tm_hour, 
//            sys_time.tm_min, 
//            sys_time.tm_sec); /*lint !e586  Standard Library Usage */ 
//    SERCOM0_USART_Write((uint8_t*)DateTime, strlen(DateTime)); 
//}
static void CommandCANTx(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    SYS_CONSOLE_MESSAGE("CAN Send Command Received\r\n");

    if (argc < 2)
    {
        SYS_CONSOLE_MESSAGE("Usage: C <CAN_NUM 0-5>\r\n");
        return;
    }

    int canNum = atoi(argv[1]);
    if (canNum < 0 || canNum > 5)
    {
        SYS_CONSOLE_MESSAGE("Invalid CAN number. Use: C <0-5>\r\n");
        return;
    }

    // Prepare data buffer
    uint8_t bytesToSend = 12;
    uint8_t CANbuffer[12] = {
        0x18, 0x00, 0xB0, (uint8_t)(0xA0 + canNum), // Different last byte per CAN
        0x01, 0x01, 0x02, 0x03,
        0x04, 0x05, 0x06, 0x07
    };

    // Send on selected CAN
    CAN_Write((uint8_t)canNum, CANbuffer, bytesToSend);
    SYS_CONSOLE_PRINT("Data sent on CAN%d\r\n", canNum);

    vTaskDelay(pdMS_TO_TICKS(100));
}

void TcpIPTest()
{   
    #define TCP_SERVER_PORT 5000
    static TCP_SOCKET tcpServerSocket = INVALID_SOCKET;    
    if (TCPIP_TCP_IsConnected(tcpServerSocket)) {
        /* Handle disconnection scenarios */
        if (!TCPIP_TCP_IsConnected(tcpServerSocket) || TCPIP_TCP_WasDisconnected(tcpServerSocket)) {
            SYS_CONSOLE_PRINT("\r\nTCP Connection Closed\r\n");
            TCPIP_TCP_Close(tcpServerSocket);
            tcpServerSocket = INVALID_SOCKET;
        }
    }

    /* Reconnection logic if socket is invalid */
    if (tcpServerSocket == INVALID_SOCKET) {
        tcpServerSocket = TCPIP_TCP_ServerOpen(IP_ADDRESS_TYPE_IPV4, TCP_SERVER_PORT, 0);

        if (tcpServerSocket == INVALID_SOCKET) {
            SYS_CONSOLE_PRINT("Failed to reopen Test socket, retrying...\r\n");
        } 
        vTaskDelay(RECONNECT_DELAY_MS);
    }    
}

static void CommandBootModeTest(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    SYS_CONSOLE_MESSAGE("Boot Mode Enter Command Received\r\n"); 
    SYS_CONSOLE_MESSAGE("\n\r####### Bootloader Triggered #######\n\r");

    SYS_CONSOLE_MESSAGE("\n\r####### Program new firmware from Bootloader #######\n\r");

    ramStart[0] = BTL_TRIGGER_PATTERN;
    ramStart[1] = BTL_TRIGGER_PATTERN;
    ramStart[2] = BTL_TRIGGER_PATTERN;
    ramStart[3] = BTL_TRIGGER_PATTERN;

    DCACHE_CLEAN_BY_ADDR(ramStart, 16);

    SYS_RESET_SoftwareReset();   
}
static void CommandTCPIPTest(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
   SYS_CONSOLE_MESSAGE("TCP/IP Test Command Received\r\n"); 
   TcpIPTest();
}

static void CommandRS485Test(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    SYS_CONSOLE_MESSAGE("Data sent to RS485_1 and RS485_2 Command Received\r\n");
    uint8_t u8UARTbuffer[10];
    // Fill the CAN buffer with data
    u8UARTbuffer[0] = 0x11;
    u8UARTbuffer[1] = 0x01;
    u8UARTbuffer[2] = 0x00;
    u8UARTbuffer[3] = 0x13;
    u8UARTbuffer[4] = 0x00; 
    u8UARTbuffer[5] = 0x25; 
    u8UARTbuffer[6] = 0x0E; 
    u8UARTbuffer[7] = 0x84;
    SYS_CONSOLE_PRINT("Sending data: ");
    for (unsigned int loop = 0; loop < 8; loop++){
        SYS_CONSOLE_PRINT("%02X ",u8UARTbuffer[loop]);
    }    
    // Transmit to RS485 UART8 using provided API
    // RS485_TransmitData_UART8(u8UARTbuffer, 8);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Transmit to RS485 UART9 directly if no specific transmit function is available
    // RS485_TransmitData_UART9(u8UARTbuffer, 8);
    vTaskDelay(pdMS_TO_TICKS(50));    
}

void Set_All_Outputs(void) {
    Digital_Output_1_Set(); Digital_Output_2_Set(); Digital_Output_3_Set(); Digital_Output_4_Set(); Digital_Output_5_Set();
    Digital_Output_6_Set(); Digital_Output_7_Set(); Digital_Output_8_Set(); Digital_Output_9_Set(); Digital_Output_10_Set();
    Digital_Output_11_Set(); Digital_Output_12_Set(); Digital_Output_13_Set(); Digital_Output_14_Set(); Digital_Output_15_Set();
    Digital_Output_16_Set(); Digital_Output_17_Set(); Digital_Output_18_Set(); Digital_Output_19_Set(); Digital_Output_20_Set();
    Digital_Output_21_Set(); Digital_Output_22_Set(); Digital_Output_23_Set(); Digital_Output_24_Set(); Relay_Output_1_Set(); Relay_Output_2_Set(); efuse1_in_Set(); efuse2_in_Set(); efuse3_in_Set(); efuse4_in_Set();
}

void Clear_All_Outputs(void) {
    Digital_Output_1_Clear(); Digital_Output_2_Clear(); Digital_Output_3_Clear(); Digital_Output_4_Clear(); Digital_Output_5_Clear();
    Digital_Output_6_Clear(); Digital_Output_7_Clear(); Digital_Output_8_Clear(); Digital_Output_9_Clear(); Digital_Output_10_Clear();
    Digital_Output_11_Clear(); Digital_Output_12_Clear(); Digital_Output_13_Clear(); Digital_Output_14_Clear(); Digital_Output_15_Clear();
    Digital_Output_16_Clear(); Digital_Output_17_Clear(); Digital_Output_18_Clear(); Digital_Output_19_Clear(); Digital_Output_20_Clear();
    Digital_Output_21_Clear(); Digital_Output_22_Clear(); Digital_Output_23_Clear(); Digital_Output_24_Clear(); Relay_Output_1_Clear(); Relay_Output_2_Clear(); efuse1_in_Clear(); efuse2_in_Clear(); efuse3_in_Clear(); efuse4_in_Clear();
}

static void CommandFullLoadTest(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    SYS_CONSOLE_MESSAGE("Full Load Test Command Received\r\n");

    uint32_t duration_minutes = 1;  // Default to 1 minute

    // Parse the duration from the second argument if provided
    if (argc > 1)
    {
        int timeVal = atoi(argv[1]);  // "5" from "F 5"

        if (timeVal >= 1 && timeVal <= 10)
        {
            duration_minutes = (uint32_t)timeVal;
            Set_All_Outputs();
            
        }
        else
        {
            SYS_CONSOLE_MESSAGE("Invalid duration. Use: F <1-10>\r\n");
            return;
        }
    }
    else
    {
        SYS_CONSOLE_MESSAGE("No time duration provided. Usage: F <1-10>\r\n");
        return;
    }

    // Calculate time limit in ticks
    TickType_t duration_ticks = pdMS_TO_TICKS(duration_minutes * 60 * 1000);
    TickType_t start_time = xTaskGetTickCount();

    while ((xTaskGetTickCount() - start_time) < duration_ticks)
    {
        SYS_CONSOLE_MESSAGE("Data sent to RS485_1 and RS485_2 Command Received at every 100msec\r\n");

        uint8_t u8UARTbuffer[10] = {0x11, 0x01, 0x00, 0x13, 0x00, 0x25, 0x0E, 0x84};
        for (unsigned int loop = 0; loop < 8; loop++)
        {
            SYS_CONSOLE_PRINT("%02X ", u8UARTbuffer[loop]);
        }
        // RS485_TransmitData_UART8(u8UARTbuffer, sizeof(u8UARTbuffer));
        vTaskDelay(pdMS_TO_TICKS(10));
        // RS485_TransmitData_UART9(u8UARTbuffer, sizeof(u8UARTbuffer));
        vTaskDelay(pdMS_TO_TICKS(10));

        SYS_CONSOLE_MESSAGE("\r\nContinue checking whether the port is open or closed.\r\n");
        TcpIPTest();

        SYS_CONSOLE_MESSAGE("Transmit CAN data to all 6 CAN bus interfaces.\r\n");  
        uint8_t bytesToSend = 12;

        uint8_t CANbuffer[12] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10, 0x11};
        
        CAN_Write(0,(uint8_t*)CANbuffer, (uint8_t)bytesToSend);
        vTaskDelay(10);
        CAN_Write(1,(uint8_t*)CANbuffer, (uint8_t)bytesToSend);
        vTaskDelay(10);
        CAN_Write(2,(uint8_t*)CANbuffer, (uint8_t)bytesToSend);
        vTaskDelay(10);
        CAN_Write(3,(uint8_t*)CANbuffer, (uint8_t)bytesToSend);
        vTaskDelay(10);
        CAN_Write(4,(uint8_t*)CANbuffer, (uint8_t)bytesToSend);
        vTaskDelay(10);
        CAN_Write(5,(uint8_t*)CANbuffer, (uint8_t)bytesToSend);  
        vTaskDelay(10);
    }
    Clear_All_Outputs();
    SYS_CONSOLE_MESSAGE("Full Load Test Completed.\r\n");
}

static void CommandIdealLoadTest(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    SYS_CONSOLE_MESSAGE("Ideal Load Test Command Received\r\n");  
}

static void CommandHelp(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv)
{
    uint32_t ix;
    int32_t groupIx;
    const SYS_CMD_DESCRIPTOR*  pDcpt;
    const SYS_CMD_DESCRIPTOR_TABLE* pTbl, *pDTbl;
    const void* cmdIoParam = pCmdIO->cmdIoParam;

    if(argc == 1)
    {   // no params help; display basic info
        bool hadHeader = false;
        pTbl = usrCmdTbl;
        for (groupIx=0; groupIx < MAX_CMD_GROUP; groupIx++)
        {
            if (pTbl->pCmd != NULL)
            {
                if(!hadHeader)
                {
                    (*pCmdIO->pCmdApi->msg)(cmdIoParam, LINE_TERM "------- Supported command groups ------");
                    hadHeader = true;
                }
                (*pCmdIO->pCmdApi->msg)(cmdIoParam, LINE_TERM " *** ");
                (*pCmdIO->pCmdApi->msg)(cmdIoParam, pTbl->cmdGroupName);
                (*pCmdIO->pCmdApi->msg)(cmdIoParam, pTbl->cmdMenuStr);
                (*pCmdIO->pCmdApi->msg)(cmdIoParam, " ***");
            }
            pTbl++;
        }

        // display the basic commands
        (*pCmdIO->pCmdApi->msg)(cmdIoParam, LINE_TERM "---------- Built in commands ----------");

        ix = 0;
        pDcpt = builtinCmdTbl;
        while(ix < (sizeof(builtinCmdTbl)/sizeof(*builtinCmdTbl)))
        {
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, LINE_TERM " *** ");
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, pDcpt->cmdStr);
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, pDcpt->cmdDescr);
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, " ***");
            ix++;
            pDcpt++;
        }

        (*pCmdIO->pCmdApi->msg)(cmdIoParam, LINE_TERM);
    }
    else
    {   // we have a command group name
        pDTbl = NULL;
        pTbl = usrCmdTbl;
        for (groupIx=0; groupIx < MAX_CMD_GROUP; groupIx++)
        {
            if (pTbl->pCmd != NULL)
            {
                if(strcmp(pTbl->cmdGroupName, argv[1]) == 0)
                {   // match
                    pDTbl = pTbl;
                    break;
                }
            }
            pTbl++;
        }

        if(pDTbl != NULL)
        {
            ix = 0;
            pDcpt = pDTbl->pCmd;
            while(ix < (uint32_t)pDTbl->nCmds)
            {
                (*pCmdIO->pCmdApi->msg)(cmdIoParam, LINE_TERM " *** ");
                (*pCmdIO->pCmdApi->msg)(cmdIoParam, pDcpt->cmdStr);
                (*pCmdIO->pCmdApi->msg)(cmdIoParam, pDcpt->cmdDescr);
                (*pCmdIO->pCmdApi->msg)(cmdIoParam, " ***");
                ix++;
                pDcpt++;
            }

            (*pCmdIO->pCmdApi->msg)(cmdIoParam, LINE_TERM);
        }
        else
        {
            (*pCmdIO->pCmdApi->msg)(cmdIoParam, LINE_TERM "Unknown command group. Try help" LINE_TERM );
        }
    }

}

static void ParseCmdBuffer(SYS_CMD_IO_DCPT* pCmdIO)
{
    int  argc = 0;
    char *argv[MAX_CMD_ARGS] = {0};
    char saveCmd[SYS_CMD_MAX_LENGTH + 1];
    char usrSaveCmd[SYS_CMD_MAX_LENGTH + 1];
    const void* cmdIoParam = pCmdIO->devNode.cmdIoParam;

    uint32_t  ix;
    int grp_ix;
    const SYS_CMD_DESCRIPTOR* pDcpt;

    (void) strncpy(saveCmd, pCmdIO->cmdBuff, sizeof(saveCmd));     // make a copy of the command

    // standard parse a command string to *argv[]
    argc = StringToArgs(saveCmd, argv, MAX_CMD_ARGS);

    if(argc == 0)
    {
        (*pCmdIO->devNode.pCmdApi->msg)(cmdIoParam, " *** Command Processor: Please type in a command***" LINE_TERM);
    }
    else
    {
        if(argc > 0)
        {   // ok, there's smth here
            // add it to the history list
            histCmdNode* pN = CmdRemoveTail(&pCmdIO->histList);
            (void) strncpy(pN->cmdBuff, pCmdIO->cmdBuff, sizeof(pN->cmdBuff)); // Need save non-parsed string
            CmdAddHead(&pCmdIO->histList, pN);
            pCmdIO->currHistN = NULL;

            // try built-in commands first
            ix = 0;
            pDcpt = builtinCmdTbl;
            while(ix < (sizeof(builtinCmdTbl)/sizeof(*builtinCmdTbl)))
            {
                if(strcmp(argv[0], pDcpt->cmdStr) == 0)
                {   // command found
                    if(argc > MAX_CMD_ARGS)
                    {
                        (*pCmdIO->devNode.pCmdApi->print)(cmdIoParam, "\n\r Too many arguments. Maximum args supported: %d!\r\n", MAX_CMD_ARGS);
                    }
                    else
                    {   // OK, call command handler
                        pDcpt->cmdFnc(&pCmdIO->devNode, argc, argv);
                    }
                    return;
                }
                ix++;
                pDcpt++;
            }

            // search user commands
            SYS_CMD_DESCRIPTOR_TABLE* pTbl = usrCmdTbl;
            grp_ix = 0;
            while(grp_ix < MAX_CMD_GROUP)
            {
                if (pTbl->pCmd != NULL)
                {
                    if (pTbl->usrCallback != NULL)
                    {
                        // external parser; give it a fresh copy of the command
                        (void) strncpy(usrSaveCmd, pCmdIO->cmdBuff, sizeof(usrSaveCmd));
                        if(pTbl->usrCallback(pTbl->pCmd, &pCmdIO->devNode, usrSaveCmd, sizeof(usrSaveCmd), pTbl->usrParam))
                        {   // command processed externally
                            return;
                        }
                        // reparse the user modified command
                        argc = StringToArgs(usrSaveCmd, argv, MAX_CMD_ARGS);
                    }

                    if(argc > MAX_CMD_ARGS)
                    {
                        (*pCmdIO->devNode.pCmdApi->print)(cmdIoParam, "\n\r Too many arguments. Maximum args supported: %d!\r\n", MAX_CMD_ARGS);
                        return;
                    }

                    ix = 0;
                    pDcpt = usrCmdTbl[grp_ix].pCmd;
                    while(ix < (uint32_t)usrCmdTbl[grp_ix].nCmds)
                    {
                        if(strcmp(argv[0], pDcpt->cmdStr) == 0)
                        {
                            // command found
                            pDcpt->cmdFnc(&pCmdIO->devNode, argc, argv);
                            return;
                        }
                        ix++;
                        pDcpt++;
                    }
                }
                grp_ix++;
                pTbl++;
            }
        }

        // command not found
        (*pCmdIO->devNode.pCmdApi->msg)(cmdIoParam, " *** Command Processor: unknown command. ***\r\n");
    }
}

/*
  parse a string into '*argv[]' tokens
  token delimitor is space, tab or comma
  parts within quotes (") are parsed as a single token
  return number of parsed tokens
  < 0 if error
*/
static int StringToArgs(char *str, char *argv[], size_t argvSize)
{
    char* pTkn;
    char* qStart, *qEnd;   // special char '"' starting position;
    int nArgs = 0;

    while(str != NULL)
    {
        qStart = strchr(str, (int32_t)'"');
        if(qStart != NULL)
        {
            *qStart = (char)0;
        }

        // parse until quote
        while((pTkn = strtok(str, " \t,")) != NULL)
        {
            str = NULL;
            if((uint32_t)nArgs < argvSize)
            {
                argv[nArgs] = pTkn;
            }
            nArgs++;
        }

        if(qStart == NULL)
        {   // done
            break;
        }

        // get end quote
        qStart++;
        qEnd = strchr(qStart, (int32_t)'"');
        if((qEnd == NULL) || ((qEnd - qStart) == 0))
        {   // no matching quote end or empty string within quotes
            return -1;
        }
        *qEnd = (char)0;
        if((uint32_t)nArgs < argvSize)
        {
            argv[nArgs] = qStart;
        }
        nArgs++;

        // continue parsing
        str = qEnd + 1;
    }


    return nArgs;
}

static void lkeyUpProcess(SYS_CMD_IO_DCPT* pCmdIO, const KEY_SEQ_DCPT* pSeqDcpt)
{   // up arrow
    histCmdNode *pNext;

    if(pCmdIO->currHistN != NULL)
    {
        pNext = pCmdIO->currHistN->next;
        if(pNext == pCmdIO->histList.head)
        {
            return; // reached the end of list
        }
    }
    else
    {
        pNext = pCmdIO->histList.head;
    }

    DisplayNodeMsg(pCmdIO, pNext);
}

static void lkeyDownProcess(SYS_CMD_IO_DCPT* pCmdIO, const KEY_SEQ_DCPT* pSeqDcpt)
{   // down arrow
    histCmdNode *pNext;

    if(pCmdIO->currHistN != NULL)
    {
        pNext = pCmdIO->currHistN->prev;
        if(pNext != pCmdIO->histList.tail)
        {
            DisplayNodeMsg(pCmdIO, pNext);
        }
    }
}

static void lkeyRightProcess(SYS_CMD_IO_DCPT* pCmdIO, const KEY_SEQ_DCPT* pSeqDcpt)
{   // right arrow
    const SYS_CMD_API* pCmdApi = pCmdIO->devNode.pCmdApi;
    const void* cmdIoParam = pCmdIO->devNode.cmdIoParam;

    if(pCmdIO->cmdPnt < pCmdIO->cmdEnd)
    {   // just advance
        (*pCmdApi->msg)(cmdIoParam, pSeqDcpt->keyCode);
        pCmdIO->cmdPnt++;
    }
}

static void lkeyLeftProcess(SYS_CMD_IO_DCPT* pCmdIO, const KEY_SEQ_DCPT* pSeqDcpt)
{   // left arrow
    const SYS_CMD_API* pCmdApi = pCmdIO->devNode.pCmdApi;
    const void* cmdIoParam = pCmdIO->devNode.cmdIoParam;

    if(pCmdIO->cmdPnt > pCmdIO->cmdBuff)
    {
        pCmdIO->cmdPnt--;
        (*pCmdApi->msg)(cmdIoParam, pSeqDcpt->keyCode);
    }
}

static void lkeyHomeProcess(SYS_CMD_IO_DCPT* pCmdIO, const KEY_SEQ_DCPT* pSeqDcpt)
{   // home key
    const SYS_CMD_API* pCmdApi = pCmdIO->devNode.pCmdApi;
    const void* cmdIoParam = pCmdIO->devNode.cmdIoParam;
    int nChars = pCmdIO->cmdPnt - pCmdIO->cmdBuff;
    if(nChars != 0)
    {
        // <ESC>[{COUNT}D
        char homeBuff[ 10 + 1];
        (void) sprintf(homeBuff, "\x1b[%dD", nChars);
        (*pCmdApi->msg)(cmdIoParam, homeBuff);
        pCmdIO->cmdPnt = pCmdIO->cmdBuff;
    }
}

static void lkeyEndProcess(SYS_CMD_IO_DCPT* pCmdIO, const KEY_SEQ_DCPT* pSeqDcpt)
{   // end key
    const SYS_CMD_API* pCmdApi = pCmdIO->devNode.pCmdApi;
    const void* cmdIoParam = pCmdIO->devNode.cmdIoParam;

    int nChars = pCmdIO->cmdEnd - pCmdIO->cmdPnt;
    if(nChars != 0)
    {
        // "<ESC>[{COUNT}C"
        char endBuff[ 10 + 1];
        (void) sprintf(endBuff, "\x1b[%dC", nChars);
        (*pCmdApi->msg)(cmdIoParam, endBuff);
        pCmdIO->cmdPnt = pCmdIO->cmdEnd;
    }

}
/* MISRAC 2012 deviation block end */

static void DisplayNodeMsg(SYS_CMD_IO_DCPT* pCmdIO, histCmdNode* pNext)
{
    int oCmdLen, nCmdLen;
    const SYS_CMD_API* pCmdApi = pCmdIO->devNode.pCmdApi;
    const void* cmdIoParam = pCmdIO->devNode.cmdIoParam;

    if((nCmdLen = (int32_t)strlen(pNext->cmdBuff)) != 0)
    {   // something there
        oCmdLen = pCmdIO->cmdEnd - pCmdIO->cmdBuff;
        while(oCmdLen > nCmdLen)
        {
            (*pCmdApi->msg)(cmdIoParam, "\b \b");     // clear the old command
            oCmdLen--;
        }
        while(oCmdLen != 0)
        {
            (*pCmdApi->msg)(cmdIoParam, "\b");
            oCmdLen--;
        }
        (void) strcpy(pCmdIO->cmdBuff, pNext->cmdBuff);
        (*pCmdApi->msg)(cmdIoParam, "\r\n>");
        (*pCmdApi->msg)(cmdIoParam, pCmdIO->cmdBuff);
        pCmdIO->cmdPnt = pCmdIO->cmdBuff + nCmdLen;
        pCmdIO->cmdEnd = pCmdIO->cmdBuff + nCmdLen;
        pCmdIO->currHistN = pNext;
    }
}


static void CmdAddHead(histCmdList* pL, histCmdNode* pN)
{
    if(pL->head == NULL)
    { // empty list, first node
        pL->head = pN;
        pL->tail = pN;
        pN->next = pN;
        pN->prev = pN;
    }
    else
    {
        pN->next = pL->head;
        pN->prev = pL->tail;
        pL->tail->next = pN;
        pL->head->prev = pN;
        pL->head = pN;
    }
}


static histCmdNode* CmdRemoveTail(histCmdList* pL)
{
    histCmdNode* pN;
    if(pL->head == pL->tail)
    {
        pN = pL->head;
        pL->head = NULL;
        pL->tail = NULL;
    }
    else
    {
        pN = pL->tail;
        pL->tail = pN->prev;
        pL->tail->next = pL->head;
        pL->head->prev = pL->tail;
    }
    return pN;
}

static void CmdAdjustPointers(SYS_CMD_IO_DCPT* pCmdIO)
{
    if(pCmdIO->cmdPnt > pCmdIO->cmdEnd)
    {
        pCmdIO->cmdEnd = pCmdIO->cmdPnt;
    }
}


