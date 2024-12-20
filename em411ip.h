//---------------------------------------------------------------------------
// COMPANY      : Confederation College
// FILE         : EM411IP.H
// FILE VERSION : 1.0.0
// PROGRAMMER   : Nikhil Premjani
//---------------------------------------------------------------------------
// REVISION HISTORY
//---------------------------------------------------------------------------
//
// 1.0, 2024-06-07, Programmer Name
//   - Initial release
//
//---------------------------------------------------------------------------
// INCLUDE FILES
//---------------------------------------------------------------------------

// Inclusion lock
#ifndef _EM411IP_H
#define _EM411IP_H

#include "global.h"
#include "driverlib/rom.h"

//---------------------------------------------------------------------------
// DEFINES
//---------------------------------------------------------------------------

// Firmware Version
#define EM411IP_VER_MAJOR            1   // Major Version Number
#define EM411IP_VER_MINOR            0   // Minor Version Number

// Limits
#define EM411IP_MAXBUFSZ           256   // Maximum buffer size

// Timeout Values
#define EM411IP_TIMEOUT            100   // Typical EM411IP Transaction
#define EM411IP_REQUESTWAIT          0   // Request wait time
#define EM411IP_PACKETTIMEOUT       50   // Packet timeout
#define EM411IP_BACKOFF            100   // Backoff before retry

// Request Codes - System
#define EM411IP_SYSPING            0x20  // Ping
#define EM411IP_SYSINFO            0x21  // System Information

// Request Codes - Operation
#define EM411IP_OPRRIS             0x30  // Read Input States
#define EM411IP_OPRWOS             0x31  // Write Output States
#define EM411IP_OPRMTP             0x32  // Move to Point
#define EM411IP_OPRGCP             0x33  // Get Current Position

// EM411IP Control Code Definitions
#define EM411IP_SOH                0x01  // Start of heading
#define EM411IP_EOT                0x04  // End of transmission
#define EM411IP_NAK                0x15  // Negative Acknowledge

// EM411IP Response Status Byte
#define EM411IP_RSB_OK             0x00
#define EM411IP_RSB_RESTRICTED     0x01
#define EM411IP_RSB_UNSUPPORTED    0x02
#define EM411IP_RSB_REQUEST_ERROR  0x03
#define EM411IP_RSB_NOT_COMPLETED  0x04
#define EM411IP_RSB_UNSOLICITED    0xf0

// EM411IP Error Response Byte
#define EM411IP_ERB_LEN_ERROR                 0x00
#define EM411IP_ERB_RECEPTION_TIMEOUT         0x01
#define EM411IP_ERB_TRANSMISSION_TIMEOUT      0x02
#define EM411IP_ERB_RECEIVER_BUFFER_OVERFLOW  0x03
#define EM411IP_ERB_UNRECOGNIZED_CODE         0x04

// EM411IP Link Status
#define EM411IP_OK           0x00000100  // Ok
#define EM411IP_LINK_ERROR   0x00000101  // Link error
#define EM411IP_COMM_ERROR   0x00000102  // Protocol error
#define EM411IP_DATA_ERROR   0x00000104  // Data error

// EM411IP Packet Offsets
#define EM411IP_O_DP    3
#define EM411IP_O_COD   (EM411IP_O_DP+0)    // Command Code Offset
#define EM411IP_O_RSB   (EM411IP_O_DP+1)    // Respond Status Byte Offset
#define EM411IP_O_TXD   (EM411IP_O_DP+2)    // Transmit Data Offset
#define EM411IP_O_RXD   (EM411IP_O_DP+1)    // Receive Data Offset

/*
// Manipulator Link Lengths (in meters)
#define MANIP_L0         0.067f
#define MANIP_L1         0.157f
#define MANIP_L2         0.155f
#define MANIP_L3         0.100f
*/

// EM411IP States
enum EM411IP_STATES
{
    EM411IP_STATE_SOH = 0,  // Start of heading
    EM411IP_STATE_LEN0,     // Length byte 0
    EM411IP_STATE_LEN1,     // Length byte 1
    EM411IP_STATE_DP,       // Data packet
    EM411IP_STATE_CRC0,     // CRC byte 0
    EM411IP_STATE_CRC1,     // CRC byte 1
    EM411IP_STATE_EOT,      // End of transmission

    EM411IP_STATE_COMPLETE,
};

// EM411IP Status
enum EM411IP_STATUS
{
    EM411IP_STATUS_TRANSACTING = 0, // Performing transaction
    EM411IP_STATUS_PACKET_RECEIVED, // Packet received without error
    EM411IP_STATUS_TIMEOUT,         // Timeout
    EM411IP_STATUS_ABORTED,         // Aborted
    EM411IP_STATUS_CRC_ERROR,       // CRC Error
};

//---------------------------------------------------------------------------
// STRUCTURES
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// CLASS DECLARATIONS
//---------------------------------------------------------------------------

class EM411IP
{

    volatile uint32_t m_ui32TxCount;
    volatile uint32_t m_ui32RxCount;

    uint8_t ProcessRequest(void);
    void PrepareResponse(uint32_t uiLength);

    uint8_t m_uiRxState;
    uint8_t m_uiTxState;

    TickType_t m_tRxTimeoutStart;

public:

     float x,y,z,iTime, iPitch, iRoll, iClamp;


    float angle_S0,angle_S1,angle_S2,angle_S3,angle_S4,angle_S5;

    float C_Beta, C_Alpha, B, iAlpha, q,R,r1, Ze;
    float x_pos,y_pos,z_pos,C_Pitch;


    uint8_t m_aRxPacket[ EM411IP_MAXBUFSZ];
    uint16_t m_uiRxPacketLen;

    uint8_t m_aTxPacket[ EM411IP_MAXBUFSZ];
    uint16_t m_uiTxPacketLen;

    uint16_t m_uiReqDPLen;
    uint16_t m_uiReqCRC;
    uint16_t m_uiCalcCRC;

    uint16_t m_uiTxCRC;

    EM411IP(void);
    ~EM411IP();

    void RxTimeout(TimerHandle_t xTimer);

    void IncTxCount(uint32_t uiCount);
    void IncRxCount(uint32_t uiCount);

    bool FlushTx();
    bool FlushRx();

    bool Initialize(void);

    uint8_t Receive(uint8_t uiData);

    // Commands - System
    uint32_t SysPing(void);
    uint32_t SysInformation(void);

    // Commands - Operations

    uint32_t OprReadInputStates(void);
    uint32_t OprWriteOutputStates(void);
    uint32_t OprMoveToPoint(void);
    uint32_t GetCurrentPosition(void);

};

#endif // _EM411IP_H

//---------------------------------------------------------------------------
// END EM411IP.H
//---------------------------------------------------------------------------
