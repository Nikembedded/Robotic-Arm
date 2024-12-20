//---------------------------------------------------------------------------
// COMPANY      : Confederation College
// FILE         : EM411IP.CPP
// FILE VERSION : 1.0.0
// PROGRAMMER   : Nikhil Premjani
//---------------------------------------------------------------------------
// REVISION HISTORY
//---------------------------------------------------------------------------
//
// 1.0, 2024-06-06, Programmer Name
//   - Initial release
//
//---------------------------------------------------------------------------
// MODULE DESCRIPTION
//---------------------------------------------------------------------------
////
//---------------------------------------------------------------------------
// INCLUDE FILES
//---------------------------------------------------------------------------

#include "em411ip.h"
#include "Tasks/exio.h"
#include <math.h>
#include "Tasks/manip.h"
#include "servo.h"

//-----------------------------------------------------------------------------
// EXTERNAL REFERENCE
//-----------------------------------------------------------------------------

extern QueueHandle_t g_qEXIOQueue;
extern QueueHandle_t g_qMANIPQueue;
extern SERVO   g_SERVO[MANIP_SERVO_NUM_SERVOS];

//-----------------------------------------------------------------------------
// GLOBALS
//-----------------------------------------------------------------------------

EM411IP g_EM411IP;

//---------------------------------------------------------------------------
// FUNCTION    : EM411IP::EM411IP( void )
// DESCRIPTION : Constructor
//---------------------------------------------------------------------------

EM411IP::EM411IP(void)
{
    m_ui32TxCount = 0;
    m_ui32RxCount = 0;
}

//---------------------------------------------------------------------------
// FUNCTION    : EM411IP::~EM411IP()
// DESCRIPTION : Destructor
//---------------------------------------------------------------------------

EM411IP::~EM411IP()
{
}

//---------------------------------------------------------------------------
// FUNCTION    : EM411IP::IncTxCount( uint32_t uiCount )
// DESCRIPTION : Increment transmit count
//---------------------------------------------------------------------------

void EM411IP::IncTxCount(uint32_t uiCount)
{
    m_ui32TxCount += uiCount;
    return;
}

//---------------------------------------------------------------------------
// FUNCTION    : EM411IP::IncRxCount( uint32_t uiCount )
// DESCRIPTION : Increment receive count
//---------------------------------------------------------------------------

void EM411IP::IncRxCount(uint32_t uiCount)
{
    m_ui32RxCount += uiCount;
    return;
}

//---------------------------------------------------------------------------
// FUNCTION    : EM411IP::FlushTx( void )
// DESCRIPTION : Flush the transmit buffer
//---------------------------------------------------------------------------

bool EM411IP::FlushTx(void)
{
    return true;
}

//---------------------------------------------------------------------------
// FUNCTION    : EM411IP::FlushRx( void )
// DESCRIPTION : Flush the receive buffer
//---------------------------------------------------------------------------

bool EM411IP::FlushRx(void)
{
    while (UARTCharsAvail( UART0_BASE))
    {
        UARTCharGet( UART0_BASE);
    }

    return true;
}

//---------------------------------------------------------------------------
// FUNCTION    : EM411IP::Initialize( void )
// DESCRIPTION : Initializes the EM411IP link
//---------------------------------------------------------------------------

bool EM411IP::Initialize(void)
{
    bool bResult = false;

    // Enable Peripherals
    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable( SYSCTL_PERIPH_UART0);

    // Set GPIO A0 and A1 as UART pins and A2 as DE
    GPIOPinConfigure( GPIO_PA0_U0RX);
    GPIOPinConfigure( GPIO_PA1_U0TX);
    GPIOPinTypeUART( GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinTypeGPIOOutput( GPIO_PORTA_BASE, GPIO_PIN_2);
    GPIOPinWrite( GPIO_PORTA_BASE, GPIO_PIN_2, 0);

    // Configure the UART for 115,200, 8-N-1 operation
    UARTConfigSetExpClk( UART0_BASE, SysCtlClockGet(), 115200,
                        ( UART_CONFIG_WLEN_8 |
                        UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE));
    UARTFIFODisable( UART0_BASE);

    m_ui32TxCount = 0;
    m_ui32RxCount = 0;

    m_uiRxState = EM411IP_STATE_SOH;
    m_uiTxState = EM411IP_STATE_SOH;

    FlushTx();
    FlushRx();

    // Enable the driver output
    GPIOPinWrite( GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_PIN_2);

    bResult = true;

    return bResult;
}

//---------------------------------------------------------------------------
// FUNCTION    : EM411IP::Receive( uint8_t uiData )
// DESCRIPTION : Receive L2 character
//---------------------------------------------------------------------------

uint8_t EM411IP::Receive(uint8_t uiData)
{
    uint8_t uiStatus = EM411IP_STATUS_TRANSACTING;

    // Check for reception timeout - !!! Check for rollover
    if (m_uiRxState != EM411IP_STATE_SOH)
    {
        if ((xTaskGetTickCount() - m_tRxTimeoutStart)
                > pdMS_TO_TICKS(EM411IP_PACKETTIMEOUT))
        {
            m_uiRxState = EM411IP_STATE_SOH;
            uiStatus = EM411IP_STATUS_TIMEOUT;
        }
    }

    if (uiStatus == EM411IP_STATUS_TRANSACTING)
    {
        // Receive Packet
        switch (m_uiRxState)
        {
        case EM411IP_STATE_SOH:

            // Prepare to receive L2 request
            m_uiRxPacketLen = 0;

            // Check for start of heading
            if (uiData == EM411IP_SOH)
            {
                m_aRxPacket[m_uiRxPacketLen++] = uiData;
                m_tRxTimeoutStart = xTaskGetTickCount();
                m_uiRxState = EM411IP_STATE_LEN0;
                IncRxCount(1);
            }
            else
            {
                // Bad start - abort
                m_uiRxState = EM411IP_STATE_SOH;
                uiStatus = EM411IP_STATUS_ABORTED;
            }
            break;
        case EM411IP_STATE_LEN0:

            // LSB of packet length
            m_uiReqDPLen = uiData;
            m_aRxPacket[m_uiRxPacketLen++] = uiData;
            IncRxCount(1);
            m_uiRxState = EM411IP_STATE_LEN1;
            break;
        case EM411IP_STATE_LEN1:

            // MSB of packet length
            m_uiReqDPLen |= uiData << 8;
            m_aRxPacket[m_uiRxPacketLen++] = uiData;
            IncRxCount(1);
            if (m_uiReqDPLen <= EM411IP_MAXBUFSZ)
            {
                m_uiRxState = EM411IP_STATE_DP;
            }
            else
            {
                // Bad packet length - abort
                m_uiRxState = EM411IP_STATE_SOH;
                uiStatus = EM411IP_STATUS_ABORTED;
            }
            break;
        case EM411IP_STATE_DP:

            // Obtain data packet
            m_aRxPacket[m_uiRxPacketLen++] = uiData;
            IncRxCount(1);

            if (m_uiReqDPLen - (m_uiRxPacketLen - EM411IP_O_DP) == 0)
            {
                m_uiRxState = EM411IP_STATE_CRC0;
            }
            break;

        case EM411IP_STATE_CRC0:

            // LSB of CRC
            m_uiReqCRC = uiData;
            m_aRxPacket[m_uiRxPacketLen++] = uiData;
            IncRxCount(1);
            m_uiRxState = EM411IP_STATE_CRC1;

            break;

        case EM411IP_STATE_CRC1:

            // MSB of CRC
            m_uiReqCRC |= uiData << 8;
            m_aRxPacket[m_uiRxPacketLen++] = uiData;
            IncRxCount(1);

            // Verify CRC
            m_uiCalcCRC = ROM_Crc16(0, &m_aRxPacket[ EM411IP_O_DP],
                                    m_uiReqDPLen);
            if (m_uiCalcCRC != m_uiReqCRC)
            {
                // CRC error
                m_uiRxState = EM411IP_STATE_SOH;
                uiStatus = EM411IP_STATUS_CRC_ERROR;
            }
            else
            {
                m_uiRxState = EM411IP_STATE_EOT;
            }
            break;
        case EM411IP_STATE_EOT:

            // Check for end of transmission
            if (uiData == EM411IP_EOT)
            {
                m_aRxPacket[m_uiRxPacketLen++] = uiData;
                IncRxCount(1);

                // If processing results in an error, flush and wait backoff time
                if (ProcessRequest() == EM411IP_RSB_OK)
                {
                    uiStatus = EM411IP_STATUS_PACKET_RECEIVED;
                }
                else
                {
                    // Bad end - abort
                    uiStatus = EM411IP_STATUS_ABORTED;
                }
            }
            m_uiRxState = EM411IP_STATE_SOH;
            break;
        default:
            // Synchronization error - abort
            uiStatus = EM411IP_STATUS_ABORTED;
            m_uiRxState = EM411IP_STATE_SOH;
            break;
        }
    }

    return uiStatus;
}

//---------------------------------------------------------------------------
// FUNCTION    : EM411IP::ProcessRequest( void )
// DESCRIPTION : Processes the client request
//---------------------------------------------------------------------------

uint8_t EM411IP::ProcessRequest()
{
    uint8_t uiResult = EM411IP_RSB_UNSUPPORTED;

    if (m_uiRxPacketLen)
    {
        switch (m_aRxPacket[ EM411IP_O_COD])
        {
        case EM411IP_SYSPING:
            uiResult = SysPing();
            break;
        case EM411IP_SYSINFO:
            uiResult = SysInformation();
            break;

        case EM411IP_OPRRIS:
            uiResult = OprReadInputStates();
            break;
        case EM411IP_OPRWOS:
            uiResult = OprWriteOutputStates();
            break;
        case EM411IP_OPRMTP:
            uiResult = OprMoveToPoint();
            break;
        case EM411IP_OPRGCP:
            uiResult = GetCurrentPosition();
            break;
        default:
            break;
        }
    }
    return uiResult;
}

//---------------------------------------------------------------------------
// FUNCTION    : EM411IP::PrepareResponse( uint32_t uiLength )
// DESCRIPTION : Prepares the host response
//---------------------------------------------------------------------------

void EM411IP::PrepareResponse(uint32_t uiLength)
{
    m_uiTxPacketLen = 0;

    // Calculate response CRC
    m_uiCalcCRC = ROM_Crc16(0, &m_aTxPacket[ EM411IP_O_DP], uiLength);

    // Prepare header
    m_aTxPacket[m_uiTxPacketLen++] = EM411IP_SOH;
    m_aTxPacket[m_uiTxPacketLen++] = (uint8_t) (uiLength >> 0) & 0x00ff;
    m_aTxPacket[m_uiTxPacketLen++] = (uint8_t) (uiLength >> 8) & 0x00ff;

    // Account for data packet
    m_uiTxPacketLen += uiLength;

    // Prepare tailer
    m_aTxPacket[m_uiTxPacketLen++] = (uint8_t) (m_uiCalcCRC >> 0) & 0x00ff;
    m_aTxPacket[m_uiTxPacketLen++] = (uint8_t) (m_uiCalcCRC >> 8) & 0x00ff;
    m_aTxPacket[m_uiTxPacketLen++] = EM411IP_EOT;

    return;
}

//---------------------------------------------------------------------------
// FUNCTION    : EM411IP::SysPing( void )
// DESCRIPTION : Respond to system ping request
//---------------------------------------------------------------------------

uint32_t EM411IP::SysPing(void)
{
    m_aTxPacket[ EM411IP_O_COD] = m_aRxPacket[ EM411IP_O_COD];
    if ((m_uiReqDPLen >= 1) && (m_uiReqDPLen <= 256))
    {
        m_aTxPacket[ EM411IP_O_RSB] = EM411IP_RSB_OK;
        for (uint8_t n = 0; n < (m_uiReqDPLen - 1); n++)
        {
            m_aTxPacket[ EM411IP_O_TXD + n] = m_aRxPacket[ EM411IP_O_RXD + n];
        }
        PrepareResponse(m_uiReqDPLen + 2);
    }
    else
    {
        m_aTxPacket[ EM411IP_O_RSB] = EM411IP_RSB_NOT_COMPLETED;
        PrepareResponse(2);
    }

    return m_aTxPacket[ EM411IP_O_RSB];
}

//---------------------------------------------------------------------------
// FUNCTION    : EM411IP::SysInformation( void )
// DESCRIPTION : Respond to system information request
//---------------------------------------------------------------------------

uint32_t EM411IP::SysInformation(void)
{
    // Determine operation
    m_aTxPacket[ EM411IP_O_COD] = m_aRxPacket[ EM411IP_O_COD];
    m_aTxPacket[ EM411IP_O_RSB] = EM411IP_RSB_OK;

    m_aTxPacket[ EM411IP_O_TXD + 0] = EM411IP_VER_MAJOR;
    m_aTxPacket[ EM411IP_O_TXD + 1] = EM411IP_VER_MINOR;
    PrepareResponse(4);

    return m_aTxPacket[ EM411IP_O_RSB];
}

//---------------------------------------------------------------------------
// FUNCTION    : EM411IP::OprReadInputStates( void )
// DESCRIPTION : Reads and returns the state of the inputs
//---------------------------------------------------------------------------

uint32_t EM411IP::OprReadInputStates(void)
{
    // Determine operation
    m_aTxPacket[ EM411IP_O_COD] = m_aRxPacket[ EM411IP_O_COD];
    m_aTxPacket[ EM411IP_O_RSB] = EM411IP_RSB_OK;

    m_aTxPacket[ EM411IP_O_TXD] = 0;
    m_aTxPacket[ EM411IP_O_TXD] |=
            GPIOPinRead( EXIO_CTRL_BASE, EXIO_CTRL_IN0) ? 0 : 0x01;
    m_aTxPacket[ EM411IP_O_TXD] |=
            GPIOPinRead( EXIO_CTRL_BASE, EXIO_CTRL_IN1) ? 0 : 0x02;
    m_aTxPacket[ EM411IP_O_TXD] |=
            GPIOPinRead( EXIO_S2_BASE, EXIO_S2) ? 0 : 0x04;
    PrepareResponse(3);

    return m_aTxPacket[ EM411IP_O_RSB];
}

//---------------------------------------------------------------------------
// FUNCTION    : EM411IP::OprWriteOutputStates( void )
// DESCRIPTION : Writes the state of the outputs
//---------------------------------------------------------------------------

uint32_t EM411IP::OprWriteOutputStates(void)
{
    // Determine operation
    m_aTxPacket[ EM411IP_O_COD] = m_aRxPacket[ EM411IP_O_COD];
    m_aTxPacket[ EM411IP_O_RSB] = EM411IP_RSB_OK;

    EXIO_QUEUE_ITEM qiMessage = 0;

    if (m_aRxPacket[ EM411IP_O_RXD] & 0x01)
    {
        qiMessage =
                m_aRxPacket[ EM411IP_O_RXD + 1] & 0x01 ?
                        EXIO_MSG_CTRL_OUT0_EN : EXIO_MSG_CTRL_OUT0_DE;
        xQueueSend(g_qEXIOQueue, &qiMessage, portMAX_DELAY);
    }

    if (m_aRxPacket[ EM411IP_O_RXD] & 0x02)
    {
        qiMessage =
                m_aRxPacket[ EM411IP_O_RXD + 1] & 0x02 ?
                        EXIO_MSG_CTRL_OUT1_EN : EXIO_MSG_CTRL_OUT1_DE;
        xQueueSend(g_qEXIOQueue, &qiMessage, portMAX_DELAY);
    }

    if (m_aRxPacket[ EM411IP_O_RXD] & 0x04)
    {
        qiMessage =
                m_aRxPacket[ EM411IP_O_RXD + 1] & 0x04 ?
                        EXIO_MSG_CTRL_LED_ON : EXIO_MSG_CTRL_LED_OFF;
        xQueueSend(g_qEXIOQueue, &qiMessage, portMAX_DELAY);
    }

    PrepareResponse(2);

    return m_aTxPacket[ EM411IP_O_RSB];
}

//--------------------------------------------------------------------------
//Move to Point
//--------------------------------------------------------------------------
uint32_t EM411IP::OprMoveToPoint(void)
{

    MANIP_QUEUE_ITEM qiMessage;

        // Determine operation
        m_aTxPacket[ EM411IP_O_COD] = m_aRxPacket[ EM411IP_O_COD];
        m_aTxPacket[ EM411IP_O_RSB] = EM411IP_RSB_OK;

    iTime = *((float*) &m_aRxPacket[4]);

    x = *((float*) &m_aRxPacket[8]);
    y = *((float*) &m_aRxPacket[12]);
    z = *((float*) &m_aRxPacket[16]);

    iPitch = *((float*) &m_aRxPacket[20]);
    iRoll = *((float*) &m_aRxPacket[24]);
    iClamp = *((float*) &m_aRxPacket[28]);

    iRoll = SERVO::D2R(iRoll);

    qiMessage.DURATION = iTime;
    qiMessage.x = x;
    qiMessage.y = y;
    qiMessage.z = z;
    qiMessage.PITCH = iPitch;
    qiMessage.ROLL = iRoll;
    qiMessage.CLAMP = iClamp;
    xQueueSend(g_qMANIPQueue, &qiMessage, portMAX_DELAY);


    PrepareResponse(2);

    return m_aTxPacket[ EM411IP_O_RSB];
}

uint32_t EM411IP::GetCurrentPosition(void)
{
    // Determine operation
    m_aTxPacket[ EM411IP_O_COD] = m_aRxPacket[ EM411IP_O_COD];
    m_aTxPacket[ EM411IP_O_RSB] = EM411IP_RSB_OK;

    // GET ANGLES FOR CALCULATION
    angle_S0 = g_SERVO[0].GetAngle() - MANIP_J0_ADJ;
    angle_S1 = g_SERVO[1].GetAngle() - MANIP_J1_ADJ;
    angle_S2 = g_SERVO[2].GetAngle() - MANIP_J2_ADJ;
    angle_S3 = g_SERVO[3].GetAngle() - MANIP_J3_ADJ;
    angle_S4 = g_SERVO[4].GetAngle();
    angle_S5 = g_SERVO[5].GetAngle();

    C_Beta = M_PI - abs(angle_S2);
    B = sqrt(
            (MANIP_L2 * MANIP_L2) + (MANIP_L1 * MANIP_L1)
                    - (cosf(C_Beta) * (2 * MANIP_L2 * MANIP_L1)));
    C_Alpha = acosf(
            (((B * B) + (MANIP_L1 * MANIP_L1)) - (MANIP_L2 * MANIP_L2))
                    / (2 * B * MANIP_L1));
    iAlpha = C_Alpha + angle_S1;

    q = (((M_PI / 2) + angle_S1) - C_Beta);
    C_Pitch = angle_S3 - q;
    Ze = MANIP_L3 * sinf(C_Pitch);      //ZZe
    r1 = MANIP_L3 * cosf(C_Pitch);
    R = r1 + B * sinf(iAlpha);
    x_pos = R / (sqrt(1 + (tanf(angle_S0) * tanf(angle_S0))));
    y_pos = x_pos * tanf(angle_S0);
    z_pos = (B * cosf(iAlpha)) + Ze + MANIP_L0;

    iClamp = ((angle_S5 + (M_PI / 2)) * 0.035f) / M_PI;

    float *pData = (float*) &m_aTxPacket[ EM411IP_O_TXD];

    uint16_t n = 0;
    pData[n++] = x_pos;
    pData[n++] = y_pos;
    pData[n++] = z_pos;
    pData[n++] = SERVO::R2D(C_Pitch);
    pData[n++] = SERVO::R2D(angle_S4);
    pData[n++] = iClamp;

    PrepareResponse(26);

    return m_aTxPacket[ EM411IP_O_RSB];

}
//---------------------------------------------------------------------------
// END EM411IP.CPP
//---------------------------------------------------------------------------
