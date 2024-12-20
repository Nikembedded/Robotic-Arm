//---------------------------------------------------------------------------
// COMPANY      : Confederation College
// FILE         : SERVO.CPP
// FILE VERSION : 1.0.0
// PROGRAMMER   : Nikhil Premjani
//---------------------------------------------------------------------------
// REVISION HISTORY
//---------------------------------------------------------------------------
//
// 1.0, 2024-06-07, Nikhil Premjani
//   - Initial release
//
//---------------------------------------------------------------------------
// MODULE DESCRIPTION
//---------------------------------------------------------------------------
//
//---------------------------------------------------------------------------
// INCLUDE FILES
//---------------------------------------------------------------------------

#include "servo.h"

//-----------------------------------------------------------------------------
// EXTERNAL REFERENCE
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// GLOBALS
//-----------------------------------------------------------------------------

//---------------------------------------------------------------------------
// FUNCTION    : SERVO::SERVO( void )
// DESCRIPTION : Constructor
//---------------------------------------------------------------------------

SERVO::SERVO( void )
{
    m_fTime   = 0.0f;
    m_fDelta  = 0.0f;
    m_fDuration = 1.0f;

    m_uiCountMin = 0;
    m_uiCountMax = 0;

    m_fPosStart   = 0.0f;
    m_fPosCurrent = 0.0f;
    m_fPosEnd     = 0.0f;

    m_fAngleSpan   = 0.0f;
    m_fAngleOffset = 0.0f;
}

//---------------------------------------------------------------------------
// FUNCTION    : SERVO::~SERVO()
// DESCRIPTION : Destructor
//---------------------------------------------------------------------------

SERVO::~SERVO()
{
}

//---------------------------------------------------------------------------
// FUNCTION    : SERVO::Initialize( float fDelta, uint32_t uiCountMin, uint32_t uiCountMax, float fAngleSpan, float fAngleOffset, float fAngleInitial )
// DESCRIPTION : Initializes the servo object
//---------------------------------------------------------------------------

bool SERVO::Initialize( float fDelta, uint32_t uiCountMin, uint32_t uiCountMax, float fAngleSpan, float fAngleOffset, float fAngleInitial )
{
    bool bResult = false;

    if( fDelta > 0.003f )
    {
        m_fDelta = fDelta;

        m_fAngleOffset = fAngleOffset;
        m_fAngleSpan   = fAngleSpan;

        if( uiCountMin < uiCountMax )
        {
            m_uiCountMin = uiCountMin;
            m_uiCountMax = uiCountMax;

            float fPosInitial;
            if( AngleToPosition( fAngleInitial, &fPosInitial ) )
            {
                if( ( fPosInitial >= 0.0f ) && ( fPosInitial <= 1.0f ) )
                {
                    m_fPosStart   = fPosInitial;
                    m_fPosCurrent = fPosInitial;
                    m_fPosEnd     = fPosInitial;

                    bResult = true;
                }
            }
        }
    }

    return bResult;
}

//---------------------------------------------------------------------------
// FUNCTION    : SERVO::GetCount( void )
// DESCRIPTION : Retrieves the current count for the PWM output of the servo
//---------------------------------------------------------------------------

uint32_t SERVO::GetCount( void )
{
    return m_uiCountMin + lroundf( ( float )( m_uiCountMax - m_uiCountMin ) * m_fPosCurrent );
}

//---------------------------------------------------------------------------
// FUNCTION    : SERVO::SetPosition( float fPosition, float fDuration )
// DESCRIPTION : Sets the current position of the servo
//---------------------------------------------------------------------------

bool SERVO::SetPosition( float fPosition, float fDuration )
{
    bool bResult = false;

    if( ( fPosition >= 0.0f ) && ( fPosition <= 1.0f ) )
    {
        if( fDuration > m_fDelta )
        {
            m_fDuration = fDuration;
            m_fTime = 0.0f;

            m_fPosEnd = fPosition;
            m_fPosStart = m_fPosCurrent;

            bResult = true;
        }
    }

    return bResult;
}

//---------------------------------------------------------------------------
// FUNCTION    : SERVO::GetPosition( void )
// DESCRIPTION : Retrieves the current position of the servo
//---------------------------------------------------------------------------

float SERVO::GetPosition( void )
{
    return m_fPosCurrent;
}

//---------------------------------------------------------------------------
// FUNCTION    : SERVO::StepPosition( void )
// DESCRIPTION : Steps the servo to an incremental position
//---------------------------------------------------------------------------

bool SERVO::StepPosition( void )
{
    bool bResult = false;

    if( m_fTime < m_fDuration )
    {
        m_fTime += m_fDelta;
        m_fPosCurrent = m_fPosStart + ( ( m_fPosEnd - m_fPosStart ) / ( 1.0f + expf( -M_PI * ( ( 4.0f * m_fTime / m_fDuration ) - 2.0f ) ) ) );
    }
    else
    {
        bResult = true;
    }

    return bResult;
}

//---------------------------------------------------------------------------
// FUNCTION    : SERVO::SetAngle( float fAngle, float fDuration )
// DESCRIPTION : Sets the angle of the servo
//---------------------------------------------------------------------------

bool SERVO::SetAngle( float fAngle, float fDuration )
{
    bool bResult = false;

    float fPosition;
    if( AngleToPosition( fAngle, &fPosition ) )
    {
        SetPosition( fPosition, fDuration );
        bResult = true;
    }

    return bResult;
}

//---------------------------------------------------------------------------
// FUNCTION    : SERVO::GetAngle( void )
// DESCRIPTION : Retrieves the current angle of the servo
//---------------------------------------------------------------------------

float SERVO::GetAngle( void )
{
    float fAngle = 0.0f;

    PositionToAngle( m_fPosCurrent, &fAngle );

    return fAngle;
}

//---------------------------------------------------------------------------
// FUNCTION    : SERVO::StepPosition( void )
// DESCRIPTION : Steps the servo to an incremental angle
//---------------------------------------------------------------------------

bool SERVO::StepAngle( void )
{
    return StepPosition();
}

//---------------------------------------------------------------------------
// FUNCTION    : SERVO::AngleToPosition( float fAngle, float *pfPosition )
// DESCRIPTION : Converts an angle to a position
//---------------------------------------------------------------------------

bool SERVO::AngleToPosition( float fAngle, float *pfPosition )
{
    bool bResult = false;

    if( ( ( ( fAngle + m_fAngleOffset ) / m_fAngleSpan ) >= 0.0f ) &&
        ( ( ( fAngle + m_fAngleOffset ) / m_fAngleSpan ) <= 1.0f ) )
        {
            *pfPosition = ( fAngle + m_fAngleOffset ) / m_fAngleSpan;
            bResult = true;
        }

    return bResult;
}

//---------------------------------------------------------------------------
// FUNCTION    : SERVO::PositionToAngle( float fPosition, float *pfAngle )
// DESCRIPTION : Converts a position to an angle
//---------------------------------------------------------------------------

bool SERVO::PositionToAngle( float fPosition, float *pfAngle )
{
    bool bResult = false;

    if( ( fPosition >= 0.0f ) && ( fPosition <= 1.0f ) )
    {
        *pfAngle = ( fPosition * m_fAngleSpan ) - m_fAngleOffset;
        bResult = true;
    }

    return bResult;
}

//---------------------------------------------------------------------------
// FUNCTION    : SERVO::D2R( float d )
// DESCRIPTION : Converts degrees to radians
//---------------------------------------------------------------------------

float SERVO::D2R( float d )
{
    d = fmod( d, 360.0f );
    d -= d > 180.0f ? 360.0f : 0.0f;

    return ( d * M_PI / 180.0f );
}

//---------------------------------------------------------------------------
// FUNCTION    : SERVO::D2RM( float d )
// DESCRIPTION : Converts degrees to radians (magnitude)
//---------------------------------------------------------------------------

float SERVO::D2RM( float d )
{
    d = fmod( d, 360.0f );

    return ( d * M_PI / 180.0f );
}

//---------------------------------------------------------------------------
// FUNCTION    : SERVO::R2D( float r )
// DESCRIPTION : Converts radians to degrees
//---------------------------------------------------------------------------

float SERVO::R2D( float r )
{
    r = fmod( r, 2.0f * M_PI );
    r -= r > M_PI ? 2.0f * M_PI : 0.0f;

    return ( r * 180.0f / M_PI );
}

//---------------------------------------------------------------------------
// END SERVO.CPP
//---------------------------------------------------------------------------
