//---------------------------------------------------------------------------
// COMPANY      : Confederation College
// FILE         : SERVO.H
// FILE VERSION : 1.0.0
// PROGRAMMER   : Programmer Name
//---------------------------------------------------------------------------
// REVISION HISTORY
//---------------------------------------------------------------------------
//
// 1.0, YYYY-MM-DD, Programmer Name
//   - Initial release
//
//----------------------------------------------------------------------------
// INCLUSION LOCK
//----------------------------------------------------------------------------

// Inclusion lock
#ifndef _SERVO_H
#define _SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

//----------------------------------------------------------------------------
// INCLUDE FILES
//----------------------------------------------------------------------------

#include "global.h"
#include "math.h"

//---------------------------------------------------------------------------
// DEFINES
//---------------------------------------------------------------------------

#define SERVO_DELTA         0.018f

#define SERVO_0_COUNT_MIN    40000
#define SERVO_0_COUNT_MAX   200000
#define SERVO_0_SPAN        SERVO::D2RM( 195.0f )
#define SERVO_0_OFFSET      SERVO::D2R( 97.5f )
#define SERVO_0_REST        SERVO::D2R( 0.0f )

#define SERVO_1_COUNT_MIN    80000
#define SERVO_1_COUNT_MAX   180000
#define SERVO_1_SPAN        SERVO::D2RM( 152.5f  )
#define SERVO_1_OFFSET      SERVO::D2R(  59.5f )
#define SERVO_1_REST        SERVO::D2R( -59.5f )

#define SERVO_2_COUNT_MIN    53000
#define SERVO_2_COUNT_MAX   200000
#define SERVO_2_SPAN        SERVO::D2RM( 187.5f )
#define SERVO_2_OFFSET      SERVO::D2R(  154.5f )
#define SERVO_2_REST        SERVO::D2R( -154.5f )

#define SERVO_3_COUNT_MIN    40000
#define SERVO_3_COUNT_MAX   200000
#define SERVO_3_SPAN        SERVO::D2RM( 195.0f )
#define SERVO_3_OFFSET      SERVO::D2R( 97.5f )
#define SERVO_3_REST        SERVO::D2R( -44.85f )

#define SERVO_4_COUNT_MIN    40000
#define SERVO_4_COUNT_MAX   200000
#define SERVO_4_SPAN        SERVO::D2RM( 195.0f )
#define SERVO_4_OFFSET      SERVO::D2R( 105.5f )
#define SERVO_4_REST        SERVO::D2R( 0.0f )

#define SERVO_5_COUNT_MIN    70000
#define SERVO_5_COUNT_MAX   190000
#define SERVO_5_SPAN        SERVO::D2RM( 195.0f )
#define SERVO_5_OFFSET      SERVO::D2R( 97.5f )
#define SERVO_5_REST        SERVO::D2R( 0.0f )


//---------------------------------------------------------------------------
// CLASS DECLARATIONS
//---------------------------------------------------------------------------

class SERVO
{
    float m_fTime;
    float m_fDuration;
    float m_fDelta;

    uint32_t m_uiCountMin;
    uint32_t m_uiCountMax;

    float m_fPosStart;
    float m_fPosCurrent;
    float m_fPosEnd;

    float m_fAngleSpan;
    float m_fAngleOffset;

public:

    SERVO( void );
    ~SERVO();

    bool  Initialize( float fDelta, uint32_t uiCountMin, uint32_t uiCountMax, float fAngleSpan, float fAngleOffset, float fAngleInitial );

    uint32_t GetCount( void );

    bool  SetPosition( float fPosition, float fDuration );
    float GetPosition( void );
    bool  StepPosition( void );

    bool  SetAngle( float fAngle, float fDuration );
    float GetAngle( void );
    bool  StepAngle( void );

    bool  AngleToPosition( float fAngle, float *pfPosition );
    bool  PositionToAngle( float fPosition, float *pfAngle );

    static float D2R( float d );
    static float D2RM( float d );
    static float R2D( float r );
};

#ifdef __cplusplus
}
#endif

#endif // _SERVO_H

//---------------------------------------------------------------------------
// END SERVO.H
//---------------------------------------------------------------------------
