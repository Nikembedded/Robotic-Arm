//----------------------------------------------------------------------------
// COMPANY      : Confederation College
// FILE         : CONTACT.H
// FILE VERSION : 1.0
// PROGRAMMER   : Nikhil Premjani
//----------------------------------------------------------------------------
// REVISION HISTORY
//----------------------------------------------------------------------------
//
// 1.0, 2024-06-06, Nikhil Premjani
//   - Initial release
//
//----------------------------------------------------------------------------
// INCLUSION LOCK
//----------------------------------------------------------------------------

#ifndef CONTACT_H_
#define CONTACT_H_

#ifdef __cplusplus
extern "C" {
#endif

//----------------------------------------------------------------------------
// INCLUDE FILES
//----------------------------------------------------------------------------

#include "global.h"

//----------------------------------------------------------------------------
// CONSTANTS
//----------------------------------------------------------------------------

typedef struct tagCONTACT
{
    uint32_t    uiReload;
    uint32_t    uiCounter;

    uint32_t    bCurrInput  :  1;
    uint32_t    bPrevInput  :  1;
    uint32_t    bEvent      :  1;
    uint32_t    bOutput     :  1;
    uint32_t    bReserved   : 28;

} CONTACT;

//----------------------------------------------------------------------------
// FUNCTION PROTOTYPES
//----------------------------------------------------------------------------

void CONTACT_Init( CONTACT *pContact, uint32_t uiReload, bool bInitialState );
bool CONTACT_Sample( CONTACT *pContact, bool bInput );

#ifdef __cplusplus
}
#endif

#endif // CONTACT_H_

//----------------------------------------------------------------------------
// END CONTACT.H
//----------------------------------------------------------------------------
