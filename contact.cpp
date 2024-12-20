//----------------------------------------------------------------------------
// COMPANY      : Confederation College
// FILE         : CONTACT.CPP
// FILE VERSION : 1.0
// PROGRAMMER   : Nikhil Premjani
//----------------------------------------------------------------------------
// REVISION HISTORY
//----------------------------------------------------------------------------
//
// 1.0, 2024-06-06, Programmer Name
//   - Initial release
//
//----------------------------------------------------------------------------
// MODULE DESCRIPTION
//----------------------------------------------------------------------------
//
// Code to support electro-mechanical contact inputs.
//
//----------------------------------------------------------------------------
// INCLUDE FILES
//----------------------------------------------------------------------------

#include "contact.h"

//----------------------------------------------------------------------------
// EXTERNAL REFERENCES
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// FUNCTION : CONTACT_Init( CONTACT *pContact, uint32_t uiReload, bool bInitialState )
// PURPOSE  : Initializes the contact interface
//----------------------------------------------------------------------------

void CONTACT_Init( CONTACT *pContact, uint32_t uiReload, bool bInitialState )
{
    pContact->bCurrInput = bInitialState;
    pContact->bPrevInput = bInitialState;

    pContact->uiCounter = 0;
    pContact->uiReload  = uiReload;

    pContact->bEvent  = 0;
    pContact->bOutput = bInitialState;

    pContact->bReserved = 0;

    return;
}

//----------------------------------------------------------------------------
// FUNCTION : CONTACT_Sample( CONTACT *pContact, bool bInput )
// PURPOSE  : Samples the specified contact input
//----------------------------------------------------------------------------

bool CONTACT_Sample( CONTACT *pContact, bool bInput )
{
    pContact->bCurrInput = bInput;

    if( pContact->bCurrInput ^ pContact->bPrevInput )
    {
        // Change detected from previous sample � reload counter
        pContact->uiCounter = pContact->uiReload;
    }
    else
    {
        // No change from previous sample
        if( pContact->uiCounter )
        {
            pContact->uiCounter--;
        }
    }
    // Prepare for next sample
    pContact->bPrevInput = pContact->bCurrInput;

    // Update the event and output
    pContact->bEvent  = pContact->uiCounter == 1;
    pContact->bOutput = pContact->bEvent ? pContact->bCurrInput : pContact->bOutput;

    return pContact->bEvent;
}

//----------------------------------------------------------------------------
// END CONTACT.CPP
//----------------------------------------------------------------------------
