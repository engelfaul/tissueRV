#include "VRPN_WiiMote.h"

// -------------------------------------------------------------------------
VRPN_Device_Callback_Macro( VRPN_WiiMote, Analog, vrpn_ANALOGCB );
VRPN_Device_Callback_Macro( VRPN_WiiMote, Button, vrpn_BUTTONCB );

// -------------------------------------------------------------------------
VRPN_WiiMote::
VRPN_WiiMote( unsigned int B )
  : Superclass( B, VRPN_WiiMote_State::NumberOfButtons )
{
}

// -------------------------------------------------------------------------
VRPN_WiiMote::
~VRPN_WiiMote( )
{
}

// -------------------------------------------------------------------------
void VRPN_WiiMote::
linkHandlers( )
{
  this->m_Device.Analog->register_change_handler(
    this,
    VRPN_Device_Callback_Name_Macro( VRPN_WiiMote, Analog )
    );
  this->m_Device.Button->register_change_handler(
    this,
    VRPN_Device_Callback_Name_Macro( VRPN_WiiMote, Button )
    );
}

// eof - VRPN_WiiMote.cxx
