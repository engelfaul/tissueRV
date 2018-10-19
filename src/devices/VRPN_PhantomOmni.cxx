#include "VRPN_PhantomOmni.h"

// -------------------------------------------------------------------------
VRPN_Device_Callback_Macro( VRPN_PhantomOmni, Tracker, vrpn_TRACKERCB );
// -------------------------------------------------------------------------
VRPN_PhantomOmni::
VRPN_PhantomOmni( unsigned int B )
  : Superclass( B )
{
}

// -------------------------------------------------------------------------
VRPN_PhantomOmni::
~VRPN_PhantomOmni( )
{
}

// -------------------------------------------------------------------------
void VRPN_PhantomOmni::
linkHandlers( )
{
  this->m_Device.Tracker->register_change_handler(
    this,
    VRPN_Device_Callback_Name_Macro( VRPN_PhantomOmni, Tracker )
    );
}

// eof - $RCSfile$
