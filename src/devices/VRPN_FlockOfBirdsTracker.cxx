#include "VRPN_FlockOfBirdsTracker.h"

// -------------------------------------------------------------------------
VRPN_Device_Callback_Macro(
  VRPN_FlockOfBirdsTracker, Tracker, vrpn_TRACKERCB
  );

// -------------------------------------------------------------------------
VRPN_FlockOfBirdsTracker::
VRPN_FlockOfBirdsTracker( unsigned int B )
  : Superclass( B )
{
}

// -------------------------------------------------------------------------
VRPN_FlockOfBirdsTracker::
~VRPN_FlockOfBirdsTracker( )
{
}

// -------------------------------------------------------------------------
void VRPN_FlockOfBirdsTracker::
linkHandlers( )
{
  this->m_Device.Tracker->register_change_handler(
    this,
    VRPN_Device_Callback_Name_Macro( VRPN_FlockOfBirdsTracker, Tracker )
    );
}

// eof - VRPN_FlockOfBirdsTracker.cxx
