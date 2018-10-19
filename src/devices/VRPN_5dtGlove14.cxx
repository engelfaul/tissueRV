#include <algorithm>
#include "VRPN_5dtGlove14.h"

// -------------------------------------------------------------------------
VRPN_Device_Callback_Macro( VRPN_5dtGlove14, Analog, vrpn_ANALOGCB );

// -------------------------------------------------------------------------
VRPN_5dtGlove14::
VRPN_5dtGlove14( unsigned int B )
  : Superclass( B )
{
  for( unsigned int c = 0; c < 14; c++ )
  {
    this->m_MinValues[ c ] = float( 0 );
    this->m_MaxValues[ c ] = float( 1 );

  } // rof
}

// -------------------------------------------------------------------------
VRPN_5dtGlove14::
~VRPN_5dtGlove14( )
{
}

// -------------------------------------------------------------------------
const VRPN_5dtGlove14_State& VRPN_5dtGlove14::
capture( ) const
{
  for( unsigned int c = 0; c < 14; c++ )
  {
    float v = this->m_CurrentState.Channel[ c ];
    v -= this->m_MinValues[ c ];
    v /= this->m_MaxValues[ c ] - this->m_MinValues[ c ];
    if( v < 0 ) v = 0;
    if( v > 1 ) v = 1;
    this->m_CurrentState.Channel[ c ] = v;

  } // rof

  return( this->Superclass::capture( ) );
}

// -------------------------------------------------------------------------
void VRPN_5dtGlove14::
setCalibration( float* min_values, float* max_values )
{
  std::copy( min_values, min_values + 14, this->m_MinValues );
  std::copy( max_values, max_values + 14, this->m_MaxValues );
}

// -------------------------------------------------------------------------
void VRPN_5dtGlove14::
linkHandlers( )
{
  this->m_Device.Analog->register_change_handler(
    this,
    VRPN_Device_Callback_Name_Macro( VRPN_5dtGlove14, Analog )
    );
}

// eof - VRPN_5dtGlove14.cxx
