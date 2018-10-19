#ifndef __VRPN_5DTGLOVE14__H__
#define __VRPN_5DTGLOVE14__H__

#include "VRPN_Device.h"

/**
 */
class ogre_EXPORT VRPN_5dtGlove14_State
{
public:
  float Channel[ 14 ];
  void setAnalog( const vrpn_ANALOGCB* a )
    {
      for( unsigned int i = 0; i < 14; i++ )
        this->Channel[ i ] = a->channel[ i ];
    }
  void clear( )
    {
      for( unsigned int i = 0; i < 14; i++ )
        this->Channel[ i ] = float( 0 );
    }
};

/**
 */
class ogre_EXPORT VRPN_5dtGlove14
  : public VRPN_DeviceBase< VRPN_5dtGlove14_State >
{
public:
  typedef VRPN_5dtGlove14                          Self;
  typedef VRPN_DeviceBase< VRPN_5dtGlove14_State > Superclass;

public:
  explicit VRPN_5dtGlove14( unsigned int B = 32 );
  virtual ~VRPN_5dtGlove14( );

  virtual const VRPN_5dtGlove14_State& capture( ) const;
  void setCalibration( float* min_values, float* max_values );

protected:
  virtual void linkHandlers( );

protected:
  float m_MinValues[ 14 ];
  float m_MaxValues[ 14 ];
};

#endif // __VRPN_5DTGLOVE14__H__

// eof - VRPN_5dtGlove14.h
