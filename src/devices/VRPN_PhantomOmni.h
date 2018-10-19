#ifndef __VRPN_PhantomOmni__h__
#define __VRPN_PhantomOmni__h__

#include "VRPN_Device.h"


  class ogre_EXPORT VRPN_PhantomOmni_State
  {
  public:
    unsigned int Id;
    float Position[ 3 ];
    float Rotation[ 4 ];

    void setTracker(
      unsigned int id,
      float px, float py, float pz,
      float qx, float qy, float qz, float qw
      )
    {
      this->Id = id;

      this->Position[ 0 ] = px;
      this->Position[ 1 ] = py;
      this->Position[ 2 ] = pz;

      this->Rotation[ 0 ] = qx;
      this->Rotation[ 1 ] = qy;
      this->Rotation[ 2 ] = qz;
      this->Rotation[ 3 ] = qw;
    }
    void clear( )
    {
      this->Id = 0;

      this->Position[ 0 ] = float( 0 );
      this->Position[ 1 ] = float( 0 );
      this->Position[ 2 ] = float( 0 );

      this->Rotation[ 0 ] = float( 0 );
      this->Rotation[ 1 ] = float( 0 );
      this->Rotation[ 2 ] = float( 0 );
      this->Rotation[ 3 ] = float( 0 );
    }
  };

  /**
   */
  class ogre_EXPORT VRPN_PhantomOmni
    : public VRPN_DeviceBase< VRPN_PhantomOmni_State >
  {
  public:
    typedef VRPN_PhantomOmni                 Self;
    typedef VRPN_DeviceBase< VRPN_PhantomOmni_State > Superclass;

  public:
    explicit VRPN_PhantomOmni( unsigned int B = 32 );
    virtual ~VRPN_PhantomOmni( );

  protected:
    virtual void linkHandlers( );
  };

#endif 
// __ogre__VRPN_PhantomOmni__h__

// eof - $RCSfile$
