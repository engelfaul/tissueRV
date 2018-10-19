#ifndef __VRPN_DEVICE__H__
#define __VRPN_DEVICE__H__

#include <queue>
#include <string>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <vrpn_Shared.h>
#include <vrpn_Tracker.h>
#include <vrpn_Button.h>
#include <vrpn_Analog.h>
#include <vrpn_Dial.h>
#include <vrpn_Text.h>

#include "../ogre_EXPORT.h"

// -------------------------------------------------------------------------
#define VRPN_Device_PushedButton( b )                                   \
  bool is##b##Pushed( ) const                                           \
  { return( ( this->PushedButtons & ( 1 << b ) ) == ( 1 << b ) ); }

// -------------------------------------------------------------------------
#define VRPN_Device_Callback_Macro( name, dev, type )                   \
  void VRPN_CALLBACK VRPN_Device_hnd##name##dev(                        \
    void* data, const type v                                            \
    )                                                                   \
  {                                                                     \
    name* devs = reinterpret_cast< name* >( data );                     \
    if( devs != NULL )                                                  \
      devs->hnd##dev( &v );                                             \
  }

// -------------------------------------------------------------------------
#define VRPN_Device_Callback_Name_Macro( name, dev )    \
  VRPN_Device_hnd##name##dev

/**
 */
class VRPN_Device
{
public:
  explicit VRPN_Device( unsigned int B = 32, unsigned short N = 0 )
    { }
  virtual ~VRPN_Device( )
    { }

  virtual bool connect( const std::string& url ) = 0;
  virtual void disconnect( ) = 0;
  virtual bool isBufferFull( ) const = 0;
  virtual void clearBuffer( ) = 0;
};

/**
 */
template< class S >
class VRPN_DeviceBase
  : public VRPN_Device
{
public:
  typedef VRPN_DeviceBase Self;
  typedef VRPN_Device     Superclass;

protected:
  typedef std::queue< S > TBuffer;
  struct TDevice
  {
    std::string          Name;
    vrpn_Tracker_Remote* Tracker;
    vrpn_Button_Remote*  Button;
    vrpn_Analog_Remote*  Analog;
    vrpn_Dial_Remote*    Dial;
    vrpn_Text_Receiver*  Text;
  };

public:
  explicit VRPN_DeviceBase( unsigned int B = 32, unsigned short N = 0 );
  virtual ~VRPN_DeviceBase( );

  bool connect( const std::string& url );
  void disconnect( );

  virtual const S& capture( ) const;
  TBuffer& buffer( );
  bool isBufferFull( ) const;
  void clearBuffer( );

protected:
  void killThread( );
  void createThread( );
  bool updateThread( );
  bool update( );

  virtual void linkHandlers( ) = 0;

public:
  /*
   * WARNING: do not call these methods by yourself, these are just
   *          wrappers.
   */
  void hndAnalog( const void* data );
  void hndButton( const void* data );
  void hndTracker( const void* data );

protected:
  boost::mutex*  m_Mutex;
  boost::thread* m_Thread;
  volatile bool  m_ThreadRunning;

  TDevice      m_Device;
  mutable S    m_CurrentState;
  TBuffer      m_Buffer;
  unsigned int m_MaxBufferSize;

  unsigned long* m_ButtonMasks;
};

#include "VRPN_Device.txx"

#endif // __VRPN_DEVICE__H__

// eof - VRPN_Device.h
