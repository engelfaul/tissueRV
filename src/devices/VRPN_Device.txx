#ifndef __VRPN_DEVICE__TXX__
#define __VRPN_DEVICE__TXX__

// -------------------------------------------------------------------------
template< class S >
VRPN_DeviceBase< S >::
VRPN_DeviceBase( unsigned int B, unsigned short N )
  : m_Mutex( NULL ),
    m_Thread( NULL ),
    m_ThreadRunning( false ),
    m_MaxBufferSize( B )
{
  this->m_Mutex = new boost::mutex( );

  if( N > 0 && N <= 64 )
  {
    this->m_ButtonMasks = new unsigned long[ N ];
    unsigned long mask = 1;
    for( unsigned short n = 1; n <= N; n++ )
    {
      this->m_ButtonMasks[ n - 1 ] = mask;
      mask <<= 1;

    } // rof
  }
  else
    this->m_ButtonMasks = NULL;
}

// -------------------------------------------------------------------------
template< class S >
VRPN_DeviceBase< S >::
~VRPN_DeviceBase( )
{
  this->killThread( );
  delete this->m_Mutex;
  if( this->m_ButtonMasks != NULL )
    delete this->m_ButtonMasks;
}

// -------------------------------------------------------------------------
template< class S >
bool VRPN_DeviceBase< S >::
connect( const std::string& url )
{
  // Create VRPN connection
  this->m_Device.Name    = url;
  this->m_Device.Tracker = new vrpn_Tracker_Remote( url.c_str( ) );
  this->m_Device.Button  = new vrpn_Button_Remote( url.c_str( ) );
  this->m_Device.Analog  = new vrpn_Analog_Remote( url.c_str( ) );
  this->m_Device.Dial    = new vrpn_Dial_Remote( url.c_str( ) );
  this->m_Device.Text    = new vrpn_Text_Receiver( url.c_str( ) );
  if(
    this->m_Device.Tracker == NULL ||
    this->m_Device.Button  == NULL ||
    this->m_Device.Analog  == NULL ||
    this->m_Device.Dial    == NULL ||
    this->m_Device.Text    == NULL 
    )
    return( false );

  // Initialize values
  this->clearBuffer( );
  this->m_CurrentState.clear( );

  // Link handlers & launch thread
  this->linkHandlers( );
  this->createThread( );

  return( true );
}

// -------------------------------------------------------------------------
template< class S >
void VRPN_DeviceBase< S >::
disconnect( )
{
  this->killThread( );
}

// -------------------------------------------------------------------------
template< class S >
const S& VRPN_DeviceBase< S >::
capture( ) const
{
  return( this->m_CurrentState );
}

// -------------------------------------------------------------------------
template< class S >
typename VRPN_DeviceBase< S >::
TBuffer& VRPN_DeviceBase< S >::
buffer( )
{
  return( this->m_Buffer );
}

// -------------------------------------------------------------------------
template< class S >
bool VRPN_DeviceBase< S >::
isBufferFull( ) const
{
  return( this->m_Buffer.size( ) > this->m_MaxBufferSize );
}

// -------------------------------------------------------------------------
template< class S >
void VRPN_DeviceBase< S >::
clearBuffer( )
{
  while( !( this->m_Buffer.empty( ) ) )
    this->m_Buffer.pop( );
}

// -------------------------------------------------------------------------
template< class S >
void VRPN_DeviceBase< S >::
killThread( )
{
  if( this->m_ThreadRunning )
  {
    this->m_ThreadRunning = false;
    this->m_Thread->join( );
    delete this->m_Thread;
    this->m_Thread = NULL;

  } // fi
}

// -------------------------------------------------------------------------
template< class S >
void VRPN_DeviceBase< S >::
createThread( )
{
  if( !this->m_ThreadRunning )
  {
    this->m_ThreadRunning = true;
    this->m_Thread =
      new boost::thread( boost::bind( &Self::updateThread, this ) );

  } // fi
}

// -------------------------------------------------------------------------
template< class S >
bool VRPN_DeviceBase< S >::
updateThread( )
{
  bool cont = true;
  while( this->m_ThreadRunning && cont )
  {
    {
      boost::mutex::scoped_lock lock( *this->m_Mutex );
      cont = this->update( );
    }

  } // elihw
  return( cont );
}

// -------------------------------------------------------------------------
template< class S >
bool VRPN_DeviceBase< S >::
update( )
{
  this->m_Device.Tracker->mainloop( );
  this->m_Device.Button->mainloop( );
  this->m_Device.Analog->mainloop( );
  this->m_Device.Dial->mainloop( );
  this->m_Device.Text->mainloop( );

  // Sleep for 1ms so we don't eat the CPU
  vrpn_SleepMsecs( 1 );

  return( true );
}

// -------------------------------------------------------------------------
template< class S >
void VRPN_DeviceBase< S >::
hndAnalog( const void* data )
{
  const vrpn_ANALOGCB* ptr_a =
    reinterpret_cast< const vrpn_ANALOGCB* >( data );
  if( ptr_a == NULL )
    return;

  this->m_CurrentState.setAnalog( ptr_a );
  if( this->m_Buffer.size( ) > this->m_MaxBufferSize )
    this->m_Buffer.pop( );
  this->m_Buffer.push( this->m_CurrentState );
}

// -------------------------------------------------------------------------
template< class S >
void VRPN_DeviceBase< S >::
hndButton( const void* data )
{
  const vrpn_BUTTONCB* ptr_b =
    reinterpret_cast< const vrpn_BUTTONCB* >( data );
  if( ptr_b == NULL )
    return;
  vrpn_BUTTONCB b = *ptr_b;

  // WARNING: this depends on the button masks
  unsigned long btn = ( unsigned long )( b.button );
  if( b.state == 1 )
    this->m_CurrentState.PushedButtons |=  this->m_ButtonMasks[ btn ];
  else
    this->m_CurrentState.PushedButtons &= ~this->m_ButtonMasks[ btn ];

  if( this->m_Buffer.size( ) > this->m_MaxBufferSize )
    this->m_Buffer.pop( );
  this->m_Buffer.push( this->m_CurrentState );
}

// -------------------------------------------------------------------------
template< class S >
void VRPN_DeviceBase< S >::
hndTracker( const void* data )
{
  const vrpn_TRACKERCB* ptr_t =
    reinterpret_cast< const vrpn_TRACKERCB* >( data );
  if( ptr_t == NULL )
    return;

  this->m_CurrentState.setTracker(
    ptr_t->sensor,
    ptr_t->pos[ 0 ], ptr_t->pos[ 1 ], ptr_t->pos[ 2 ],
    ptr_t->quat[ 0 ], ptr_t->quat[ 1 ], ptr_t->quat[ 2 ], ptr_t->quat[ 3 ]
    );
}

#endif // __VRPN_DEVICE__TXX__

// eof - VRPN_Device.txx
