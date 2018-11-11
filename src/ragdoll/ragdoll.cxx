#include <iostream>
#include <pujOgreBullet/Application.h>

#include <OgreManualObject.h>
#include <OgreCamera.h>
#include <OgreEntity.h>
#include <OgreMeshManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreMesh.h>
#include <OgreSubMesh.h>
#include <OgreRay.h>  
#include <OgrePlane.h>
#include <OgreMeshManager.h>
#include <OgreColourValue.h>
#include <OgreRoot.h>
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"

#include "../softObject/SceneSoftObject.h" 

//include vrpn descomentariar cuando se tenga instalado
#include "../devices/VRPN_Device.h"
#include "../devices/VRPN_PhantomOmni.h"
#include <OgreLogManager.h>

//libreias openhaptics  :O
#include <HD/hd.h>
//#include <HD/hdDevice.h>
#include <HDU/hduVector.h>


/**
 */
class RagDollApp
  : public pujOgreBullet::Application
{
public:
  typedef RagDollApp                 Self;
  typedef pujOgreBullet::Application Superclass;
  virtual void updateSimulation( const Ogre::Real& timeoffset );

public:
  RagDollApp( );
  virtual ~RagDollApp( );
  HHD hHD;
  Ogre::Vector3 direccion;
  Ogre::Vector3 posTool;
  typedef std::vector< VRPN_Device* > TDevices; 
//  Ogre::Vector3 *vertices;
//  Ogre::uint32 *indices;
//  size_t vertex_count;
//  size_t index_count;
  void setMesh();
  void createColourCube();
  void updateMesh(int trian );
  void UpdatePhysics(std::chrono::duration<double> deltaTime);
  void updateToolPosition ( const Ogre::FrameEvent& evt, double x, double y, double z );
  void updateCatheterPosition( );
  bool isCollisionDetectedBetweenExistingNodes( );
  void applyCollisionForceFeedback( );
  double getDx();
  double getDy();
  double getDz();	
protected:
  virtual void createScene( ) override;
  virtual void createCamera( ) override;
  virtual bool frameRenderingQueued( const Ogre::FrameEvent& evt ) override;
  bool keyPressed( const OIS::KeyEvent& arg ) override;
  TDevices m_Devices;  
  Ogre::AnimationState* m_AnimationState;

  //pos
  double dx,dy,dz;
};

// -------------------------------------------------------------------------
// Main
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
#endif

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
  INT WINAPI WinMain( HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT )
#else // OGRE_PLATFORM == OGRE_PLATFORM_WIN32
    int main( int argc, char* argv[] )
#endif // OGRE_PLATFORM == OGRE_PLATFORM_WIN32
  {
    // Create application object
    RagDollApp app;
    try
    {
      app.go( );
    }
    catch( Ogre::Exception& e )
    {
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
      MessageBox( NULL, e.getFullDescription().c_str(), "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else // OGRE_PLATFORM == OGRE_PLATFORM_WIN32
      std::cerr
        << "An exception has occured: "
        << e.getFullDescription( ).c_str( )
        << std::endl;
#endif // OGRE_PLATFORM == OGRE_PLATFORM_WIN32
    } // yrt
    return( 0 );
  }
#ifdef __cplusplus
}
#endif // __cplusplus

// ----------------------------------------------

// ----------------------------------------------------------------------------------
RagDollApp::
RagDollApp( )
  : Superclass( )
{
}

// -------------------------------------------------------------------------
RagDollApp::
~RagDollApp( )
{
}

// -------------------------------------------------------------------------
void RagDollApp::
createScene( )
{
  this->Superclass::createScene( );


 // Create a plane
  Ogre::Plane plane( Ogre::Vector3::UNIT_Y, 0 );
  Ogre::MeshManager::getSingleton( ).
    createPlane(
      "plane",
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      plane,
      150, 150, 20, 20, true, 1, 5, 5,
      Ogre::Vector3::UNIT_Z
      );

  // Associate a floor entity with the created plane
  Ogre::Entity* floor = this->m_SceneMgr->createEntity( "floor", "plane" );
  floor->setMaterialName( "Mat2" );
  this->m_SceneMgr->getRootSceneNode( )->attachObject( floor );

  // Associate plane to the physical world
  this->addPhysicsPlane( plane, "plane_physics", 0.000001, 0.00001 );


 

//prueba clase manual object
Ogre::ManualObject* man = this->m_SceneMgr->createManualObject("test");
man->begin("Mat", Ogre::RenderOperation::OT_TRIANGLE_LIST);

int numeroVertices = 20;
//int largo = 10;
//double resx = numeroVertices/largo;
for(int n=0; n<numeroVertices;n++){
     for(int m=0; m<numeroVertices;m++){
       std::cout << "vertice:  "<< m << " "<< n << "\n" << std::endl;
        man->position(m, 10, n);
        man->normal(0, 1, 0);
     } 
  }


// https://forums.ogre3d.org/viewtopic.php?t=32747 revisar
    
    int totalVertices = numeroVertices * numeroVertices;

    for(int o=0;o<(totalVertices-numeroVertices);o+=numeroVertices){
      for(int p=0;p<numeroVertices-1;p++){
        //se ajusta la forma en que se arman los triangulos para que queden igual como los arma bullet
        if((o+p)&1){
          man->triangle(o+p,o+numeroVertices+p,o+numeroVertices+p+1);
          man->triangle(o+p,o+numeroVertices+p+1,o+1+p); 
        }else{
          man->triangle(o+1+p,o+p,o+numeroVertices+p);
          man->triangle(o+1+p,o+numeroVertices+p,o+numeroVertices+p+1);
        }   
      }
    }

    man->end();
    //Ogre::SceneNode* manNode = this->m_SceneMgr->getRootSceneNode()->createChildSceneNode();
    //manNode->attachObject(man); 

    Ogre::MeshPtr meshTest = man->convertToMesh("test");
    //Agregar el objeto blando al mundo geometrico y al mundo fisico
    Ogre::Entity* thisEntity = this->m_SceneMgr->createEntity("pielEnity", "test");
      //thisEntity->setMaterialName("Mat");
      Ogre::AxisAlignedBox bboxSoft = thisEntity->getBoundingBox( );
      Ogre::SceneNode* thisSceneNode = this->m_SceneMgr->getRootSceneNode()->createChildSceneNode("piel_node");
      //thisSceneNode->setPosition(0, 5, 0);
      thisSceneNode->attachObject(thisEntity);
      std::cout << "nombre de la entidad:  "<< thisEntity->getName() << "\n" << std::endl;

    //Agregar el objeto blando al mundo fisico
      Ogre::Quaternion qsoft( 1, 1, 1, 1 );
      qsoft.normalise( );
      this->addSoftPhysicsTrimesh(
        thisEntity, thisSceneNode, "soft_physics", 0.0009, 0.0009, 10,
        Ogre::Vector3( 0, 0, 0 ),
        qsoft
        );

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Lights
  this->m_SceneMgr->setAmbientLight( Ogre::ColourValue( 1, 1, 1 ) );
  this->m_SceneMgr->setShadowTechnique( Ogre::SHADOWTYPE_STENCIL_ADDITIVE );

  Ogre::Light* light1 = this->m_SceneMgr->createLight( "Light1" );
  light1->setType( Ogre::Light::LT_POINT );
  light1->setPosition( Ogre::Vector3( 50, 50, 50 ) );
  light1->setDiffuseColour( 1.0, 1.0, 1.0 );
  light1->setSpecularColour( 1.0, 1.0, 1.0 );

  Ogre::Light* light2 = this->m_SceneMgr->createLight( "Light2" );
  light2->setType( Ogre::Light::LT_POINT );
  light2->setPosition( Ogre::Vector3( 0, 50, -50 ) );
  light2->setDiffuseColour( 0.5, 0.5, 0.5 );
  light2->setSpecularColour( 0.5, 0.5, 0.5 );

 
// -------------------------------------------------------
/* 
  // Load model entity
  Ogre::Entity* ninja =
    this->m_SceneMgr->createEntity(
      "ninja", "cyborg_ninja.mesh"
      );
  ninja->setCastShadows( true );
  Ogre::AxisAlignedBox bbox = ninja->getBoundingBox( );

  // Associate it to a node
  Ogre::SceneNode* ninja_node =
    this->m_SceneMgr->getRootSceneNode( )->createChildSceneNode(
      "ninja_node"
      );
  ninja_node->attachObject( ninja );
  ninja_node->translate(5,10,5);
  */

  /* el objeto cargado de blender no se deja actualizar por el objeto de bullet debido a que viene por defecto con el parametro de sharevertex en true y debe ser false
  //////////Objeto piel
    // Load model entity
  Ogre::Entity* tissue =
    this->m_SceneMgr->createEntity(
      "tissue", "Tissue.mesh"
      );
  tissue->setCastShadows( true );
  Ogre::AxisAlignedBox bbox3 = tissue->getBoundingBox( );
  tissue->setMaterialName( "Mat" );
  // Associate it to a node
  Ogre::SceneNode* tissue_node =
    this->m_SceneMgr->getRootSceneNode( )->createChildSceneNode(
      "tissue_node"
      );
  tissue_node->attachObject( tissue );
  tissue_node->translate(0,1,0);
  setMesh();   

    //Agregar el objeto blando al mundo fisico
      Ogre::Quaternion qsoft2( 1, 1, 1, 1 );
      qsoft2.normalise( );
      this->addSoftPhysicsTrimesh(
        tissue, tissue_node, "tiss_physics", 0.0009, 0.0009, 10,
        Ogre::Vector3( 0, 10, 0 ),
        qsoft2
        );
*/


//Se puede transformar el objeto piel en ogre o en blender (para blender recordar aplicar con ctrl + A)

  ////////////////////////////Objeto herramienta/////////////////////////////////////
    // Load model entity
  Ogre::Entity* tool =
    this->m_SceneMgr->createEntity(
      "tool", "tool2.mesh"
      );
  tool->setMaterialName( "Mat3" );    
  tool->setCastShadows( true );
  Ogre::AxisAlignedBox bbox4 = tool->getBoundingBox( );
  
  // Associate it to a node
  Ogre::SceneNode* tool_node =
    this->m_SceneMgr->getRootSceneNode( )->createChildSceneNode(
      "tool_node"
      );
  tool_node->attachObject( tool );
  //tool_node->scale(2,2,2);
  tool_node->translate(0,0,0);

  posTool   = Ogre::Vector3(0,15,0); 
  direccion = Ogre::Vector3(0,14,0);

// Associate tool to the physical world
  Ogre::Quaternion qTool( 5, 1, 1, 1 );
  qTool.normalise( );
    // Associate ninja to the physical world
  //addPhysicsCylinder addPhysicsConvex  addPhysicsSphere addRigidPhysicsTrimesh
  
  this->addPhysicsSphere(
    tool, tool_node, "tool_physics", 0.0009, 0.0009, 0,
    Ogre::Vector3( 0, 0, 0 ),
    qTool
    );

/////////////////////////////////////////////////////////////////////////////////////
/*
  ////////////////////////////Objeto de prueba/////////////////////////////////////
    // Load model entity
  Ogre::Entity* pruebaEntity =
    this->m_SceneMgr->createEntity(
      "pruebaEntity", "Sphere.mesh"
      );
  pruebaEntity->setMaterialName( "Mat3" );    
  pruebaEntity->setCastShadows( true );
  Ogre::AxisAlignedBox bbox5 = pruebaEntity->getBoundingBox( );
  
  // Associate it to a node
  Ogre::SceneNode* test_node =
    this->m_SceneMgr->getRootSceneNode( )->createChildSceneNode(
      "test_node"
      );
  test_node->attachObject( pruebaEntity );
  //tool_node->scale(2,2,2);
  test_node->translate(8,30,8);

// Associate tool to the physical world
  Ogre::Quaternion qTest( 5, 1, 1, 1 );
  qTest.normalise( );
    // Associate ninja to the physical world
  //addPhysicsCylinder addPhysicsConvex  addPhysicsSphere addRigidPhysicsTrimesh
  
  this->addPhysicsSphere(
    pruebaEntity, test_node, "test_physics", 1, 1, 20,
    Ogre::Vector3( 8, 30, 8 ),
    qTest
    );
*/
/////////////////////////////////////////////////////////////////////////////////////

/*
  // Associate ninja to the physical world
  Ogre::Quaternion q( 1, 1, 2, 3 );
  q.normalise( );
  this->addPhysicsConvex(
    ninja, ninja_node, "ninja_physics", 0.0009, 0.0009, 75,
    Ogre::Vector3( -10, 5, -10 ),
    q
    );

  */  

  //conectando phantom
  //creando phantom, OJO: correr el archivo vrpn_server de la maquina del servidor de  VRPN/vrpn-build/Debug
  VRPN_PhantomOmni* phantom0 = new VRPN_PhantomOmni( );

  if( !( phantom0->connect( "Phantom@10.3.137.56" ) ) )
  {
    Ogre::LogManager::getSingletonPtr( )->
      logMessage( "!!! Error connecting to Phantom@host !!!" );
    delete phantom0;
    phantom0 = NULL;
  }
  this->m_Devices.push_back( phantom0 );

  hHD = hdInitDevice("Phantom");

  hdEnable(3000);
  hdStartScheduler();
  //hdMakeCurrentDevice(hHD);


}

// -------------------------------------------------------------------------
void RagDollApp::
createCamera( )
{
  this->Superclass::createCamera( );
  this->m_Camera->setPosition( Ogre::Vector3( 10, 30, 40 ) );
  this->m_Camera->lookAt( Ogre::Vector3( 10, 10, 0 ) );
  this->m_Camera->setNearClipDistance( 5 );
}




bool RagDollApp::
keyPressed( const OIS::KeyEvent& arg )
{
  
  
  OIS::KeyCode a = arg.key;
  ///moviendo la camara
  //this->m_Camera->setPosition( Ogre::Vector3( 25, 25, 25 ) );
    
   // if( this->m_Keyboard->isKeyDown( OIS::KC_LSHIFT ) )
    //  moveScale *= 10;
    Ogre::Vector3 posCamera = this->m_Camera->getPosition();
    if( this->m_Keyboard->isKeyDown( OIS::KC_J ) ) {
        // Move camera left
      posCamera.x = posCamera.x + 1;
      this->m_Camera->setPosition( posCamera );
      }

    if( this->m_Keyboard->isKeyDown( OIS::KC_L ) ){
      posCamera.x = posCamera.x - 1;
      this->m_Camera->setPosition( posCamera );  // Move camera right
    }
      

    if(this->m_Keyboard->isKeyDown( OIS::KC_I )){
      posCamera.z = posCamera.z + 0.5;
      this->m_Camera->setPosition( posCamera );// Move camera forward
    }
      

    if(this->m_Keyboard->isKeyDown( OIS::KC_K )){
      posCamera.z = posCamera.z - 0.5;
      this->m_Camera->setPosition( posCamera );  // Move camera backward
    }
      

    if( this->m_Keyboard->isKeyDown( OIS::KC_Y ) ){
      posCamera.y = posCamera.y + 0.5;
      this->m_Camera->setPosition( posCamera );  // Move camera up
    }
      

    if( this->m_Keyboard->isKeyDown( OIS::KC_H ) ){
      posCamera.y = posCamera.y - 0.5;
      this->m_Camera->setPosition( posCamera ); // Move camera down
    }
      
   Ogre::Degree rotate = Ogre::Degree(1);
  
    if( this->m_Keyboard->isKeyDown( OIS::KC_RIGHT ) ){
      std::cout <<"rotando derecha "<< "\n";
      this->m_Camera->yaw( -rotate );
    }
      

    if( this->m_Keyboard->isKeyDown( OIS::KC_LEFT ) ){
      std::cout <<"rotando izquierda "<< "\n";
      this->m_Camera->yaw( rotate );
    }
       

    if(
      this->m_Keyboard->isKeyDown( OIS::KC_ESCAPE ) ||
      this->m_Keyboard->isKeyDown( OIS::KC_Q )
      )
      return( false );


  //moviendo el instrumento
  Ogre::SceneNode* planeBlender_node = this->m_SceneMgr->getSceneNode("tool_node");
  float dx1  = 0;
  float dy1  = 0;
  float dz1  = 0;
  if(OIS::KC_W==a){
    dx1 = 0.1;
  }
  
  if(OIS::KC_S==a){
    dx1 = -0.1;
  }

  if(OIS::KC_A==a){
    dz1 = -0.1;
  }

  if(OIS::KC_D==a){
    dz1 = 0.1;
  }

  if(OIS::KC_UP==a){
    dy1 = 0.1;
  }

  if(OIS::KC_DOWN==a){
    dy1 = -0.1;
  } 
    planeBlender_node->translate(dx1,dy1,dz1);
    Ogre::Vector3 vpos = Ogre::Vector3(dx1,dy1,dz1);
    this->updatePositionBullet(vpos,planeBlender_node); //pendiente verificar esto, la punta no colisiona


  return( true );
}






bool RagDollApp::
frameRenderingQueued( const Ogre::FrameEvent& evt )
{

  

  if( this->m_Devices[ 0 ] != NULL ) {
    VRPN_PhantomOmni* phantom = dynamic_cast< VRPN_PhantomOmni* >( this->m_Devices[ 0 ] );
    VRPN_PhantomOmni_State phantom_state = phantom->capture( );
    //std::cout << "actualizando !!! " <<std::endl;
      //hduVector3Dd feedbackForce(5 , 5, 5 );
   hduVector3Dd (40,40.0,17.0);
    this->updateToolPosition( evt, phantom_state.Position[0], phantom_state.Position[1], phantom_state.Position[2] );

      
  }

  std::chrono::duration<double> deltaTime; 
  deltaTime = static_cast<std::chrono::duration<double>>(evt.timeSinceLastFrame);
    if( this->pujOgre::Application::frameRenderingQueued( evt ) )
  {
    //std::cout <<"Loop: " << evt.timeSinceLastFrame <<"\n";
    UpdatePhysics(deltaTime);
    //this->m_AnimationState->addTime( evt.timeSinceLastFrame );
    return( true );
  }
  else
    return( false );
}


void RagDollApp::UpdatePhysics(std::chrono::duration<double> deltaTime) {
  //std::cout <<"Loop: " << deltaTime.count() <<"\n";
}

void RagDollApp::updateToolPosition ( const Ogre::FrameEvent& evt, double x, double y, double z ){
  this->dx = x; this->dy = y; this->dz = z;
  //std::cout <<"Loop: " << deltaTime.count() <<"\n";
  

  if(this->isCollisionDetectedBetweenExistingNodes()){
    this->applyCollisionForceFeedback();
  }else {
    std::cout <<"Sin fuerza " <<"\n";

  }
  Ogre::Real timeoffset = evt.timeSinceLastFrame;
 
  this->updateSimulation( timeoffset );
  
}

void RagDollApp::
updateSimulation( const Ogre::Real& timeoffset )
{
  //this->m_AnimationState->addTime( timeoffset );
  this->updateCatheterPosition();
  //this->updateVesselDeformations();
}

void RagDollApp::
updateCatheterPosition( ) {


  const unsigned int movementScale = 70;
  const unsigned int movementTranslate = 10;
  
	double movx = (movementScale * this->dx)+movementTranslate;
	double movy = (movementScale * this->dy)+movementTranslate;
	double movz = (movementScale * this->dz)+movementTranslate;

  
	Ogre::SceneNode* catheterNode = this->m_SceneMgr->getSceneNode("tool_node");
  
  Ogre::Vector3 befpos = catheterNode->getPosition();
  
  catheterNode->setPosition( movx, movy, movz );
  Ogre::Vector3 vpos = Ogre::Vector3(movx,movy,movz);
  Ogre::Vector3 transObject = vpos-befpos;
  this->updatePositionBullet(transObject,catheterNode); //sincronizacion
 
}

bool RagDollApp::
isCollisionDetectedBetweenExistingNodes( ){

  Ogre::SceneNode* catheterNode = this->m_SceneMgr->getSceneNode("tool_node");
  Ogre::SceneNode* vesselNode = this->m_SceneMgr->getSceneNode("piel_node");
  return ( this->isCollisionDetected( catheterNode, vesselNode ) );
  //return true;
}

void RagDollApp::
applyCollisionForceFeedback( ){
  //retroalimentacion
  
  HDErrorInfo error;
  std::cout <<"Devolviendo fuerza: " <<"\n";
  //hduVector3Dd feedbackForce(0.0,0.95,0.68);
  //std::cout <<"HD_CURRENT_FORCE: " <<HD_CURRENT_FORCE<<"\n";
  //hdSetDoublev ( HD_CURRENT_FORCE, feedbackForce );
  //HDdouble baseTorque[3] = {100, 250, 200}; //Base Torque in mNm
  //hdSetDoublev(HD_CURRENT_JOINT_TORQUE, baseTorque );
  hdMakeCurrentDevice(hHD);
  std::cout <<"Device numero: "<< hdGetCurrentDevice() <<"\n";
   
   
  // hdBeginFrame(0);

  

    HDdouble force[3] = {100, 107, 100};
    hdGetDoublev(3000,force);
  //hdEndFrame(0);

  //hdStopScheduler();
  //hdUnschedule(scheduleCallbackHandle);
  //hdDisableDevice(hdGetCurrentDevice());
    
 

}

// eof - $RCSfile$
 