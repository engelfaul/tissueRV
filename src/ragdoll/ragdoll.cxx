#include <iostream>
#include <pujOgreBullet/Application.h>

#include <OgreCamera.h>
#include <OgreEntity.h>
#include <OgreMeshManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

/**
 */
class RagDollApp
  : public pujOgreBullet::Application
{
public:
  typedef RagDollApp                 Self;
  typedef pujOgreBullet::Application Superclass;

public:
  RagDollApp( );
  virtual ~RagDollApp( );

protected:
  virtual void createScene( ) override;
  virtual void createCamera( ) override;
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

// -------------------------------------------------------------------------
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
  floor->setMaterialName( "Mat" );
  this->m_SceneMgr->getRootSceneNode( )->attachObject( floor );

  // Associate plane to the physical world
  this->addPhysicsPlane( plane, "plane_physics", 0.000001, 0.00001 );

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

  // Associate ninja to the physical world
  Ogre::Quaternion q( 1, 1, 2, 3 );
  q.normalise( );
  this->addSoftPhysicsTrimesh(
    ninja, ninja_node, "ninja_physics", 0.0009, 0.0009, 75,
    Ogre::Vector3( 0, -bbox.getMinimum( )[ 1 ] * 2, 0 ),
    q
    );
}

// -------------------------------------------------------------------------
void RagDollApp::
createCamera( )
{
  this->Superclass::createCamera( );
  this->m_Camera->setPosition( Ogre::Vector3( 25, 25, 25 ) );
  this->m_Camera->lookAt( Ogre::Vector3( 0, 10, 0 ) );
  this->m_Camera->setNearClipDistance( 5 );
}

// eof - $RCSfile$
