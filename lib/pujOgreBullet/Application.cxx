#include <iostream>

#include <pujOgreBullet/Application.h>

#include <OgreBulletCollisionsBoxShape.h>
#include <OgreBulletCollisionsConvexHullShape.h>
#include <OgreBulletCollisionsCylinderShape.h>
#include <OgreBulletCollisionsMeshToShapeConverter.h>
#include <OgreBulletCollisionsSphereShape.h>
#include <OgreBulletCollisionsStaticPlaneShape.h>
#include <OgreBulletCollisionsTrimeshShape.h>
#include <OgreBulletDynamicsRigidBody.h>
#include <OgreBulletDynamicsSoftBody.h>
#include <OgreBulletDynamicsWorld.h>

// -------------------------------------------------------------------------
pujOgreBullet::Application::
Application( )
  : Superclass( )
{
}

// -------------------------------------------------------------------------
pujOgreBullet::Application::
~Application( )
{
}

// -------------------------------------------------------------------------
void pujOgreBullet::Application::
createScene( )
{
  this->initBullet( );
}

// -------------------------------------------------------------------------
bool pujOgreBullet::Application::
frameStarted( const Ogre::FrameEvent& evt )
{
  
  if( this->Superclass::frameStarted( evt ) )
  {
    this->m_BulletWorld->stepSimulation( evt.timeSinceLastFrame );
    return( true );
  }
  else
    return( false );
}

// -------------------------------------------------------------------------
void pujOgreBullet::Application::
initBullet(
  const Ogre::Vector3& gravityVector,
  const Ogre::AxisAlignedBox& bounds
  )
{
  this->m_BulletWorld =
    new OgreBulletDynamics::DynamicsWorld(
      this->m_SceneMgr, bounds, gravityVector, true, true, 10000
      );
}

// -------------------------------------------------------------------------
void pujOgreBullet::Application::
addPhysicsPlane(
  const Ogre::Plane& plane,
  const std::string& name,
  float bodyRestitution,
  float bodyFriction
  )
{
  OgreBulletDynamics::RigidBody* body =
    new OgreBulletDynamics::RigidBody( name, this->m_BulletWorld );
  body->setStaticShape(
    new OgreBulletCollisions::StaticPlaneCollisionShape(
      plane.normal, plane.d
      ),
    bodyRestitution, bodyFriction
    );
}

// -------------------------------------------------------------------------
void pujOgreBullet::Application::
addPhysicsSphere(
  Ogre::Entity* entity,
  Ogre::SceneNode* node,
  const std::string& name,
  float bodyRestitution,
  float bodyFriction,
  float bodyMass,
  const Ogre::Vector3& position,
  const Ogre::Quaternion& orientation
  )
{
  OgreBulletCollisions::StaticMeshToShapeConverter* conv =
    new OgreBulletCollisions::StaticMeshToShapeConverter( entity );
  OgreBulletDynamics::RigidBody* body =
    new OgreBulletDynamics::RigidBody( name, this->m_BulletWorld );
  body->setShape(
    node, conv->createSphere( ),
    bodyRestitution, bodyFriction, bodyMass,
    position, orientation
    );
}

// -------------------------------------------------------------------------
void pujOgreBullet::Application::
addPhysicsBox(
  Ogre::Entity* entity,
  Ogre::SceneNode* node,
  const std::string& name,
  float bodyRestitution,
  float bodyFriction,
  float bodyMass,
  const Ogre::Vector3& position,
  const Ogre::Quaternion& orientation
  )
{
  OgreBulletCollisions::StaticMeshToShapeConverter* conv =
    new OgreBulletCollisions::StaticMeshToShapeConverter( entity );
  OgreBulletDynamics::RigidBody* body =
    new OgreBulletDynamics::RigidBody( name, this->m_BulletWorld );
  body->setShape(
    node, conv->createBox( ),
    bodyRestitution, bodyFriction, bodyMass,
    position, orientation
    );
}

// -------------------------------------------------------------------------
void pujOgreBullet::Application::
addPhysicsCylinder(
  Ogre::Entity* entity,
  Ogre::SceneNode* node,
  const std::string& name,
  float bodyRestitution,
  float bodyFriction,
  float bodyMass,
  const Ogre::Vector3& position,
  const Ogre::Quaternion& orientation
  )
{
  OgreBulletCollisions::StaticMeshToShapeConverter* conv =
    new OgreBulletCollisions::StaticMeshToShapeConverter( entity );
  OgreBulletDynamics::RigidBody* body =
    new OgreBulletDynamics::RigidBody( name, this->m_BulletWorld );
  body->setShape(
    node, conv->createCylinder( ),
    bodyRestitution, bodyFriction, bodyMass,
    position, orientation
    );
}

// -------------------------------------------------------------------------
void pujOgreBullet::Application::
addPhysicsConvex(
  Ogre::Entity* entity,
  Ogre::SceneNode* node,
  const std::string& name,
  float bodyRestitution,
  float bodyFriction,
  float bodyMass,
  const Ogre::Vector3& position,
  const Ogre::Quaternion& orientation
  )
{
  OgreBulletCollisions::StaticMeshToShapeConverter* conv =
    new OgreBulletCollisions::StaticMeshToShapeConverter( entity );
  OgreBulletDynamics::RigidBody* body =
    new OgreBulletDynamics::RigidBody( name, this->m_BulletWorld );
  body->setShape(
    node, conv->createConvex( ),
    bodyRestitution, bodyFriction, bodyMass,
    position, orientation
    );
}

// -------------------------------------------------------------------------
void pujOgreBullet::Application::
addRigidPhysicsTrimesh(
  Ogre::Entity* entity,
  Ogre::SceneNode* node,
  const std::string& name,
  float bodyRestitution,
  float bodyFriction,
  float bodyMass,
  const Ogre::Vector3& position,
  const Ogre::Quaternion& orientation
  )
{
  OgreBulletCollisions::StaticMeshToShapeConverter* conv =
    new OgreBulletCollisions::StaticMeshToShapeConverter( entity );
  OgreBulletDynamics::RigidBody* body =
    new OgreBulletDynamics::RigidBody( name, this->m_BulletWorld );
  body->setShape(
    node, conv->createTrimesh( ),
    bodyRestitution, bodyFriction, bodyMass,
    position, orientation
    );
    body->setDebugDisplayEnabled(true);    
}
//--------------------------------------------------------------------------------------------
void pujOgreBullet::Application::
addStaticRigidPhysicsTrimesh(
  Ogre::Entity* entity,
  Ogre::SceneNode* node,
  const std::string& name,
  float bodyRestitution,
  float bodyFriction,
  float bodyMass,
  const Ogre::Vector3& position,
  const Ogre::Quaternion& orientation
  )
{
  OgreBulletCollisions::StaticMeshToShapeConverter* conv =
    new OgreBulletCollisions::StaticMeshToShapeConverter( entity );
  OgreBulletDynamics::RigidBody* body =
    new OgreBulletDynamics::RigidBody( name, this->m_BulletWorld );
  body->setShape(
    node, conv->createTrimesh( ),
    bodyRestitution, bodyFriction, bodyMass,
    position, orientation
    );
    body->setDebugDisplayEnabled(true);    
}
// -------------------------------------------------------------------------
void pujOgreBullet::Application::
addSoftPhysicsTrimesh(
  Ogre::Entity* entity,
  Ogre::SceneNode* node,
  const std::string& name,
  float bodyRestitution,
  float bodyFriction,
  float bodyMass,
  const Ogre::Vector3& position,
  const Ogre::Quaternion& orientation
  )
{
 //quitar
  OgreBulletCollisions::StaticMeshToShapeConverter* conv =
    new OgreBulletCollisions::StaticMeshToShapeConverter( entity );
  /////
  OgreBulletDynamics::SoftBody* body =
    new OgreBulletDynamics::SoftBody( name, this->m_BulletWorld );
  OgreBulletCollisions::TriangleMeshCollisionShape* collisionShape =
    conv->createTrimesh( );
  btTriangleMesh* trimesh = collisionShape->getTrimesh();

    body->setShape(
      entity, node, collisionShape, trimesh,
      bodyRestitution, bodyFriction, bodyMass,
      position, orientation
    );
    body->setDebugDisplayEnabled(true);
}

// -------------------------------------------------------------------------
void pujOgreBullet::Application::
updatePhysics()
{
  std::cout <<"Actualizando fisica " <<"\n";
  
}

// eof - $RCSfile$
