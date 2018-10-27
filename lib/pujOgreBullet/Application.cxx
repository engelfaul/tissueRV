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
  /*
  body->setShape(
    node, conv->createTrimesh( ),
    bodyRestitution, bodyFriction, bodyMass,
    position, orientation
    );*/
  body->setStaticShape(
    node, conv->createTrimesh(),
    bodyRestitution, bodyFriction,
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

// -------------------------------------------------------------------------
void pujOgreBullet::Application::
updatePositionBullet(
   Ogre::Vector3& posVector, Ogre::SceneNode* node1
  )
{

    OgreBulletCollisions::Object *object_obj1 = this->m_BulletWorld->findObject( node1 );
    
    //object_obj1->setPosition(posVector);
    static_cast<btRigidBody*>(object_obj1->mObject)->translate(btVector3(posVector.x,posVector.y,posVector.z) );
  /*
  this->m_BulletWorld =
    new OgreBulletDynamics::DynamicsWorld(
      this->m_SceneMgr, bounds, gravityVector, true, true, 10000
      );
      */
}

bool pujOgreBullet::Application::
isCollisionDetected( Ogre::SceneNode* node1, Ogre::SceneNode* node2 ){
    
    
    this->m_collisionsWorld =
          this->m_BulletWorld->getBulletCollisionWorld( );
      
     
      unsigned int amountOfManifolds = 
          this->m_collisionsWorld->getDispatcher( )->getNumManifolds( );
      
      for ( unsigned int i = 0; i < amountOfManifolds; i++ ) {
          btPersistentManifold* contactManifold = 
              this->m_collisionsWorld->getDispatcher( )->getManifoldByIndexInternal( i );
          const btCollisionObject* objectA = contactManifold->getBody0( );
          const btCollisionObject* objectB = contactManifold->getBody1( );
          
          OgreBulletCollisions::Object *object_obj1 = this->m_BulletWorld->findObject( node1 );
          OgreBulletCollisions::Object *object_obj2 = this->m_BulletWorld->findObject( node2 );
          
          OgreBulletCollisions::Object *object_objA = this->m_BulletWorld->findObject( objectA );
          OgreBulletCollisions::Object *object_objB = this->m_BulletWorld->findObject( objectB );

          if ( ( object_objA == object_obj1 ) && ( object_objB == object_obj2 ) ) {
              delete object_objA; delete object_objB;
              return true;
          } else if ( ( object_objA == object_obj1 ) && ( object_objB == object_obj1 ) ) { 
              delete object_objA; delete object_objB;
              return true;
          } else {
              return false;
          }
      }
      
    return true; 
}

// -------------------------------------------------------------------------


// eof - $RCSfile$
