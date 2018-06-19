#ifndef __pujOgreBullet__Application__h__
#define __pujOgreBullet__Application__h__

#include <pujOgre/Application.h>
#include <OgreAxisAlignedBox.h>

namespace OgreBulletDynamics
{
  class DynamicsWorld;
} // end namespace

namespace pujOgreBullet
{
  /**
   */
  class Application
    : public pujOgre::Application
  {
  public:
    typedef Application          Self;
    typedef pujOgre::Application Superclass;

  public:
    Application( );
    virtual ~Application( );

  protected:
    virtual void createScene( ) override;
    virtual bool frameStarted( const Ogre::FrameEvent& evt ) override;

    virtual void initBullet(
      const Ogre::Vector3& gravityVector = Ogre::Vector3( 0, -9.81, 0 ),
      const Ogre::AxisAlignedBox& bounds =
      Ogre::AxisAlignedBox(
        Ogre::Vector3( -10000, -10000, -10000 ),
        Ogre::Vector3(  10000,  10000,  10000 )
        )
      );

    void addPhysicsPlane(
      const Ogre::Plane& plane,
      const std::string& name,
      float bodyRestitution,
      float bodyFriction
      );

    void addPhysicsSphere(
      Ogre::Entity* entity,
      Ogre::SceneNode* node,
      const std::string& name,
      float bodyRestitution,
      float bodyFriction,
      float bodyMass,
      const Ogre::Vector3& position = Ogre::Vector3::ZERO,
      const Ogre::Quaternion& orientation = Ogre::Quaternion::IDENTITY
      );

    void addPhysicsBox(
      Ogre::Entity* entity,
      Ogre::SceneNode* node,
      const std::string& name,
      float bodyRestitution,
      float bodyFriction,
      float bodyMass,
      const Ogre::Vector3& position = Ogre::Vector3::ZERO,
      const Ogre::Quaternion& orientation = Ogre::Quaternion::IDENTITY
      );

    void addPhysicsCylinder(
      Ogre::Entity* entity,
      Ogre::SceneNode* node,
      const std::string& name,
      float bodyRestitution,
      float bodyFriction,
      float bodyMass,
      const Ogre::Vector3& position = Ogre::Vector3::ZERO,
      const Ogre::Quaternion& orientation = Ogre::Quaternion::IDENTITY
      );

    void addPhysicsConvex(
      Ogre::Entity* entity,
      Ogre::SceneNode* node,
      const std::string& name,
      float bodyRestitution,
      float bodyFriction,
      float bodyMass,
      const Ogre::Vector3& position = Ogre::Vector3::ZERO,
      const Ogre::Quaternion& orientation = Ogre::Quaternion::IDENTITY
      );

    void addRigidPhysicsTrimesh(
      Ogre::Entity* entity,
      Ogre::SceneNode* node,
      const std::string& name,
      float bodyRestitution,
      float bodyFriction,
      float bodyMass,
      const Ogre::Vector3& position = Ogre::Vector3::ZERO,
      const Ogre::Quaternion& orientation = Ogre::Quaternion::IDENTITY
      );

   void addSoftPhysicsTrimesh(
      Ogre::Entity* entity,
      Ogre::SceneNode* node,
      const std::string& name,
      float bodyRestitution,
      float bodyFriction,
      float bodyMass,
      const Ogre::Vector3& position,
      const Ogre::Quaternion& orientation
    );

  protected:
    OgreBulletDynamics::DynamicsWorld* m_BulletWorld;
  };

} // ecapseman

#endif // __pujOgre__Application__h__

// eof - $RCSfile$
