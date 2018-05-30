#ifndef _OGREBULLETDYNAMICS_SoftObject_H
#define _OGREBULLETDYNAMICS_SoftObject_H

#include "OgreBulletDynamicsPreRequisites.h"

#include "OgreBulletCollisionsObject.h"
#include "OgreBulletCollisionsWorld.h"

#include "OgreBulletDynamicsWorld.h"

namespace OgreBulletDynamics {

    class SoftBody : public OgreBulletCollisions::Object
    {
        public:
            SoftBody(const Ogre::String &name,
                    DynamicsWorld *world,
                    const short collisionGroup = 0,
                    const short collisionMask = 0);
            
            virtual ~SoftBody();

            void setShape(Ogre::SceneNode *node,
                      OgreBulletCollisions::CollisionShape *shape,
                      const float bodyRestitution,
                      const float bodyFriction,
                      const float bodyMass,
                      const Ogre::Vector3 &pos = Ogre::Vector3::ZERO,
                      const Ogre::Quaternion &quat = Ogre::Quaternion::IDENTITY);

            inline btSoftBody *getBulletSoftBody() const;
            inline DynamicsWorld *getDynamicsWorld();

        protected:
            short mCollisionGroup;
            short mCollisionMask;
    };
    // -------------------------------------------------------------------------        
    inline btSoftBody *SoftBody::getBulletSoftBody() const
    {
        return static_cast<btSoftBody *>(mObject);
    }
    // -------------------------------------------------------------------------
    inline DynamicsWorld *SoftBody::getDynamicsWorld()
    { 
        return static_cast<DynamicsWorld *>(mWorld);
    }
}

#endif