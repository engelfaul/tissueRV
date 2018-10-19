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

            void setShape(Ogre::Entity *ent,Ogre::SceneNode *node,
                      OgreBulletCollisions::CollisionShape *shape,
                      btTriangleMesh* trimesh,
                      const float bodyRestitution,
                      const float bodyFriction,
                      const float bodyMass,
                      const Ogre::Vector3 &pos = Ogre::Vector3::ZERO,
                      const Ogre::Quaternion &quat = Ogre::Quaternion::IDENTITY);
            
            //actualizando el mesh en cada delta t
            void UpdateMesh();          

            //evento de corte
            void UpdateCut(Ogre::Real cutnode);

            inline btSoftBody *getBulletSoftBody() const;
            inline DynamicsWorld *getDynamicsWorld();
            inline btDynamicsWorld *getBulletDynamicsWorld() const;

        protected:
            Ogre::Entity * mEntity;
            int * indexBulletNodes;
            int * indexBulletFaces;
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
    // -------------------------------------------------------------------------
    inline btDynamicsWorld *SoftBody::getBulletDynamicsWorld() const
    { 
        return static_cast<btDynamicsWorld *>(mWorld->getBulletCollisionWorld());
    }
    
}

#endif