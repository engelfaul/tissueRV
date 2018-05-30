#include "OgreBulletDynamics.h"

#include "OgreBulletCollisionsShape.h"
#include "OgreBulletCollisionsObject.h"
#include "OgreBulletCollisionsWorld.h"
#include "OgreBulletCollisionsObjectState.h"

#include "OgreBulletDynamicsWorld.h"
#include "OgreBulletDynamicsSoftBody.h"


using namespace Ogre;
using namespace OgreBulletCollisions;

namespace OgreBulletDynamics
{
    // -------------------------------------------------------------------------
    SoftBody::SoftBody(const Ogre::String &name, DynamicsWorld *world, const short collisionGroup, const short collisionMask)
        : Object(name, world, false),
          mCollisionGroup(collisionGroup),
          mCollisionMask(collisionMask)
    {
    }
    // -------------------------------------------------------------------------
    SoftBody::~SoftBody()
    {
    }
    // -------------------------------------------------------------------------
    void SoftBody::setShape(Ogre::SceneNode *node,
                      OgreBulletCollisions::CollisionShape *shape,
                      const float bodyRestitution,
                      const float bodyFriction,
                      const float bodyMass,
                      const Ogre::Vector3 &pos,
                      const Ogre::Quaternion &quat)
    {
        /*const btScalar* vertices;
        const int* triangles;
        int ntriangles;
        btSoftBody *body = btSoftBodyHelpers::CreateFromTriMesh(getDynamicsWorld()->getWorldInfo(),
                                                                &vertices,
                                                                &triangles,
                                                                ntriangles
                                                                );
        mObject = body;
        getDynamicsWorld()->addSoftBody(this, mCollisionGroup, mCollisionMask);*/

    }
}