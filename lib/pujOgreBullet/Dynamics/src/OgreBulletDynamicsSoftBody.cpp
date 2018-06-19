#include <iostream>

#include "OgreBulletDynamics.h"

#include "OgreBulletCollisionsShape.h"
#include "OgreBulletCollisionsObject.h"
#include "OgreBulletCollisionsWorld.h"
#include "OgreBulletCollisionsObjectState.h"

#include "OgreBulletDynamicsWorld.h"
#include "OgreBulletDynamicsSoftBody.h"

#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"

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
                      btTriangleMesh* trimesh,
                      const float bodyRestitution,
                      const float bodyFriction,
                      const float bodyMass,
                      const Ogre::Vector3 &pos,
                      const Ogre::Quaternion &quat)
    {
        mState = new ObjectState(this);
        
        mRootNode = node;
        mShapeNode = mRootNode->createChildSceneNode(mName + "Node");
        mShapeNode->attachObject(this);

        node->setPosition(pos);
        node->setOrientation(quat);

        mShape = shape;
        std::cout << "\n\nTipo de *shape: " << typeid( *(shape->getBulletShape()) ).name() << std::endl;
        std::cout << "Tipo de shape: " << typeid( shape->getBulletShape() ).name() << std::endl;
        std::cout << "Tipo de shape (planito): " << typeid( shape ).name() << std::endl;
        std::cout << "Tipo de *mShape: " << typeid( *(mShape->getBulletShape()) ).name() << std::endl;
        std::cout << "Tipo de mShape: " << typeid( mShape   ->getBulletShape() ).name() << std::endl;
        std::cout << "Tipo de mShape (planito): " << typeid( mShape ).name() << std::endl;
        std::cout << "Tipo de trimesh: " << typeid( trimesh ).name() << std::endl;

        showDebugShape(mWorld->getShowDebugShapes());
        
        btVector3 localInertiaTensor = btVector3(0.f, 0.f, 0.f);
        if (bodyMass > 0.f)
        {
            mShape->getBulletShape()->calculateLocalInertia( bodyMass, localInertiaTensor );
        }

        // -------------- Create Soft Body from TriMesh ---------------
        const btVector3 meshScaling = trimesh->getScaling();
        btAlignedObjectArray<btScalar> trimeshVertices;
        btAlignedObjectArray<int> triangles;

        for ( int part=0;part< trimesh->getNumSubParts(); part++ ) {
            const unsigned char * vertexbase;
            const unsigned char * indexbase;

            int indexstride;
            int stride, numverts, numtriangles;
            PHY_ScalarType type, gfxindextype;

            trimesh->getLockedReadOnlyVertexIndexBase(   &vertexbase,
                                                            numverts,
                                                            type,
                                                            stride,
                                                            &indexbase,
                                                            indexstride,
                                                            numtriangles,
                                                            gfxindextype,
                                                            part
                                                        );

            for (int gfxindex=0; gfxindex < numverts; gfxindex++) {
                float* graphicsbase = (float*)(vertexbase+gfxindex*stride);
                trimeshVertices.push_back(graphicsbase[0]*meshScaling.getX());
                trimeshVertices.push_back(graphicsbase[1]*meshScaling.getY());
                trimeshVertices.push_back(graphicsbase[2]*meshScaling.getZ());
            }

            for (int gfxindex=0;gfxindex < numtriangles; gfxindex++) {
                unsigned int* tri_indices= (unsigned int*)(indexbase+gfxindex*indexstride);
                triangles.push_back(tri_indices[0]);
                triangles.push_back(tri_indices[1]);
                triangles.push_back(tri_indices[2]);
            }
        }

        
        btSoftRigidDynamicsWorld* myWorld =
          dynamic_cast< btSoftRigidDynamicsWorld* >( getDynamicsWorld()->getBulletDynamicsWorld() );
        if( myWorld == NULL )
          throw std::runtime_error( "No sea bruto" );

        std::cout << "\n----- MUNDO EXISTENTE ------" << std::endl;
        std::cout << "- Tipo Dispatcher: " << typeid(*(myWorld->getWorldInfo().m_dispatcher)).name() << std::endl;
        std::cout << "- Tipo Broadphase: " << typeid(*(myWorld->getWorldInfo().m_broadphase)).name() << std::endl;
        std::cout << "- Tipo Mundo: " << typeid(*myWorld).name() << std::endl << std::endl;
        std::cout << "- Gravedad: " << *(myWorld->getWorldInfo().m_gravity) << std::endl;
        
        btSoftBody *body = btSoftBodyHelpers::CreateFromTriMesh( myWorld->getWorldInfo(),
                                                                &trimeshVertices[0],
                                                                &triangles[0],
                                                                trimesh->getNumTriangles()
                                                                );

       // myWorld->getWorldInfo().m_gravity =  btVector3(0,10,0);
        std::cout << "- Gravedad2: " << *(myWorld->getWorldInfo().m_gravity) << std::endl;                                                        
        body->generateBendingConstraints(2);
        mObject = body;
	    body->getCollisionShape()->setMargin( 0.1f );
        body->setTotalMass( bodyMass );
	    //cloth->generateBendingConstraints(2,cloth->appendMaterial());
	    body->m_cfg.citerations = 10;
        body->m_cfg.diterations = 10;
	    body->m_cfg.piterations = 5;
        body->m_cfg.kDF = 0.75;
	    body->m_cfg.kDP = 0.005f;
        body->m_cfg.collisions |= btSoftBody::fCollision::VF_SS;	
        body->randomizeConstraints();
        getDynamicsWorld()->addSoftBody(this, mCollisionGroup, mCollisionMask);

        std::cout << "\n----- CONTEO ------" << std::endl;
        std::cout << "Cantidad de Soft bodies: " << myWorld->getSoftBodyArray().size() << std::endl;
    }    
}
