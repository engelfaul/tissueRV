#include <iostream>

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
        mState = new ObjectState(this);
        
        mRootNode = node;
        mShapeNode = mRootNode->createChildSceneNode(mName + "Node");
        mShapeNode->attachObject(this);

        node->setPosition(pos);
        node->setOrientation(quat);

        mShape = shape;

        Ogre::Entity* objEntity = static_cast<Entity*> ( node->getAttachedObject(0) );
        Ogre::MeshPtr objMesh = objEntity->getMesh( );
        Ogre::Mesh::SubMeshList subMeshesList = objMesh->getSubMeshes( );
        Ogre::Mesh::SubMeshIterator::iterator subMeshesIterator = subMeshesList.begin();

        btVector3 vert0, vert1, vert2;
	    std::vector<Ogre::Vector3> vertices;
	    std::vector<unsigned long> indices;

        btTriangleMesh *objTriMesh = new btTriangleMesh( );
        
        for ( ; subMeshesIterator !=  subMeshesList.end() ; subMeshesIterator++) {
            Ogre::SubMesh* tempSubMesh = *subMeshesIterator;
            Ogre::IndexData*  indexData = tempSubMesh->indexData;
            Ogre::VertexData* vertexData = tempSubMesh->vertexData;

            const Ogre::VertexElement* elementPosition = 
                vertexData->vertexDeclaration->findElementBySemantic ( Ogre::VES_POSITION );
            Ogre::HardwareVertexBufferSharedPtr vertexBuffer = 
                vertexData->vertexBufferBinding->getBuffer ( elementPosition->getSource() );
            Ogre::HardwareIndexBufferSharedPtr indexBuffer = indexData->indexBuffer;

            vertices.reserve ( vertexData->vertexCount );
    	    indices.reserve ( indexData->indexCount );

            // -------- Read from Vertex Buffer --------------
		    unsigned char* vertex = 
			static_cast<unsigned char*> ( vertexBuffer->lock ( Ogre::HardwareBuffer::HBL_READ_ONLY ) );
		    float* pReal = NULL;
		    for ( size_t j = 0; j < vertexData->vertexCount; ++j, vertex += vertexBuffer->getVertexSize( ) ) {
			    elementPosition->baseVertexPointerToElement( vertex, &pReal );
			    Ogre::Vector3 p ( pReal[0], pReal[1], pReal[2] );
			    vertices.push_back( p );
		    }
		    vertexBuffer->unlock( );

            // -------- Read from Index Buffer --------------
            unsigned long* pLong =
			static_cast<unsigned long*> ( indexBuffer->lock ( Ogre::HardwareBuffer::HBL_READ_ONLY ) );
		    unsigned short* pShort = reinterpret_cast<unsigned short*> ( pLong );
		    for ( size_t k = 0; k < indexData->indexCount; ++k )
        		indices.push_back ( static_cast<unsigned long> ( pShort[k] ) );
		    
            indexBuffer->unlock( );

            // -------------- Create TriMesh ---------------
            unsigned int i = 0;
            for ( size_t j = 0; j < indexData->indexCount / 3; j++ ) {
			    vert0.setValue ( vertices[indices[i]].x, vertices[indices[i]].y, vertices[indices[i]].z );
      		    vert1.setValue ( vertices[indices[i+1]].x, vertices[indices[i+1]].y, vertices[indices[i+1]].z );
      		    vert2.setValue ( vertices[indices[i+2]].x, vertices[indices[i+2]].y, vertices[indices[i+2]].z );

      		    objTriMesh->addTriangle ( vert0, vert1, vert2 );
      		    i += 3;
    	    }
        }

        std::cout << "\n # Vertices: " << vertices.size();
        std::cout << "\n # Triangles: " << objTriMesh->getNumTriangles() << std::endl;


        // -------------- Create Soft Body from TriMesh ---------------
        
        const btVector3 meshScaling = objTriMesh->getScaling();
        btAlignedObjectArray<btScalar> trimeshVertices;
        btAlignedObjectArray<int> triangles;

        for ( int part=0;part< objTriMesh->getNumSubParts(); part++ ) {
            const unsigned char * vertexbase;
            const unsigned char * indexbase;

            int indexstride;
            int stride, numverts, numtriangles;
            PHY_ScalarType type, gfxindextype;

            objTriMesh->getLockedReadOnlyVertexIndexBase(   &vertexbase,
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
        
        btSoftBody *body = btSoftBodyHelpers::CreateFromTriMesh( getDynamicsWorld()->getWorldInfo(),
                                                                &trimeshVertices[0],
                                                                &triangles[0],
                                                                objTriMesh->getNumTriangles()
                                                                );
        
        mObject = body;
        getDynamicsWorld()->addSoftBody(this, mCollisionGroup, mCollisionMask);
    }
}