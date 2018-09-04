#include <iostream>

#include "OgreBulletDynamics.h"

#include "OgreBulletCollisionsShape.h"
#include "OgreBulletCollisionsObject.h"
#include "OgreBulletCollisionsWorld.h"
#include "OgreBulletCollisionsObjectState.h"

#include "OgreBulletDynamicsWorld.h"
#include "OgreBulletDynamicsSoftBody.h"

#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"

#include <vector>

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
    void SoftBody::setShape(Ogre::Entity *ent, Ogre::SceneNode *node,
                      OgreBulletCollisions::CollisionShape *shape,
                      btTriangleMesh* trimesh,
                      const float bodyRestitution,
                      const float bodyFriction,
                      const float bodyMass,
                      const Ogre::Vector3 &pos,
                      const Ogre::Quaternion &quat)
    {
        mState = new ObjectState(this);
        mEntity = ent;
        mRootNode = node;
        mShapeNode = mRootNode->createChildSceneNode(mName + "Node");
        mShapeNode->attachObject(this);

        node->setPosition(pos);
        //node->setOrientation(quat);

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
       body->m_worldInfo = &(myWorld->getWorldInfo());
       btSoftBody::Material *pm = body->appendMaterial();
       pm->m_kLST = 0.1;
        pm->m_kAST = 0.1;
        pm->m_kVST = 0.1;
        body->generateBendingConstraints(2, pm);
        body->m_cfg.collisions |= btSoftBody::fCollision::VF_SS;
        body->m_cfg.kVCF = 1;
        body->m_cfg.kDP = 0;
        body->m_cfg.kDG = 0;
        body->m_cfg.kLF = 0;
        body->m_cfg.kPR = 0;
        body->m_cfg.kVC = 0;
        body->m_cfg.kDF = 1;
        body->m_cfg.kMT = 0;
        body->m_cfg.kCHR = 1;
        body->m_cfg.kKHR = 1;
        body->m_cfg.kSHR = 1;
        body->m_cfg.kAHR = 0.1;
        body->m_cfg.maxvolume = 1;
        body->m_cfg.timescale = 1;
        body->m_cfg.viterations = 0;
        body->m_cfg.piterations = 5;
        body->m_cfg.diterations = 0;
        body->randomizeConstraints();
        body->setTotalMass(bodyMass, true);

        //std::cout << "- Gravedad2: " << *(myWorld->getWorldInfo().m_gravity) << std::endl;                                                        
        mObject = body;
	    body->getCollisionShape()->setMargin( 0.1f );
        //body->setTotalMass( bodyMass );
	    //cloth->generateBendingConstraints(2,cloth->appendMaterial());
	    
        
        getDynamicsWorld()->addSoftBody(this, mCollisionGroup, mCollisionMask);

        std::cout << "\n----- CONTEO ------" << std::endl;
        std::cout << "Cantidad de Soft bodies: " << myWorld->getSoftBodyArray().size() << std::endl;
    }

    void SoftBody::UpdateMesh(){
           std::cout << "\n----- Actualizando objeto fisico!!! ------" << std::endl;
           std::cout << "nombre del nodo:  "<< this->mShapeNode->getName() << "\n" << std::endl;
           std::cout << "nombre de la entidad:  "<< this->mEntity->getName() << "\n" << std::endl;
           
           //this->mShapeNode recuperar la malla del scene node? debo recuperar la entidad?
           //por ahora se sabe apriori el nombre de la entidad del cuerpo blando
           //pendiente por definir como obtener la entidad a apartir del sceneNode
           
           
           int numNodes = static_cast<btSoftBody*>(mObject)->m_nodes.size();
           
           std::cout << "numero de nodos:  "<< numNodes << "\n" << std::endl;
           //Ogre::Vector3 npoints;
           std::deque<Ogre::Vector3> npoints;
           for (size_t i = 0; i < numNodes; i++) {
               btVector3 pos = static_cast<btSoftBody*>(mObject)->m_nodes[i].m_x; 
               npoints.push_back(Ogre::Vector3(pos[0],pos[1],pos[2]));
           //    std::cout << "actualizando puntos:  "<< pos[0] << " " << pos[1] << " "<< pos[2]<< "\n" << std::endl;
           }

            Ogre::Matrix4 mTransform = Ogre::Matrix4::IDENTITY;
            //modificar los nodos de este scene node
           //this->mShapeNode o this->mEntity
           for (unsigned int i = 0; i < this->mEntity->getNumSubEntities(); ++i){
               Ogre::SubMesh *sub_mesh = this->mEntity->getSubEntity(i)->getSubMesh();
               if (!sub_mesh->useSharedVertices){
                    Ogre::VertexData *data = sub_mesh->vertexData;
                    const Ogre::VertexElement *posElem = data->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION); 
                    Ogre::HardwareVertexBufferSharedPtr vbuf = data->vertexBufferBinding->getBuffer(posElem->getSource());
                    VertexDeclaration* decl = HardwareBufferManager::getSingleton().createVertexDeclaration();
                    
                     const unsigned int vSize = (unsigned int)vbuf->getVertexSize();
                    const unsigned int vertexCount = (unsigned int)data->vertexCount;
                    Ogre::Vector3 *tmp_vert = new Ogre::Vector3[vertexCount];
                    Ogre::Vector3 *mVertexBuffer;    
                        mVertexBuffer = tmp_vert;
                    
                    unsigned char *pVert = static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
                    Real* pReal;

                    //float *pReal = NULL;
            

                    std::cout << "vertex count: " <<vertexCount <<std::endl;
                    
                    Ogre::Vector3 * curVertices = &mVertexBuffer[vertexCount];  
                    for (unsigned int j = 0; j < vertexCount; ++j)
                        {
                                
                            
                            posElem->baseVertexPointerToElement(pVert , &pReal);
                            pVert += vSize;
                            //pendiente transformar
                             //npoints = mTransform * (npoints);
                            (*pReal++) = npoints[j].x;    
                            (*pReal++) = npoints[j].y;    
                            (*pReal++) = npoints[j].z;
                            /*
                            curVertices->x = (*pReal++);
                            curVertices->y = (*pReal++);
                            curVertices->z = (*pReal++);
                            *curVertices = mTransform * (*curVertices);
                            curVertices++;
                            */
                            
                        }
                    
                    
                    vbuf->unlock();      
               }
           }

            std::cout << "FIN DE ACTUALIZACION DE OBJETO SUAVE:  \n" << std::endl;       

    }    
}
