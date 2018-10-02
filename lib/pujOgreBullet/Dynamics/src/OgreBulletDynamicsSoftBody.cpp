#include <iostream>
#include <math.h>

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
        int vertexc;
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

        int tempvertex;
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
                //std::cout << "Recuperando info de los triangulos:" << tri_indices[0] << " " << tri_indices[1] << " " <<  tri_indices[2]<< std::endl;        
                triangles.push_back(tri_indices[0]);
                triangles.push_back(tri_indices[1]);
                triangles.push_back(tri_indices[2]);
            }
            tempvertex = numverts;
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
        std::cout << "- num vertices al crears: " << tempvertex << std::endl;
        /*
        btSoftBody *body = btSoftBodyHelpers::CreateFromTriMesh( myWorld->getWorldInfo(),
                                                                &trimeshVertices[0],
                                                                &triangles[0],
                                                                trimesh->getNumTriangles()
                                                                );
        */
        const btScalar s=4; //size of cloth patch
		const int NUM_X=10; //vertices on X axis
		const int NUM_Z=10; //vertices on Z axis
                                                            
        btSoftBody *body = btSoftBodyHelpers::CreatePatch(myWorld->getWorldInfo(),
		                                            btVector3(0,10,0),
		                                            btVector3(9,10,0),
		                                            btVector3(0,10,9),
		                                            btVector3(9,10,9),
		                                            NUM_X,NUM_Z, 
		                                            1+2,true);
        
        //investigando valores de los nodos
       int numNodes = body->m_nodes.size();
       std::deque<Ogre::Vector3> npoints;
       std::cout << "- numero de nodos: " << numNodes << std::endl;

        
        for (size_t i = 0; i < numNodes; i++) {
            btVector3 pos = body->m_nodes[i].m_x; 
            npoints.push_back(Ogre::Vector3(pos[0],pos[1],pos[2]));
               std::cout << "nodo "<< i <<": "<< pos[0] << " " << pos[1] << " "<< pos[2]<< "\n" << std::endl;
               
        }
       
///////////////////////////////verificando los VertexBuffer///////////////////////////////////////////////////////////
           for (unsigned int i = 0; i < this->mEntity->getNumSubEntities(); ++i){
               Ogre::SubMesh *sub_mesh = this->mEntity->getSubEntity(i)->getSubMesh();
                
               if (!sub_mesh->useSharedVertices ){ 
                
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

                    std::cout << "1. vertex count: " <<vertexCount <<std::endl;
                    vertexc = vertexCount;
                    indexBulletNodes = new int[vertexCount];
                    Ogre::Vector3 * curVertices = &mVertexBuffer[vertexCount];  
                    for (unsigned int j = 0; j < vertexCount; j++)
                        {
                            
                            posElem->baseVertexPointerToElement(pVert , &pReal);
                            pVert += vSize;
                            //el primer buffer es Z!
                            int vx =(*(pReal++));
                            int vy =(*(pReal++));
                            int vz =(*(pReal++));
                            std::cout << "bucando vertex" << j <<": " << vx << " " << vy << " " << vz  <<std::endl;
                            
                            for (size_t i = 0; i < numNodes; i++) {
                                btVector3 pos = body->m_nodes[i].m_x; 
                              //  std::cout << "nodo "<< i <<": "<< pos[0] << " " << pos[1] << " "<< pos[2]<< "\n" << std::endl;
                                if(vx == pos[0] && vy == pos[1] && vz == pos[2] ){
                                   //si encuentra un nodo igual al vertice sale del for
                                   indexBulletNodes[j] = i;        
                                   std::cout << "encontre nodo "<< i <<" vertice "<< j << "\n" << std::endl;
                                   break;         
                                }
                            }
                
                        }
                    
                    
                    vbuf->unlock();      
               }
           }


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// _softBody->m_cfg.kMT = 0.01; // Pose matching coefficient
// _softBody->m_cfg.kPR = 2500;
// _softBody->m_cfg.kCHR = 1; // Rigid contacts hardness
// _softBody->m_cfg.kKHR = 0.8; // Kinetic contacts hardness
// _softBody->m_cfg.kSHR = 1; // Soft contacts hardness

//_softBody->m_materials[0]->m_kLST = 0.45;


// _softBody->setPose(false, true); // XXX Set current state as a pose
// _softBody->generateBendingConstraints(2, softBodyMaterial);

       // myWorld->getWorldInfo().m_gravity =  btVector3(0,10,0);
       body->m_worldInfo = &(myWorld->getWorldInfo());
       btSoftBody::Material *pm = body->appendMaterial();
       //body->m_material[0]->m_kLST = 0.45;
        pm->m_kLST = 0.5;
        pm->m_kAST = 0.1;
        pm->m_kVST = 0.5;
        body->generateBendingConstraints(2, pm);
        body->m_cfg.collisions |= btSoftBody::fCollision::VF_SS;
        body->m_cfg.kVCF = 1;
        body->m_cfg.kDP = 0.001; // Damping coefficient [0,1]
        body->m_cfg.kDG = 1;
        body->m_cfg.kLF = 0;
        body->m_cfg.kPR = 0;
        body->m_cfg.kVC = 0;
        body->m_cfg.kDF = 0.5;
        body->m_cfg.kMT = 0;
        body->m_cfg.kCHR = 1;
        body->m_cfg.kKHR = 1;
        body->m_cfg.kSHR = 1;
        body->m_cfg.kAHR = 1;
        body->m_cfg.maxvolume = 1;
        body->m_cfg.timescale = 1;
        body->m_cfg.viterations = 0;
        body->m_cfg.piterations = 2;
        body->m_cfg.diterations = 0;
        body->m_cfg.citerations = 4;
        
        body->m_cfg.kSRHR_CL = 1;
        body->randomizeConstraints();
        //body->setTotalMass(bodyMass, true);
        //body->setTotalMass(0, true);
        
        //Fijar el objeto configurando vertices con masa igual a cero
        int res =  sqrt(vertexc);
        int cont = 0;
        for(int n=0;n<res-1;n++){
          body->setMass(cont,0);
          body->setMass(cont+4,0);
          body->setMass(cont+5,0);
          cont+=6;      
        }
      
        
        body->getCollisionShape()->setMargin(0.001f);
        body->getCollisionShape()->setUserPointer((void*)body);
	    body->generateBendingConstraints(2,body->appendMaterial());
	    //body->setTotalMass(10); 
	    //cloth->m_cfg.citerations = 10;
        //	cloth->m_cfg.diterations = 10;
	    //body->m_cfg.kDP = 0.01;
	

        std::cout << "RestLengh: " << body->getRestLengthScale() << std::endl;
        //body->setMass(18,0);
        //std::cout << "- Gravedad2: " << *(myWorld->getWorldInfo().m_gravity) << std::endl;                                                        
        mObject = body;
	    
        //body->setTotalMass( bodyMass );
	    //cloth->generateBendingConstraints(2,cloth->appendMaterial());
	    
        
        getDynamicsWorld()->addSoftBody(this, mCollisionGroup, mCollisionMask);
        //getSoftDynamicsWorld()->addSoftBody(cloth);

        std::cout << "\n----- CONTEO ------" << std::endl;
        std::cout << "Cantidad de Soft bodies: " << myWorld->getSoftBodyArray().size() << std::endl;
    }

    void SoftBody::UpdateMesh(){
           /*
           std::cout << "\n----- Actualizando objeto fisico!!! ------" << std::endl;
           std::cout << "nombre del nodo:  "<< this->mShapeNode->getName() << "\n" << std::endl;
           std::cout << "nombre de la entidad:  "<< this->mEntity->getName() << "\n" << std::endl;
           */       

           int numNodes = static_cast<btSoftBody*>(mObject)->m_nodes.size();
           
           //std::cout << "numero de nodos:  "<< numNodes << "\n" << std::endl;
           
           std::deque<Ogre::Vector3> npoints;
           for (size_t i = 0; i < numNodes; i++) {
               btVector3 pos = static_cast<btSoftBody*>(mObject)->m_nodes[i].m_x; 
               npoints.push_back(Ogre::Vector3(pos[0],pos[1],pos[2]));
               //std::cout << "actualizando puntos:  "<< pos[0] << " " << pos[1] << " "<< pos[2]<< "\n" << std::endl;
           }

            //std::cout << "tamanio npoints:  "<< npoints.size() << "\n" << std::endl;

            Ogre::Matrix4 mTransform = Ogre::Matrix4::IDENTITY;
            //modificar los nodos de este scene node
           //this->mShapeNode o this->mEntity
           for (unsigned int i = 0; i < this->mEntity->getNumSubEntities(); ++i){
               Ogre::SubMesh *sub_mesh = this->mEntity->getSubEntity(i)->getSubMesh();
                
              //  std::cout << "Antes del if " <<std::endl;
               if (!sub_mesh->useSharedVertices ){ 
               //    std::cout << "Adentro del if " <<std::endl;
                    Ogre::VertexData *data = sub_mesh->vertexData;
                    const Ogre::VertexElement *posElem = data->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION); 
                    Ogre::HardwareVertexBufferSharedPtr vbuf = data->vertexBufferBinding->getBuffer(posElem->getSource());
                    VertexDeclaration* decl = HardwareBufferManager::getSingleton().createVertexDeclaration();
                    
                     const unsigned int vSize = (unsigned int)vbuf->getVertexSize();
                    const unsigned int vertexCount = (unsigned int)data->vertexCount;
                    Ogre::Vector3 *tmp_vert = new Ogre::Vector3[vertexCount];
                    Ogre::Vector3 *mVertexBuffer;    
                        mVertexBuffer = tmp_vert;
                    
                    unsigned char *pVert = static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_NORMAL));
                    Real* pReal;

                    //float *pReal = NULL;
            

                    //    std::cout << "vertex count: " <<vertexCount <<std::endl;
                    
                    Ogre::Vector3 * curVertices = &mVertexBuffer[vertexCount];  
                    for (unsigned int j = 0; j < vertexCount; j++)
                        {
                                
                            
                            posElem->baseVertexPointerToElement(pVert , &pReal);
                            pVert += vSize;

                                (*(pReal++)) = npoints[indexBulletNodes[j]].x;    
                                (*(pReal++)) = npoints[indexBulletNodes[j]].y;    
                                (*(pReal++)) = npoints[indexBulletNodes[j]].z;

                                if(j==12){
                                    std::cout << "vertex de prueba: " << npoints[indexBulletNodes[j]].x << " " << npoints[indexBulletNodes[j]].y  << " " <<npoints[indexBulletNodes[j]].z <<std::endl;
                                }
                        }
                    
                    
                    vbuf->unlock();      
               }
           }

           // std::cout << "FIN DE ACTUALIZACION DE OBJETO SUAVE:  \n" << std::endl;       

    }    
}
