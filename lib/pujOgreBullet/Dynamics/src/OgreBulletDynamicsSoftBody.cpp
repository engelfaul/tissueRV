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

        //node->setPosition(pos);
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
		const int NUM_X=20; //vertices on X axis
		const int NUM_Z=20; //vertices on Z axis
        int posy= node->getPosition().y;                                                   
        btSoftBody *body = btSoftBodyHelpers::CreatePatch(myWorld->getWorldInfo(),
		                                            btVector3(0,10,0),
		                                            btVector3(19,10,0),
		                                            btVector3(0,10,19),
		                                            btVector3(19,10,19),
		                                            NUM_X,NUM_Z, 
		                                            1+2+4+8,true);
        
        //investigando valores de los nodos
       int numNodes = body->m_nodes.size();
       std::deque<Ogre::Vector3> npoints;
       std::cout << "- numero de nodos: " << numNodes << std::endl;
        
        for (size_t i = 0; i < numNodes; i++) {
            btVector3 pos = body->m_nodes[i].m_x; 
            npoints.push_back(Ogre::Vector3(pos[0],pos[1],pos[2]));
               //std::cout << "nodo "<< i <<": "<< pos[0] << " " << pos[1] << " "<< pos[2]<< "\n" << std::endl;
               
        }
        int numLinksTest = body->m_links.size();

        std::cout << "- numero de lnks: " <<  numLinksTest << std::endl;
       
       body->m_worldInfo = &(myWorld->getWorldInfo());
       btSoftBody::Material *pm = body->appendMaterial();
       //body->m_material[0]->m_kLST = 0.45;
        pm->m_kLST = 0.5;
        pm->m_kAST = 0.1;
        pm->m_kVST = 0.5;
        body->generateBendingConstraints(2, pm);
        body->m_cfg.collisions |= btSoftBody::fCollision::VF_SS;
        body->m_cfg.kVCF = 1;
        body->m_cfg.kDP = 0.0001; // Damping coefficient [0,1]
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
        //body->randomizeConstraints();
        //body->setTotalMass(bodyMass, true);
        //body->setTotalMass(0, true);
        
        //Fijar el objeto configurando vertices con masa igual a cero
        /*
        int res =  sqrt(vertexc);
        int cont = 0;
        for(int n=0;n<res-1;n++){
          body->setMass(cont,0);
          body->setMass(cont+4,0);
          body->setMass(cont+5,0);
          cont+=6;      
        }
      */
        
        body->getCollisionShape()->setMargin(0.001f);
        body->getCollisionShape()->setUserPointer((void*)body);
	    body->generateBendingConstraints(2,body->appendMaterial());


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
                            
                            int vx =(*(pReal++));
                            int vy =(*(pReal++));
                            int vz =(*(pReal++));
                            std::cout << "bucando vertex" << j <<": " << vx << " " << vy << " " << vz  <<std::endl;
                            
                            for (size_t i = 0; i < numNodes; i++) {
                                btVector3 pos = body->m_nodes[i].m_x; 
                                //std::cout << "nodo "<< i <<": "<< pos[0] << " " << pos[1] << " "<< pos[2]<< "\n" << std::endl;
                                if(vx == pos[0] && vy == pos[1] && vz == pos[2] ){
                                   //si encuentra un nodo igual al vertice sale del for
                                   indexBulletNodes[j] = i;        
                                   std::cout << "encontre nodo "<< i <<" vertice "<< j << "\n" << std::endl;
                                   break;         
                                }
                            }
                
                        }
                    
                    
                    vbuf->unlock();      
               
                    ///////////////////////Investigando informacion de los faces en ogre///////////////////////////////////////////////////    
                    //VertexIndexToShape::addIndexData(sub_mesh->indexData, mVertexCount);
                    std::cout << "Investigando informacion de los faces en ogre" << std::endl;
                    Ogre::IndexData *data2 = sub_mesh->indexData;
                    int numIndices = data2->indexCount;
                    int mIndexCount;
                 //   std::cout << "index count: " <<numIndices <<std::endl;
                    //VertexIndexToShape::addStaticVertexData(sub_mesh->vertexData);
                    
                        
                        
                        mIndexCount = (unsigned int)data2->indexCount;
                        const unsigned int prev_size = mIndexCount;
                        std::cout << "prev_size: " <<prev_size <<std::endl;
                        unsigned int *tmp_ind = new unsigned int[mIndexCount];
                        unsigned int  *mIndexBuffer;
                 
                        mIndexBuffer = tmp_ind;
                        
                        const unsigned int numTris = (unsigned int) data2->indexCount / 3;
                        //std::cout << "numTris: " <<numTris <<std::endl;
                        
                        Ogre::HardwareIndexBufferSharedPtr ibuf = data2->indexBuffer;
                        
                        const bool use32bitindexes = (ibuf->getType() == Ogre::HardwareIndexBuffer::IT_32BIT);
                        
                        unsigned int index_offset = prev_size;
                        unsigned int offset  = vertexCount;
                        if (use32bitindexes) 
                        {
                            std::cout << "entre al if " <<std::endl;
                            /* //Aqui voy
                            const unsigned int *pInt = static_cast<unsigned int*>(ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
                            for (unsigned int k = 0; k < numTris; ++k)
                            {
                                mIndexBuffer[index_offset ++] = offset + *pInt++;
                                mIndexBuffer[index_offset ++] = offset + *pInt++;
                                mIndexBuffer[index_offset ++] = offset + *pInt++;
                            }
                            ibuf->unlock();
                            */
                        }
                        else 
                        {
                            const unsigned short *pShort = static_cast<unsigned short*>(ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
                            //std::cout << "primer valor pshort: " <<  *pShort << std::endl;
                            //std::cout << "prueba pShort[1]: " << pShort[1] << std::endl;
                            //std::cout << "prueba pShort[2]: " << pShort[2] << std::endl;
                            //std::cout << "prueba pShort[3]: " << pShort[3] << std::endl;
                            indexBulletFaces = new int[numTris];
                            for(unsigned int k = 0; k < numTris; ++k)
                            {
                                /*
                                offset + static_cast<unsigned int>(*pShort++) = mIndexBuffer[index_offset ++] ;
                                offset + static_cast<unsigned int>(*pShort++) = mIndexBuffer[index_offset ++] ;
                                offset + static_cast<unsigned int>(*pShort++) = mIndexBuffer[index_offset ++] ;
                                */
                               
                               int ix = vertexCount + static_cast<unsigned int>(*pShort++) ;
                                    //vertice z
                                    int vertexZ1 = ix%10;
                                    //vertice x
                                    int tempy1= ix - vertexZ1;
                                    tempy1= tempy1%100;
                                    int vertexX1 = tempy1/10; 
                               int iy = vertexCount + static_cast<unsigned int>(*pShort++) ;
                                    //vertice z
                                    int vertexZ2 = iy%10;
                                    //vertice x
                                    int tempy2= iy - vertexZ2;
                                    tempy2= tempy2%100;
                                    int vertexX2 = tempy2/10;
                               
                               int iz = vertexCount + static_cast<unsigned int>(*pShort++) ;
                                    //vertice z
                                    int vertexZ3 = iz%10;
                                    //vertice x
                                    int tempy3= iz - vertexZ3;
                                    tempy3= tempy3%100;
                                    int vertexX3 = tempy3/10;

                             
                             //  std::cout << "recuperando index del triangulo:  " << k <<": " << ix << " " << iy << " " << iz  <<std::endl; 
                             //  std::cout << "vertex 0: " << vertexX1 <<" "<< vertexZ1 << std::endl;
                             //  std::cout << "vertex 1: " << vertexX2 <<" "<< vertexZ2 << std::endl;
                             //  std::cout << "vertex 2: " << vertexX3 <<" "<< vertexZ3 << std::endl;
                               
                                //diccionario para tener relacion entre los triangulos de ogre y bullet
                                
                                ////////////Investigando informacion de los faces en bullet/////////////////////////////////////
                             //       std::cout << "detectando triangulos" << std::endl;
                                    int numFaces = body->m_faces.size();
                                    int numLinks = body->m_links.size();

                                    for (size_t i = 0; i < numFaces; i++) {
                                        btVector3 pos = body->m_faces[i].m_n[0]->m_x;
                                        btVector3 pos1 = body->m_faces[i].m_n[1]->m_x;
                                        btVector3 pos2 = body->m_faces[i].m_n[2]->m_x; 
                                        
                                        if(int(pos[0])==vertexX1 && int(pos[2])==vertexZ1 && int(pos1[0])==vertexX2 && int(pos1[2])==vertexZ2 && int(pos2[0])==vertexX3 && int(pos2[2])==vertexZ3 ){
                               //             std::cout << "triangulos encontrado k: " << k<< " i "<< i <<std::endl;
                               //             std::cout << "triangulo: " << " i "<< i <<std::endl;
                               //             std::cout << "pos[0]: " << pos[0] << " pos[1]: " << pos[1] << "pos[2]: " << pos[2] <<std::endl;
                                //            std::cout << "pos1[0]: " << pos1[0] << " pos1[1]: " << pos1[1] << "pos1[2]: " << pos1[2] <<std::endl;
                                            indexBulletFaces[k] = i; //cambia el diccionario, se debe modificar y buscar la relacion entre triangulos al memento de cortar el triangulo
                                            break;    
                                        }
                                    //    std::cout << "Triangulo : "<< i << "\n" << std::endl;
                                    //    std::cout << " nodo: 0 -> "<< pos[0] << " " << pos[1] << " "<< pos[2]<< "\n" << std::endl;
                                    //    std::cout << " nodo: 1 -> "<< pos1[0] << " " << pos1[1] << " "<< pos1[2]<< "\n" << std::endl;
                                    //    std::cout << " nodo: 2 -> "<< pos2[0] << " " << pos2[1] << " "<< pos2[2]<< "\n" << std::endl;
                                    }
                                    ////////////////////////////////////////////////////////////////
                                    
                                ///////////////////////////////////////////////////////////////////
                            }
                            
                            ibuf->unlock();
                            
                        }


                    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
               
               }
           }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	    
        ////////////Investigando informacion de los faces en bullet/////////////////////////////////////
        std::cout << "Investigando informacion de los faces en bullet" << std::endl;
        int numFaces = body->m_faces.size();
        int numLinks = body->m_links.size();

           std::cout << "numero de faces:  "<< numFaces <<std::endl;
           std::cout << "numero de links:  "<< numLinks <<std::endl;
           
           indexBulletFaces = new int[numFaces];
           
           
           for (size_t i = 0; i < numFaces; i++) {
               btVector3 pos = body->m_faces[i].m_n[0]->m_x;
               btVector3 pos1 = body->m_faces[i].m_n[1]->m_x;
               btVector3 pos2 = body->m_faces[i].m_n[2]->m_x; 
        
          //     std::cout << "Triangulo Bullet: "<< i << "\n" << std::endl;
          //     std::cout << " nodo: 0 -> "<< pos[0] << " " << pos[1] << " "<< pos[2]<< "\n" << std::endl;
          //     std::cout << " nodo: 1 -> "<< pos1[0] << " " << pos1[1] << " "<< pos1[2]<< "\n" << std::endl;
          //     std::cout << " nodo: 2 -> "<< pos2[0] << " " << pos2[1] << " "<< pos2[2]<< "\n" << std::endl;

                
            }
          
          //Investigando links
          /*
          for(size_t i = 0; i< numLinks; i++){
               std::cout << "Mostrando Link: "<< i << "\n" << std::endl;
               btVector3 pos = body->m_links[i].m_n[0]->m_x;
               btVector3 pos1 = body->m_links[i].m_n[1]->m_x;

               std::cout << " Link: 0 -> "<< pos[0] << " " << pos[1] << " "<< pos[2]<< "\n" << std::endl;
               std::cout << " Link: 1 -> "<< pos1[0] << " " << pos1[1] << " "<< pos1[2]<< "\n" << std::endl;

          }*/

        ////////////////////////////////////////////////////////////////
        mObject = body;
	    	    
        
        getDynamicsWorld()->addSoftBody(this, mCollisionGroup, mCollisionMask);
        //getSoftDynamicsWorld()->addSoftBody(cloth);

        //std::cout << "\n----- CONTEO ------" << std::endl;
        //std::cout << "Cantidad de Soft bodies: " << myWorld->getSoftBodyArray().size() << std::endl;

        //pendiente revisar diccionario
        for(int z = 0; z<numFaces;z++){
          // std::cout << "diccionario bullet (i): " << z<< " Ogre (k): "<< indexBulletFaces[z] <<std::endl; 
        }
    }

    void SoftBody::UpdateMesh(){
           
           
           /*
           std::cout << "\n----- Actualizando objeto fisico!!! j ------"<< std::endl;
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
               //    actualizando informacion de los vertices
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
                   // std::cout << "probando  vertex count: " << vertexCount <<"\n" << std::endl;
                    for (unsigned int j = 0; j < vertexCount; j++)
                        {
                                  
                            posElem->baseVertexPointerToElement(pVert , &pReal);
                            pVert += vSize;

                               // (*(pReal++)) = npoints[indexBulletNodes[j]].x;    
                                //(*(pReal++)) = npoints[indexBulletNodes[j]].y;    
                                //(*(pReal++)) = npoints[indexBulletNodes[j]].z;
                                (*(pReal++)) = npoints[j].x;    
                                (*(pReal++)) = npoints[j].y;    
                                (*(pReal++)) = npoints[j].z;

                            //std::cout << "probando  vertex num: " << j << "indexBuleetNodes: "<< indexBulletNodes[j]  <<"\n" << std::endl; 
                          
                               
                        }
                    
                   
                    vbuf->unlock();
   ///////////////////////////////////Actualizando indices    
                    //VertexIndexToShape::addIndexData(sub_mesh->indexData, mVertexCount);
                    Ogre::IndexData *data2 = sub_mesh->indexData;
                    int numIndices = data2->indexCount;
                    int mIndexCount;
                    //std::cout << "index count: " <<numIndices <<std::endl;
                    //VertexIndexToShape::addStaticVertexData(sub_mesh->vertexData);
                    
                        
                        
                        mIndexCount = (unsigned int)data2->indexCount;
                        const unsigned int prev_size = mIndexCount;
                      //  std::cout << "prev_size: " <<prev_size <<std::endl;
                        unsigned int *tmp_ind = new unsigned int[mIndexCount];
                        unsigned int  *mIndexBuffer;
                     /*
                        if (mIndexBuffer)
                        {
                            memcpy (tmp_ind, mIndexBuffer, sizeof(unsigned int) * prev_size);
                            delete[] mIndexBuffer;
                        }
                     */   
                        mIndexBuffer = tmp_ind;
                        
                        const unsigned int numTris = (unsigned int) data2->indexCount / 3;
                        //std::cout << "numTris: " <<numTris <<std::endl;
                        
                        Ogre::HardwareIndexBufferSharedPtr ibuf = data2->indexBuffer;
                        
                        const bool use32bitindexes = (ibuf->getType() == Ogre::HardwareIndexBuffer::IT_32BIT);
                        
                        unsigned int index_offset = prev_size;
                        unsigned int offset  = vertexCount;
                        if (use32bitindexes) 
                        {
                            std::cout << "entre al if " <<std::endl;
                            /* //Aqui voy
                            const unsigned int *pInt = static_cast<unsigned int*>(ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
                            for (unsigned int k = 0; k < numTris; ++k)
                            {
                                mIndexBuffer[index_offset ++] = offset + *pInt++;
                                mIndexBuffer[index_offset ++] = offset + *pInt++;
                                mIndexBuffer[index_offset ++] = offset + *pInt++;
                            }
                            ibuf->unlock();
                            */
                        }
                        else 
                        {
                            const unsigned short *pShort = static_cast<unsigned short*>(ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
                            
                            for(unsigned int k = 0; k < numTris; ++k)
                            {
                                /*
                                offset + static_cast<unsigned int>(*pShort++) = mIndexBuffer[index_offset ++] ;
                                offset + static_cast<unsigned int>(*pShort++) = mIndexBuffer[index_offset ++] ;
                                offset + static_cast<unsigned int>(*pShort++) = mIndexBuffer[index_offset ++] ;
                                */
                                offset + static_cast<unsigned int>(*pShort++) ;
                                offset + static_cast<unsigned int>(*pShort++) ;
                                offset + static_cast<unsigned int>(*pShort++) ;
                            }
                            
                            ibuf->unlock();
                            
                        }
                        
               }
           }
          

           // std::cout << "FIN DE ACTUALIZACION DE OBJETO SUAVE:  \n" << std::endl;       

    }

    void SoftBody::UpdateCut(Ogre::Real cutnode){
           
           int numFaces = static_cast<btSoftBody*>(mObject)->m_faces.size();
           int numLinks = static_cast<btSoftBody*>(mObject)->m_links.size();

           //std::cout << "numero de faces:  "<< numFaces <<std::endl;
           //std::cout << "numero de links:  "<< numLinks <<std::endl;
           
           std::deque<Ogre::Vector3> npoints;
           
           for (size_t i = 0; i < numFaces; i++) {
               btVector3 pos = static_cast<btSoftBody*>(mObject)->m_faces[i].m_n[0]->m_x;
               btVector3 pos1 = static_cast<btSoftBody*>(mObject)->m_faces[i].m_n[1]->m_x;
               btVector3 pos2 = static_cast<btSoftBody*>(mObject)->m_faces[i].m_n[2]->m_x; 
               npoints.push_back(Ogre::Vector3(pos[0],pos[1],pos[2]));
               /*
               std::cout << "Triangulo before : "<< i << "\n" << std::endl;
               std::cout << " nodo: 0 -> "<< pos[0] << " " << pos[1] << " "<< pos[2]<< "\n" << std::endl;
               std::cout << " nodo: 1 -> "<< pos1[0] << " " << pos1[1] << " "<< pos1[2]<< "\n" << std::endl;
               std::cout << " nodo: 2 -> "<< pos2[0] << " " << pos2[1] << " "<< pos2[2]<< "\n" << std::endl;*/
           }

            std::cout << "CORTANDO TRIANGULO: " << cutnode << "!!!!!!"<<std::endl;
            int cutN = static_cast<int>(cutnode);
           // static_cast<btSoftBody*>(mObject)->m_faces.removeAtIndex(indexBulletFaces[cutN] );
           //static_cast<btSoftBody*>(mObject)->m_faces.removeAtIndex(cutN);

            /*
              for(int i = 0;i<numLinks;i++){
                  static_cast<btSoftBody*>(mObject)->m_links.removeAtIndex(i);
              }
            */          
            //static_cast<btSoftBody*>(mObject)->m_nodes.removeAtIndex(cutN);
            
            //btSoftBodySolver * bsolver = new btSoftBodySolver::btSoftBodySolver();
            //bsolver->updateSoftBodies();
            //btSoftBodySolver::updateSoftBodies();
            numFaces = static_cast<btSoftBody*>(mObject)->m_faces.size();
           // std::cout << "nuevo size: " << numFaces <<std::endl;
            /* For para vericar que se quito un trinagulo
            for (size_t i = 0; i < numFaces; i++) {
                  
               btVector3 pos = static_cast<btSoftBody*>(mObject)->m_faces[i].m_n[0]->m_x;
               btVector3 pos1 = static_cast<btSoftBody*>(mObject)->m_faces[i].m_n[1]->m_x;
               btVector3 pos2 = static_cast<btSoftBody*>(mObject)->m_faces[i].m_n[2]->m_x; 
               npoints.push_back(Ogre::Vector3(pos[0],pos[1],pos[2]));
               
               std::cout << "Triangulo after: "<< i << "\n" << std::endl;
               std::cout << " nodo: 0 -> "<< pos[0] << " " << pos[1] << " "<< pos[2]<< "\n" << std::endl;
               std::cout << " nodo: 1 -> "<< pos1[0] << " " << pos1[1] << " "<< pos1[2]<< "\n" << std::endl;
               std::cout << " nodo: 2 -> "<< pos2[0] << " " << pos2[1] << " "<< pos2[2]<< "\n" << std::endl;
           }*/
            //std::cout << "Recuperando entidad para cortarla : "<< this->mEntity->getName() << "\n" << std::endl;

//actualzando indices en ogre
        /*
            for (unsigned int i = 0; i < this->mEntity->getNumSubEntities(); ++i){
               Ogre::SubMesh *sub_mesh = this->mEntity->getSubEntity(i)->getSubMesh();
                Ogre::VertexData *data = sub_mesh->vertexData;
                const unsigned int vertexCount = (unsigned int)data->vertexCount;

               ///////////////////////////////////Actualizando indices    
                    //VertexIndexToShape::addIndexData(sub_mesh->indexData, mVertexCount);
                    Ogre::IndexData *data2 = sub_mesh->indexData;
                    int numIndices = data2->indexCount;
                    int mIndexCount;
                    //std::cout << "index count: " <<numIndices <<std::endl;
                    //VertexIndexToShape::addStaticVertexData(sub_mesh->vertexData);
                    
                        
                        
                        mIndexCount = (unsigned int)data2->indexCount;
                        const unsigned int prev_size = mIndexCount;
                      //  std::cout << "prev_size: " <<prev_size <<std::endl;
                        unsigned int *tmp_ind = new unsigned int[mIndexCount];
                        unsigned int  *mIndexBuffer;
                  
                        mIndexBuffer = tmp_ind;
                        
                        const unsigned int numTris = (unsigned int) data2->indexCount / 3;
                        //std::cout << "numTris: " <<numTris <<std::endl;
                        
                        Ogre::HardwareIndexBufferSharedPtr ibuf = data2->indexBuffer;
                        
                        const bool use32bitindexes = (ibuf->getType() == Ogre::HardwareIndexBuffer::IT_32BIT);
                        
                        unsigned int index_offset = prev_size;
                        unsigned int offset  = vertexCount;
                        if (use32bitindexes) 
                        {
                            std::cout << "entre al if " <<std::endl;
       
                        }
                        else 
                        {
                           
                             unsigned short *pShort = static_cast<unsigned short*>(ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
                           
                            //pShort[3*cutN]= 0;
                           
                            for(unsigned int k = 0; k < numTris; ++k)
                            {
                                    
                                if (k==cutN){
                                    *(pShort++) = 200;
                                    *pShort++ = 200;
                                    *pShort++ = 200;
                                }else {
                                    offset + static_cast<unsigned int>(*pShort++) ;
                                    offset + static_cast<unsigned int>(*pShort++) ;
                                    offset + static_cast<unsigned int>(*pShort++) ;    
                                }
                                
                                
                                
                                //offset + static_cast<unsigned int>(*pShort++) = mIndexBuffer[index_offset ++] ;
                                //offset + static_cast<unsigned int>(*pShort++) = mIndexBuffer[index_offset ++] ;
                                //offset + static_cast<unsigned int>(*pShort++) = mIndexBuffer[index_offset ++] ;

                            }
                             
                            ibuf->unlock();
                           
                        }
            
            }
            */
            //actualizar la animacion en bullet
            //(myWorld->getWorldInfo()
            static_cast<btSoftBody*>(mObject)->m_worldInfo->m_sparsesdf.Reset();
            
    }    
}
