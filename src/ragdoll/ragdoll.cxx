#include <iostream>
#include <pujOgreBullet/Application.h>

#include <OgreManualObject.h>
#include <OgreCamera.h>
#include <OgreEntity.h>
#include <OgreMeshManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreMesh.h>
#include <OgreSubMesh.h>
#include <OgreRay.h>  
#include <OgrePlane.h>
#include <OgreMeshManager.h>
#include <OgreColourValue.h>
#include <OgreRoot.h>
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"

#include "../softObject/SceneSoftObject.h" 

//include vrpn descomentariar cuando se tenga instalado
#include "../devices/VRPN_Device.h"
#include "../devices/VRPN_PhantomOmni.h"
#include <OgreLogManager.h>


/**
 */
class RagDollApp
  : public pujOgreBullet::Application
{
public:
  typedef RagDollApp                 Self;
  typedef pujOgreBullet::Application Superclass;

public:
  RagDollApp( );
  virtual ~RagDollApp( );
  Ogre::Vector3 direccion;
  Ogre::Vector3 posTool;
  typedef std::vector< VRPN_Device* > TDevices; 
//  Ogre::Vector3 *vertices;
//  Ogre::uint32 *indices;
//  size_t vertex_count;
//  size_t index_count;
  void setMesh();
  void createColourCube();
  void updateMesh(int trian );
  void UpdatePhysics(std::chrono::duration<double> deltaTime);	
protected:
  virtual void createScene( ) override;
  virtual void createCamera( ) override;
  virtual bool frameRenderingQueued( const Ogre::FrameEvent& evt ) override;
  TDevices m_Devices;  
  Ogre::AnimationState* m_AnimationState;
};

// -------------------------------------------------------------------------
// Main
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
#endif

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
  INT WINAPI WinMain( HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT )
#else // OGRE_PLATFORM == OGRE_PLATFORM_WIN32
    int main( int argc, char* argv[] )
#endif // OGRE_PLATFORM == OGRE_PLATFORM_WIN32
  {
    // Create application object
    RagDollApp app;
    try
    {
      app.go( );
    }
    catch( Ogre::Exception& e )
    {
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
      MessageBox( NULL, e.getFullDescription().c_str(), "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else // OGRE_PLATFORM == OGRE_PLATFORM_WIN32
      std::cerr
        << "An exception has occured: "
        << e.getFullDescription( ).c_str( )
        << std::endl;
#endif // OGRE_PLATFORM == OGRE_PLATFORM_WIN32
    } // yrt
    return( 0 );
  }
#ifdef __cplusplus
}
#endif // __cplusplus

// ----------------------------------------------

// ----------------------------------------------------------------------------------
RagDollApp::
RagDollApp( )
  : Superclass( )
{
}

// -------------------------------------------------------------------------
RagDollApp::
~RagDollApp( )
{
}

// -------------------------------------------------------------------------
void RagDollApp::
createScene( )
{
  this->Superclass::createScene( );



 // Create a plane
  Ogre::Plane plane( Ogre::Vector3::UNIT_Y, 0 );
  Ogre::MeshManager::getSingleton( ).
    createPlane(
      "plane",
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      plane,
      150, 150, 20, 20, true, 1, 5, 5,
      Ogre::Vector3::UNIT_Z
      );

  // Associate a floor entity with the created plane
  Ogre::Entity* floor = this->m_SceneMgr->createEntity( "floor", "plane" );
  floor->setMaterialName( "Mat2" );
  this->m_SceneMgr->getRootSceneNode( )->attachObject( floor );

  // Associate plane to the physical world
  this->addPhysicsPlane( plane, "plane_physics", 0.000001, 0.00001 );


 
   ///Prueba generacion manual de un mesh /////comentariado para probar el manualobject
  /*
  createColourCube(); 
  Ogre::Entity* thisEntity2 = this->m_SceneMgr->createEntity("pielEnity2", "ColourCube");
  thisEntity2->setMaterialName("Mat");
  Ogre::AxisAlignedBox bboxSoft2 = thisEntity2->getBoundingBox( );
  Ogre::SceneNode* thisSceneNode2 = this->m_SceneMgr->getRootSceneNode()->createChildSceneNode();
  thisSceneNode2->setPosition(0, 5, 0);
  thisSceneNode2->attachObject(thisEntity2);
  //std::cout << "nombre de la entidad:  "<< thisEntity->getName() << "\n" << std::endl;

//Agregar el objeto blando al mundo fisico
  Ogre::Quaternion qsoft1( 1, 1, 1, 1 );
  qsoft1.normalise( );
  this->addSoftPhysicsTrimesh(
    thisEntity2, thisSceneNode2, "soft_physics2", 0.0009, 0.0009, 10,
    Ogre::Vector3( 0, 5, 0 ),
    qsoft1
    );
 */
   ///prueba clase tipo suave
   //std::shared_ptr<SceneSoftObject> softObject(new SceneSoftObject(thisEntity));
  //SceneSoftObject* so = new SceneSoftObject(thisEntity);
  //softObject->InitSoftBody(this->m_BulletWorld->softBodyWorldInfo)

//prueba clase manual object
Ogre::ManualObject* man = this->m_SceneMgr->createManualObject("test");
man->begin("Mat", Ogre::RenderOperation::OT_TRIANGLE_LIST);

int numeroVertices = 10;
for(int n=0; n<numeroVertices;n++){
     for(int m=0; m<numeroVertices;m++){
       std::cout << "vertice:  "<< m << " "<< n << "\n" << std::endl;
        man->position(m, 10, n);
        man->normal(0, 1, 0);
     } 
  }


// https://forums.ogre3d.org/viewtopic.php?t=32747 revisar
    
    int totalVertices = numeroVertices * numeroVertices;

    for(int o=0;o<(totalVertices-numeroVertices);o+=numeroVertices){
      for(int p=0;p<numeroVertices-1;p++){
        //se ajusta la forma en que se arman los triangulos para que queden igual como los arma bullet
        //man->triangle(o+p,o+numeroVertices+p,o+numeroVertices+p+1);
        //man->triangle(o+numeroVertices+p+1,o+1+p,o+p);
        if((o+p)&1){
          man->triangle(o+p,o+numeroVertices+p,o+numeroVertices+p+1);
          man->triangle(o+p,o+numeroVertices+p+1,o+1+p); 
        }else{
          man->triangle(o+1+p,o+p,o+numeroVertices+p);
          man->triangle(o+1+p,o+numeroVertices+p,o+numeroVertices+p+1);
        }   
      }
    }

    man->end();
    //Ogre::SceneNode* manNode = this->m_SceneMgr->getRootSceneNode()->createChildSceneNode();
    //manNode->attachObject(man); 

    Ogre::MeshPtr meshTest = man->convertToMesh("test");
    //Agregar el objeto blando al mundo geometrico y al mundo fisico
    Ogre::Entity* thisEntity = this->m_SceneMgr->createEntity("pielEnity", "test");
      //thisEntity->setMaterialName("Mat");
      Ogre::AxisAlignedBox bboxSoft = thisEntity->getBoundingBox( );
      Ogre::SceneNode* thisSceneNode = this->m_SceneMgr->getRootSceneNode()->createChildSceneNode();
      //thisSceneNode->setPosition(0, 5, 0);
      thisSceneNode->attachObject(thisEntity);
      std::cout << "nombre de la entidad:  "<< thisEntity->getName() << "\n" << std::endl;

    //Agregar el objeto blando al mundo fisico
      Ogre::Quaternion qsoft( 1, 1, 1, 1 );
      qsoft.normalise( );
      this->addSoftPhysicsTrimesh(
        thisEntity, thisSceneNode, "soft_physics", 0.0009, 0.0009, 10,
        Ogre::Vector3( 0, 0, 0 ),
        qsoft
        );

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Lights
  this->m_SceneMgr->setAmbientLight( Ogre::ColourValue( 1, 1, 1 ) );
  this->m_SceneMgr->setShadowTechnique( Ogre::SHADOWTYPE_STENCIL_ADDITIVE );

  Ogre::Light* light1 = this->m_SceneMgr->createLight( "Light1" );
  light1->setType( Ogre::Light::LT_POINT );
  light1->setPosition( Ogre::Vector3( 50, 50, 50 ) );
  light1->setDiffuseColour( 1.0, 1.0, 1.0 );
  light1->setSpecularColour( 1.0, 1.0, 1.0 );

  Ogre::Light* light2 = this->m_SceneMgr->createLight( "Light2" );
  light2->setType( Ogre::Light::LT_POINT );
  light2->setPosition( Ogre::Vector3( 0, 50, -50 ) );
  light2->setDiffuseColour( 0.5, 0.5, 0.5 );
  light2->setSpecularColour( 0.5, 0.5, 0.5 );

 
// -------------------------------------------------------
  // Load model entity
  Ogre::Entity* ninja =
    this->m_SceneMgr->createEntity(
      "ninja", "cyborg_ninja.mesh"
      );
  ninja->setCastShadows( true );
  Ogre::AxisAlignedBox bbox = ninja->getBoundingBox( );
/*
  // Associate it to a node
  Ogre::SceneNode* ninja_node =
    this->m_SceneMgr->getRootSceneNode( )->createChildSceneNode(
      "ninja_node"
      );
  ninja_node->attachObject( ninja );
  ninja_node->translate(5,10,5);
  */
  
  //////////Objeto de prueba
    // Load model entity
  Ogre::Entity* ship =
    this->m_SceneMgr->createEntity(
      "ship", "Sphere.mesh"
      );
  ship->setCastShadows( true );
  Ogre::AxisAlignedBox bbox2 = ship->getBoundingBox( );

  // Associate it to a node
  Ogre::SceneNode* ship_node =
    this->m_SceneMgr->getRootSceneNode( )->createChildSceneNode(
      "ship_node"
      );
  ship_node->attachObject( ship );

  /* el objeto cargado de blender no se deja actualizar por el objeto de bullet debido a que viene por defecto con el parametro de sharevertex en true y debe ser false
  //////////Objeto piel
    // Load model entity
  Ogre::Entity* tissue =
    this->m_SceneMgr->createEntity(
      "tissue", "Tissue.mesh"
      );
  tissue->setCastShadows( true );
  Ogre::AxisAlignedBox bbox3 = tissue->getBoundingBox( );
  tissue->setMaterialName( "Mat" );
  // Associate it to a node
  Ogre::SceneNode* tissue_node =
    this->m_SceneMgr->getRootSceneNode( )->createChildSceneNode(
      "tissue_node"
      );
  tissue_node->attachObject( tissue );
  tissue_node->translate(0,1,0);
  setMesh();   

    //Agregar el objeto blando al mundo fisico
      Ogre::Quaternion qsoft2( 1, 1, 1, 1 );
      qsoft2.normalise( );
      this->addSoftPhysicsTrimesh(
        tissue, tissue_node, "tiss_physics", 0.0009, 0.0009, 10,
        Ogre::Vector3( 0, 10, 0 ),
        qsoft2
        );
*/


//Se puede transformar el objeto piel en ogre o en blender (para blender recordar aplicar con ctrl + A)

  ////////////////////////////Objeto herramienta/////////////////////////////////////
    // Load model entity
  Ogre::Entity* tool =
    this->m_SceneMgr->createEntity(
      "tool", "tool.mesh"
      );
  tool->setCastShadows( true );
  Ogre::AxisAlignedBox bbox4 = tool->getBoundingBox( );
  
  // Associate it to a node
  Ogre::SceneNode* tool_node =
    this->m_SceneMgr->getRootSceneNode( )->createChildSceneNode(
      "tool_node"
      );
  tool_node->attachObject( tool );
  tool_node->translate(0,15,0);

  posTool   = Ogre::Vector3(0,15,0); 
  direccion = Ogre::Vector3(0,14,0);

// Associate tool to the physical world
  Ogre::Quaternion qTool( 1, 1, 2, 3 );
  qTool.normalise( );
  //this->addPhysicsPlane(
  //  tool,  "tool_physics", 0.000001, 0.00001);

//  this->addPhysicsPlane( plane, "plane_physics", 0.000001, 0.00001 );
/////////////////////////////////////////////////////////////////////////////////////
/*
  // Associate ninja to the physical world
  Ogre::Quaternion q( 1, 1, 2, 3 );
  q.normalise( );
  this->addPhysicsConvex(
    ninja, ninja_node, "ninja_physics", 0.0009, 0.0009, 75,
    Ogre::Vector3( 5, 20, 5 ),
    q
    );
*/

 //indi = new int[ibufCount/3];

  for(short i=0 ; i<(ibufCount/3) ; i++){
    indi[i]= 1;
  }


  //conectando phantom
  //creando phantom, OJO: correr el archivo vrpn_server de la maquina del servidor
  VRPN_PhantomOmni* phantom0 = new VRPN_PhantomOmni( );

  if( !( phantom0->connect( "Phantom@10.3.137.56" ) ) )
  {
    Ogre::LogManager::getSingletonPtr( )->
      logMessage( "!!! Error connecting to Phantom@host !!!" );
    delete phantom0;
    phantom0 = NULL;
  }
  this->m_Devices.push_back( phantom0 );

}

// -------------------------------------------------------------------------
void RagDollApp::
createCamera( )
{
  this->Superclass::createCamera( );
  this->m_Camera->setPosition( Ogre::Vector3( 25, 25, 25 ) );
  this->m_Camera->lookAt( Ogre::Vector3( 0, 10, 0 ) );
  this->m_Camera->setNearClipDistance( 5 );
}

void RagDollApp::setMesh(){
    std::cout << "entrando" << std::endl;
    Ogre::Entity* planeBlender = this->m_SceneMgr->getEntity("tissue");

//////////////////////////Obteniendo informacion de la malla de la piel//////////////////////////////////////////
      const Ogre::MeshPtr mesh = planeBlender->getMesh();           
      const Ogre::Vector3 position = planeBlender->getParentNode()->_getDerivedPosition();
      const Ogre::Quaternion orient = planeBlender->getParentNode()->_getDerivedOrientation();
      const Ogre::Vector3 scale = planeBlender->getParentNode()->_getDerivedScale();
      bool  added_shared = false;

      size_t current_offset = 0;
      size_t shared_offset = 0;
      size_t next_offset = 0;
      size_t index_offset = 0;

      vertex_count = index_count = 0;

      // Calculate how many vertices and indices we're going to need
    for ( unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
    {
        Ogre::SubMesh* submesh = mesh->getSubMesh(i);
        // We only need to add the shared vertices once
        if(submesh->useSharedVertices)
        {
            if( !added_shared )
            {
                vertex_count += mesh->sharedVertexData->vertexCount;
                added_shared = true;
            }
        }
        else
        {
            vertex_count += submesh->vertexData->vertexCount;
        }
        // Add the indices
        index_count += submesh->indexData->indexCount;
    }

    // Allocate space for the vertices and indices
    vertices = new Ogre::Vector3[vertex_count];
    indices = new  Ogre::uint32[index_count];

    added_shared = false;

    // Run through the submeshes again, adding the data into the arrays
    for (unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i)
    {
        Ogre::SubMesh* submesh = mesh->getSubMesh(i);

        Ogre::VertexData* vertex_data = submesh->useSharedVertices ? mesh->sharedVertexData : submesh->vertexData;

        if ((!submesh->useSharedVertices) || (submesh->useSharedVertices && !added_shared))
        {
            if(submesh->useSharedVertices)
            {
                added_shared = true;
                shared_offset = current_offset;
            }

            const Ogre::VertexElement* posElem =
                vertex_data->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);

            Ogre::HardwareVertexBufferSharedPtr vbuf =
                vertex_data->vertexBufferBinding->getBuffer(posElem->getSource());

            unsigned char* vertex =
                static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

            // There is _no_ baseVertexPointerToElement() which takes an Ogre::Real or a double
            //  as second argument. So make it float, to avoid trouble when Ogre::Real will
            //  be comiled/typedefed as double:
            //Ogre::Real* pReal;
            float* pReal;

            for( size_t j = 0; j < vertex_data->vertexCount; ++j, vertex += vbuf->getVertexSize())
            {
                posElem->baseVertexPointerToElement(vertex, &pReal);
                Ogre::Vector3 pt(pReal[0], pReal[1], pReal[2]);
                vertices[current_offset + j] = (orient * (pt * scale)) + position;
            }
            
            vbuf->unlock();
            next_offset += vertex_data->vertexCount;
        }

        Ogre::IndexData* index_data = submesh->indexData;
        size_t numTris = index_data->indexCount / 3;
        Ogre::HardwareIndexBufferSharedPtr ibuf = index_data->indexBuffer;
        
        bool use32bitindexes = (ibuf->getType() == Ogre::HardwareIndexBuffer::IT_32BIT);

        unsigned long* pLong = static_cast<unsigned long*>(ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
        unsigned short* pShort = reinterpret_cast<unsigned short*>(pLong);

        size_t offset = (submesh->useSharedVertices)? shared_offset : current_offset;

        if ( use32bitindexes )
        {
            for ( size_t k = 0; k < numTris*3; ++k)
            {
                indices[index_offset++] = pLong[k] + static_cast<unsigned long>(offset);
            }
        }
        else
        {
            for ( size_t k = 0; k < numTris*3; ++k)
            {
                indices[index_offset++] = static_cast<unsigned long>(pShort[k]) +
                                          static_cast<unsigned long>(offset);
            }
        }

        ibuf->unlock();
        current_offset = next_offset;
    }
 
    bool new_closest_found = false;
    for (size_t i = 0; i < index_count; i += 3)
    {
      // check for a hit against this triangle
     // std::cout<<"triangle "<<i<<": "<<vertices[indices[i]]<<" "<<
     // vertices[indices[i+1]]<<" "<< vertices[indices[i+2]]<<std::endl;
    }


////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}


bool pujOgre::Application::
keyPressed( const OIS::KeyEvent& arg )
{
  
  
  OIS::KeyCode a = arg.key;
  ///moviendo la camara
  //this->m_Camera->setPosition( Ogre::Vector3( 25, 25, 25 ) );
    
   // if( this->m_Keyboard->isKeyDown( OIS::KC_LSHIFT ) )
    //  moveScale *= 10;
    Ogre::Vector3 posCamera = this->m_Camera->getPosition();
    if( this->m_Keyboard->isKeyDown( OIS::KC_J ) ) {
        // Move camera left
      posCamera.x = posCamera.x + 1;
      this->m_Camera->setPosition( posCamera );
      }

    if( this->m_Keyboard->isKeyDown( OIS::KC_L ) ){
      posCamera.x = posCamera.x - 1;
      this->m_Camera->setPosition( posCamera );  // Move camera right
    }
      

    if(this->m_Keyboard->isKeyDown( OIS::KC_I )){
      posCamera.z = posCamera.z + 0.5;
      this->m_Camera->setPosition( posCamera );// Move camera forward
    }
      

    if(this->m_Keyboard->isKeyDown( OIS::KC_K )){
      posCamera.z = posCamera.z - 0.5;
      this->m_Camera->setPosition( posCamera );  // Move camera backward
    }
      

    if( this->m_Keyboard->isKeyDown( OIS::KC_Y ) ){
      posCamera.y = posCamera.y + 0.5;
      this->m_Camera->setPosition( posCamera );  // Move camera up
    }
      

    if( this->m_Keyboard->isKeyDown( OIS::KC_H ) ){
      posCamera.y = posCamera.y - 0.5;
      this->m_Camera->setPosition( posCamera ); // Move camera down
    }
      
   Ogre::Degree rotate = Ogre::Degree(1);
  
    if( this->m_Keyboard->isKeyDown( OIS::KC_RIGHT ) ){
      std::cout <<"rotando derecha "<< "\n";
      this->m_Camera->yaw( -rotate );
    }
      

    if( this->m_Keyboard->isKeyDown( OIS::KC_LEFT ) ){
      std::cout <<"rotando izquierda "<< "\n";
      this->m_Camera->yaw( rotate );
    }
       

    if(
      this->m_Keyboard->isKeyDown( OIS::KC_ESCAPE ) ||
      this->m_Keyboard->isKeyDown( OIS::KC_Q )
      )
      return( false );


  //moviendo el instrumento
  Ogre::SceneNode* planeBlender_node = this->m_SceneMgr->getSceneNode("tool_node");
  float dx  = 0;
  float dy  = 0;
  float dz  = 0;
  if(OIS::KC_W==a){
    dx = 0.1;
  }
  
  if(OIS::KC_S==a){
    dx = -0.1;
  }

  if(OIS::KC_A==a){
    dz = -0.1;
  }

  if(OIS::KC_D==a){
    dz = 0.1;
  }

  if(OIS::KC_UP==a){
    dy = 0.1;
  }

  if(OIS::KC_DOWN==a){
    dy = -0.1;
  } 
    planeBlender_node->translate(dx,dy,dz);
    


  /*
/////////////////////////////////////Buscando si hay colision/////////////////////////////////////////////////////////////////
  //Ogre::Vector3 posTool = Ogre::Vector3(posBall[0],posBall[1],posBall[2]);
  Ogre::Plane plano;
  Ogre::Vector3 posTool = planeBlender_node->getPosition();
  posTool.y=posTool.y-6.5; //la punta de instrumento esta siete unidades abajo del centro
  //std::cout <<"posx "<< posTool.x << " posy " << posTool.y << " posz " << posTool.z <<"\n";
  Ogre::Vector3 normal=Ogre::Vector3(posTool.x,-0.5,posTool.z);
  //std::cout <<"dirx "<< normal.x << " diry " << normal.y << " dirz " << normal.z <<"\n";
  Ogre::Ray toolRay = Ogre::Ray(posTool, normal);
  //std::cout<< "origen rayo " << toolRay.getOrigin() << " direccion " << toolRay.getDirection() << "\n";
  
  //Plano blender
  for (size_t i = 0; i < index_count; i += 3){
    plano = Ogre::Plane(vertices[indices[i]],vertices[indices[i+1]],vertices[indices[i+2]]);
    std::pair<bool,Ogre::Real> inter = Ogre::Math::intersects(toolRay,vertices[indices[i]],vertices[indices[i+1]],vertices[indices[i+2]],true , true);
    
    if(inter.first && inter.second < 1){
    //  std::cout <<"colision triangulo objetivo: "<< i <<"\n"; 
    }
  }

  //Plano manual
  int aux=0;
  for(size_t i = 0; i< ibufCount ; i += 3){
      //std::cout <<"tirangulo "<< aux <<" : "<< faces[i] << " " << faces[i+1] << " " << faces[i+2]<<"\n"; 
      int side_a = faces[i];
      int side_b = faces[i+1];
      int side_c = faces[i+2];
      
      Ogre::Vector3  pointA =  Ogre::Vector3(verts[side_a*6],verts[(side_a*6)+1],verts[(side_a*6)+2]);
      Ogre::Vector3  pointB =  Ogre::Vector3(verts[side_b*6],verts[(side_b*6)+1],verts[(side_b*6)+2]);
      Ogre::Vector3  pointC =  Ogre::Vector3(verts[side_c*6],verts[(side_c*6)+1],verts[(side_c*6)+2]);
      
      //std::cout <<"tirangulo "<< aux <<" : "<< faces[i] << " " << faces[i+1] << " " << faces[i+2]<<"\n";
      std::pair<bool,Ogre::Real> inter = Ogre::Math::intersects(toolRay, pointA , pointB , pointC ,true , true);
      //std::cout <<"first: "<< inter.first << "second: " << inter.second <<"\n";
      if(inter.first && inter.second < 1){
      //std::cout <<"colision triangulo objetivo: "<< aux <<"\n";
        RagDollApp  * a  = new RagDollApp();
         a->RagDollApp::updateMesh( aux ); //renderizar de nuevo el mesh sin el triangulo colisionado 
      }

      aux++;

  }
////////////////////////////////////Fin Buscando colision/////////////////////////////////////////////////////////////////////
*/
  return( true );
}



void RagDollApp::createColourCube()
{
    /// Create the mesh via the MeshManager
    Ogre::MeshPtr msh = Ogre::MeshManager::getSingleton().createManual("ColourCube", "General");

    /// Create one submesh
    Ogre::SubMesh* sub = msh->createSubMesh();

    //const float sqrt13 = 0.577350269f; /* sqrt(1/3) */

    /// Define the vertices (16 vertices, each have 3 floats for position and 3 for normal)
    //const size_t nVertices = 16;
    //const size_t vbufCount = 3*2*nVertices;
    //float verts[vbufCount] = {
    /*
     float  verts[]   = {  
            0.0,0.0,0.0,                //0 position
            -sqrt13,sqrt13,-sqrt13,     //0 normal
            2.0,0.0,0.0,                //1 position
            sqrt13,sqrt13,-sqrt13,      //1 normal
            4.0,0.0,0.0,                //2 position
            sqrt13,-sqrt13,-sqrt13,     //2 normal
            6.0,0.0,0.0,                //3 position
            -sqrt13,-sqrt13,-sqrt13,    //3 normal
            0.0,0.0,2.0,                //4 position
            -sqrt13,sqrt13,sqrt13,      //4 normal
            2.0,0.0,2.0,                //5 position
            sqrt13,sqrt13,sqrt13,       //5 normal
            4.0,0.0,2.0,                //6 position
            sqrt13,-sqrt13,sqrt13,      //6 normal
            6.0,0.0,2.0,                //7 position
            -sqrt13,-sqrt13,sqrt13,     //7 normal
            0.0,0.0,4.0,                //8 position
            -sqrt13,sqrt13,-sqrt13,     //8 normal
            2.0,0.0,4.0,                //9 position
            sqrt13,sqrt13,-sqrt13,      //9 normal
            4.0,0.0,4.0,                //10 position
            sqrt13,-sqrt13,-sqrt13,     //10 normal
            6.0,0.0,4.0,                //11 position
            -sqrt13,-sqrt13,-sqrt13,    //11 normal
            0.0,0.0,6.0,                //12 position
            -sqrt13,sqrt13,sqrt13,      //12 normal
            2.0,0.0,6.0,                //13 position
            sqrt13,sqrt13,sqrt13,       //13 normal
            4.0,0.0,6.0,                //14 position
            sqrt13,-sqrt13,sqrt13,      //14 normal
            6.0,0.0,6.0,                //15 position
            -sqrt13,-sqrt13,sqrt13,     //15 normal
          };
    */
   //cube mesh 
  //  float vertices[vbufCount] = {
  //          -10.0,10.0,-10.0,        //0 position
  //          -sqrt13,sqrt13,-sqrt13,     //0 normal
  //          10.0,10.0,-10.0,         //1 position
  //          sqrt13,sqrt13,-sqrt13,      //1 normal
  //          10.0,-10.0,-10.0,        //2 position
  //          sqrt13,-sqrt13,-sqrt13,     //2 normal
  //          -10.0,-10.0,-10.0,       //3 position
  //          -sqrt13,-sqrt13,-sqrt13,    //3 normal
  //          -10.0,10.0,10.0,         //4 position
  //          -sqrt13,sqrt13,sqrt13,      //4 normal
  //          10.0,10.0,10.0,          //5 position
  //          sqrt13,sqrt13,sqrt13,       //5 normal
  //          10.0,-10.0,10.0,         //6 position
  //          sqrt13,-sqrt13,sqrt13,      //6 normal
  //          -10.0,-10.0,10.0,        //7 position
  //          -sqrt13,-sqrt13,sqrt13,     //7 normal
  //  };

    
    Ogre::RenderSystem* rs = this->m_Root->getSingleton().getRenderSystem();
    //Root::getSingleton().getRenderSystem();
    Ogre::RGBA colours[nVertices];
    Ogre::RGBA *pColour = colours;
    // Use render system to convert colour value since colour packing varies
    rs->convertColourValue(Ogre::ColourValue(1.0,0.0,0.0), pColour++); //0 colour
    rs->convertColourValue(Ogre::ColourValue(1.0,1.0,0.0), pColour++); //1 colour
    rs->convertColourValue(Ogre::ColourValue(0.0,1.0,0.0), pColour++); //2 colour
    rs->convertColourValue(Ogre::ColourValue(0.0,0.0,0.0), pColour++); //3 colour
    rs->convertColourValue(Ogre::ColourValue(1.0,0.0,1.0), pColour++); //4 colour
    rs->convertColourValue(Ogre::ColourValue(1.0,1.0,1.0), pColour++); //5 colour
    rs->convertColourValue(Ogre::ColourValue(0.0,1.0,1.0), pColour++); //6 colour
    rs->convertColourValue(Ogre::ColourValue(0.0,0.0,1.0), pColour++); //7 colour

    /// Define 12 triangles (two triangles per cube face)
    /// The values in this table refer to vertices in the above table
    /*
    const size_t ibufCount = 54;
    unsigned short faces[ibufCount] = {
            0,4,5,
            0,5,1,
            1,5,2,
            2,5,6,
            2,6,7,
            2,7,3,
            4,8,5,
            5,8,9,
            5,9,10,
            5,10,6,
            6,10,7,
            7,10,11,
            8,12,13,
            8,13,9,
            9,13,10,
            13,14,10,
            15,10,14,
            10,15,11
    };
    */
    /// Create vertex data structure for 8 vertices shared between submeshes
    msh->sharedVertexData = new Ogre::VertexData();
    msh->sharedVertexData->vertexCount = nVertices;

    /// Create declaration (memory format) of vertex data
    Ogre::VertexDeclaration* decl = msh->sharedVertexData->vertexDeclaration;
    size_t offset = 0;
    // 1st buffer
    decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
    decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
    /// Allocate vertex buffer of the requested number of vertices (vertexCount) 
    /// and bytes per vertex (offset)
    Ogre::HardwareVertexBufferSharedPtr vbuf = 
        Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
        offset, msh->sharedVertexData->vertexCount, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
    /// Upload the vertex data to the card
    vbuf->writeData(0, vbuf->getSizeInBytes(), verts, true);

    /// Set vertex buffer binding so buffer 0 is bound to our vertex buffer
    Ogre::VertexBufferBinding* bind = msh->sharedVertexData->vertexBufferBinding; 
    bind->setBinding(0, vbuf);

    // 2nd buffer
    offset = 0;
    decl->addElement(1, offset, Ogre::VET_COLOUR, Ogre::VES_DIFFUSE);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_COLOUR);
    /// Allocate vertex buffer of the requested number of vertices (vertexCount) 
    /// and bytes per vertex (offset)
    vbuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
        offset, msh->sharedVertexData->vertexCount, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
    /// Upload the vertex data to the card
    vbuf->writeData(0, vbuf->getSizeInBytes(), colours, true);

    /// Set vertex buffer binding so buffer 1 is bound to our colour buffer
    bind->setBinding(1, vbuf);

    /// Allocate index buffer of the requested number of vertices (ibufCount) 
    Ogre::HardwareIndexBufferSharedPtr ibuf = Ogre::HardwareBufferManager::getSingleton().
        createIndexBuffer(
        Ogre::HardwareIndexBuffer::IT_16BIT, 
        ibufCount, 
        Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);

    /// Upload the index data to the card
    ibuf->writeData(0, ibuf->getSizeInBytes(), faces, true);

    /// Set parameters of the submesh
    sub->useSharedVertices = true;
    sub->indexData->indexBuffer = ibuf;
    sub->indexData->indexCount = ibufCount;
    sub->indexData->indexStart = 0;

    /// Set bounding information (for culling)
    msh->_setBounds(Ogre::AxisAlignedBox(-100,-100,-100,100,100,100));
    msh->_setBoundingSphereRadius(Ogre::Math::Sqrt(3*100*100));

    /// Notify -Mesh object that it has been loaded
    msh->load();
}

 void RagDollApp::updateMesh(int trian ){
   std::cout <<"Aquiiiiiiiiiiiiiiiiiiiiiiiiiiiii Antes "<< "\n";
   //Borrando triangulo
   //Ogre::Entity* planeManual = this->m_SceneMgr->getEntity("cc");
   std::cout <<"Aquiiiiiiiiiiiiiiiiiiiiiiiiiiiii "<< "\n";
    // Associate it to a node
   //updatePhysics(planeManual, cc_node, 0.0009, 0.0009, 75, Ogre::Vector3( 0, -bbox4.getMinimum( )[ 1 ] * 2, 0 ) ,q4)
   updatePhysics();
   std::cout <<"Borrar Triangulo: "<< trian <<"\n";
   std::cout <<"borrar vertices: "<< faces[3 * trian]  << " " << faces[(3 * trian)+1] << " " << faces[(3 * trian)+2] <<"\n";
   //
   
   
   indi[trian] = 0;
   unsigned short *facesTemp;
    size_t temp = 0;

    for(size_t i = 0; i< (ibufCount/3); i++){
      
      if(indi[i] == 1){
        int k=i*3;
        //facesTemp[temp]   =   faces[k];
        //facesTemp[temp+1] =   faces[(k)+1];
        //facesTemp[temp+2] =   faces[(k)+2];
        //temp              = temp + 3; 
      }else {
       // std::cout <<"Eliminando triangulo: "<< i <<"\n";
      }
      //esta imprimiento mal el indicador
      std::cout <<"Indi "<< i <<": "<< indi[i]<<"\n";
    }

   
   /* 
   for (size_t i = 0; i< ibufCount ; i += 3){
     
     if(i == (3* trian)){
        std::cout <<"Eliminando triangulo: "<< i/3 <<"\n";
     }else {
       //std::cout <<"Copiando triangulos "<< i/3 << " al " << temp/3 <<"\n";
        facesTemp[temp]   =   faces[i];
        facesTemp[temp+1] =   faces[(i)+1];
        facesTemp[temp+2] =   faces[(i)+2];
        //facesTemp[temp]   =   0;
        //facesTemp[temp+1] =   0;
        //facesTemp[temp+2] =   0;
        //temp = temp + 3 ;   
    } 

   }
   */
   //facesTemp[0] = 0;
  // std::cout <<"Copiado de triangulos finalizado " <<"\n";
 }


bool RagDollApp::
frameRenderingQueued( const Ogre::FrameEvent& evt )
{

  

  if( this->m_Devices[ 0 ] != NULL ) {
    VRPN_PhantomOmni* phantom = dynamic_cast< VRPN_PhantomOmni* >( this->m_Devices[ 0 ] );
    VRPN_PhantomOmni_State phantom_state = phantom->capture( );
    //pendiente crear funcion de actualizar pos herramienta
    //this->updateCatheterPosition( evt, phantom_state.Position[2], phantom_state.Position[1], phantom_state.Position[0] );
  }

  std::chrono::duration<double> deltaTime; 
  deltaTime = static_cast<std::chrono::duration<double>>(evt.timeSinceLastFrame);
    if( this->pujOgre::Application::frameRenderingQueued( evt ) )
  {
    //std::cout <<"Loop: " << evt.timeSinceLastFrame <<"\n";
    UpdatePhysics(deltaTime);
    //this->m_AnimationState->addTime( evt.timeSinceLastFrame );
    return( true );
  }
  else
    return( false );
}


void RagDollApp::UpdatePhysics(std::chrono::duration<double> deltaTime) {
  //std::cout <<"Loop: " << deltaTime.count() <<"\n";
}
// eof - $RCSfile$
 