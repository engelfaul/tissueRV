#include <iostream>
#include <pujOgreBullet/Application.h>

#include <OgreCamera.h>
#include <OgreEntity.h>
#include <OgreMeshManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreMesh.h>
#include <OgreSubMesh.h>
#include <OgreRay.h>  
#include <OgrePlane.h>

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
//  Ogre::Vector3 *vertices;
//  Ogre::uint32 *indices;
//  size_t vertex_count;
//  size_t index_count;
  void setMesh();
protected:
  virtual void createScene( ) override;
  virtual void createCamera( ) override;
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


// -------------------------------------------------------
  // Load model entity
  Ogre::Entity* ninja =
    this->m_SceneMgr->createEntity(
      "ninja", "cyborg_ninja.mesh"
      );
  ninja->setCastShadows( true );
  Ogre::AxisAlignedBox bbox = ninja->getBoundingBox( );

  // Associate it to a node
  Ogre::SceneNode* ninja_node =
    this->m_SceneMgr->getRootSceneNode( )->createChildSceneNode(
      "ninja_node"
      );
  ninja_node->attachObject( ninja );

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
  tool_node->translate(0,10,0);

  posTool   = Ogre::Vector3(0,10,0); 
  direccion = Ogre::Vector3(0,9,0);

/////////////////////////////////////////////////////////////////////////////////////
// Associate ship to the physical world
  Ogre::Quaternion q2( 1, 1, 2, 3 );
  q2.normalise( );
  this->addSoftPhysicsTrimesh(
    ship, ship_node, "ship_physics", 0.0009, 0.0009, 75,
    Ogre::Vector3( 0, -bbox.getMinimum( )[ 1 ] * 2, 0 ),
    q2
    );

  // Associate ninja to the physical world
  Ogre::Quaternion q( 1, 1, 2, 3 );
  q.normalise( );
  this->addPhysicsConvex(
    ninja, ninja_node, "ninja_physics", 0.0009, 0.0009, 75,
    Ogre::Vector3( 0, -bbox.getMinimum( )[ 1 ] * 1, 0 ),
    q
    );
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
      std::cout<<"triangle "<<i<<": "<<vertices[indices[i]]<<" "<<
      vertices[indices[i+1]]<<" "<< vertices[indices[i+2]]<<std::endl;
    }


////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}


bool pujOgre::Application::
keyPressed( const OIS::KeyEvent& arg )
{
  OIS::KeyCode a = arg.key;
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
    

/////////////////////////////////////Buscando si hay colision/////////////////////////////////////////////////////////////////
  //Ogre::Vector3 posTool = Ogre::Vector3(posBall[0],posBall[1],posBall[2]);
  Ogre::Plane plano;
  Ogre::Vector3 posTool = planeBlender_node->getPosition();
  posTool.y=posTool.y-6.5; //la punta de instrumento esta siete unidades abajo del centro
  std::cout <<"posx "<< posTool.x << " posy " << posTool.y << " posz " << posTool.z <<"\n";
  Ogre::Vector3 normal=Ogre::Vector3(posTool.x,-0.5,posTool.z);
  std::cout <<"dirx "<< normal.x << " diry " << normal.y << " dirz " << normal.z <<"\n";
  Ogre::Ray toolRay = Ogre::Ray(posTool, normal);
  std::cout<< "origen rayo " << toolRay.getOrigin() << " direccion " << toolRay.getDirection() << "\n";
  for (size_t i = 0; i < index_count; i += 3){
    plano = Ogre::Plane(vertices[indices[i]],vertices[indices[i+1]],vertices[indices[i+2]]);
    std::pair<bool,Ogre::Real> inter = Ogre::Math::intersects(toolRay,vertices[indices[i]],vertices[indices[i+1]],vertices[indices[i+2]],true , true);
    
    if(inter.first && inter.second < 1){
      std::cout <<"colision triangulo objetivo: "<< i <<"\n"; 
    }
  }
////////////////////////////////////Fin Buscando colision/////////////////////////////////////////////////////////////////////

  return( true );
}

// eof - $RCSfile$
