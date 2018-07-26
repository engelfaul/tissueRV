// Based from: http://www.ogre3d.org/tikiwiki/

#ifndef __pujOgre__Application__h__
#define __pujOgre__Application__h__

#include <OgreException.h>
#include <OgreFrameListener.h>
#include <Bites/OgreWindowEventUtilities.h>

#include <OISKeyboard.h>
#include <OISMouse.h>

namespace Ogre
{
  class OverlaySystem;
}

namespace OgreBites
{
  class CameraMan;
}

namespace pujOgre
{
  /**
   */
  class Application
    : public Ogre::FrameListener,
      public OgreBites::WindowEventListener,
      public OIS::KeyListener,
      public OIS::MouseListener
  {
  public:
    Application( );
    virtual ~Application( );

    virtual void go( );
    size_t vertex_count;
    size_t index_count;
    Ogre::Vector3 *vertices;
    Ogre::uint32 *indices;
    /// Define the vertices (16 vertices, each have 3 floats for position and 3 for normal)
    static const size_t nVertices = 16;
    static const size_t vbufCount = 3*2*nVertices;
    //float * verts;
    const float sqrt13 = 0.577350269f; /* sqrt(1/3) */
    float  verts[vbufCount]   = {  
            0.0,5.0,0.0,                //0 position
            -sqrt13,sqrt13,-sqrt13,     //0 normal
            2.0,5.0,0.0,                //1 position
            sqrt13,sqrt13,-sqrt13,      //1 normal
            4.0,5.0,0.0,                //2 position
            sqrt13,-sqrt13,-sqrt13,     //2 normal
            6.0,5.0,0.0,                //3 position
            -sqrt13,-sqrt13,-sqrt13,    //3 normal
            0.0,5.0,2.0,                //4 position
            -sqrt13,sqrt13,sqrt13,      //4 normal
            2.0,5.0,2.0,                //5 position
            sqrt13,sqrt13,sqrt13,       //5 normal
            4.0,5.0,2.0,                //6 position
            sqrt13,-sqrt13,sqrt13,      //6 normal
            6.0,5.0,2.0,                //7 position
            -sqrt13,-sqrt13,sqrt13,     //7 normal
            0.0,5.0,4.0,                //8 position
            -sqrt13,sqrt13,-sqrt13,     //8 normal
            2.0,5.0,4.0,                //9 position
            sqrt13,sqrt13,-sqrt13,      //9 normal
            4.0,5.0,4.0,                //10 position
            sqrt13,-sqrt13,-sqrt13,     //10 normal
            6.0,5.0,4.0,                //11 position
            -sqrt13,-sqrt13,-sqrt13,    //11 normal
            0.0,5.0,6.0,                //12 position
            -sqrt13,sqrt13,sqrt13,      //12 normal
            2.0,5.0,6.0,                //13 position
            sqrt13,sqrt13,sqrt13,       //13 normal
            4.0,5.0,6.0,                //14 position
            sqrt13,-sqrt13,sqrt13,      //14 normal
            6.0,5.0,6.0,                //15 position
            -sqrt13,-sqrt13,sqrt13,     //15 normal
          };

    /// Define 12 triangles (two triangles per cube face)
    /// The values in this table refer to vertices in the above table
    static const size_t ibufCount = 54;
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

    //unsigned short *facesTemp;     
  
  protected:
    virtual void createScene( ) = 0;

    virtual bool setup( );
    virtual bool configure( );
    virtual void chooseSceneManager( );
    virtual void createCamera( );
    virtual void createFrameListener( );
    virtual void destroyScene( );
    virtual void createViewports( );
    virtual void setupResources( );
    virtual void createResourceListener( );
    virtual void loadResources( );

    // Ogre::FrameListener
    virtual bool frameRenderingQueued( const Ogre::FrameEvent& evt ) override;

    // OIS::KeyListener
    virtual bool keyPressed( const OIS::KeyEvent& arg ) override;
    virtual bool keyReleased( const OIS::KeyEvent& arg ) override;

    // OIS::MouseListener
    virtual bool mouseMoved( const OIS::MouseEvent& arg ) override;
    virtual bool mousePressed( const OIS::MouseEvent& arg, OIS::MouseButtonID id ) override;
    virtual bool mouseReleased( const OIS::MouseEvent& arg, OIS::MouseButtonID id ) override;

    // OgreBites::WindowEventListener
    virtual void windowResized( Ogre::RenderWindow* rw ) override;
    virtual void windowClosed( Ogre::RenderWindow* rw ) override;

  protected:
    // Configuration
    Ogre::String m_ResourcesCfg;
    Ogre::String m_PluginsCfg;

    // Rendering graph
    Ogre::Root* m_Root;

    // Some objects created from m_Root
    Ogre::Camera*        m_Camera;
    Ogre::SceneManager*  m_SceneMgr;
    Ogre::RenderWindow*  m_Window;
    Ogre::OverlaySystem* m_OverlaySystem;

    // Bites related objects
    OgreBites::CameraMan* m_CameraMan;

    // OIS Input devices
    OIS::InputManager* m_InputManager;
    OIS::Mouse*    m_Mouse;
    OIS::Keyboard* m_Keyboard;

    // Other attributes
    bool m_ShutDown;
  };

} // ecapseman

#endif // __pujOgre__Application__h__

// eof - $RCSfile$
