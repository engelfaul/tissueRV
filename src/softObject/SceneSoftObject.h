#ifndef SCENE_SOFT_OBJECT_H
#define SCENE_SOFT_OBJECT_H

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <BulletSoftBody/btSoftBody.h>
#include <LinearMath/btAlignedObjectArray.h>
#include <LinearMath/btVector3.h>
#include <btBulletDynamicsCommon.h>

//#include <vtkActor.h>
//#include <vtkSmartPointer.h>
//#include <vtkTransform.h>

#include <OgreEntity.h>

class SceneSoftObject {
public:
  std::string name;
  Ogre::Entity* actor;
  std::shared_ptr<btSoftBody> softBody;

  SceneSoftObject();
  SceneSoftObject(Ogre::Entity* actor);
  ~SceneSoftObject();
  void InitSoftBody(btSoftBodyWorldInfo &worldInfo);
  void UpdateMesh();
  virtual void UpdatePhysics(std::chrono::duration<double> deltaTime);
  btVector3 GetCenterOfGeometry();
};

#endif // !SCENE_SOFT_OBJECT_H
