#include <iostream>
#include "SceneSoftObject.h"

#include <memory>

#include "BulletSoftBody/btSoftBody.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"

//#include <vtkDataSet.h>
//#include <vtkMapper.h>
//#include <vtkPolyData.h>
//#include <vtkProperty.h>
//#include <vtkTransform.h>

//#include "../util/ToString.h"
//#include "RigidMotionState.h"

SceneSoftObject::SceneSoftObject() {}
SceneSoftObject::SceneSoftObject(Ogre::Entity* actor) {
  this->actor = actor;
}


SceneSoftObject::~SceneSoftObject() {}
void SceneSoftObject::InitSoftBody(btSoftBodyWorldInfo &worldInfo) {
  std::cout << "obteniendo info del mundo fisico" << std::endl;
}

void SceneSoftObject::UpdateMesh() {
//probando
}

btVector3 SceneSoftObject::GetCenterOfGeometry() {
  btVector3 centerOfGeometry(0, 0, 0);
  for (size_t i = 0; i < this->softBody->m_nodes.size(); i++) {
    btSoftBody::Node node = this->softBody->m_nodes[i];
    centerOfGeometry += node.m_x;
  }
  centerOfGeometry /= double(this->softBody->m_nodes.size());
  return centerOfGeometry;
}

void SceneSoftObject::UpdatePhysics(std::chrono::duration<double> deltaTime) {}