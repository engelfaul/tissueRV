/***************************************************************************

This source file is part of OGREBULLET
(Object-oriented Graphics Rendering Engine Bullet Wrapper)

Copyright (c) 2007 tuan.kuranes@gmail.com (Use it Freely, even Statically, but have to contribute any changes)



Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
-----------------------------------------------------------------------------
*/

#include <iostream>

#include "OgreBulletDynamics.h"

#include "OgreBulletCollisionsShape.h"

#include "OgreBulletDynamicsWorld.h"
#include "OgreBulletDynamicsObjectState.h"
#include "OgreBulletDynamicsRigidBody.h"
#include "OgreBulletDynamicsSoftBody.h"
#include "OgreBulletDynamicsConstraint.h"

#include "Constraints/OgreBulletDynamicsRaycastVehicle.h"

#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"

#include "BulletSoftBody/btDefaultSoftBodySolver.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"

#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"

using namespace Ogre;
using namespace OgreBulletCollisions;

namespace OgreBulletDynamics
{

    DynamicsWorld::DynamicsWorld(Ogre::SceneManager *mgr,
                                 const Ogre::AxisAlignedBox &bounds,
                                 const Ogre::Vector3 &gravity,
                                 bool init,
                                 bool set32BitAxisSweep,
                                 unsigned int maxHandles)
        : CollisionsWorld(mgr, bounds, false, set32BitAxisSweep, maxHandles)
    {
        mConstraintsolver = new btSequentialImpulseConstraintSolver();
        
        if (init)
        {
            std::cout << "\n----- ANTES DEl GENESIS ------" << std::endl;
            std::cout << "- Tipo Dispatcher: " << typeid(*mDispatcher).name() << std::endl;
            std::cout << "- Tipo Collision Config: " << typeid(mDefaultCollisionConfiguration).name() << std::endl;
            std::cout << "- Tipo Broadphase: " << typeid(mBroadphase).name() << std::endl;
            
            // Algoritmo de emparejamiento de colisionadores (Broadphase).
            // En este caso un Dynamic AABBTree.
            mBroadphase = new btDbvtBroadphase();

            // Configuración de detección de colisiones entre dos pares y el despachador.
            btDefaultCollisionConfiguration *collisionConfiguration =
            new btSoftBodyRigidBodyCollisionConfiguration();

            // Soporta colisiones convex-convex y convex-concave.
            btCollisionDispatcher *dispatcher =
            new btCollisionDispatcher(collisionConfiguration);

            btSoftBodySolver* solver1 = new btDefaultSoftBodySolver( );
            // Solucionador del estado siguiente de la simulación a partir de la lógica de
            // simulación. Es un punto importante para el rendimiento de la simulación.
            // Este solucionador implementa PGS para LCP.
            btSequentialImpulseConstraintSolver* solver =
                new btSequentialImpulseConstraintSolver();

            mWorld = new btSoftRigidDynamicsWorld( dispatcher, mBroadphase, mConstraintsolver, collisionConfiguration, solver1 );
            //(DynamicsWorld*)mWorld->getDynamicsWorld()->getBulletDynamicsWorld()->setGravity(btVector3(0, -10, 0));
            
            std::cout << "- Tipo Mundo: " << typeid(mWorld).name() << std::endl << std::endl;
            static_cast<btSoftRigidDynamicsWorld *>(mWorld)->setGravity(convert(gravity));

            // Configuración para la simulación de cuerpos suaves.
            this->softBodyWorldInfo.m_broadphase = mBroadphase;
            this->softBodyWorldInfo.m_dispatcher = dispatcher;
            this->softBodyWorldInfo.m_gravity = static_cast<btSoftRigidDynamicsWorld *>(mWorld)->getGravity();
            this->softBodyWorldInfo.m_sparsesdf.Initialize();
		}

    }
    // -------------------------------------------------------------------------
    DynamicsWorld::~DynamicsWorld()
    {
        delete mConstraintsolver;
        mConstraintsolver = NULL;
    }

    // -------------------------------------------------------------------------
    void DynamicsWorld::addRigidBody(RigidBody *rb, short collisionGroup, short collisionMask)
    {
        mObjects.push_back(static_cast <Object *>(rb));

		if (collisionGroup == 0 && collisionMask == 0)
		{
			// use default collision group/mask values (dynamic/kinematic/static)
            static_cast<btSoftRigidDynamicsWorld *>(mWorld)->addRigidBody(rb->getBulletRigidBody());
		}
		else
		{
            static_cast<btSoftRigidDynamicsWorld *>(mWorld)->addRigidBody(rb->getBulletRigidBody(), collisionGroup, collisionMask);
		}
    }

    // -------------------------------------------------------------------------
    void DynamicsWorld::addSoftBody(SoftBody *sb, short collisionGroup, short collisionMask)
    {
        //ojo statico
        mObjects.push_back(static_cast <Object *>(sb));
        mSoftObjects.push_back(static_cast <Object *>(sb));
		if (collisionGroup == 0 && collisionMask == 0)
		{
			// use default collision group/mask values (dynamic/kinematic/static)
            static_cast<btSoftRigidDynamicsWorld *>(mWorld)->addSoftBody(sb->getBulletSoftBody());
		}
		else
		{
            static_cast<btSoftRigidDynamicsWorld *>(mWorld)->addSoftBody(sb->getBulletSoftBody(), collisionGroup, collisionMask);
		}
    }

    // -------------------------------------------------------------------------
    void DynamicsWorld::stepSimulation(const Ogre::Real elapsedTime, int maxSubSteps, const Ogre::Real fixedTimestep)
    {
         //std::cout <<"loop num softbodies: "<< static_cast<btSoftRigidDynamicsWorld *> (mWorld)->getSoftBodyArray().size() <<"\n";
        //std::cout <<"loop num softbodies: "<< mSoftObjects.size() <<"\n";
        
        //actualizando el mesh
       // stepTime=stepTime+elapsedTime;
      
       stepTime=stepTime+1;
        std::cout <<"time test!!!: "<< stepTime <<"\n";
        int numSoft = static_cast<btSoftRigidDynamicsWorld *> (mWorld)->getSoftBodyArray().size();
          // Llamar actualización de física de cada objeto suave.
        //this->dynamicsWorld->stepSimulation(deltaTime.count(), 10);
        
        for (size_t i = 0; i < numSoft; i++) {
            /*
          if(stepTime == 54 ){
            static_cast<SoftBody*>(mSoftObjects[i])->UpdateCut(stepTime);
            //static_cast<SoftBody*>(mSoftObjects[i])->UpdateCut(stepTime+1);    
          } 
               
          if (stepTime == 64){
              static_cast<SoftBody*>(mSoftObjects[i])->UpdateCut(55);
          }
          if (stepTime == 74){
              static_cast<SoftBody*>(mSoftObjects[i])->UpdateCut(56);
          }
          if (stepTime == 84){
              static_cast<SoftBody*>(mSoftObjects[i])->UpdateCut(57);
          }
          if (stepTime == 94){
              static_cast<SoftBody*>(mSoftObjects[i])->UpdateCut(58);
          }
          if (stepTime == 104){
              static_cast<SoftBody*>(mSoftObjects[i])->UpdateCut(59);
          }
          if (stepTime == 114){
              static_cast<SoftBody*>(mSoftObjects[i])->UpdateCut(60);
          }
          if (stepTime == 124){
              static_cast<SoftBody*>(mSoftObjects[i])->UpdateCut(61);
          }
          if (stepTime == 134){
              static_cast<SoftBody*>(mSoftObjects[i])->UpdateCut(62);
          }
          if (stepTime == 144){
              static_cast<SoftBody*>(mSoftObjects[i])->UpdateCut(63);
          }
          if (stepTime == 154){
              static_cast<SoftBody*>(mSoftObjects[i])->UpdateCut(64);
          }
          if (stepTime == 154){
              static_cast<SoftBody*>(mSoftObjects[i])->UpdateCut(65);
          }
          if (stepTime == 164){
              static_cast<SoftBody*>(mSoftObjects[i])->UpdateCut(66);
          }
          if (stepTime == 174){
              static_cast<SoftBody*>(mSoftObjects[i])->UpdateCut(67);
          }
          if (stepTime == 184){
              static_cast<SoftBody*>(mSoftObjects[i])->UpdateCut(68);
          }
          if (stepTime == 194){
              static_cast<SoftBody*>(mSoftObjects[i])->UpdateCut(69);
          }
          if (stepTime == 204){
              static_cast<SoftBody*>(mSoftObjects[i])->UpdateCut(70);
          }
          if (stepTime == 214){
              static_cast<SoftBody*>(mSoftObjects[i])->UpdateCut(71);
          }


            */

          static_cast<SoftBody*>(mSoftObjects[i])->UpdateMesh();
                  
        }
    
        
        // Reset Debug Lines
        if (mDebugDrawer)
        {
            mDebugDrawer->clear();
        }

        if(mDebugContactPoints)
        {
            mDebugContactPoints->clear();
        }
        
        //static_cast<btSoftRigidDynamicsWorld *> (mWorld)->stepSimulation(elapsedTime, maxSubSteps, fixedTimestep);
        static_cast<btSoftRigidDynamicsWorld *> (mWorld)->stepSimulation(elapsedTime, 10);
		if (mDebugContactPoints) 
		{
            
        	///one way to draw all the contact points is iterating over contact manifolds / points:
            const unsigned int numManifolds = mWorld->getDispatcher()->getNumManifolds();
            for (unsigned int i = 0; i < numManifolds; ++i)
			{
				btPersistentManifold* contactManifold = mWorld->getDispatcher()->getManifoldByIndexInternal(i);

                const btCollisionObject* obA = contactManifold->getBody0();
                const btCollisionObject* obB = contactManifold->getBody1();

                contactManifold->refreshContactPoints(obA->getWorldTransform(), obB->getWorldTransform());

				const unsigned int numContacts = contactManifold->getNumContacts();
                for (unsigned int j = 0; j < numContacts; j++)
				{
					btManifoldPoint& pt = contactManifold->getContactPoint(j);

					if (mShowDebugContactPoints)
					{
						btVector3 ptA = pt.getPositionWorldOnA();
						btVector3 ptB = pt.getPositionWorldOnB();
                        btVector3 ptDistB = ptB  + pt.m_normalWorldOnB * 100;

                        mDebugContactPoints->addLine(ptA.x(), ptA.y(), ptA.z(),
                                                     ptB.x(), ptB.y(), ptB.z());

                        mDebugContactPoints->addLine(ptB.x(), ptB.y(), ptB.z(),
                                                     ptDistB.x(), ptDistB.y(), ptDistB.z());
					}
				}
				//you can un-comment out this line, and then all points are removed
				//contactManifold->clearManifold();	
			}
			// draw lines that step Simulation sent.
			mDebugContactPoints->draw();
		}

		if (mDebugDrawer) 
		{
			// draw lines that step Simulation sent.
			mDebugDrawer->draw();

            /** TODO: */
            /*
            const bool drawFeaturesText = ((mDebugDrawer->getDebugMode () & btIDebugDraw::DBG_DrawFeaturesText) != 0);
			if (drawFeaturesText)
			{
				// on all bodies we have
				// we get all shapes and draw more information
				//depending on mDebugDrawer mode.
				std::deque<Object*>::iterator it = mObjects.begin();
				while (it != mObjects.end())
				{
					//(*it)->drawFeaturesText();
					++it;
				}
			}
            */            
		} 
           
    }
    // -------------------------------------------------------------------------
    void DynamicsWorld::removeConstraint(TypedConstraint *constraint)
    {
        getBulletDynamicsWorld()->removeConstraint(constraint->getBulletTypedConstraint());
        std::deque <TypedConstraint*>::iterator it = mConstraints.begin();
        while (it != mConstraints.end())
        {
            if ((*it) == constraint)
            {
                mConstraints.erase(it);
                break;
            }
            ++it;
        }
    }
    // -------------------------------------------------------------------------
    void DynamicsWorld::addConstraint(TypedConstraint *constraint)
    {
        getBulletDynamicsWorld()->addConstraint(constraint->getBulletTypedConstraint());
        mConstraints.push_back(constraint);
    }
    // -------------------------------------------------------------------------
    void DynamicsWorld::addVehicle(RaycastVehicle *v)
    {
        getBulletDynamicsWorld()->addVehicle(v->getBulletVehicle());
        mActionInterface.push_back(static_cast<ActionInterface *>(v));

        //mVehicles.push_back(v);
    }
    // -------------------------------------------------------------------------
    bool DynamicsWorld::isConstraintRegistered(TypedConstraint *constraint) const
    {
        std::deque <TypedConstraint*>::const_iterator it = mConstraints.begin();
        while (it != mConstraints.end())
        {
            if ((*it) == constraint)
            {
                return true;
            }
            ++it;
        }
        return false;
    }
}

