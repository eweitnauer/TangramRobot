// Copyright 2009 Erik Weitnauer
#include "MinimalVisualized.h"
#include <btBulletDynamicsCommon.h>
#include "GlutStuff.h"
#include "LinearMath/btAlignedObjectArray.h"
#include <iostream>

using namespace std;

void MinimalVisualizer::clientMoveAndDisplay() {
  float ms = getDeltaTimeMicroseconds();
  if (m_dynamicsWorld) {
    m_dynamicsWorld->stepSimulation(ms / 1.e6, 1);
  }
  displayCallback();
}

void MinimalVisualizer::initPhysics() {
	// for alternative btAxisSweep3 broadphaser see "broadphase init" code snipet
  m_broadphase = new btDbvtBroadphase();
  
  m_collisionConfiguration = new btDefaultCollisionConfiguration();
  m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
  m_solver = new btSequentialImpulseConstraintSolver();

    // instanciate the dynamics world
  m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
  m_dynamicsWorld->setGravity(btVector3(0,-10,0));
  
  setTexturing(true);
	setShadows(true);
	setCameraDistance(btScalar(10.));
}

void MinimalVisualizer::initScene() {
  	m_dynamicsWorld->addCollisionObject(createGround());

  // sphere with radius 1
  btCollisionShape* fallShape = new btSphereShape(1);
  m_collisionShapes.push_back(fallShape);

  // add the falling 1 kg sphere 5 m above ground
  btDefaultMotionState* fallMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), btVector3(0,5,0)));
  btScalar mass = 1.;
  btVector3 fallInertia(0,0,0);
  fallShape->calculateLocalInertia(mass, fallInertia);
  btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass,fallMotionState,fallShape,fallInertia);
  btRigidBody* fallRigidBody = new btRigidBody(fallRigidBodyCI);
  m_dynamicsWorld->addRigidBody(fallRigidBody);
}

btCollisionObject* MinimalVisualizer::createGround() {
	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(20.),btScalar(.5),btScalar(20.)));
	m_collisionShapes.push_back(groundShape);
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-0.5,0));
	btCollisionObject* fixedGround = new btCollisionObject();
	fixedGround->setCollisionShape(groundShape);
	fixedGround->setWorldTransform(groundTransform);
	return fixedGround;
/**
  // infinite static plane parallel to x-z plane, at y=1
  btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),1);
  m_collisionShapes.push_back(groundShape);
  
  btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), btVector3(0,-1,0)));
  // pass 0 as mass = infinite mass
  btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0,groundMotionState,groundShape,btVector3(0,0,0));
  btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
  // add ground to the world
  m_dynamicsWorld->addRigidBody(groundRigidBody);
  */
}

void MinimalVisualizer::displayCallback() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
  renderme();
      // optional but useful: debug drawing to detect problems
  if (m_dynamicsWorld)
    m_dynamicsWorld->debugDrawWorld();
  if (m_shapeDrawer)
  	m_shapeDrawer->drawCoordSystem();
  glFlush();
  glutSwapBuffers();
}

void MinimalVisualizer::freeScene() {
	for (int i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--) {
    btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
    btRigidBody* body = btRigidBody::upcast(obj);
    if (body && body->getMotionState()) {
      delete body->getMotionState();
    }
    m_dynamicsWorld->removeCollisionObject( obj );
    delete obj;
  }
  // delete collision shapes
  for (int j=0;j<m_collisionShapes.size();j++) {
    btCollisionShape* shape = m_collisionShapes[j];
    delete shape;
  }
  m_collisionShapes.clear();
}

void MinimalVisualizer::freePhysics() {
	delete m_dynamicsWorld;
  delete m_solver;
  delete m_dispatcher;
  delete m_collisionConfiguration;
  delete m_broadphase;
}
