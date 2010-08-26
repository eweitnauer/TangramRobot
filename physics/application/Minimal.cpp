// Copyright 2009 Erik Weitnauer
/// Written after the tutorial on
/// http://www.bulletphysics.com/mediawiki-1.5.8/index.php?title=Hello_World
/// Minimal usage example for bullet physics library.

#include <btBulletDynamicsCommon.h>
#include <iostream>

using namespace std;

void do_something_awesome(btDiscreteDynamicsWorld* dynamicsWorld) {
  dynamicsWorld->setGravity(btVector3(0,-10,0));
  
  // infinite static plane parallel to x-z plane, at y=1
  btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),1);
  // sphere with radius 1
  btCollisionShape* fallShape = new btSphereShape(1);
  
  // motion state to make the visualisation easier
  btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), btVector3(0,-1,0)));
  // pass 0 as mass = infinite mass
  btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0,groundMotionState,groundShape,btVector3(0,0,0));
  btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
  
  // add ground to the world
  dynamicsWorld->addRigidBody(groundRigidBody);
  
  // add the falling 1 kg sphere 50 m above ground
  btDefaultMotionState* fallMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), btVector3(0,50,0)));
  btScalar mass = 1;
  btVector3 fallInertia(0,0,0);
  fallShape->calculateLocalInertia(mass, fallInertia);
  btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass,fallMotionState,fallShape,fallInertia);
  btRigidBody* fallRigidBody = new btRigidBody(fallRigidBodyCI);
  dynamicsWorld->addRigidBody(fallRigidBody);
  
  for (int i=0 ; i<30 ; i++) {
    dynamicsWorld->stepSimulation(1/60.f, 10);
    btTransform trans;
    fallRigidBody->getMotionState()->getWorldTransform(trans);
    
    cout << "sphere height: " << trans.getOrigin().getY() << endl;
  }
  
  dynamicsWorld->removeRigidBody(fallRigidBody);
  delete fallRigidBody->getMotionState();
  delete fallRigidBody;
  dynamicsWorld->removeRigidBody(groundRigidBody);
  delete groundRigidBody->getMotionState();
  delete groundRigidBody;
  
  delete fallShape;
  delete groundShape;
}

int main() {
  // for alternative btAxisSweep3 broadphaser see "broadphase init" code snipet
  btDbvtBroadphase* broadphase = new btDbvtBroadphase();
  
  btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
  btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
  btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();
   
  // instanciate the dynamics world
  btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
  
  do_something_awesome(dynamicsWorld);
  
  delete dynamicsWorld;
  delete solver;
  delete dispatcher;
  delete collisionConfiguration;
  delete broadphase;
  
  return 0;
}

