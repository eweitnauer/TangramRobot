#include <ICLQuick/Common.h>
#include <ICLUtils/StackTimer.h>
#include <btBulletDynamicsCommon.h>
#include <vector>
#include "Shapes.h"
#include "PushMovement.h"

#define SCALING 20.0

using namespace std;
btDiscreteDynamicsWorld* dynamicsWorld;

void cleanTheWorld() {
	for (int i=dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--) {
    btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
    btRigidBody* body = btRigidBody::upcast(obj);
    if (body && body->getMotionState()) {
      delete body->getMotionState();
    }
    dynamicsWorld->removeCollisionObject( obj );
    delete obj;
  }
}

float rnd() {
	return (float) rand()/RAND_MAX;
}

void addSphere(btCollisionShape *fallShape) {
	// add the falling 1 kg sphere 50 m above ground
  btDefaultMotionState* fallMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), btVector3(rnd()*100,50+rnd()*100,rnd()*100)));
  btScalar mass = 1;
  btVector3 fallInertia(0,0,0);
  fallShape->calculateLocalInertia(mass, fallInertia);
  btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass,fallMotionState,fallShape,fallInertia);
  btRigidBody* fallRigidBody = new btRigidBody(fallRigidBodyCI);
  dynamicsWorld->addRigidBody(fallRigidBody);
}

void one_second_ten_balls_falling() {
	BENCHMARK_THIS_FUNCTION;
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
  
  for (int i=0; i<10; i++) addSphere(fallShape);
  
  for (int i=0 ; i<60 ; i++) {
  	dynamicsWorld->stepSimulation(1./60.);
  }

	cleanTheWorld();  
  delete fallShape;
  delete groundShape;
}

btRigidBody* createDynamicRigidBody(btTransform transform, btCollisionShape* shape,
                                      float mass, float friction, float restitution) {
  btDefaultMotionState* motionState = new btDefaultMotionState(transform);
  btVector3 inertia(0,0,0);
  shape->calculateLocalInertia(mass, inertia);
  btRigidBody::btRigidBodyConstructionInfo bodyCI(mass, motionState, shape, inertia);
  bodyCI.m_friction = friction;
  bodyCI.m_restitution = restitution;
  btRigidBody* body = new btRigidBody(bodyCI);
  body->setActivationState(DISABLE_DEACTIVATION);
  return body;
}

btRigidBody* createDynamicRigidBody(btVector3 position, btCollisionShape* shape,
    float mass, float friction, float restitution) {
  return createDynamicRigidBody(btTransform(btQuaternion(0,0,0,1), position), shape, mass, friction, restitution);
}

void movePushingObject(btRigidBody *pusher, const PushMovement &push, float progress) {
  btTransform newTrans;
  pusher->getMotionState()->getWorldTransform(newTrans);
  newTrans.getOrigin() = push.getPosition(progress);
  pusher->getMotionState()->setWorldTransform(newTrans);
}

void ten_seconds_tangram_pushing() {
	BENCHMARK_THIS_FUNCTION;
	float time_in_s = 10;
  float time_step=1./60.;
  dynamicsWorld->setGravity(btVector3(0,-10,0));
  
  // infinite static plane parallel to x-z plane, at y=1
  btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),1);
  // motion state to make the visualisation easier
  btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), btVector3(0,-1,0)));
  // pass 0 as mass = infinite mass
  btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0,groundMotionState,groundShape,btVector3(0,0,0));
  btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
  
  // add ground to the world
  dynamicsWorld->addRigidBody(groundRigidBody);
    
  float a = 0.165*SCALING; // base length in m, scaled
  float h = 0.017*SCALING; // heigth in m, scaled
  float m = 0.180; // don't scale the mass (in kg)
  
  // cylinder with radius 1, length 5
  btCollisionShape* cylinderShape = new btCylinderShape(btVector3(h,a/2,h));
  // triangle shapes
  btCollisionShape* smallTriangleShape = Shapes::createTriangleShape(a,h);
  btCollisionShape* mediumTriangleShape = Shapes::createTriangleShape(sqrt(2)*a,h);
  btCollisionShape* bigTriangleShape = Shapes::createTriangleShape(2*a,h);
  // square shape
  btCollisionShape* squareShape = Shapes::createSquareShape(sqrt(2)/2*a,h);
  // parallelogram shape
  btCollisionShape* prllShape = Shapes::createParallelogramShape(a,h);
  
  // add cylinder shape to the world 3 m above ground
  btDefaultMotionState* cylinderMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), btVector3(2.5*a,a/2+h/2,0)));//btVector3(0,a/2+2*h,0)));
  btRigidBody::btRigidBodyConstructionInfo cylinderRigidBodyCI(0,cylinderMotionState,cylinderShape,btVector3(0,0,0));
  btRigidBody* cylinderRigidBody = new btRigidBody(cylinderRigidBodyCI);
  cylinderRigidBody->setCollisionFlags( cylinderRigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
  cylinderRigidBody->setActivationState(DISABLE_DEACTIVATION);
  dynamicsWorld->addRigidBody(cylinderRigidBody);
	PushMovement push(btVector3(2.5*a,a/2+h/2,0), btVector3(-2.5*a,a/2+h/2,0));

  a += a/10;
  h *= 10;
  btQuaternion q;
  // add triangles to the world
  q.setRotation(btVector3(0,1,0),-M_PI/2);
  dynamicsWorld->addRigidBody(
    createDynamicRigidBody(btTransform(q,btVector3(a/4,h/2,0)), smallTriangleShape, m, 0.9, 0.5));
  q.setRotation(btVector3(0,1,0),M_PI);
  dynamicsWorld->addRigidBody(
    createDynamicRigidBody(btTransform(q,btVector3(-a/2,h/2,3*a/4)), smallTriangleShape, m, 0.9, 0.5));
  q.setRotation(btVector3(0,1,0),M_PI/4);
  dynamicsWorld->addRigidBody(
    createDynamicRigidBody(btTransform(q,btVector3(3*a/4,h/2,3*a/4)), mediumTriangleShape, 2*m, 0.9, 0.5));
  dynamicsWorld->addRigidBody(
    createDynamicRigidBody(btTransform(btQuaternion(0,0,0,1), btVector3(0,h/2,-a/2)), bigTriangleShape, 4*m, 0.9, 0.5));
  q.setRotation(btVector3(0,1,0),M_PI/2);
  dynamicsWorld->addRigidBody(
    createDynamicRigidBody(btTransform(q, btVector3(-a/2,h/2,0)), bigTriangleShape, 4*m, 0.9, 0.5));
  // add square
  q.setRotation(btVector3(0,1,0),M_PI/4);
  btRigidBody *squareBody = createDynamicRigidBody(btTransform(q, btVector3(0,h/2,a/2)), squareShape, 4*m, 0.9, 0.05);
  dynamicsWorld->addRigidBody(squareBody);
  // add parallelogram
  q.setRotation(btVector3(0,1,0),M_PI/2);
  btRigidBody *prllBody = createDynamicRigidBody(btTransform(q, btVector3(3*a/4,h/2,-a/4)), prllShape, 2*m, 0.9, 0.05);
  dynamicsWorld->addRigidBody(prllBody);
  
  
	int step_count = time_in_s/time_step;
	for (int i=0; i<=step_count; i++) {
		movePushingObject(cylinderRigidBody, push, time_step*i / time_in_s);
		dynamicsWorld->stepSimulation(time_step, 1, time_step);
	}
	
  cleanTheWorld();  
  delete cylinderShape;
  delete smallTriangleShape;
  delete mediumTriangleShape;
  delete bigTriangleShape;
  delete squareShape;   
  delete prllShape;
  delete groundShape;
}

int main() {
  // for alternative btAxisSweep3 broadphaser see "broadphase init" code snipet
  btDbvtBroadphase* broadphase = new btDbvtBroadphase();
  
  btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
  btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
  btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();
   
  // instanciate the dynamics world
  dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
  dynamicsWorld->setGravity(btVector3(0,-10,0));
  
  for (int i=0; i<10; i++) one_second_ten_balls_falling();
  for (int i=0; i<10; i++) ten_seconds_tangram_pushing();
  
  delete dynamicsWorld;
  delete solver;
  delete dispatcher;
  delete collisionConfiguration;
  delete broadphase;
	return 0;
}
