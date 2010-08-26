#include <PushingSimulatorFast.h>
#include <iostream>
#include <ICLUtils/StackTimer.h>

using namespace std;
using namespace icl;

float PushingSimulatorFast::simulate(const PushMovement &push, std::vector<btRigidBody*> &bodies,
			float before_time_in_s, float after_time_in_s) {
	resetSolver(m_dynamicsWorld);
	createPusher(push.pusher_dims);
	applyParameters((btDynamicsWorld*)m_dynamicsWorld, bodies);
	// add all the rigid bodies to the scene, remove the pusher
	for (unsigned int i=0; i<bodies.size(); i++)
		m_dynamicsWorld->addRigidBody(bodies[i]);
	
	float time_step = m_parameters["sim_stepsize"];

	float time_in_s = 1.25 * push.getLength() / push.speed;	// 1.25 for security - to ensure we really arrive at the target
	
	m_pusher_speed = 0;
  // now let the engine simulate for 'init time' without any pushing
	// assure that: timeStep(1st) < maxSubSteps(2nd) * fixedTimeStep(3rd)
	if (before_time_in_s > 0)
		m_dynamicsWorld->stepSimulation(before_time_in_s, before_time_in_s/time_step+1, time_step);
	
	// now simulate the pushing action with time resolution of 60 Hz
	// first add pusher
	m_pusher->setCenterOfMassTransform(btTransform(btQuaternion(0,0,0,1), push.start + btVector3(0,m_pusher_dims.getY(),0)));
	m_dynamicsWorld->addRigidBody(m_pusher);
	m_pusher->setGravity(btVector3(0,0,0));
	m_pusher_speed = push.speed;
	m_pusher_target = push.end + btVector3(0,m_pusher_dims.getY(),0);
	
//	m_dynamicsWorld->stepSimulation(time_in_s, time_in_s/time_step+1, time_step);
	int n=m_substeps; 	if (n>time_in_s/time_step) n = time_in_s/time_step;
	for (int i=1; i<=n; i++) {
		m_dynamicsWorld->stepSimulation(time_in_s/n, time_in_s/n/time_step+1, time_step);
	}

	m_pusher_speed = 0;
	if (after_time_in_s > 0)
		m_dynamicsWorld->stepSimulation(after_time_in_s, after_time_in_s/time_step+1, time_step);
	
	// remove all the objects	
	for (unsigned int i=0; i<bodies.size(); i++)
		m_dynamicsWorld->removeRigidBody(bodies[i]);
	m_dynamicsWorld->removeRigidBody(m_pusher);
	
	return before_time_in_s + after_time_in_s + time_in_s;
}

void PushingSimulatorFast::initPhysics() {
	// for alternative btAxisSweep3 broadphaser see "broadphase init" code snipet
  m_broadphase = new btDbvtBroadphase();
  
  m_collisionConfiguration = new btDefaultCollisionConfiguration();
  m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
  m_solver = new btSequentialImpulseConstraintSolver();

    // instanciate the dynamics world
  m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
  m_dynamicsWorld->setGravity(btVector3(0,-10,0));
  m_dynamicsWorld->setInternalTickCallback(PushingSimulator::myTickCallback, static_cast<void *>((PushingSimulator*)this));
}

void PushingSimulatorFast::freePhysics() {
	delete m_dynamicsWorld;
  delete m_solver;
  delete m_dispatcher;
  delete m_collisionConfiguration;
  delete m_broadphase;
}

void PushingSimulatorFast::initScene() {
	m_dynamicsWorld->addRigidBody(createGround());
}

void PushingSimulatorFast::freeScene() {
	if (m_dynamicsWorld == NULL) return; // the init functions were not called
	for (int i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--) {
    btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
    btRigidBody* body = btRigidBody::upcast(obj);
    if (body && body->getMotionState()) {
      delete body->getMotionState();
    }
    m_dynamicsWorld->removeCollisionObject( obj );
    delete obj; obj = NULL;
  }
  if (m_pusher && m_pusher->getMotionState()) delete m_pusher->getMotionState();
  delete m_pusher;
  
  for (int j=0;j<m_collisionShapes.size();j++) {
    btCollisionShape* shape = m_collisionShapes[j];
    delete shape;
  }
  m_collisionShapes.clear();
	delete m_pusher_shape;
	delete m_ground_shape;
}

