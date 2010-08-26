#include <PushingSimulatorDebug.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <ICLUtils/StackTimer.h>
#include <LinearMath/btMatrix3x3.h>

using namespace std;
using namespace icl;

template <class T> string str(const T& t) {
  std::stringstream ss; ss << t; return ss.str();
}

template<> string str(const btQuaternion &q) {
  stringstream s;
  s << "(" << q.x() << " | " << q.y() << " | " << q.z() << " | " << q.w() << ")";
  return s.str();
}

template<> string str(const btVector3 &v) {
  stringstream s;
  s << "(" << v.x() << " | " << v.y() << " | " << v.z() << ")";
  return s.str();
}

template<> string str(const btTransform &t) {
  stringstream s;
  s << "(origin: " << str(t.getOrigin()) << ", rot: " << str(t.getRotation()) << ")";
  return s.str();
}

template<> string str(const btMatrix3x3 &m) {
  stringstream s;
  s << "(c0:" << str(m.getColumn(0)) << "c1:" << str(m.getColumn(1)) << "c2:" << str(m.getColumn(2)) << ")";
  return s.str(); 
}

template<> string str(const btRigidBody &b) {
  stringstream s;
  s << "str(const btRigidBody &b) not implemented!!!" << endl;
//  s << "m_invInertiaTensorWorld: " << str(b.m_invInertiaTensorWorld) << endl
//    << "m_linearVelocity: " << str(b.m_linearVelocity) << endl
//    << "m_angularVelocity: " << str(b.m_angularVelocity) << endl
//    << "m_inverseMass: " << str(b.m_inverseMass) << endl
//    << "m_linearFactor: " << str(b.m_linearFactor) << endl
//    << "m_gravity: " << str(b.m_gravity) << endl
//    << "m_gravity_acceleration: " << str(b.m_gravity_acceleration) << endl
//    << "m_invInertiaLocal: " << str(b.m_invInertiaLocal) << endl
//    << "m_totalForce: " << str(b.m_totalForce) << endl
//    << "m_totalTorque: " << str(b.m_totalTorque) << endl
//    << "m_linearDamping: " << str(b.m_linearDamping) << endl
//    << "m_angularDamping: " << str(b.m_angularDamping) << endl
//    << "m_additionalDamping: " << str(b.m_additionalDamping) << endl
//    << "m_linearSleepingThreshold: " << str(b.m_linearSleepingThreshold) << endl
//    << "m_angularSleepingThreshold: " << str(b.m_angularSleepingThreshold) << endl
//    << "m_deltaAngularVelocity: " << str(b.m_deltaAngularVelocity) << endl
//    << "m_angularFactor: " << str(b.m_angularFactor) << endl
//    << "m_invMass: " << str(b.m_invMass) << endl
//    << "m_turnVelocity: " << str(b.m_turnVelocity) << endl
//    << "m_pushVelocity: " << str(b.m_pushVelocity) << endl;
    
//	btScalar	m_additionalDampingFactor;
//	btScalar	m_additionalLinearDampingThresholdSqr;
//	btScalar	m_additionalAngularDampingThresholdSqr;
//	btScalar	m_additionalAngularDampingFactor;
//  btMotionState*	m_optionalMotionState;
//  btAlignedObjectArray<btTypedConstraint*> m_constraintRefs;
//	int	m_rigidbodyFlags;
//  int	m_debugBodyId;
  return s.str();
}

void PushingSimulatorDebug::myTickCallback(btDynamicsWorld *world, btScalar timeStep) {
  
  static btTransform transform;
  static btTransform transform2;
  static btVector3 vel;
  static int tick_count = 0;
  tick_count++;
	PushingSimulatorDebug *self = static_cast<PushingSimulatorDebug *>(world->getWorldUserInfo());
	
	self->m_pusher->getMotionState()->getWorldTransform(transform);
	transform2 = self->m_pusher->getCenterOfMassTransform();
	if (self->m_pusher_speed<=0) {
	  vel = btVector3(0,0,0);
	} else {
  	vel = (self->m_pusher_target - transform.getOrigin()) / timeStep * 0.7;
    if (vel.length() > self->m_pusher_speed) vel *= self->m_pusher_speed/vel.length();
  }
  
  self->m_pusher->setLinearVelocity(vel);

 	static btTransform bodyTransform;
 	((*self->m_bodylist)[0])->getMotionState()->getWorldTransform(bodyTransform);
  	  
//  stringstream s;
//  s << "[TICK] #" << tick_count << " (" << timeStep*1000 << "ms)";
//  self->log(s.str());
//  self->log("[TICK] MotionState->WorldTransform of pusher is " + str(transform));
//  self->log("[TICK] CenterOfMassTransform of pusher is       " + str(transform2));
//  self->log("[TICK] Setting velocitiy of pusher to           " + str(vel));
//  self->log("[TICK] Pos. of body 1 is                        " + str(bodyTransform));
  self->log("[TICK] Body 1 properties:                         ");
  self->log(str(*(*self->m_bodylist)[0]));
}

float PushingSimulatorDebug::simulate(const PushMovement &push, std::vector<btRigidBody*> &bodies,
			float before_time_in_s, float after_time_in_s) {
	// sets the protected m_localTime to 0 by calling
  m_dynamicsWorld->stepSimulation(0,0,0);
	//resetSolver(m_dynamicsWorld);
	createPusher(push.pusher_dims);
	{
	  btTransform start_pos(btQuaternion(0,0,0,1), push.start + btVector3(0,m_pusher_dims.getY(),0));
	  btDefaultMotionState* myMotionState = (btDefaultMotionState*)m_pusher->getMotionState();
		myMotionState->m_startWorldTrans = start_pos;
		myMotionState->m_graphicsWorldTrans = myMotionState->m_startWorldTrans;
		m_pusher->setCenterOfMassTransform(myMotionState->m_graphicsWorldTrans );
		m_pusher->setInterpolationWorldTransform(myMotionState->m_startWorldTrans);
	}
	//m_pusher->setCenterOfMassTransform(btTransform(btQuaternion(0,0,0,1), push.start + btVector3(0,m_pusher_dims.getY(),0)));
	applyParameters(m_dynamicsWorld, bodies);
	m_bodylist = &bodies;
	// add all the rigid bodies to the scene, remove the pusher
	for (unsigned int i=0; i<bodies.size(); i++)
		m_dynamicsWorld->addRigidBody(bodies[i]);
	
	float time_step = m_parameters["sim_stepsize"];

	float time_in_s = 1.25 * push.getLength() / push.speed;	// 1.25 for security - to ensure we really arrive at the target
	
	m_pusher_speed = 0;
  // now let the engine simulate for 'init time' without any pushing
	// assure that: timeStep(1st) < maxSubSteps(2nd) * fixedTimeStep(3rd)
	log("[SIMU] before");
	log(str(m_dynamicsWorld->getSolverInfo().m_solverMode));
	if (before_time_in_s > 0)
		m_dynamicsWorld->stepSimulation(before_time_in_s, before_time_in_s/time_step+1, time_step);
	btTransform transform;
	bodies[0]->getMotionState()->getWorldTransform(transform);
	log("[SIMU] MotionState->WorldTransform of body is " + str(transform));
	
	
	// now simulate the pushing action with time resolution of 60 Hz
	// first add pusher
	m_dynamicsWorld->addRigidBody(m_pusher);
	m_pusher->setGravity(btVector3(0,0,0));
	m_pusher_speed = push.speed;
	m_pusher_target = push.end + btVector3(0,m_pusher_dims.getY(),0);
	
//	m_dynamicsWorld->stepSimulation(time_in_s, time_in_s/time_step+1, time_step);
	log("[SIMU] main");
	int n=m_substeps; 	if (n>time_in_s/time_step) n = time_in_s/time_step;
	for (int i=1; i<=n; i++) {
		m_dynamicsWorld->stepSimulation(time_in_s/n, time_in_s/n/time_step+1, time_step);
	}

	m_pusher_speed = 0;
  log("[SIMU] after");
	if (after_time_in_s > 0)
		m_dynamicsWorld->stepSimulation(after_time_in_s, after_time_in_s/time_step+1, time_step);
	
	// remove all the objects	
	for (unsigned int i=0; i<bodies.size(); i++)
		m_dynamicsWorld->removeRigidBody(bodies[i]);
	m_dynamicsWorld->removeRigidBody(m_pusher);
	
	m_bodylist = 0;
	
	return before_time_in_s + after_time_in_s + time_in_s;
}

void PushingSimulatorDebug::resetSolver(btDynamicsWorld *world) {
  cout << "resetting everything" << endl;
  world->getBroadphase()->resetPool(world->getDispatcher());
  world->getConstraintSolver()->reset();
}

void PushingSimulatorDebug::initPhysics() {
	// for alternative btAxisSweep3 broadphaser see "broadphase init" code snipet
  m_broadphase = new btDbvtBroadphase();
  
  m_collisionConfiguration = new btDefaultCollisionConfiguration();
  m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
  m_solver = new btSequentialImpulseConstraintSolver();

    // instanciate the dynamics world
  m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
  m_dynamicsWorld->setGravity(btVector3(0,-10,0));
  m_dynamicsWorld->setInternalTickCallback(PushingSimulatorDebug::myTickCallback, static_cast<void *>((PushingSimulatorDebug*)this),true);
}

void PushingSimulatorDebug::freePhysics() {
	delete m_dynamicsWorld;
  delete m_solver;
  delete m_dispatcher;
  delete m_collisionConfiguration;
  delete m_broadphase;
}

void PushingSimulatorDebug::initScene() {
	m_dynamicsWorld->addRigidBody(createGround());
}

void PushingSimulatorDebug::freeScene() {
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

string PushingSimulatorDebug::openFileLogStream(string prefix) {
  stringstream s;
  s << prefix << m_fstream_counter++;
  m_logstream = new fstream();
  ((fstream*)m_logstream)->open(s.str().c_str(), fstream::out | fstream::trunc);
  return s.str();
}

void PushingSimulatorDebug::closeFileLogStream() {
  ((fstream*)m_logstream)->close();
}
