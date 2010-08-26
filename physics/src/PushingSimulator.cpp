#include <PushingSimulator.h>
#include <iostream>
#include <ICLUtils/StackTimer.h>
#include <btBulletDynamicsCommon.h>

using namespace std;
using namespace icl;

//static ostream& operator<<(std::ostream &out, const btVector3 &v) {
//	return out << v.getX() << ", " << v.getY() << ", " << v.getZ();
//}

void PushingSimulator::myTickCallback(btDynamicsWorld *world, btScalar timeStep) {
  static btTransform transform;
  static btVector3 vel;
	PushingSimulator *psim = static_cast<PushingSimulator *>(world->getWorldUserInfo());
	if (psim->m_pusher_speed<=0) {
		psim->m_pusher->setLinearVelocity(btVector3(0,0,0));
		return;
	}
  psim->m_pusher->getMotionState()->getWorldTransform(transform);
  vel = (psim->m_pusher_target - transform.getOrigin()) / timeStep * 0.7;
  if (vel.length() > psim->m_pusher_speed) vel *= psim->m_pusher_speed/vel.length();
//  cout << "Target " << psim->m_pusher_target << endl;
//  cout << "Current " << transform.getOrigin() << endl;
//	cout << "Velocity " << vel << endl;
  psim->m_pusher->setLinearVelocity(vel);
}

void PushingSimulator::applyParameters(btDynamicsWorld *world, std::vector<btRigidBody*> &bodies) {
	world->setGravity(m_parameters.getGravity());
	world->getSolverInfo().m_numIterations = m_parameters["solver_iterations"];
	world->getSolverInfo().m_solverMode = m_parameters.getSolverMode();
	world->getSolverInfo().m_splitImpulse = m_parameters["split_impulse"];
	world->getSolverInfo().m_splitImpulsePenetrationThreshold = m_parameters["split_impulse_penetration_threshold"];
	world->getSolverInfo().m_erp = m_parameters["erp"];
	world->getSolverInfo().m_tau = m_parameters["tau"];
	for (unsigned int i=0; i<bodies.size(); i++) {
		bodies[i]->setDamping(m_parameters["lin_damping"], m_parameters["ang_damping"]);
		bodies[i]->setFriction(m_parameters["friction_polygon"]);
		bodies[i]->setRestitution(m_parameters["restitution_polygon"]);
		bodies[i]->getCollisionShape()->setMargin(m_parameters["collision_margin"]);
		if (btConvexHullShape* shape = dynamic_cast<btConvexHullShape*>(bodies[i]->getCollisionShape())) {
			shape->recalcLocalAabb();
		}
		bodies[i]->setLinearFactor(m_parameters.getLinearFactor());
		bodies[i]->setAngularFactor(m_parameters.getAngularFactor());
	}
	if (m_ground) {
		m_ground->setFriction(m_parameters["friction_ground"]);
		m_ground->setRestitution(m_parameters["restitution_ground"]);
	}
	if (m_pusher) {
		m_pusher->setFriction(m_parameters["friction_pusher"]);
		m_pusher->setRestitution(m_parameters["restitution_pusher"]);
	} 
}

btRigidBody *PushingSimulator::createPusher(const btVector3 &dims, float mass) {
  // only create if size changed or if not created yet
  //if (m_pusher_dims == dims && m_pusher) return m_pusher;
  // if existent, free memory
  if (m_pusher && m_pusher->getMotionState()) delete m_pusher->getMotionState();
  delete m_pusher_shape;
  delete m_pusher;
	m_pusher_dims = dims;
	m_pusher_shape = new btCylinderShape(dims);
	btDefaultMotionState* motionState = new btDefaultMotionState(
		btTransform(btQuaternion(0,0,0,1),
		btVector3(0,(dims[1]+dims[0]/2),0)));
  btVector3 inertia(0,0,0);
  m_pusher_shape->calculateLocalInertia(mass, inertia);
  btRigidBody::btRigidBodyConstructionInfo bodyCI(mass, motionState, m_pusher_shape, inertia);
  m_pusher = new btRigidBody(bodyCI);
  m_pusher->setActivationState(DISABLE_DEACTIVATION);
  m_pusher->setAngularFactor(0); // no tilting allowed!
  m_pusher->setGravity(btVector3(0,0,0));
  return m_pusher;
}

btRigidBody *PushingSimulator::createGround() {
	// infinite static plane parallel to x-z plane, at y=1
	m_ground_shape = new btStaticPlaneShape(btVector3(0,1,0),1);
  btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1), btVector3(0,-1,0)));
  // pass 0 as mass = infinite mass
  btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0,groundMotionState,m_ground_shape,btVector3(0,0,0));
  m_ground = new btRigidBody(groundRigidBodyCI);
  return m_ground;
}

void PushingSimulator::resetSolver(btDynamicsWorld *world) {
  world->getBroadphase()->resetPool(world->getDispatcher());
  world->getConstraintSolver()->reset();
  // sets the protected m_localTime to 0 by calling
  ((btDiscreteDynamicsWorld*)world)->stepSimulation(0,0,0);
}

