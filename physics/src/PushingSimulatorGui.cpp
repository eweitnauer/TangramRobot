// Copyright 2009 Erik Weitnauer
#include "PushingSimulatorGui.h"
#include <btBulletDynamicsCommon.h>
#include "GlutStuff.h"
#include "LinearMath/btAlignedObjectArray.h"
#include <iostream>

using namespace std;

float PushingSimulatorGui::simulate(const PushMovement &push, std::vector<btRigidBody*> &bodies,
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
	// do the movement in real time in 20 steps
	int n=20;
	for (int i=1; i<=n; i++) {
		m_dynamicsWorld->stepSimulation(time_in_s/n, time_in_s/n/time_step+1, time_step);
		usleep(time_in_s/n*1000000/m_fast_forward_factor);
		update_display();
	}
	
	m_pusher_speed = 0;
	if (after_time_in_s > 0) {
		for (int i=1; i<=4; i++) {
			m_dynamicsWorld->stepSimulation(after_time_in_s/4, after_time_in_s/4/time_step+1, time_step);
			usleep(after_time_in_s/4*1000000/m_fast_forward_factor);
			update_display();
		}
	}

	// remove all the objects	
	for (unsigned int i=0; i<bodies.size(); i++)
		m_dynamicsWorld->removeRigidBody(bodies[i]);
	m_dynamicsWorld->removeRigidBody(m_pusher);
		
	return before_time_in_s + after_time_in_s + time_in_s;
}

void PushingSimulatorGui::initScene() {
	m_dynamicsWorld->addCollisionObject(MinimalVisualizer::createGround());
  m_dynamicsWorld->setInternalTickCallback(PushingSimulator::myTickCallback, static_cast<void *>((PushingSimulator*)this));
}

void PushingSimulatorGui::freeScene() {
	if (m_pusher && m_pusher->getMotionState()) delete m_pusher->getMotionState();
  delete m_pusher;
	delete m_pusher_shape;
}

