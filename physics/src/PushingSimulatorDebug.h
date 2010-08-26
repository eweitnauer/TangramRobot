// Copyright 2010 Erik Weitnauer
#ifndef __PUSHING_SIMULATOR_DEBUG_EWEITNAU_H__
#define __PUSHING_SIMULATOR_DEBUG_EWEITNAU_H__

#include <btBulletDynamicsCommon.h>
#include <PushingSimulator.h>
#include <iostream>

/// Simulates pushing actions and stores information of object states over time.
/** Pass a vector of pointers to RigidBodies and a pushing action. Then call
simulate() with the amount of time to simulate. The position changes are written
in place into the passed RigidBodies. */
class PushingSimulatorDebug : public PushingSimulator {
	public:
		PushingSimulatorDebug(): m_dynamicsWorld(NULL), m_broadphase(NULL),
			m_dispatcher(NULL), m_solver(NULL), m_collisionConfiguration(NULL),
			m_substeps(1), m_logstream(&std::cout), m_fstream_counter(0) { }
		virtual ~PushingSimulatorDebug() { freeScene(); freePhysics(); }
		virtual void init() { initPhysics(); initScene(); }
		/// Simulates a pushing action performed on the rigid bodies passed.
		/** First, a ground at y=0 is added to the scene, then all bodies passed
		 * in 'bodies'. Next, 'before_time_in_s' time this scene is simulated. Afterwards
		 * a pushing object (small upright cylinder) is added to the scene at the
		 * position 'push.start' and is moved linearly to the position 'push.end',
		 * which moves with push.speed velocity. Afterwards, another the after_time_in_s
		 * seconds are simulated without moving the cylinder.
		 * The position changes are written directly into the passed rigid bodies.
		 * Returns how much world time was simulated (in seconds). */
		virtual float simulate(const PushMovement &push, std::vector<btRigidBody*> &bodies,
			float before_time_in_s=0.1, float after_time_in_s=0.5);
	
	  void initPhysics();
	  void freePhysics();

		/// Set the number of steps used to simulate the pushing.
		void setSubsteps(int value) {m_substeps = value;}
		
		/// Creates the m_pusher and m_ground objects and adds them to scene.
		void initScene();
		/// Removes and deletes all objects in the world. <omfg>
		void freeScene();
		
		std::string openFileLogStream(std::string prefix="log");
		void closeFileLogStream();
				
		void log(const std::string &s) { (*m_logstream) << s << std::endl; }
		
		void setLogStream(std::ostream &s) { m_logstream = &s; }
		
		virtual void resetSolver(btDynamicsWorld *world);
		
	protected:
	  static void myTickCallback(btDynamicsWorld *world, btScalar timeStep);
		
  private:
		btDiscreteDynamicsWorld *m_dynamicsWorld;
    btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
    btBroadphaseInterface* m_broadphase;
    btCollisionDispatcher* m_dispatcher;
    btConstraintSolver* m_solver;
    btDefaultCollisionConfiguration* m_collisionConfiguration;
    
    std::vector<btRigidBody*> *m_bodylist;
    
    int m_substeps;
    std::ostream *m_logstream;
    int m_fstream_counter;
};


#endif /* __PUSHING_SIMULATOR_DEBUG_EWEITNAU_H__ */
