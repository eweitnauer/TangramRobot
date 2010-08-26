// Copyright 2009 Erik Weitnauer
#ifndef __PUSHING_SIMULATOR_FAST_EWEITNAU_H__
#define __PUSHING_SIMULATOR_FAST_EWEITNAU_H__

#include <btBulletDynamicsCommon.h>
#include <PushingSimulator.h>

/// Simulates pushing actions.

/** Pass a vector of pointers to RigidBodies and a pushing action. Then call
simulate() with the amount of time to simulate. The position changes are written
in place into the passed RigidBodies. */
class PushingSimulatorFast : public PushingSimulator {
public:

    PushingSimulatorFast() : m_dynamicsWorld(NULL), m_broadphase(NULL),
    m_dispatcher(NULL), m_solver(NULL), m_collisionConfiguration(NULL),
    m_substeps(1) {
    }

    virtual ~PushingSimulatorFast() {
        freeScene();
        freePhysics();
    }

    virtual void init() {
        initPhysics();
        initScene();
    }
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
            float before_time_in_s = 0.1, float after_time_in_s = 0.5);

    void initPhysics();
    void freePhysics();

    /// Set the number of steps used to simulate the pushing.
    void setSubsteps(int value) {
        m_substeps = value;
    }

    /// Creates the m_pusher and m_ground objects and adds them to scene.
    void initScene();
    /// Removes and deletes all objects in the world. <omfg>
    void freeScene();

private:
    btDiscreteDynamicsWorld *m_dynamicsWorld;
    btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
    btBroadphaseInterface* m_broadphase;
    btCollisionDispatcher* m_dispatcher;
    btConstraintSolver* m_solver;
    btDefaultCollisionConfiguration* m_collisionConfiguration;

    int m_substeps;
};


#endif /* __PUSHING_SIMULATOR_FAST_EWEITNAU_H__ */
