// Copyright 2009 Erik Weitnauer
#ifndef __PUSHING_SIMULATOR_EWEITNAU_H__
#define __PUSHING_SIMULATOR_EWEITNAU_H__

#include <vector>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <PushMovement.h>
#include <PhysicsParameters.h>

/// Abstract class with a method for pushing action simulation.

class PushingSimulator {
public:

    PushingSimulator() : m_pusher(NULL), m_pusher_shape(NULL), m_pusher_speed(1.),
    m_ground_shape(NULL) {
    }
    /// Simulates a pushing action performed on the rigid bodies passed.
    /** Returns how much world time was simulated (in seconds). */
    virtual float simulate(const PushMovement &push, std::vector<btRigidBody*> &bodies,
            float before_time_in_s = 0.1, float after_time_in_s = 0.5) = 0;

    virtual void init() = 0;

    void setPhysicsParameters(const PhysicsParameters &params) {
        m_parameters = params;
    }

    PhysicsParameters &getPhysicsParameters() {
        return m_parameters;
    }

    const PhysicsParameters &getPhysicsParameters() const {
        return m_parameters;
    }

protected:
    static void myTickCallback(btDynamicsWorld *world, btScalar timeStep);
    btRigidBody *createGround();
    btRigidBody *createPusher(const btVector3 &dims, float mass = 1000.);
    void applyParameters(btDynamicsWorld *world, std::vector<btRigidBody*> &bodies);

    /// Reset some internal cached data in the broadphase.
    virtual void resetSolver(btDynamicsWorld *world);

    PhysicsParameters m_parameters;
    btRigidBody *m_pusher;
    btCollisionShape* m_pusher_shape;
    float m_pusher_speed;
    btVector3 m_pusher_dims;
    btVector3 m_pusher_target;
    btRigidBody *m_ground;
    btCollisionShape* m_ground_shape;
};


#endif /* __PUSHING_SIMULATOR_EWEITNAU_H__ */
