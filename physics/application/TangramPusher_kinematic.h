// Copyright 2009 Erik Weitnauer
#ifndef OBJECT_PUSHER_H
#define OBJECT_PUSHER_H

#include "MinimalVisualized.h"
#include <btBulletDynamicsCommon.h>
#include "LinearMath/btAlignedObjectArray.h"
#include <PhysicsParameters.h>

/// Let the user push around a whole set of tangram pieces with the a, s, w, d keys.
class TangramPusherKin : public MinimalVisualizer {
  private:
    btRigidBody* m_convexBody0, *m_convexBody1, *m_cylinder;
    bool m_shouldApplyForce;
    float dx,dz;
    PhysicsParameters m_params;
    float m_target;
    
  protected:
    btRigidBody* createDynamicRigidBody(btTransform transform, btCollisionShape* shape,
                                 float mass, float friction, float restitution);
    btRigidBody* createDynamicRigidBody(btTransform transform, btCollisionShape* shape,
                                 float mass, float friction, float restitution, btVector3 inertia);
    btRigidBody* createDynamicRigidBody(btVector3 position, btCollisionShape* shape,
                                        float mass, float friction, float restitution);

  public:
    TangramPusherKin():m_shouldApplyForce(0),dx(0),dz(0) {/* initPhysics(); initScene();*/ }
    virtual ~TangramPusherKin() { freeScene(); freePhysics(); }
    
    virtual void clientMoveAndDisplay();
    
    virtual void initPhysics();
    virtual void initScene();

    static void myTickCallback(btDynamicsWorld *world, btScalar timeStep);
    virtual void keyboardCallback(unsigned char key, int x, int y);
    virtual void keyboardUpCallback(unsigned char key, int x, int y);
};
#endif
