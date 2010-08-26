// Copyright 2009 Erik Weitnauer
/// Pushing around a complete tangram puzzle with a, s, w, d keys.
#include "TangramPusher.h"
#include "Shapes.h"
#include <btBulletDynamicsCommon.h>
#include "GlutStuff.h"
#include "LinearMath/btAlignedObjectArray.h"
#include <iostream>

using namespace std;

bool g_use_custom_inertia;

#define SCALING 5.0
/* Scaling the world:
Obviously any collision shapes must be scaled appropriately, but there are also
some other things that need to be changed. In general, if you are scaling the
world by a factor of X then you must do the following:

    * Scale collision shapes about origin by X
    * Scale all positions by X
    * Scale all linear (but not angular) velocities by X
    * Scale linear Sleep Threshold by X ==> [Anm.: no sleeping anyway, so no need]
    * Scale gravity by X ==> [Anm.: not a good idea - becomes instable]
    * Scale all impulses you supply by X
    * Scale all torques by X^2
    * Scale all inertias by X if not computed by Bullet 

Damping is a ratio so this does not need to be changed.
Angular velocity should not need to be changed either.

[http://www.bulletphysics.com/mediawiki-1.5.8/index.php?title=Scaling_The_World]
*/

void TangramPusher::myTickCallback(btDynamicsWorld *world, btScalar timeStep) {
  TangramPusher *tp = static_cast<TangramPusher *>(world->getWorldUserInfo());
//  tp->m_cylinder->setGravity(btVector3(0,0,0));
  tp->m_cylinder->setAngularFactor(btVector3(0,0,0));
  tp->m_cylinder->setLinearVelocity(btVector3(2.*tp->dx*SCALING/20.,0.,2.*SCALING/20.*tp->dz)); 
}

void TangramPusher::clientMoveAndDisplay() {
  float usec = getDeltaTimeMicroseconds();
  static bool firsttime = true;
  if (m_dynamicsWorld) {
    if (firsttime) {
      m_dynamicsWorld->setInternalTickCallback(TangramPusher::myTickCallback, static_cast<void *>(this));
      firsttime = false;
    }
    m_dynamicsWorld->stepSimulation(usec / 1.e6, 100, m_params["sim_stepsize"]);
  }
  displayCallback();
}

void TangramPusher::keyboardUpCallback(unsigned char key, int x, int y) {
  switch (key) {
    case 'a': case 'd': dx = 0; break;
    case 's': case 'w': dz = 0; break;
  }
}

void TangramPusher::keyboardCallback(unsigned char key, int x, int y) {
  switch (key) {
    case 'a': dx = 1; break;
    case 'd': dx = -1; break;
    case 's': dz = -1; break;
    case 'w': dz = 1; break;
    case ' ': m_shouldApplyForce = !m_shouldApplyForce; break;
    default: DemoApplication::keyboardCallback(key, x, y);
  }
}

void TangramPusher::initPhysics() {
  MinimalVisualizer::initPhysics();
}

btRigidBody* TangramPusher::createDynamicRigidBody(btVector3 position, btCollisionShape* shape,
    float mass, float friction, float restitution) {
  return createDynamicRigidBody(btTransform(btQuaternion(0,0,0,1), position), shape, mass, friction, restitution);
}

btRigidBody* TangramPusher::createDynamicRigidBody(btTransform transform, btCollisionShape* shape,
                                      float mass, float friction, float restitution) {
	btVector3 inertia(0,0,0);
  shape->calculateLocalInertia(mass, inertia);
  return createDynamicRigidBody(transform, shape, mass, friction, restitution, inertia);
}

btVector3 getInertia(btConvexHullShape* shape, float mass) {
	btScalar margin = shape->getMargin();

	btTransform ident;
	ident.setIdentity();
	btVector3 aabbMin,aabbMax;
	shape->getAabb(ident,aabbMin,aabbMax);
	btVector3 halfExtents = (aabbMax-aabbMin)*btScalar(0.5);

	btScalar lx=btScalar(2.)*(halfExtents.x()+margin);
	btScalar ly=btScalar(2.)*(halfExtents.y()+margin);
	btScalar lz=btScalar(2.)*(halfExtents.z()+margin);
	cout << "x="<<lx<<" y="<<ly<<" z="<<lz<<" including 2x margin "<< margin << endl;
	cout << "mass=" << mass;
	const btScalar x2 = lx*lx;
	const btScalar y2 = ly*ly;
	const btScalar z2 = lz*lz;
	const btScalar scaledmass = mass * btScalar(0.08333333);

	return scaledmass * (btVector3(y2+z2,x2+z2,x2+y2));
}

btRigidBody* TangramPusher::createDynamicRigidBody(btTransform transform, btCollisionShape* shape,
                                      float mass, float friction, float restitution, btVector3 inertia) {
  btDefaultMotionState* motionState = new btDefaultMotionState(transform);
  if (!g_use_custom_inertia) {
  	inertia = getInertia((btConvexHullShape*)shape, mass);
  	shape->calculateLocalInertia(mass, inertia);
		cout << "bullet inertia: " << inertia.getX() << ", " << inertia.getY() << ", " << inertia.getZ() << endl;
  } else {
	  cout << "custom inertia: " << inertia.getX() << ", " << inertia.getY() << ", " << inertia.getZ() << endl;
	}
	btRigidBody::btRigidBodyConstructionInfo bodyCI(mass, motionState, shape, inertia);
  bodyCI.m_friction = friction;
  bodyCI.m_restitution = restitution;
  bodyCI.m_linearDamping = m_params["lin_damping"];
  bodyCI.m_angularDamping = m_params["ang_damping"];
  btRigidBody* body = new btRigidBody(bodyCI);
  body->setActivationState(DISABLE_DEACTIVATION);
  return body;
}

void TangramPusher::initScene() {
  m_dynamicsWorld->setGravity(m_params.getGravity());
 	m_dynamicsWorld->getSolverInfo().m_numIterations = m_params["solver_iterations"];
	m_dynamicsWorld->getSolverInfo().m_solverMode = m_params.getSolverMode();
	m_dynamicsWorld->getSolverInfo().m_splitImpulse = m_params["split_impulse"];
	m_dynamicsWorld->getSolverInfo().m_erp = m_params["erp"];
	m_dynamicsWorld->getSolverInfo().m_tau = m_params["tau"];
	  
  // add static ground plane at y=0
  btCollisionObject *ground = createGround();
  ground->setFriction(m_params["friction_ground"]);
  ground->setRestitution(m_params["restitution_ground"]);
  m_dynamicsWorld->addCollisionObject(ground);
  
  float a = 0.1*SCALING; // base length in m, scaled
  float h = 0.018*SCALING; // heigth in m, scaled
  float m = 0.180*SCALING*SCALING*SCALING; // don't scale the mass (in kg)
  
  // cylinder with radius 1, length 5
  btCollisionShape* cylinderShape = new btCylinderShape(btVector3(h/2,a/2,h/2));
  cylinderShape->setMargin(m_params["collision_margin"]);
  m_collisionShapes.push_back(cylinderShape);
  // triangle shapes
  btCollisionShape* smallTriangleShape = Shapes::createTriangleShape(a,h,Shapes::getCorrectShapeFactor(Shapes::SMALL_TRIANGLE),m_params["collision_margin"]);
  smallTriangleShape->setMargin(m_params["collision_margin"]);
  ((btConvexHullShape*)smallTriangleShape)->recalcLocalAabb();
  btCollisionShape* mediumTriangleShape = Shapes::createTriangleShape(sqrt(2)*a,h,Shapes::getCorrectShapeFactor(Shapes::MEDIUM_TRIANGLE),m_params["collision_margin"]);
  mediumTriangleShape->setMargin(m_params["collision_margin"]);
  ((btConvexHullShape*)mediumTriangleShape)->recalcLocalAabb();
  btCollisionShape* bigTriangleShape = Shapes::createTriangleShape(2*a,h,Shapes::getCorrectShapeFactor(Shapes::LARGE_TRIANGLE),m_params["collision_margin"]);
  bigTriangleShape->setMargin(m_params["collision_margin"]);
  ((btConvexHullShape*)bigTriangleShape)->recalcLocalAabb();
  cout << "Margin of Triangle Shape: " << bigTriangleShape->getMargin() << endl;
  m_collisionShapes.push_back(smallTriangleShape);
  m_collisionShapes.push_back(mediumTriangleShape);
  m_collisionShapes.push_back(bigTriangleShape);
  // square shape
  btCollisionShape* squareShape = Shapes::createSquareShape(a,h,Shapes::getCorrectShapeFactor(Shapes::SQUARE),m_params["collision_margin"]);
  squareShape->setMargin(m_params["collision_margin"]);
  ((btConvexHullShape*)squareShape)->recalcLocalAabb();
  cout << "Margin of Square Shape: " << squareShape->getMargin() << endl;
  m_collisionShapes.push_back(squareShape);
  // parallelogram shape
  btCollisionShape* prllShape = Shapes::createParallelogramShape(a,h,Shapes::getCorrectShapeFactor(Shapes::PARALLELOGRAM),m_params["collision_margin"]);
  prllShape->setMargin(m_params["collision_margin"]);
  ((btConvexHullShape*)prllShape)->recalcLocalAabb();
  m_collisionShapes.push_back(prllShape);
  
  // add cylinder shape to the world 3 m above ground
  btTransform cylinder_transform(btQuaternion(0,0,0,1), btVector3(1.5*a,a/2,0));
  m_cylinder = createDynamicRigidBody(cylinder_transform, cylinderShape, 10*SCALING*SCALING*SCALING,
                 m_params["friction_pusher"],  m_params["restitution_pusher"]);
  m_dynamicsWorld->addRigidBody(m_cylinder);
  m_cylinder->setGravity(btVector3(0,0,0));
  m_cylinder->setAngularFactor(0);
  
  float friction = m_params["friction_polygon"];
  float restitution = m_params["restitution_polygon"];
  g_use_custom_inertia = m_params["use_custom_inertia_tensor"]==1;
  
  float a2 = a+a/20;
  float h2 = h*2;
  a += 2*m_params["collision_margin"];
  h += 2*m_params["collision_margin"];
  btQuaternion q;
  float sqrt2 = sqrt(2);
  // add triangles to the world
  cout << "=== small triangles ===" << endl;
  q.setRotation(btVector3(0,1,0),M_PI/2);
  m_dynamicsWorld->addRigidBody(
    createDynamicRigidBody(btTransform(q,btVector3(-a2*sqrt2/3.,h/2,0)), smallTriangleShape, m, friction, restitution, Shapes::getInertiaTensor(Shapes::SMALL_TRIANGLE, a, h, m)));
  q.setRotation(btVector3(0,1,0),-M_PI);
  m_dynamicsWorld->addRigidBody(
    createDynamicRigidBody(btTransform(q,btVector3(a2/sqrt2,h2/2,a2*(sqrt2/2+sqrt2/3))), smallTriangleShape, m, friction, restitution, Shapes::getInertiaTensor(Shapes::SMALL_TRIANGLE, a, h, m)));
  cout << "=== medium triangle ===" << endl;
  q.setRotation(btVector3(0,1,0),-M_PI/4);
    m_dynamicsWorld->addRigidBody(
  createDynamicRigidBody(btTransform(q,btVector3(-a2*2*sqrt2/3,h2/2,a2*2*sqrt2/3)), mediumTriangleShape, 2*m, friction, restitution, Shapes::getInertiaTensor(Shapes::MEDIUM_TRIANGLE, a, h, 2*m)));
  cout << "=== large triangles ===" << endl;
  m_dynamicsWorld->addRigidBody(
    createDynamicRigidBody(btTransform(btQuaternion(0,0,0,1), btVector3(0,h2/2,-a2*2*sqrt2/3)), bigTriangleShape, 4*m, friction, restitution, Shapes::getInertiaTensor(Shapes::LARGE_TRIANGLE, a, h, 4*m)));
  q.setRotation(btVector3(0,1,0),-M_PI/2);
  m_dynamicsWorld->addRigidBody(
    createDynamicRigidBody(btTransform(q, btVector3(a2*2*sqrt2/3,h2/2,0)), bigTriangleShape, 4*m, friction, restitution, Shapes::getInertiaTensor(Shapes::LARGE_TRIANGLE, a, h, 4*m)));
  // add square
    cout << "=== square ===" << endl;
  q.setRotation(btVector3(0,1,0),-M_PI/4);
  btRigidBody *squareBody = createDynamicRigidBody(btTransform(q, btVector3(0,h2/2,a2/sqrt2)), squareShape, 4*m, friction, restitution, Shapes::getInertiaTensor(Shapes::SQUARE, a, h, 4*m));
  m_dynamicsWorld->addRigidBody(squareBody);
  // add parallelogram
  cout << "=== parallelogram ===" << endl;
  q.setRotation(btVector3(0,1,0),-M_PI/2);
  btRigidBody *prllBody = createDynamicRigidBody(btTransform(q, btVector3(-3.*a2*sqrt2/4,h2/2,-a2*sqrt2/4)), prllShape, 2*m, friction, restitution, Shapes::getInertiaTensor(Shapes::PARALLELOGRAM, a, h, 2*m));
  m_dynamicsWorld->addRigidBody(prllBody);
  
  setCameraDistance(0.5*SCALING);
}

int main(int argc,char** argv) {
  TangramPusher mvis;
  mvis.init();
  return glutmain(argc, argv,640,480,"Minimal Visualization Example",&mvis);
}
