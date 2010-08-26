#include <PushingSimulatorDebug.h>
#include <Shapes.h>
#include <sstream>
#include <iostream>
#include <fstream>

using namespace std;

btTransform g_start_position;

btRigidBody* createDynamicBody(btTransform transform, Shapes::ShapeType shape_type, float base_length, float height, float mass) {
  btCollisionShape *shape = Shapes::createShape(shape_type, base_length, height, 1, 0);
  //btCollisionShape *shape = new btBoxShape(btVector3(base_length/2, height/2, base_length/2));
  btDefaultMotionState* motionState = new btDefaultMotionState(transform);
  btVector3 inertia(0,0,0);
  shape->calculateLocalInertia(mass, inertia);
  btRigidBody::btRigidBodyConstructionInfo bodyCI(mass, motionState, shape, inertia);
  btRigidBody* body = new btRigidBody(bodyCI);
  body->setActivationState(DISABLE_DEACTIVATION);
  return body;
}

btRigidBody* createSquare() {
  // create body
  float base_length = 0.3;
  float height = 0.06;
  float mass = 0.138;
  g_start_position = btTransform(btQuaternion(0,0,0,1), btVector3(50,height/2+0.04,0));
  return createDynamicBody(g_start_position, Shapes::SQUARE, base_length, height, mass);
}

void resetBody(btRigidBody* body) {
  btDefaultMotionState* myMotionState = (btDefaultMotionState*)body->getMotionState();
	myMotionState->m_startWorldTrans = g_start_position;
	myMotionState->m_graphicsWorldTrans = myMotionState->m_startWorldTrans;
	body->setCenterOfMassTransform(myMotionState->m_graphicsWorldTrans );
	body->setInterpolationWorldTransform(myMotionState->m_startWorldTrans);
	body->forceActivationState(ACTIVE_TAG);
	body->activate();
	body->setDeactivationTime(0);
}

void run_simulation(PushingSimulatorDebug &psim, btRigidBody* body) {
  // create pusher
  btVector3 push_start(0.5,0,0);
  btVector3 push_end(6,0,0);
  float push_speed = 2.0;
  btVector3 pusher_dims(0.5,2.0,0.5);
  PushMovement push_movement(push_start, push_end, pusher_dims, push_speed);
  
  vector<btRigidBody*> bodies;
  bodies.push_back(body);

  // simulate pushing
  //float time_in_s = 0.2;
  psim.simulate(push_movement, bodies);
}

template <class T> inline std::string to_str(const T& t) {
  std::stringstream ss; ss << t; return ss.str();
}

void two_runs() {
  PushingSimulatorDebug psim;
  psim.init();
  int N=2;
  fstream log[N];
  btRigidBody *body = createSquare();
  for (int i =0; i<N; ++i) {
    string name = "log"+to_str(i);
    cout << name << endl;
    log[i].open(name.c_str(), fstream::out | fstream::trunc);
    psim.setLogStream(log[i]);
    resetBody(body);
    run_simulation(psim,body);
    log[i].close();
  }
}

int main() {
  two_runs();
  
  return 0;
}
