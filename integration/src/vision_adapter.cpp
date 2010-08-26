#include "vision_adapter.h"
#include "btBulletDynamicsCommon.h"
#include "ICLUtils/Point32f.h"
#include <vector>
#include <iostream>

using namespace std;
using namespace icl;

btConvexHullShape* VisionAdapter::to_bullet(const PolygonShape& shape) const {
  btConvexHullShape* bt_shape = new btConvexHullShape();
  // We translate the corners, so the center is at 0,0,0
  float tx = shape.getCenter().x;
  float ty = shape.getCenter().y;
  const vector<Point32f> &corners = shape.getCorners();
  for (unsigned int i=0; i<corners.size(); i++)
    bt_shape->addPoint(btVector3(m_scaling*(corners[i].x-tx),
    														 -m_scaling*shape.getHeight()/2,
    														 m_scaling*(corners[i].y-ty)));
  for (int i=corners.size()-1; i>=0; i--)
		bt_shape->addPoint(btVector3(m_scaling*(corners[i].x-tx),
																 m_scaling*shape.getHeight()/2,
																 m_scaling*(corners[i].y-ty)));
  return bt_shape;
}

btRigidBody* VisionAdapter::to_bullet(const PolygonObject& polygon, btCollisionShape* shape, float friction, float restitution) const {
	btDefaultMotionState *motionState = new btDefaultMotionState(to_bullet(polygon.getTransformation(), polygon.getShape().getHeight()/2));
  btVector3 inertia(0,0,0);
  shape->calculateLocalInertia(polygon.getMass(), inertia);
  btRigidBody::btRigidBodyConstructionInfo bodyCI(polygon.getMass(), motionState, shape, inertia);
  bodyCI.m_friction = friction;
  bodyCI.m_restitution = restitution;
  btRigidBody* body = new btRigidBody(bodyCI);
  body->setActivationState(DISABLE_DEACTIVATION);
  return body;
}

btRigidBody* VisionAdapter::to_bullet(const PolygonObject& polygon, float friction, float restitution) const {
  btCollisionShape *shape = to_bullet(polygon.getShape());
 	return to_bullet(polygon, shape, friction, restitution);
}

btTransform VisionAdapter::to_bullet(const Transformation& transform, float y, float y_offset) const {
	btQuaternion rotation(btVector3(0,1,0), clip_angle(transform.getRotation()));
	btVector3 translation(m_scaling*transform.getTx(), m_scaling*y+y_offset, m_scaling*transform.getTy());
	return btTransform(rotation, translation);
}

btVector3 VisionAdapter::to_bullet(const Vec &v) const {
	return btVector3(v[0]*m_scaling,v[2]*m_scaling,v[1]*m_scaling);
}
		
Transformation VisionAdapter::to_vision(const btTransform &transform) const {
	const btQuaternion &rotation = transform.getRotation();
	float rot = rotation.getAngle();
	if (rotation.getY() < 0) rot = -rot;
	rot = clip_angle(rot);
	float x = transform.getOrigin().getX()/m_scaling;
	float y = transform.getOrigin().getZ()/m_scaling;
	return Transformation(rot,x,y);
}

float VisionAdapter::clip_angle(float angle) {
	if (angle < 0) return angle + 2*M_PI*((int)(-angle/2/M_PI+1));
	if (angle >= 2*M_PI) return angle - 2*M_PI*((int)(angle/2/M_PI));
	return angle;
}
