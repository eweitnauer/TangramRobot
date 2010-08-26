// Copyright 2009 Erik Weitnauer
#ifndef __VISION_ADAPTER_EWEITNAU_H__
#define __VISION_ADAPTER_EWEITNAU_H__

#include "polygon_object.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

/// Conversion between vision classen (e.g. PolygonObject) and Bullet classes (e.g. btRigidBody).
/** When constructing the class, a scaling factor for converting vision classes
to bullet classes can be set. It will be applied to the polygon shape and to the
transformation (position of the object).
*/
class VisionAdapter {
	public:
		/// Constructor. Scaling factor is applied to position and corners when converting to bullet classes.
		VisionAdapter(float scaling=1.0): m_scaling(scaling) {}
		
		/// Converts PolygonShape to a bullet btConvexHullShape. Points are moved so center is at 0,0,0.
		/// CAUTION: The caller must call delete on the returned pointer!
		btConvexHullShape* to_bullet(const PolygonShape& shape) const;

		/// Converts PolygonObject to a bullet btRigidBody using the passed collsion shape.
		/** CAUTION: The caller must call delete on:
		 * <pre>
		 delete body->getMotionState();
   	 delete body;
   	 </pre>
   	 where body is the returned pointer!
   	 */
		btRigidBody* to_bullet(const PolygonObject& polygon, btCollisionShape* shape, float friction=0.9, float restitution=0.5) const;

		/// Converts PolygonObject to a bullet btRigidBody.
		/** Creates a new btCollisionShape from the shape information of the PolygonObject.
		 * CAUTION: The caller must call delete on:
		 * <pre>
		 delete body->getMotionState();
		 delete body->getCollisionShape();
   	 delete body;
   	 </pre>
   	 where body is the returned pointer!
   	 */
		btRigidBody* to_bullet(const PolygonObject& polygon, float friction=0.9, float restitution=0.5) const;

		/// Converts a vision Transformation to a bullet btTransform.
		/** The y_offset is in PE units to the y position and is set to 0.04 which,
		at the moment of writing this, is the additional space around each object in
		which bullet already detects collisions with other objects / the ground. It
		get added to the y value in this function, so objects lie exactly on the
		ground after putting them onto it.*/
		btTransform to_bullet(const Transformation& transform, float y=0, float y_offset=0.04) const;

		/// Converts a Vec vector to a btVector3.
		btVector3 to_bullet(const icl::Vec &v) const;

		/// Converts a bullet btTransform to a vision Transformation.
		Transformation to_vision(const btTransform &b_transform) const;
		
		/// clips the angle to the interval [0, 2*PI[
		static float clip_angle(float angle);
	private:
		float m_scaling; ///< scaling factor from vision to bullet
};
		
#endif /* __VISION_ADAPTER_EWEITNAU_H__ */

