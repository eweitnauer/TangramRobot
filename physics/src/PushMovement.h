// Copyright 2009 Erik Weitnauer
#ifndef __PUSH_MOVEMENT_EWEITNAU_H__
#define __PUSH_MOVEMENT_EWEITNAU_H__

#include <iostream>
#include "LinearMath/btVector3.h"

/// Represents pushing action with 3d start and end point and movement with constant speed.
/** The points are the position of the 'finger tip'. */
struct PushMovement {
	btVector3 start;
	btVector3 end;
	btVector3 pusher_dims;
	float speed;
	
	PushMovement(): speed(2.) {}
	
	PushMovement(btVector3 start, btVector3 end,
		btVector3 pusher_dims=btVector3(0.5,2,0.5), float speed=2.)
		:	start(start), end(end),
		  pusher_dims(pusher_dims), speed(speed) {}
	
	/// Progress in 0...1. Formular use: start + ((end-start)*progress)
	btVector3 getPosition(float progress) const {
		return start + ((end-start)*progress);
	}
	
	float getLength() const {
		return (end-start).length();
	}
	
	static void writeHeader(std::ostream &out) {
		out << "pusher_x0 pusher_y0 pusher_x1 pusher_y1 pusher_radius pusher_speed";
	}
	
	void writeData(std::ostream &out, float factor) {
		out << factor*start.getX() << " " << factor*start.getZ() << " "
		    << factor*end.getX() << " " << factor*end.getZ() << " "
		    << factor*pusher_dims.getX() << " " << factor*speed;
	}
};

std::ostream& operator<<(std::ostream &out, const PushMovement &x);

#endif /* __PUSH_MOVEMENT_EWEITNAU_H__ */

