#ifndef __PUSHING_SCENE_EWEITNAU_H__
#define __PUSHING_SCENE_EWEITNAU_H__

#include <PushedBody.h>
#include <iostream>
#include <PushMovement.h>
#include <PhysicsParameters.h>

struct PushingScene {
	std::vector<PushedBody> pbodies;
	PushMovement push;

	virtual ~PushingScene() { clearBodies(); }

	void clearBodies() {
		for (unsigned int i=0; i<pbodies.size(); ++i) {
			if (pbodies[i].getBody() && pbodies[i].getBody()->getMotionState()) delete pbodies[i].getBody()->getMotionState();
			delete pbodies[i].getBody();
		}
		pbodies.clear();
	}
	
	void resetStatistics() {
		for (unsigned int i=0; i<pbodies.size(); ++i) pbodies[i].clearEndPositions();
	}

	/** As reference transformation the resulting mean transformation of a trial
	 * with a default parameter setting can be passed. The calculated statistics
	 * include variance of distance and max distance of the current body
	 * transformations to the reference transformation. */
	void calcStatistics(Transformation reference_t=Transformation(0,0,0)) {
		for (unsigned int i=0; i<pbodies.size(); ++i) pbodies[i].calcStatistics(reference_t);
	}
	
	static void writeStatisticsHeader(std::ostream &out, const PhysicsParameters &params, const std::string *additional_param = NULL) {
		out << "n ";
		params.writeHeader(out); out << " ";
		if (additional_param != NULL) out << *additional_param << " ";
		PushMovement::writeHeader(out); out << " ";
		out << "x0 y0 rot0 dx_mean dy_mean rot_mean dx_min dy_min rot_min dx_max dy_max rot_max variance ref_dist" << std::endl;
	}

	void writeStatistics(std::ostream &out, PhysicsParameters params, const float *additional_param = NULL) {
		for (unsigned int i=0; i<pbodies.size(); ++i) {
			const Transformation &start = pbodies[i].getStartPosition();
			const Transformation &mean = pbodies[i].getMeanDelta();
			const Transformation &min = pbodies[i].getMinDelta();
			const Transformation &max = pbodies[i].getMaxDelta();
			float variance = pbodies[i].getVariance();
			float ref_dist = pbodies[i].getDistanceToReference();
			out << pbodies[i].getNumberOfEndPositions() << " ";
			params.writeData(out); out << " ";
			if (additional_param != NULL) out << *additional_param << " ";
			push.writeData(out,1/params["world_scaling_factor"]); out << " ";
			out << start.getTx() << " " << start.getTy() << " " << start.getRotation() << " "
					<< mean.getTx() << " "  << mean.getTy() << " " << mean.getRotation() << " "
			    << min.getTx() << " "  << min.getTy() << " " << min.getRotation() << " "
			    << max.getTx() << " "  << max.getTy() << " " << max.getRotation() << " "
			    << variance << " " << ref_dist << std::endl;
		}
	}
	
	/// moves all bodies to their start positions
	void init() {
		for (unsigned int i=0; i<pbodies.size(); ++i) pbodies[i].moveToStart();
	}
	
	/// adds the current body positions to the PushedBody objects
	void record() {
		for (unsigned int i=0; i<pbodies.size(); ++i) pbodies[i].addEndPosition();
	}
	
	std::vector<btRigidBody*> getBodies() {
		std::vector<btRigidBody*> result;
		for (unsigned int i=0; i<pbodies.size(); ++i) { result.push_back(pbodies[i].getBody()); }
		return result;
	}	
};

#endif /* __PUSHING_SCENE_EWEITNAU_H__ */

