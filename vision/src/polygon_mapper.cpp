// Copyright 2009 Erik Weitnauer
#include "polygon_mapper.h"
#include <math.h>
#include <algorithm>

using namespace std;
using namespace icl;

int PolygonMapper::getBestMapping(const vector<PolygonObject> &mappings,
		float maxRotation, float maxTranslation) {
	// take the mapping in which the corners travel the smallest distance but only
	// take mappings with rotation and translation below max values into account
  float dist=-1;
	int index=-1;
	for (unsigned int i=0; i<mappings.size(); i++) {
	 	if ( ((maxRotation < 0) or (mappings[i].getTransformation().getRotation() < maxRotation)) and
       ((maxTranslation < 0) or (mappings[i].getTransformation().length() < maxTranslation)) ) {
			float cur_dist = mappings[i].getTransformationCornerMovement();
			if (dist == -1 or dist>cur_dist) {
				index = i; dist = cur_dist;
			}
		}
	}
	return index;
}

int PolygonMapper::getBestMapping(const vector<PolygonObject*> &mappings,
		float maxRotation, float maxTranslation) {
	// take the mapping in which the corners travel the smallest distance but only
	// take mappings with rotation and translation below max values into account
  float dist=-1;
	int index=-1;
	for (unsigned int i=0; i<mappings.size(); i++) {
	 	if ( (mappings[i] != NULL) and
	 	     ((maxRotation < 0) or (mappings[i]->getTransformation().getRotation() < maxRotation)) and
       ((maxTranslation < 0) or (mappings[i]->getTransformation().length() < maxTranslation)) ) {
			float cur_dist = mappings[i]->getTransformationCornerMovement();
			if (dist == -1 or dist>cur_dist) {
				index = i; dist = cur_dist;
			}
		}
	}
	return index;
}

bool PolygonMapper::mapPolygonObjects(
		const PolygonShape &target,
		vector<PolygonObject> &polygons,
		int cur_time,
		float dynThresh,
		float maxRotation,
		float maxTranslation) {
	// collect the best mapping for each polygon object
	int index;
	vector<PolygonObject*> best_mappings;
	for (unsigned int i=0; i<polygons.size(); i++) {
		// only check if polygon was not mapped to something already this timestep
		if (polygons[i].isActive(cur_time)) best_mappings.push_back(NULL);
		else {
			vector<PolygonObject> mappings = mapPolygonShape(target, polygons[i].getTransformedShape(), dynThresh, false, false);
			index = getBestMapping(mappings, maxRotation, maxTranslation);
			if (index == -1) best_mappings.push_back(NULL);
			else best_mappings.push_back(new PolygonObject(mappings[index]));
		}
	}
	
	index = getBestMapping(best_mappings, maxRotation, maxTranslation);
	
	// apply the mapping with the smallest corner distance
	bool success = false;
	if (index != -1) {
		polygons[index].addTransformation(best_mappings[index]->getTransformation());
		polygons[index].setActive(cur_time);
		success = true;
	}
	// free the best_mappings vector
	for (unsigned int i=0; i<best_mappings.size(); i++) delete best_mappings[i];
	return success;
}

bool PolygonMapper::mapPolygonObject(
		const PolygonShape &target,
		PolygonObject &polygon,
		int cur_time,
		float dynThresh,
		float maxRotation,
		float maxTranslation) {
	// get all possible mappings with error below dynThresh
	vector<PolygonObject> mappings = mapPolygonShape(target, polygon.getTransformedShape(),
		dynThresh, false, false);
	
	// get the best mapping (smallest corner movement under transformation)
	int index = getBestMapping(mappings, maxRotation, maxTranslation);
	
	// apply the mapping with the smallest corner distance
	if (index != -1) {
		polygon.addTransformation(mappings[index].getTransformation());
		polygon.setActive(cur_time);
		return true;
	} else return false;
}


/// Calls mapPolygon() for each of the model polygons and collects results.
vector<PolygonObject> PolygonMapper::mapPolygonShapes(
		const PolygonShape &target,
	  const std::vector<PolygonShape> &models,
	  float dynThresh,
	  bool sortResult,
	  bool useScaling) {
	vector<PolygonObject> maps;
	
	for (unsigned int i=0; i<models.size(); i++) {
		vector<PolygonObject> curr_maps = mapPolygonShape(target, models[i], dynThresh, false, useScaling);
		maps.insert(maps.end(), curr_maps.begin(), curr_maps.end());
	}
	
	if (sortResult) sort(maps.begin(), maps.end());
	return maps;
}
		  		  
/*
This function will map an (model) polygon to an (observed, target) polygon.
All mappings, whose error is below a threshold, will be returned in a
vector.

Algorithm:
(1) align the center of model to center of target
(2) chose corner cm[0] from model
(3) loop through all target corners ct[i]
	(3a) set rotation and scaling so that cm[0] is mapped onto ct[i]
	(3b) use this rotation and scaling on all other model corners (==> cm_t[])
	(3c) clac quad. error by iterating over all target corners and
			 sum the quad. distance to the closest model corner:
			 E_t[i] = Sum_j=1..nt(||ct[j]-nearest(ct[j], cm_t)||^2)
	(3d) clac quad. error by iterating over all model corners and
			 sum the quad. distance to the closest target corner:
			 E_m[i] = Sum_j=1..nm(||cm_t[j]-nearest(cm_t[j], ct)||^2)
	(3e) add the mapping to the result vector, if E_t[i] and E_m[i] are
		   both smaller than a threshold
*/
vector<PolygonObject> PolygonMapper::mapPolygonShape(
		const PolygonShape &target,
		const PolygonShape &model,
		float dynThresh,
		bool sortResult,
		bool useScaling) {
	vector<PolygonObject> maps;

  Point32f centerm = model.getCenter();
  Point32f centerr = target.getCenter();
  
	// step (1)
	float tx = centerr.x - centerm.x;
	float ty = centerr.y - centerm.y;
	//cout << "tx=" << tx << " ty=" << ty << endl;
	
	// step (2)
	Point32f cm0 = model.getCorners()[0]; // model corner for fitting
	//cout << "cm0=" << cm0 << endl;
	
	// step (3)
	vector<Point32f> ct = target.getCorners();
  vector<Point32f> cm = model.getCorners();
	float error_m[cm.size()];
	float error_t[ct.size()];
	for (unsigned int j = 0; j < ct.size(); j++) {
	  //cout << "==Fitting " << cm0 << " to " << ct[j] << ":" << endl;
		// step (3a) todo: div by zero
		float scaling = useScaling ? sqrt( LinAlg::distance2(ct[j],centerr) /
													       LinAlg::distance2(cm0, centerm) )
													     : 1.;
		//cout << "scaling=" << scaling << endl;
		float angle_m = atan2(cm0.y-centerm.y, cm0.x-centerm.x);
		float angle_t = atan2(ct[j].y-centerr.y, ct[j].x-centerr.x);
		float theta = angle_m-angle_t;
		//cout << "angle_m=" << angle_m << endl;
		//cout << "angle_t=" << angle_t << endl;
		//cout << "theta=" << theta << endl;
		// step (3b)
		error_m[j] = 0; error_t[j] = 0;
		Mat3 T = LinAlg::createTransformationMatrix2d(theta, tx, ty, scaling);
		vector<Point32f> cm_t = LinAlg::transformedAround(centerm, cm, T);
		// step (3c)
		for (unsigned int i = 0; i < ct.size(); i++)
			error_t[j] += LinAlg::closestDistanceSqr(ct[i], cm_t);
		// step (3d)
		for (unsigned int i = 0; i < cm.size(); i++)
			error_m[j] += LinAlg::closestDistanceSqr(cm_t[i], ct);
		// step (3e)
		float threshold = pow(scaling*LinAlg::diagonalLength(model.getBoundingBox())*dynThresh,2) * 
											model.getCorners().size();
		//cout << "Error threshold is " << threshold << endl;
		float error = max(error_t[j], error_m[j]);
	  if (error <= threshold) 
	  	maps.push_back(PolygonObject(Transformation(theta, tx, ty), scaling, model, error));
	}
	
	if (sortResult) sort(maps.begin(), maps.end());
	return maps;	
}
		
