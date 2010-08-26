// Copyright 2009 Erik Weitnauer
#ifndef __POLYGON_MAPPER_EWEITNAU_H__
#define __POLYGON_MAPPER_EWEITNAU_H__

#include <string>
#include <vector>

#include <ICLUtils/Point32f.h>

#include "polygon_shape.h"
#include "polygon_object.h"

/// Searches for a 2d transformation (rotation, translation, scaling) that will map a polygon shape / polygon object as closely as possible on another polygon shape.
/**
PolygonMapper defines four static methods.

Two of them search for an affine transformation[1] to map one / several polygon shape onto
another polygon shape. The results are given back (optinally) sorted by mapping
error (how much is the result of the transformated polygon off the target polygon)
as a list of PolygonObjects (which represent the mappings).
Also, an error threshold to sort out unlikely or bad mappings is used.

The other two methods map one / several polygon objects onto a polygon shape.
Here, if the mapping is successful, the polygon object that fits best will be
updated. Its tranformation will be changed, so it has the same position as the
target shape. Also its last_active_time will be set to the current time.
These two methods can be used to track polygons. You can just pass a list of
formerly detected polygon objects and the methods will chose the best fit and
update the polygon object.

[1] translation, rotation and (optional) scaling, but here without shearing
*/
class PolygonMapper {
	public:
		/// Maps a (model) polygon shape or polygon object to a (observed, target) polygon shape.
		/**
		 All mappings, whose error is below a threshold, will be returned in a
		 vector.
		 
		 Algorithm:
		 <pre>
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
		 </pre>

		 However, in the default configuration of the algorithm here, it does not
		 use any scaling.

		 The threshold is chosen dynamically, so that all mappings in which in
		 average each corner is no more than dynThresh * model.diameter off. The
		 default value for dynThresh is 10%.

		 Outlook:
		 Later this algorithm can be extended to do some optimization (e.g. gradient
		 stepping on the error function), so a solution in which all the corners are
		 off a bit can also be found.
		*/
		static std::vector<PolygonObject> mapPolygonShape(
			const PolygonShape &target,
		  const PolygonShape &model,
		  float dynThresh=0.1,
		  bool sortResult=true,
		  bool useScaling=false);
		
		/// Tries to map each of the model polygons onto the target polygon.
		/** Collects
		 all mappings and returns them. */
		static std::vector<PolygonObject> mapPolygonShapes(
			const PolygonShape &target,
		  const std::vector<PolygonShape> &models,
		  float dynThresh=0.1,
		  bool sortResult=true,
		  bool useScaling=false);

		/// Transforms the passed polygon object onto the polygon shape if possible.
		/** If the polygon object could be mapped onto the shape, its transformation
		 * is updated, its last_active_time is set to the passed current time and
		 * true is returned. Amoung all possible mappings the one which moves the
		 * corners of the PolygonObject the smallest distance is chosen.
		 * The PolygonObject is only considered for mapping, if its isActive() method
		 * returns false. This way, the same PolygonObject will not be mapped to more
		 * than one shape at the same time.
		 * The mapping of a PolygonObject is only successful, if the mapping error is
		 * below dynThresh, the rotation not bigger than maxRotation and the translation
		 * not bigger than maxTranslation. If -1 is passed for maxRotation or maxTranslation,
		 * it is not taken into account. */
		static bool mapPolygonObject(
			const PolygonShape &target,
		  PolygonObject &polygon,
		  int cur_time,
		  float dynThresh=0.1,
		  float maxRotation=-1,
		  float maxTranslation=-1);
		  
		/// Transformes one of the passed polygon objects onto the polygon shape if possible.
		/** This method is very similar to the mapPolygonObject() method. The difference
		 * is that now a list of PolygonObjects is passed. For each of the PolygonObjects
		 * possible mappings are calculated and the best amoung all of them is chosen.
		 * The PolygonObject with this mapping gets updated.
		 * So, if successful, only one of the PolygonObjects is modified. If no valid
		 * mapping could be found for any of the PolygonObjects, none of them is modified. */
		static bool mapPolygonObjects(
			const PolygonShape &target,
		  std::vector<PolygonObject> &polygons,
		  int cur_time,
		  float dynThresh=0.1,
		  float maxRotation=-1,
		  float maxTranslation=-1);
		  
	private:
	 	/** The index of the mapping which moves the corners under its transformation
	   * the smallest distance is taken. Only mappings with rotation and translation
	   * not bigger than maxRotation and maxTranslation are considered */
	 	static int getBestMapping(const std::vector<PolygonObject> &mappings,
	 		float maxRotation, float maxTranslation);
	 	static int getBestMapping(const std::vector<PolygonObject*> &mappings,
	 		float maxRotation, float maxTranslation);
};

#endif /* __POLYGON_MAPPER_EWEITNAU_H__ */

