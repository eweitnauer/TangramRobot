// Copyright 2009 Erik Weitnauer
#ifndef __TANGRAM_CLASSIFIER_EWEITNAU_H__
#define __TANGRAM_CLASSIFIER_EWEITNAU_H__

#include <vector>
#include "polygon_shape.h"
#include <ICLUtils/XMLDocument.h>

/// Classification of polygons. Returns candidate tangram tiles for a observed target polygon. 
/**
There are two classification modes: One with scaling allowed, one with no scaling
allowed. If scaling is allowed, some of the shapes [triangle, square, parallelogram]
might be returned as candidate shapes.
Without scaling, some of the shapes [small triangle, medium triangle, square,
parallelogram] might be returned as candidate shapes. Without scaling, size
information about the observed tangram tiles must be provided in form of the base_length
of the tiles (same as the side length of the square).
<pre>
==================
|\ st  /\        | 
|  \ /    \  mt  |
|    \ sq  /     |
| lt   \ /st|\   |
|      / \  |  \ |
|    /     \| pa |
|  /   lt    \   |
|/             \ |
==================
</pre>

By now, the class selects the candidate tangram shapes only by counting the
corners and -- in case the mode without scaling is used -- by comparing the
area of the observed and model shapes.
*/

class TangramClassifier {
	public:
		/// Standard Constructor that creates a TangramClassifier with an empty list of classification model shapes.
		TangramClassifier(): m_base_length(0), m_height(0) {}
	  
	  /// Creates the 5 different tangram shapes according to the passed base length and height.
	  /** Small, medium and large triangle shapes, a square and a parallelogram
	   * shape are created. The base legth is the length of one square side.
	   * When not passing a height of the polygon, it is set to (base_length/7).*/
	  void loadStandardTangramShapes(float base_length=1, float height=-1);

		/// Loads polygon shapes for classification from a xml document.
		void loadShapesFromXML(const std::string &filename);
		
		/// Clears all shapes.
		void clearShapes() { m_shapes.clear(); }
		
	  /// Returns a vector of tangram tiles (PolygonShapes), which are similar to the observed object.
	  /** At the moment, all the tangram tiles are returned whose corner count is
	   equal or at most smaller by two to the passed data shape.
	   In case size_matters=true is passed, the size of the shapes is also taken into accout.
		 Then, the tolerance value is used to decide, whether the observed shape is
		 too big or too small to be classified as any of the models. A tolerance of
		 0.2 means that the area of the model polygon can differ up to 20% from
		 the area of the observed polygon in order to be taken as a candidate match.
		 If no matching candidate tangram tile is found, an empty vector is returned.
		 WARNING: The returned vector is only valid until the next call of the function!*/
		const std::vector<PolygonShape> &classify(PolygonShape &data, float tolerance=0.2, bool size_matters=false);
		
		/// Returns a vector with Triangle, Square and Parallelogram shape in it.
		const std::vector<PolygonShape> &getAllShapes() {return m_shapes;}
		
		inline float getBaseLength() const { return m_base_length; }
		inline float getHeight() const { return m_height; }
		
		/// Change the baselength and height of the shapes (all shapes get rescaled).
		/// If no height or a negative height is passed, the height remains unchanged.
		void setSize(float base_length, float height=-1);
		
	friend std::ostream& operator<<(std::ostream &out, const TangramClassifier &tc);
	
	private:
		std::vector< PolygonShape > m_shapes;
		std::vector< PolygonShape > m_candidate_shapes;
		
		float m_base_length;
		float m_height;
};

std::ostream& operator<<(std::ostream &out, const TangramClassifier &tc);

#endif /* __TANGRAM_CLASSIFIER_EWEITNAU_H__ */

