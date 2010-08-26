// Copyright 2009 Erik Weitnauer
#ifndef __POLYGON_DESCRIPTION_EWEITNAU_H__
#define __POLYGON_DESCRIPTION_EWEITNAU_H__

#include <string>
#include <vector>
#include <ICLUtils/Point32f.h>
#include <ICLUtils/Rect32f.h>
#include "lin_alg.h"

/// Representation of a polygon shape. Has corners. Automatic calculation of centroid and area.
/**
 Add corners to the polygon with the addCorner() method. The polygon has a height,
 which can be used for building a 3d model for a physics engine. You can give your
 polygon a name. The center of gravity (centroid), area and the bounding box
 are calculated automatically.
 
 Call the transformAroundCenter() method to apply a matrix transformation to all
 corners of the polygon. For convinience, also scale() and translate() methods
 are available.
 
 One way to initialized a PolygonShape is passing a vector<Point32f> to the constructor.
*/
class PolygonShape {
	public:
		/// Constructor.
		/** The height can be used for calculation a 3d model of the polygon shape
		 and has no other use yet. */
	  PolygonShape(const std::vector<icl::Point32f> &cs, float height=1.):
	  	m_name(""), m_height(height) { addCorners(cs); }
	  /// Contructor with corners as float array. Format: [x0,y0,x1,y0,...].
	  PolygonShape(const std::vector<float> &cs, float height=1.):
	  	m_name(""), m_height(height) { addCorners(cs); }
	  /// Constructor for PolygonShape without corners.
	  PolygonShape(float height=1.): m_name(""), m_height(height) { resetGeometry(); }
	  /// Constructor for PolygonShape with name.
	  PolygonShape(const std::string &name, float height=1.):
	  	m_name(name),	m_height(height) { resetGeometry(); }
		
		/// Transforms all corners and the center using the transformation matrix.
		/** Scaling and rotating are done relative to own center. */
		void transformAroundCenter(const Mat3 &T);
		
		/// Scales the polygon relative to its center.
		/// The height of the polygon gets not scaled, if false is passed as second argument.
		void scale(float s, bool scale_height=true);
		
		/// Translates the polygon.
		void translate(float tx, float ty);
		
		std::string getName() const { return m_name ;}
		void setName(const std::string &name) { this->m_name = name; }
		
		/** If true is passed as second argument, the area, center, etc. are updated
		 * after adding the corner. */
		void addCorner(icl::Point32f c, bool updateGeometry=false);
		/// Adds all passed corners to the corner vector.
		/** If true is passed as second argument, the area, center, etc. are updated
		 * after adding all corners. */
		void addCorners(const std::vector<icl::Point32f> &cs, bool updateGeometry=true);
		/// Adds all passed corners to the corner vector.
		/** If true is passed as second argument, the area, center, etc. are updated
		 * after adding all corners. Format of the corner vector is [x0,y0,x1,y0,...].*/
		void addCorners(const std::vector<float> &cs, bool updateGeometry=true);
		
		const std::vector<icl::Point32f> &getCorners() const { return m_corners; }
		/// Removes all corners from the polygon. Center, area and diameter are set to zero.
		void clearCorners();
				
		void setHeight(float value) { m_height = value; }
		float getHeight() const { return m_height; }
		
		/// Returns the center of gravity (centroid) of the polygon.
		icl::Point32f getCenter() const { return m_center; }
		
		/// Returns the area of the polygon.
		float getArea() const { return m_area; }
		
		/// Returns the volume of the polygon.
		float getVolume() const { return m_height*m_area; }

		/// Returns the length of the bounding box diagonal.
		const icl::Rect32f &getBoundingBox() const { return m_bbox; } 
		
		bool hasCorners() const { return !m_corners.empty(); }
		
		/// Returns the mean euclidian distance between the corresponding corner pairs of itself and the passed PolygonShape.
		float getMeanCornerDistance(const PolygonShape &other) const;
		
		/// write to stream
		std::ostream& operator>>(std::ostream &out) const;

		/// convert to xml
//		std::ostream& toXML(std::ostream &out) const;

		/// parse xml
//		static PolygonShape createFromXML(const std::string &xml);
		
		/// Calculates the center of gravity, area and bounding box.
		/** You should only have to call this manually after adding corners to the
		 * shape with updateGeometry=false argument. Only correct for corners
		 * forming a not self intersecting polygon. */
		void updateGeometry();

	protected:		
		/// sets bbox, area and center to zero
		void resetGeometry();
		
		/// calculates bounding box from the corners
		void updateBoundingBox();

	private:
	  std::vector<icl::Point32f> m_corners;
	  std::string m_name;
	  icl::Point32f m_center;
	  icl::Rect32f m_bbox;
	  float m_area;
	  float m_height;
};

std::ostream& operator<<(std::ostream &out, const PolygonShape &x);

#endif /* __POLYGON_DESCRIPTION_EWEITNAU_H__ */

