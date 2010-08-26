#ifndef __CAMERA_TRANSFORMER_EWEITNAU_H__
#define __CAMERA_TRANSFORMER_EWEITNAU_H__

#include <ICLUtils/ConfigFile.h>
#include <ICLGeom/Camera.h>
#include <ICLGeom/PlaneEquation.h>
#include <ICLUtils/Point32f.h>

#include <polygon_shape.h>

class CameraTransformer {
	public:
		CameraTransformer(const icl::Camera cam = icl::Camera(),
			icl::PlaneEquation plane=icl::PlaneEquation(icl::Vec(0,0,0),icl::Vec(0,0,1)))
			: m_camera(cam), m_plane(plane) {};
		
		/// Sets the plane equation, that will be used for screen to world transformation.
		void setPlaneEquation(icl::PlaneEquation value) { m_plane = value; }
		
		/// Get the plane equation used for screen to world transformation.
		inline const icl::PlaneEquation getPlaneEquation() const { return m_plane; }
		
		/// Transformation of a point from world to screen coordinates.
		/// world_pos must be a vector like | x y z 1 |.
		icl::Point32f transformedWorldToScreen(const icl::Vec &world_pos) const;
		
		/// Transformation of a point from screen to world coordinates.
		icl::Vec transformedScreenToWorld(const icl::Point32f &screen_pos) const;

		/// Transformation of a point from screen to world coordinates.
		/** z value of the intersection plane's offset vector is set to z */
		icl::Vec transformedScreenToWorld(const icl::Point32f &screen_pos, float z) const;

		/// Inplace transformation of a polygon shape from world to screen coordinates.
		/// As z coordinate the passed value is taken.
		void transformWorldToScreen2D(PolygonShape &shape, float z) const;

		/// Inplace transformation of a point vector from screen to world coordinates.
		void transformScreenToWorld2D(std::vector<icl::Point32f> &points) const;

		/// Inplace transformation of a point vector from screen to world coordinates.
		/** z value of the intersection plane's offset vector is set to z */
		void transformScreenToWorld2D(std::vector<icl::Point32f> &points, float z) const;
		
	private:
		icl::Camera m_camera;
		icl::PlaneEquation m_plane;
};

/*
/// Inplace transformation of a point vector from world to screen coordinates.
void transformWorldToScreen2D(vector<Point32f> &points);

/// Inplace transformation of a polygon shape from screen to world coordinates.
void transformScreenToWorld2D(Point32f &point);
/// Inplace transformation of a point vector from screen to world coordinates.
void transformScreenToWorld2D(vector<Point32f> &points);
*/
		
#endif /* __CAMERA_TRANSFORMER_EWEITNAU_H__ */

