#include "camera_transformer.h"

using namespace std;
using namespace icl;

icl::Point32f CameraTransformer::transformedWorldToScreen(const icl::Vec &world_pos) const {
	return m_camera.project(world_pos);
}

icl::Vec CameraTransformer::transformedScreenToWorld(const icl::Point32f &screen_pos) const {
	return m_camera.getViewRay(screen_pos).getIntersection(m_plane);
}

icl::Vec CameraTransformer::transformedScreenToWorld(const icl::Point32f &screen_pos, float z) const {
	icl::Vec result;
  icl::PlaneEquation plane(icl::Vec(m_plane.offset[0], m_plane.offset[1], z), m_plane.normal);
	result = m_camera.getViewRay(screen_pos).getIntersection(plane);
	return result;
}

void CameraTransformer::transformWorldToScreen2D(PolygonShape &shape, float z) const {
	// we need to transform all corner points and then update the geometry...
	vector<Point32f> corners = shape.getCorners();
	vector<Point32f> transformed_corners;
	for (unsigned int i=0; i<corners.size(); ++i) {
		Point32f p = transformedWorldToScreen(Vec(corners[i].x, corners[i].y, z, 1));
		transformed_corners.push_back(p);
	}
	shape.clearCorners();
	shape.addCorners(transformed_corners);
}

void CameraTransformer::transformScreenToWorld2D(vector<Point32f> &points) const {
	for (unsigned int i=0; i<points.size(); ++i) {
		Vec v = transformedScreenToWorld(points[i]);
		points[i].x = v[0];
		points[i].y = v[1];
	}
}

void CameraTransformer::transformScreenToWorld2D(vector<Point32f> &points, float height) const {
	for (unsigned int i=0; i<points.size(); ++i) {
		Vec v = transformedScreenToWorld(points[i], height);
		points[i].x = v[0];
		points[i].y = v[1];
	}
}
