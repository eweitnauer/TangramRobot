// Copyright 2009 Erik Weitnauer
#include <math.h>
#include <ICLUtils/Point32f.h>

#include "polygon_shape.h"
#include "lin_alg.h"

using namespace icl;
using namespace std;

float PolygonShape::getMeanCornerDistance(const PolygonShape &other) const {
	if (m_corners.empty() || other.m_corners.empty()) return 0;
	int n = min(m_corners.size(), other.m_corners.size());
	float distance=0;
	for (int i=0; i<n; i++) {
		distance += sqrt(pow(m_corners[i].x-other.m_corners[i].x,2) +
										 pow(m_corners[i].y-other.m_corners[i].y,2));	
	}
	return distance / n;
}

void PolygonShape::clearCorners() {
	m_corners.clear();
	resetGeometry();
}

void PolygonShape::resetGeometry() {
	m_center.x = 0; m_center.y = 0;
	m_bbox = icl::Rect32f(0,0,0,0);
	m_area = 0;
}

// Adds the corner to the corner vector and updates the diameter.
void PolygonShape::addCorner(Point32f c, bool updateGeometry) {
	m_corners.push_back(c);
	if (updateGeometry) this->updateGeometry();
}

void PolygonShape::addCorners(const vector<Point32f> &cs, bool updateGeometry) {
	for (unsigned int i=0; i<cs.size(); i++) addCorner(cs[i], false);
	if (updateGeometry) this->updateGeometry();
}

void PolygonShape::addCorners(const vector<float> &cs, bool updateGeometry) {
	for (unsigned int i=0; i<cs.size(); i+=2) addCorner(Point32f(cs[i],cs[i+1]), false);
	if (updateGeometry) this->updateGeometry();
}

/** Uses algorithm described at http://local.wasp.uwa.edu.au/~pbourke/geometry/polyarea/ */
void PolygonShape::updateGeometry() {
	// calculate area and center
  m_area = 0;
  m_center.x = 0; m_center.y = 0;
  float x,xn,y,yn;
  for (unsigned int i=0; i<m_corners.size(); i++) {
  	x = m_corners[i].x; y = m_corners[i].y;
  	xn = (i==m_corners.size()-1) ? m_corners[0].x : m_corners[i+1].x;
  	yn = (i==m_corners.size()-1) ? m_corners[0].y : m_corners[i+1].y;  	
  	m_area += x*yn - xn*y;
  	m_center.x += (x + xn) * (x*yn - xn*y);
		m_center.y += (y + yn) * (x*yn - xn*y);		  	              
  }
  m_center.x /= 3*m_area;
  m_center.y /= 3*m_area;
  m_area = fabs(m_area)/2;
  // calculate the bounding box
  updateBoundingBox();
}

void PolygonShape::updateBoundingBox() {
	if (m_corners.empty()) m_bbox = Rect32f(0,0,0,0);
  else {
		m_bbox = Rect32f(m_corners[0].x,m_corners[0].y,0,0);
		for (unsigned int i=1; i<m_corners.size(); i++)
			m_bbox |= Rect32f(m_corners[i].x,m_corners[i].y,0,0);
  }
}

void PolygonShape::transformAroundCenter(const Mat3 &T) {
	LinAlg::transformAround(m_center, m_corners, T);
	updateGeometry();
}

void PolygonShape::scale(float s, bool scale_height) {
	if (scale_height) m_height *= s;
	transformAroundCenter(LinAlg::createTransformationMatrix2d(0,0,0,s));
	updateGeometry();
}

void PolygonShape::translate(float tx, float ty) {
	m_center.x += tx; m_center.y += ty;
	m_bbox.x += tx; m_bbox.y += ty;
	for (unsigned int i=0; i<m_corners.size(); i++) {
  	m_corners[i].x += tx; m_corners[i].y += ty;
  }
}
/*
std::ostream& PolygonShape::toXML(std::ostream &out) const {
	static string base_xml =
      "<POLYGONSHAPE name=''>"
      "<CORNERS count=''/>"
      "</POLYGONSHAPE>";
  Location lROOT(base_xml);
  
  Location lSHAPE(lROOT,"/POLYGONSHAPE");
  lSHAPE["name"] = m_name;
 
  Location lCORNERS(lROOT,"/POLYGONSHAPE/CORNERS");
  lCORNERS["count"] = m_corners.size();
  for (unsigned int i=0; i<m_corners.size(); i++) {
  
  }
  
  std::cout << "sending this doc:\n" << lDOC.getDocumentText(true) << "\n";
  m_data->mem->insert(lDOC.getDocumentText());
	return out;
}

		/// parse xml
		static PolygonShape createFromXML(const std::string &xml);
*/
std::ostream& PolygonShape::operator>>(std::ostream &out) const {
	out << "Name=" << m_name << " Centroid=" << m_center;
	out << " Area=" << m_area << " Height=" << m_height;
	out << " Bounding Box=" << m_bbox << " Corners=";
	for (unsigned int i=0; i<m_corners.size(); i++) {
		out << m_corners[i];
		if (i != m_corners.size()-1) out << ", ";
	}
	return out;
}

std::ostream& operator<<(std::ostream &out, const PolygonShape &x) {
	return x >> out;
}

