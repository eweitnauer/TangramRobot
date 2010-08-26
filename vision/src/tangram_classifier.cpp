// Copyright 2009 Erik Weitnauer
#include "tangram_classifier.h"
#include "polygon_shape.h"
#include <xml_reader.h>

using namespace std;
using namespace icl;


void TangramClassifier::setSize(float base_length, float height) {
	// do we need to scale the shapes?
	if (m_base_length != base_length) {
		for (unsigned int i=0; i<m_shapes.size(); ++i)
			m_shapes[i].scale(base_length / m_base_length, false);
		m_base_length = base_length;
	}
	if (height<0) return;
	// maybe we also need to update the heights...
	if (m_height != height) {
		for (unsigned int i=0; i<m_shapes.size(); ++i)
			m_shapes[i].setHeight(height);
		m_height = height;		
	}
}

void TangramClassifier::loadStandardTangramShapes(float base_length, float height) {
	int i;
	float sqrt2 = sqrt(2);
	m_base_length = base_length;
	m_height = height;
	if (m_height < 0) m_height = m_base_length/7;
		
	// init triangle models
	PolygonShape t_shape("Triangle", m_height);
	float t_p[] = {-sqrt2/2.,-sqrt2/6.,   0,sqrt2/3.,   sqrt2/2.,-sqrt2/6.};
	for (i = 0; i < 3; i++)	t_shape.addCorner(Point32f(t_p[2*i], t_p[2*i+1]));
	t_shape.updateGeometry();
	
	// small
	PolygonShape ts_shape = t_shape;
	ts_shape.scale(m_base_length, false);
	ts_shape.setName("Triangle Small");
	ts_shape.updateGeometry();
	// medium
	PolygonShape tm_shape = t_shape;
	tm_shape.scale(sqrt2*m_base_length, false);
	tm_shape.setName("Triangle Medium");
	tm_shape.updateGeometry();
	// large
	PolygonShape tl_shape = t_shape;
	tl_shape.scale(2*m_base_length, false);
	tl_shape.setName("Triangle Large");
	tl_shape.updateGeometry();

	// init square model
	PolygonShape sq_shape("Square", m_height);
	float sq_p[] = {-0.5,-0.5,   0.5,-0.5,   0.5,0.5, -0.5,0.5};
	for (i = 0; i < 4; i++)	sq_shape.addCorner(Point32f(sq_p[2*i], sq_p[2*i+1]));
	sq_shape.scale(m_base_length, false);
	sq_shape.updateGeometry();
	
	// init parallelogram model
	PolygonShape pa_shape("Parallelogram", m_height);
	float pa_p[] = {-0.75*sqrt2,-0.25*sqrt2,   -0.25*sqrt2, 0.25*sqrt2,
								   0.75*sqrt2, 0.25*sqrt2,    0.25*sqrt2,-0.25*sqrt2};
	for (i = 0; i < 4; i++)	pa_shape.addCorner(Point32f(pa_p[2*i], pa_p[2*i+1]));
	pa_shape.scale(m_base_length, false);
	pa_shape.updateGeometry();
	
	m_shapes.push_back(ts_shape);
	m_shapes.push_back(tm_shape);
	m_shapes.push_back(tl_shape);
	m_shapes.push_back(pa_shape);
	m_shapes.push_back(sq_shape);
}

void TangramClassifier::loadShapesFromXML(const string &filename) {
	XMLReader::readShapes(filename, m_shapes, m_base_length, m_height);
}

/// For now, just return all models if corner number in {3..5}.
const vector<PolygonShape> &TangramClassifier::classify(PolygonShape &data, float tolerance, bool size_matters) {
	int data_corners = data.getCorners().size(); 
	m_candidate_shapes.clear();
	for (unsigned int i=0; i<m_shapes.size(); ++i) {
		int model_corners = m_shapes[i].getCorners().size();
		if (model_corners > data_corners || model_corners < data_corners-2) continue;
		if (!size_matters) m_candidate_shapes.push_back(m_shapes[i]);
		else { // size matters
			if (fabs(m_shapes[i].getArea() - data.getArea()) < m_shapes[i].getArea()*tolerance)
				m_candidate_shapes.push_back(m_shapes[i]);
		}
	}
	return m_candidate_shapes;
}

std::ostream& operator<<(std::ostream &out, const TangramClassifier &tc) {
	out << "BaseLength: " << tc.m_base_length <<
				 " Height: " << tc.m_height << std::endl;
	out << "Shapes: " << std::endl;
	for (unsigned int i=0; i<tc.m_shapes.size(); ++i)
		out << "[" << i << "] " << tc.m_shapes[i] << std::endl;
	return out;
}
