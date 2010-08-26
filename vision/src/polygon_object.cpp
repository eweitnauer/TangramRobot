// Copyright 2009 Erik Weitnauer
#include "polygon_object.h"

using namespace std;
using namespace icl;

int PolygonObject::ID = 0;

void PolygonObject::performTransformation(bool doScaling) {
	m_shape.transformAroundCenter(m_transform.getMatrix());
	m_transform.setIdentity();
	if (doScaling) {
		m_shape.scale(m_scaling);
		m_scaling = 1;
	}
	updateTransformedShape();
}

float PolygonObject::getTransformationCornerMovement() const {
	return m_shape.getMeanCornerDistance(m_transformed_shape);
}

void PolygonObject::addTransformation(const Transformation &t) {
	m_transform += t;
	updateTransformedShape();
}
		
void PolygonObject::updateTransformedShape() {
	m_transformed_shape = m_shape;
	m_transformed_shape.transformAroundCenter(m_transform.getMatrix());
	m_transformed_shape.scale(m_scaling);	
}

void PolygonObject::updatePredictedShape() {
	m_predicted_shape = m_shape;
	m_predicted_shape.transformAroundCenter(m_predicted_transform.getMatrix());
	m_predicted_shape.scale(m_scaling);	
}
		
std::ostream& PolygonObject::operator>>(std::ostream &out) const {
	out << "Id=" << m_id << " Transformation=[" << m_transform << ", scaling=";
	out << m_scaling << "] Error=" << m_error << " Mass=" << m_mass;
	out << " lastActiveTime=" << m_last_active;
	out << " Shape=[" << m_shape << "]";
	out << " TransformedShape=[" << m_transformed_shape << "]";
	return out;
}

std::ostream& operator<<(std::ostream &out, const PolygonObject &x) {
	return x >> out;
}
