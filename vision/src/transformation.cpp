// Copyright 2009 Erik Weitnauer
#include "transformation.h"

/// write to stream, numbers only
std::ostream& Transformation::write_plain(std::ostream &out) const {
	return out << m_rotation << " " << m_tx << " " << m_ty;
}

Transformation Transformation::operator+(const Transformation &other) const {
	return Transformation(m_rotation+other.m_rotation, m_tx+other.m_tx, m_ty+other.m_ty);
}

Transformation& Transformation::operator+=(const Transformation &other) {
	m_rotation += other.m_rotation;
	m_tx += other.m_tx;
	m_ty += other.m_ty;
	updateMatrix();
	return *this;
}

Transformation Transformation::operator-(const Transformation &other) const {
	return Transformation(m_rotation-other.m_rotation, m_tx-other.m_tx, m_ty-other.m_ty);
}

Transformation& Transformation::operator-=(const Transformation &other) {
	m_rotation -= other.m_rotation;
	m_tx -= other.m_tx;
	m_ty -= other.m_ty;
	updateMatrix();
	return *this;
}

PolygonShape Transformation::operator*(const PolygonShape &shape) const {
	PolygonShape result = shape;
	result.transformAroundCenter(this->m_T);
	return result;
}

std::ostream& operator<<(std::ostream &out, const Transformation &t) {
	out << "Rotation=" << t.m_rotation << " Translation=(" << t.m_tx;
	out << ", " << t.m_ty << ")";
	return out;
}
