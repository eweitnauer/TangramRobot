// Copyright 2009 Erik Weitnauer
#include "Shapes.h"
#include <vector>
#include <LinearMath/btGeometryUtil.h>
#include <LinearMath/btAlignedObjectArray.h>
#include <iostream>
#include <stdexcept>

using namespace std;

float Shapes::sf_sq = 0.5411;
float Shapes::sf_pa = 0.5524;
float Shapes::sf_st = 0.461;
float Shapes::sf_mt = 0.461;
float Shapes::sf_lt = 0.461;

btVector3 Shapes::getInertiaTensor(ShapeType shape, float base_length, float height, float mass) {
	float a = base_length;
	float h = height;
	switch(shape) {
		case UNKNOWN:;
		case SQUARE: return (mass/12.)*btVector3(a*a+h*h, 2.*a*a, a*a+h*h);
		case PARALLELOGRAM: return (mass/12.)*btVector3(0.5*a*a+h*h, 3.*a*a, 2.5*a*a+h*h);
		case SMALL_TRIANGLE: return (mass/8.)*btVector3((5./9.)*a*a+0.5*h*h, (65./36.)*a*a, (5./4.)*a*a+h*h);
		case MEDIUM_TRIANGLE: return (mass/8.)*btVector3((10./9.)*a*a+0.5*h*h, (65./18.)*a*a, (5./2.)*a*a+h*h);
		case LARGE_TRIANGLE: return (mass/8.)*btVector3((20./9.)*a*a+0.5*h*h, (65./9.)*a*a, 5.*a*a+h*h);
		default: throw std::runtime_error("Undefined shape type.");
	}
}

void Shapes::setCorrectShapeFactors(float sq, float pa, float st, float mt, float lt) {
	sf_sq = sq; sf_pa = pa; sf_st = st; sf_mt = mt; sf_lt = lt;
}

float Shapes::getCorrectShapeFactor(ShapeType shape) {
	switch(shape) {
		case UNKNOWN: return 1.;
		case SQUARE: return sf_sq;
		case PARALLELOGRAM: return sf_pa;
		case SMALL_TRIANGLE: return sf_st;
		case MEDIUM_TRIANGLE: return sf_mt;
		case LARGE_TRIANGLE: return sf_lt;
		default: throw std::runtime_error("Undefined shape type.");
	}
	return 1;
}
   
btConvexHullShape* Shapes::createShape(ShapeType shape, float base_length, float height, float ground_scale, float collision_margin) {
	if (ground_scale<=0) {
		// use default ground scaling
		switch(shape) {
			case SQUARE: return createSquareShape(base_length, height);
			case PARALLELOGRAM: return createParallelogramShape(base_length, height);
			case SMALL_TRIANGLE: return createTriangleShape(base_length, height);
			case MEDIUM_TRIANGLE: return createTriangleShape(sqrt(2)*base_length, height);
			case LARGE_TRIANGLE: return createTriangleShape(2*base_length, height);
			default: throw std::runtime_error("Unknown shape type.");
		}
	} else {
		// used passed ground scaling
		switch(shape) {
			case SQUARE: return createSquareShape(base_length, height, ground_scale);
			case PARALLELOGRAM: return createParallelogramShape(base_length, height, ground_scale);
			case SMALL_TRIANGLE: return createTriangleShape(base_length, height, ground_scale);
			case MEDIUM_TRIANGLE: return createTriangleShape(sqrt(2)*base_length, height, ground_scale);
			case LARGE_TRIANGLE: return createTriangleShape(2*base_length, height, ground_scale);
			default: throw std::runtime_error("Unknown shape type.");
		}
	}
	return NULL;
}

std::vector<float> &Shapes::scale(std::vector<float> &vec, float a) {
	for (unsigned int i=0; i<vec.size(); ++i) vec[i] *= a;
	return vec;
}

std::vector<float> Shapes::getCorners(ShapeType shape, float base_length) {
	static float sqrt2 = sqrt(2.);
	static float sq[] = {-0.5,-0.5,   -0.5,0.5,   0.5,0.5,   0.5,-0.5};
//	static float pa[] = {-0.75*sqrt2,+0.25*sqrt2,   +0.25*sqrt2,+0.25*sqrt2,
//								   		 +0.75*sqrt2,-0.25*sqrt2,   -0.25*sqrt2,-0.25*sqrt2};
	static float pa_[] = {-0.75*sqrt2,-0.25*sqrt2,   +0.25*sqrt2,-0.25*sqrt2,
								   		 +0.75*sqrt2,+0.25*sqrt2,   -0.25*sqrt2,+0.25*sqrt2};
	static float tr[] = {-sqrt2/2.,-sqrt2/6., 0,sqrt2/3., sqrt2/2.,-sqrt2/6.};
	static vector<float> v;
	switch(shape) {
		case SQUARE: v = vector<float>(sq,sq+8); return scale(v, base_length);
		case PARALLELOGRAM: v = vector<float>(pa_,pa_+8); return scale(v, base_length);
		case SMALL_TRIANGLE: v = vector<float>(tr,tr+6); return scale(v, base_length);
		case MEDIUM_TRIANGLE: v = vector<float>(tr,tr+6); return scale(v, base_length*sqrt2);
		case LARGE_TRIANGLE: v = vector<float>(tr,tr+6); return scale(v, base_length*2);
		default: throw std::runtime_error("Unknown shape type.");
	}
}

btConvexHullShape* Shapes::createFlatObject(const std::vector<float> &corners2d,
		float base_length, float height, float ground_scale) {
	btConvexHullShape* shape = new btConvexHullShape();
  for (unsigned int i=0; i<corners2d.size()/2; i++) {
    btVector3 vtx(corners2d[i*2]*base_length, 8*height/16, corners2d[i*2+1]*base_length);
    shape->addPoint(vtx);
  }
  for (int i=corners2d.size()/2-1; i>=0; i--) {
    btVector3 vtx(corners2d[i*2]*base_length, -6*height/16, corners2d[i*2+1]*base_length);
    shape->addPoint(vtx);
  }
  for (int i=corners2d.size()/2-1; i>=0; i--) {
    btVector3 vtx(corners2d[i*2]*base_length*ground_scale, -8*height/16, corners2d[i*2+1]*base_length*ground_scale);
    shape->addPoint(vtx);
  }
  return shape;
}

btConvexHullShape* Shapes::createTriangleShape(float base_length, float height, float ground_scale, float collision_margin) {
	if (collision_margin == 0) 
		return createFlatObject(getCorners(SMALL_TRIANGLE,1), base_length, height, ground_scale);
  else return createFlatObjectInclCollMargin(getCorners(SMALL_TRIANGLE,1), base_length, height, ground_scale, collision_margin);
}

btConvexHullShape* Shapes::createSquareShape(float base_length, float height, float ground_scale, float collision_margin) {
  if (collision_margin == 0) 
  	return createFlatObject(getCorners(SQUARE,1), base_length, height, ground_scale);
  else return createFlatObjectInclCollMargin(getCorners(SQUARE,1), base_length, height, ground_scale, collision_margin);
}

btConvexHullShape* Shapes::createParallelogramShape(float base_length, float height, float ground_scale, float collision_margin) {
  if (collision_margin == 0) 
  	return createFlatObject(getCorners(PARALLELOGRAM,1), base_length, height, ground_scale);
	else return createFlatObjectInclCollMargin(getCorners(PARALLELOGRAM,1), base_length, height, ground_scale, collision_margin);
}

//btBoxShape* Shapes::createSquareShapeNative(float base_length, float height) {
//  btBoxShape* shape = new btBoxShape(btVector3(base_length/2, height/2, base_length/2));
//  return shape;
//}

btConvexHullShape* Shapes::createFlatObjectInclCollMargin(const std::vector<float> &corners2d, float base_length, float height, float ground_scale, float collision_margin) {
  btAlignedObjectArray<btVector3> vertices;
	for (unsigned int i=0; i<corners2d.size()/2; i++) {
    btVector3 vtx(corners2d[i*2]*base_length, 8*height/16, corners2d[i*2+1]*base_length);
    vertices.push_back(vtx);
  }
  for (int i=corners2d.size()/2-1; i>=0; i--) {
    btVector3 vtx(corners2d[i*2]*base_length, -6*height/16, corners2d[i*2+1]*base_length);
    vertices.push_back(vtx);
  }
  for (int i=corners2d.size()/2-1; i>=0; i--) {
    btVector3 vtx(corners2d[i*2]*base_length*ground_scale, -8*height/16, corners2d[i*2+1]*base_length*ground_scale);
    vertices.push_back(vtx);
  }

	btAlignedObjectArray<btVector3> planeEquations;
	btGeometryUtil::getPlaneEquationsFromVertices(vertices,planeEquations);
	btAlignedObjectArray<btVector3> shiftedPlaneEquations;
	
	for (int p=0;p<planeEquations.size();p++) {
		btVector3 plane = planeEquations[p];
		plane[3] += collision_margin;
		shiftedPlaneEquations.push_back(plane);
	}
	btAlignedObjectArray<btVector3> shiftedVertices;
	btGeometryUtil::getVerticesFromPlaneEquations(shiftedPlaneEquations,shiftedVertices);
	
  return new btConvexHullShape(&(shiftedVertices[0].getX()),shiftedVertices.size());
}

