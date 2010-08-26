// Copyright 2009 Erik Weitnauer
#ifndef SHAPES_H
#define SHAPES_H

#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <vector>

/// Convinient tangram shape creation.
class Shapes {
  public:
    enum ShapeType {
    	UNKNOWN=0,
      SQUARE,
      PARALLELOGRAM,
      SMALL_TRIANGLE,
      MEDIUM_TRIANGLE,
      LARGE_TRIANGLE
    };
    
	  static std::vector<float> getCorners(ShapeType shape, float base_length);
	  
    static btConvexHullShape* createShape(ShapeType shape, float base_length, float height, float ground_scale=-1, float collision_margin=0);

    /// Corners2d must have corners_count*2 elements.
    static btConvexHullShape* createFlatObject(const std::vector<float> &corners2d,
  		float base_length, float height, float ground_scale);
	  static btConvexHullShape* createFlatObjectInclCollMargin(const std::vector<float> &corners2d,
	  	float base_length, float height, float ground_scale, float collision_margin);

	  static btConvexHullShape* createTriangleShape(float base_length, float height, float ground_scale=0.461, float collision_margin=0);
    static btConvexHullShape* createSquareShape(float base_length, float height, float ground_scale=0.5411, float collision_margin=0);
    static btConvexHullShape* createParallelogramShape(float base_length, float height, float ground_scale=0.5524, float collision_margin=0);

    static btVector3 getInertiaTensor(ShapeType shape, float base_length, float height, float mass);
    static float getCorrectShapeFactor(ShapeType shape);
    static void setCorrectShapeFactors(float sq, float pa, float st, float mt, float lt);
  private:
	  static std::vector<float> &scale(std::vector<float> &vec, float a);
    static float sf_sq, sf_pa, sf_st, sf_mt, sf_lt;
};

#endif
