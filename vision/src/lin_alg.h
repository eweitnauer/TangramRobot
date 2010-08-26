// Copyright 2009 Erik Weitnauer
#ifndef __LIN_ALG_EWEITNAU_H__
#define __LIN_ALG_EWEITNAU_H__

#include <vector>
#include <math.h>
#include <ICLUtils/Point32f.h>
#include <ICLUtils/Rect32f.h>
#include <ICLGeom/GeomDefs.h>

typedef icl::FixedMatrix< icl::icl32f, 3, 3 > Mat3;
typedef icl::FixedColVector< icl::icl32f, 3 > Vec3;

/// Collection of linear algebra helper functions.
/**
All methods are static. Most of them are about applying a 3x3 Matrix onto an
icl::Point32f, to do a 2d affine transformation.

Methods called transform...() will do the transformation in place (alter the
passed points), while methods called transformed...() will not change the
points but return the transformation result as new points.
*/
class LinAlg {
	public:
		/// Creates 3x3 transformation matrix for rotating (theta), scaling (s) and
		/// translating (tx,ty) a 2d Vector.
		static Mat3 createTransformationMatrix2d(float theta, float tx, float ty, float s=1) {
			Mat3 T;
			return setTransformationMatrix2d(T, theta, tx, ty, s);
		}
		
		/// Sets a 3x3 transformation matrix for rotating (theta), scaling (s) and
		/// translating (tx,ty) a 2d Vector.
		static Mat3 &setTransformationMatrix2d(Mat3 &T, float theta, float tx, float ty, float s=1) {
			T[0] = s*cos(theta);  T[1] = s*sin(theta); T[2] = tx;
			T[3] = s*-sin(theta); T[4] = s*cos(theta); T[5] = ty;
			T[6] = 0;             T[7] = 0;            T[8] = 1;   
			return T;
		}
		
		/// Returns the square of the smallest distance between a Point and
		/// any of the points in the vector. 
		static float closestDistanceSqr(const icl::Point32f &p, const std::vector<icl::Point32f> &v);
		
		/// Returns the length of the diagonal.
		static float diagonalLength(const icl::Rect32f &r);
		
		/// Returns all passed points transformed with the Matrix: p' = T*p.
		static std::vector<icl::Point32f> 
			transformed(const std::vector<icl::Point32f> &v, const Mat3 &T);

		/// In-palce transforms all passed points with the Matrix, taking *center*
		/// as the center for scaling and rotation.
		static void
			transformAround(const icl::Point32f center,
			                std::vector<icl::Point32f> &v, const Mat3 &T);
			                      
		/// Returns the passed points transformed with the Matrix, taking *center*
		/// as the center for scaling and rotation.
		static std::vector<icl::Point32f> 
			transformedAround(const icl::Point32f center,
			                  const std::vector<icl::Point32f> &v, const Mat3 &T);
			                      

		/// In-place transformation of point p: p=T*p.
		static void transform(icl::Point32f &p, const Mat3 &T);

		/// transformation of point p: p=T*p.
		static icl::Point32f transformed(const icl::Point32f &p, const Mat3 &T);

		/// In-place transformation of rectangle.
		static void transform(icl::Rect32f &r, const Mat3 &T);

		/// In-place transformation of rectangle. Rotation and scaling are relative
		/// to center.
		static void transformAround(const icl::Point32f &center, icl::Rect32f &r, const Mat3 &T);
		
		// Returns square of distance between 2 points.
		static inline float distance2(icl::Point32f a, icl::Point32f b) {
			return (a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y);
		}
};

#endif /* __LIN_ALG_EWEITNAU_H__ */

