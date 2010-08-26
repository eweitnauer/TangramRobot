// Copyright 2009 Erik Weitnauer
#ifndef __TRANSFORMATION_EWEITNAU_H__
#define __TRANSFORMATION_EWEITNAU_H__

#include "lin_alg.h"
#include "polygon_shape.h"

/// A linear transformation for 2d Vectors with rotation and translation but no scaling or shearing.
class Transformation {
	public:
		/// Empty constructor. Initializes identity transformation.
		Transformation(): m_rotation(0), m_tx(0), m_ty(0)
			{ updateMatrix(); }
		/// Constructor setting up a transformation with the passed rotation, translation and scaling.
		Transformation(float rotation, float tx, float ty)
		  :m_rotation(rotation), m_tx(tx), m_ty(ty)
		  { updateMatrix(); }

		// getters
		float getRotation() const { return m_rotation; }
		float getTx() const { return m_tx; }
		float getTy() const { return m_ty; }
		const Mat3 &getMatrix() const { return m_T; }
		/// Returns the length of the translation vector.
		float length() const { return sqrt(m_tx*m_tx + m_ty*m_ty); }
		
		// setters
		inline void setRotation(float value) { m_rotation = value; updateMatrix(); }
		inline void setTranslation(float tx, float ty) { m_tx=tx; m_ty=ty; m_T[2] = tx; m_T[5] = ty; }
		inline void setTx(float value) { m_tx = value; }
		inline void setTy(float value) { m_ty = value; }
		
		/// sets the transformation to identity
		void setIdentity() {m_rotation=0; m_tx=0; m_ty=0; updateMatrix();}
		
		/// Combine the effects of two transformations.
		Transformation operator+(const Transformation &other) const;
		/// Combine the effects of two transformations.
		Transformation& operator+=(const Transformation &other);
		/// Combine the effects of two transformations.
		Transformation operator-(const Transformation &other) const;
		/// Combine the effects of two transformations.
		Transformation& operator-=(const Transformation &other);
		
		/// Copy and transform a PolygonShape.
		PolygonShape operator*(const PolygonShape &shape) const;
		
		/// write to stream, numbers only
		std::ostream& write_plain(std::ostream &out) const;
	
		friend std::ostream& operator<<(std::ostream &out, const Transformation &x);
	protected:
		void updateMatrix()
		  { LinAlg::setTransformationMatrix2d(m_T, m_rotation, m_tx, m_ty, 1.); }
	
	private:
		float m_rotation;
		float m_tx;
		float m_ty;
		Mat3 m_T;
};

std::ostream& operator<<(std::ostream &out, const Transformation &x);

#endif /* __TRANSFORMATION_EWEITNAU_H__ */

