// Copyright 2009 Erik Weitnauer
#ifndef __POLYGON_MAPPING_EWEITNAU_H__
#define __POLYGON_MAPPING_EWEITNAU_H__

#include "lin_alg.h"
#include "transformation.h"
#include "polygon_shape.h"

/// Representation of a polygon-shaped object with its position, shape and a unique id.
/**
 Each polygon has a unique id. Rotation and position of the center of gravity
 of the poygon are in the transformation member. An additional scaling factor
 can be set. The shape member describes the shape of the polygon. Also an error
 value can be set - e.g. useful when mapping a polygon model to observed data.
 Fpr polygon tracking, the object can hold information about when was the last
 time it was detected on e.g. a camera image. See isActive() and setActive()
 methods below.
 
 For transforming the polygon into a physical model, the mass of the polygon can
 be set, too.
*/
class PolygonObject {
	public:
		/// Default constructor.
		PolygonObject(): m_id(ID++), m_scaling(1), m_error(0),
		 m_mass(1), m_last_active(0) { }
		/// Constructor with transformation, mapping error and the source polygon as parameters.
		PolygonObject(Transformation t, float scaling, const PolygonShape &shape, float error):
			m_id(ID++), m_transform(t), m_scaling(scaling), m_shape(shape),
			m_error(error), m_mass(1), m_last_active(0)
			{updateTransformedShape();}
			
	  /// < operator, uses error for comparison.
		bool operator< (const PolygonObject &a) const { return m_error < a.m_error; }

		/// Applies the transformation to the corners of the shape and resets it to identity.
		/** If scale=true is passed, also the scaling is performed on the corners of
		 * the shape and is reset to 1 afterwards.
		 * It might be better to use the getTransformedShape() method instead. */
		void performTransformation(bool doScaling=true);
		
		/// Adds the effects of the passed transformation its own transformation.
		void addTransformation(const Transformation &t);

		// setters
		inline void setShape(const PolygonShape &shape) { m_shape = shape; updateTransformedShape(); }
		inline void setError(float value) { m_error = value; }
		inline void setTranformation(const Transformation &t) { m_transform = t; updateTransformedShape(); }
		inline void setPredictedTransformation(const Transformation &t) { m_predicted_transform = t; updatePredictedShape(); }
		inline void setMass(float value) { m_mass = value; }
		/// Sets the activation state to active.
		inline void setActive(int cur_time) { m_last_active = cur_time; }
		
		// getters
		int getId() const { return m_id; }
		const Transformation &getTransformation() const { return m_transform; }
		const Transformation &getPredictedTransformation() const { return m_predicted_transform; }
		const PolygonShape &getShape() const { return m_shape; }
		/// Returns the transformed PolygonShape.
		const PolygonShape &getTransformedShape() const { return m_transformed_shape; }
		// returns the shape transformed to the predicted postion.
		/** Will be NULL, when not set before by setPredictedTransformation. */
		const PolygonShape &getPredictedShape() const { return m_predicted_shape; }
		float getError() const { return m_error; }		
		float getMass() const { return m_mass; }
		/// Returns the last time, the object was set to active
		int getActiveTime() const { return m_last_active; }
		/// Returns true if the passed time is smaller or equal the time the object was set to active last.
		bool isActive(int cur_time) const { return m_last_active >= cur_time; }
		
		/// Returns the sum of distances each corner would move when applying the transformation to them.
		float getTransformationCornerMovement() const;
		
		/// write to stream
		std::ostream& operator>>(std::ostream &out) const;
		
		protected:
			/// copies the shape and applies transformation + scaling on it
			void updateTransformedShape();
			
			void updatePredictedShape();
		
		private:
			static int ID;
			int m_id;
			Transformation m_transform;
			Transformation m_predicted_transform;
			float m_scaling;
			PolygonShape m_shape;
			PolygonShape m_transformed_shape;
			PolygonShape m_predicted_shape;
			float m_error;
			float m_mass;
			int m_last_active;
};

std::ostream& operator<<(std::ostream &out, const PolygonObject &x);

#endif /* __POLYGON_MAPPING_EWEITNAU_H__ */

