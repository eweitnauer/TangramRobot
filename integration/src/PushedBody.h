#ifndef __PUSHED_BODY_EWEITNAU_H__
#define __PUSHED_BODY_EWEITNAU_H__

#include <transformation.h>
#include <vision_adapter.h>
#include <algorithm>
#include <iostream>
#include <vector>
#include <algorithm>
#include "LinearMath/btDefaultMotionState.h"

class PushedBody {
	public:
		PushedBody(btRigidBody *body, PolygonShape unscaled_pshape, float scaling):
				m_adapter(scaling), m_body(body), m_pshape(unscaled_pshape) {
			body->getMotionState()->getWorldTransform(m_start_position);
			m_start_t = m_adapter.to_vision(m_start_position);
		}
		
		btRigidBody* getBody() { return m_body; }
		
		void addEndPosition() {
			btTransform t;
			m_body->getMotionState()->getWorldTransform(t);
			m_end_positions.push_back(t);
		}
		
		void moveToStart() {
		  // all the following members must be reset for determinism!
		  btDefaultMotionState* myMotionState = (btDefaultMotionState*)m_body->getMotionState();
			myMotionState->m_startWorldTrans = m_start_position;
			m_body->setCenterOfMassTransform(m_start_position);
			m_body->setLinearVelocity(btVector3(0,0,0));
			m_body->setAngularVelocity(btVector3(0,0,0));

      // not sure about this one...
      m_body->setInterpolationWorldTransform(m_start_position);
		  
		  // these don't have to be reset for determinism...
      //m_body->updateInertiaTensor();
      //m_body->setHitFraction(1);
      //m_body->forceActivationState(ACTIVE_TAG);
      //m_body->activate();
      //m_body->setDeactivationTime(0);
      //myMotionState->m_graphicsWorldTrans = m_start_position;
		}
		
		int getNumberOfEndPositions() { return m_end_positions.size(); }
		
		void clearEndPositions() { m_end_positions.clear(); }
		
		/// Will calculate various statistic data. A reference transformation can be passed.
		/** Calculates the mean, max and min delta translations between the start
		 * and all end points added before. Also calculates the variance of the
		 * distance to the passed reference transformation as well as the maximum
		 * distance to the passed reference transformation. With "distance" it is
		 * refered to the mean corner distance between two transformed shapes. */
		void calcStatistics(const Transformation &reference_delta_t=Transformation(0,0,0)) {
			if (m_end_positions.empty()) return;
			int n = m_end_positions.size();
			PolygonShape ref_shape = reference_delta_t*m_pshape;
			// calculate min, max and mean translations
			for (int i=0; i<n; ++i) {
				Transformation end_t = m_adapter.to_vision(m_end_positions[i]);
				Transformation delta_t = end_t - m_start_t;
				if (i==0) {
					m_mean_t = delta_t;	m_max_t = delta_t; m_min_t = delta_t;
				} else {
					m_mean_t += delta_t;
					m_max_t.setTranslation(std::max(m_max_t.getTx(), delta_t.getTx()), std::max(m_max_t.getTy(), delta_t.getTy()));
					m_max_t.setRotation(std::max(m_max_t.getRotation(), delta_t.getRotation()));
					m_min_t.setTranslation(std::min(m_min_t.getTx(), delta_t.getTx()), std::min(m_min_t.getTy(), delta_t.getTy()));
					m_min_t.setRotation(std::min(m_min_t.getRotation(), delta_t.getRotation()));
				}
			}
			m_mean_t.setTranslation(m_mean_t.getTx()/n, m_mean_t.getTy()/n);
			m_mean_t.setRotation(m_mean_t.getRotation()/n);
			
			// calculate distance to reference
			PolygonShape mean_shape = m_mean_t*m_pshape;
			
			// calculate variance of distance to mean
			m_variance = 0;
			m_ref_distance = 0;
			m_start_end_dist = 0;
			for (int i=0; i<n; ++i) {
				Transformation end_t = m_adapter.to_vision(m_end_positions[i]);
				Transformation delta_t = end_t - m_start_t;
				PolygonShape curr_shape = delta_t*m_pshape;
				float mean_dist = mean_shape.getMeanCornerDistance(curr_shape);
				float ref_dist = curr_shape.getMeanCornerDistance(ref_shape);
				float start_end_dist = curr_shape.getMeanCornerDistance(m_pshape);
				m_variance += mean_dist*mean_dist;
				m_ref_distance += ref_dist;
				m_start_end_dist += start_end_dist;
			}
			m_variance /= n;
			m_ref_distance /= n;
			m_start_end_dist /= n;
		}

		const Transformation &getMeanDelta() const { return m_mean_t; }
		const Transformation &getMaxDelta() const { return m_max_t; }
		const Transformation &getMinDelta() const { return m_min_t; }
		const Transformation &getStartPosition() const { return m_start_t; }
		float getDistanceToReference() const { return m_ref_distance; }
		float getVariance() const { return m_variance; }
		float getStartToEndDistance() const { return m_start_end_dist; }
				
	private:
		VisionAdapter m_adapter;
		btRigidBody *m_body;
		PolygonShape m_pshape;
		btTransform m_start_position;
		std::vector<btTransform> m_end_positions;
		Transformation m_mean_t;
		Transformation m_max_t;
		Transformation m_min_t;
		Transformation m_start_t;
		float m_variance;
		float m_ref_distance;
		float m_start_end_dist;
};

#endif /* __PUSHED_BODY_EWEITNAU_H__ */

