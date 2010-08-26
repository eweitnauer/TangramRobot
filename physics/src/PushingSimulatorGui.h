// Copyright 2009 Erik Weitnauer
#ifndef __PUSHING_SIMULATOR_GUI_EWEITNAU_H__
#define __PUSHING_SIMULATOR_GUI_EWEITNAU_H__

#include <vector>
#include "MinimalVisualized.h"
#include "PushMovement.h"
#include "PushingSimulator.h"
#include "PhysicsParameters.h"

/// Simulates pushing actions and visualizates the simulation.
/** Pass a vector of pointers to RigidBodies and a pushing action. Then call
simulate() with the amount of time to simulate. The position changes are written
in place into the passed RigidBodies. */
class PushingSimulatorGui : public MinimalVisualizer, public PushingSimulator {
	public:
		PushingSimulatorGui(bool always_update_display=true) :
			m_always_update_display(always_update_display), m_fast_forward_factor(1)
			{ freeScene(); }
		virtual ~PushingSimulatorGui() { freeScene(); }
	
		virtual void init() { MinimalVisualizer::init(); }
		/// Simulates a pushing action performed on the rigid bodies passed.
		/** First, a ground at y=0 is added to the scene, then all bodies passed
		 * in 'bodies'. Next, 'before_time_in_s' time this scene is simulated. Afterwards
		 * a pushing object (small upright cylinder) is added to the scene at the
		 * position 'push.start' and is moved linearly to the position 'push.end',
		 * which moves with push.speed velocity. Afterwards, another the after_time_in_s
		 * seconds are simulated without moving the cylinder.
		 * The position changes are written directly into the passed rigid bodies.size
 		 * Returns how much world time was simulated (in seconds). */
	virtual float simulate(const PushMovement &push, std::vector<btRigidBody*> &bodies,
			float before_time_in_s=0.1, float after_time_in_s=0.5);

		/// Creates the m_pusher and m_ground objects, but does not add them to the scene yet.
		virtual void initScene();
		
		/// set the factor by which the visulaization should play faster than real time
		void setFastForward(float factor) { m_fast_forward_factor = factor; }
	
		/// this method gets called all the time in the glut main loop - in this case
		/// don't do any physcis simulation, just show what we have at the moment.
		virtual void clientMoveAndDisplay() { displayCallback(); }  
		virtual void displayCallback() { if (should_update_display || m_always_update_display) MinimalVisualizer::displayCallback(); should_update_display = false; }
		void update_display() { should_update_display = true; }

		/// Calls clear scene and deletes the m_pusher and m_ground objects.
		void freeScene();
		
  private:
  	bool should_update_display;
  	bool m_always_update_display;
  	float m_fast_forward_factor;
};


#endif /* __PUSHING_SIMULATOR_GUI_EWEITNAU_H__ */
