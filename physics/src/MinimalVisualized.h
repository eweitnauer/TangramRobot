// Copyright 2009 Erik Weitnauer
#ifndef MINIMAL_VisualizED_H
#define MINIMAL_VisualizED_H

#include "DemoApplication.h"
#include <btBulletDynamicsCommon.h>
#include "LinearMath/btAlignedObjectArray.h"

/**
 * This class is a minimal example of how to use Bullet PE to simulate
 * a physical scene and to visualize it by using the DemoApplication class.
 * 
 * Usage example:
 * \code
 * #include "MinimalVisualized.h"
 * #include "GlutStuff.h"
 * 
 * int main(int argc,char** argv) {
 *   MinimalVisualizer mvis;
 *   mvis.init();
 *   return glutmain(argc, argv,640,480,"Minimal Visualization Example",&mvis);
 * }
 * \endcode
 */
class MinimalVisualizer : public DemoApplication {
  public:
    /// standard constructor
    MinimalVisualizer() {}
    /// destructor calling exitPhysics for cleaning up
    virtual ~MinimalVisualizer() { freeScene(); freePhysics(); }

		/// Calls initPhysics and initScene.
		virtual void init() { initPhysics(); initScene(); }

    /// sets up the physics engine by creating the standard components
    /** Standard components are Broadphase, CollisionConfiguration,
     * CollisionDispatcher and SequentialImpulseConstraintSolver objects.
     */
    void initPhysics();
    
    /// Puts dynamic and static objects into the scene.
    /** Override for custom initialization. By default, it will create a ball
     * with radius 1 m and mass of 1 kg and the ground. */
    virtual void initScene();

    /// Removes all objects from the simulation and frees their allocated memory.
    virtual void freeScene();
    
    /// Deletes the standard components from memory.
    void freePhysics();

    /// this method gets called all the time in the glut main loop
    /** use for stepping the physics engine and displaying the result */
    virtual void clientMoveAndDisplay();
  
    /// gets called when a redraw is needed.
    virtual void displayCallback();
	
    static DemoApplication* Create() {
      MinimalVisualizer* demo = new MinimalVisualizer;
      demo->myinit();
      demo->initPhysics();
      demo->initScene();
      return demo;
    }
    
    virtual void swapBuffers() {}
    virtual void updateModifierKeys() {}

  protected:
    btCollisionObject* createGround();
    btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
    btBroadphaseInterface* m_broadphase;
    btCollisionDispatcher* m_dispatcher;
    btConstraintSolver* m_solver;
    btDefaultCollisionConfiguration* m_collisionConfiguration;

};
#endif
