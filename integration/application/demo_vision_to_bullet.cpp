// Copyright 2009 Erik Weitnauer
/// Demo: Adds all tangram pieces detected in vision directly into the physcis engine.

#include "ICLQuick/Common.h"
#include "vision_adapter.h"
#include "tangram_gui.h"
#include "MinimalVisualized.h"
#include "GlutStuff.h"
#include <vector>
#include <iostream>

using namespace std;

#define SCALING 0.03333 // *(1/600)*20 ... pixel->meter and meter->pe units 

/// Class for dynamic adding and removing of rigid bodies to physics scene.
class PhysicsVisualizer : public MinimalVisualizer {
	public:
		/// Setup camera and disable help text.
		virtual void initScene() {
			keyboardCallback('h',0,0);
			m_cameraDistance = 8.0;	m_ele = 50.f;	m_azi = 180.f;
		}
	
		/// this method gets called all the time in the glut main loop - in this case
		/// don't do any physcis simulation, just show what we have at the moment.
		virtual void clientMoveAndDisplay() { displayCallback(); }
		
		virtual void displayCallback() { Mutex::Locker l(m_mutex); MinimalVisualizer::displayCallback(); }
		
		/// First calls freeScene, then adds a ground and the passed objects to the scene.
		void setScene(const vector<btRigidBody*> bodies);
	private:
		Mutex m_mutex;
};

void PhysicsVisualizer::setScene(const vector<btRigidBody*> bodies) {
	Mutex::Locker l(m_mutex);
	freeScene();
	m_dynamicsWorld->addCollisionObject(createGround());
	for (unsigned int i=0; i<bodies.size(); i++) {
		m_dynamicsWorld->addRigidBody(bodies[i]);
		m_collisionShapes.push_back(bodies[i]->getCollisionShape());
	}
}

////////////////////////////////////////////////////////////////////////////////
TangramGui vision_gui;
PhysicsVisualizer physics_gui;
VisionAdapter adapter(SCALING);

void set_physics_scene(const vector<PolygonObject*> &polygons) {
	static vector<btRigidBody*> bodyList;
	for (unsigned int i=0; i<polygons.size(); i++) {
		PolygonObject po = *polygons[i];
		po.addTransformation(Transformation(0,-160,-120));
		btRigidBody *b = adapter.to_bullet(po);
		bodyList.push_back(b);
	}
	physics_gui.setScene(bodyList);
	bodyList.clear();
}

void vision_loop() {
	vision_gui.vision_loop();
	set_physics_scene(vision_gui.getActivePolygons());	
  Thread::msleep(50);
}

void show_physics_gui() {
	glutmain(0, NULL, 640, 480, "Demo: Vision To Physics", &physics_gui);
	cout << "closed physic visulaization, terminating" << endl;
	QApplication::quit();	
}

void init() {
	GenericGrabber::resetBus();
	physics_gui.init();
	vision_gui.init();
	// load tangram shapes
	if (pa("-tangram-cfg")) {
		vision_gui.loadShapesFromXMLFile(pa("-tangram-cfg"));
	} else {
		vision_gui.loadStandardTangramShapes();
	}
	// load camera configuration
	if (pa("-cam-cfg")) {
		Camera cam(pa("-cam-cfg").as<string>());
		// the camera sees the top of the tangrams, so the intersection plane for the
		// viewrays must be at z=height_of_tangrams
		PlaneEquation z_plane(Vec(0,0,vision_gui.getTangramHeight()),Vec(0,0,1));
	  vision_gui.setCameraTransformer(CameraTransformer(cam, z_plane));
	}
}

int main(int n, char **args) {
	paex("-input","define input grabber e.g. -input dc 0 or -input file images/*.ppm");
	paex("-size","image size of the camera, e.g. VGA or UXGA");
	paex("-cam-cfg","camera configuration file for screen to world transformation");
	paex("-tangram-cfg","xml file describing the polygon classes for classifying");
	ICLApplication app(n, args, "-input|-i(2) -size|-s(1) -cam-cfg(1) -tangram-cfg(1)", init, vision_loop);
	
	ExecThread y(show_physics_gui);
	y.run(false); // no loop
	
	return app.exec();
}

