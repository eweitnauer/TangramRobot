// Copyright 2009 Erik Weitnauer
/// Integrates vision and physics. At the moment, its just a demo.

#include "tangram_gui.h"
#include "ICLQuick/Common.h"
#include "MinimalVisualized.h"
#include "GlutStuff.h"
#include "tangram_classifier.h"
#include "polygon_mapper.h"
#include "PushingSimulatorGui.h"
#include "vision_adapter.h"
#include <GL/freeglut_ext.h>
#include <vector>
#include <iostream>

using namespace std;

#define SCALING 0.005 // *0.001*5 ... mm->meter and meter->pe units 

TangramGui vision_gui;
PushingSimulatorGui psim(false);
VisionAdapter adapter(SCALING);

vector<PolygonObject*> polygons;

std::ostream& operator<<(std::ostream &out, const btVector3 &x) {
	return out << x.getX() << ", " << x.getY() << ", " << x.getZ();
}

std::ostream& operator<<(std::ostream &out, const btQuaternion &x) {
	return out << x.getX() << ", " << x.getY() << ", " << x.getZ() << ", " << x.getW();
}

void simulate_physics(const vector<PolygonObject*> &polygons) {
	static float base_size = vision_gui.getTangramBaseLength() * SCALING;
	static vector<btRigidBody*> bodyList;
	static btVector3 pusher_size(base_size/5, base_size, base_size/5);
	static float pusher_speed = 2.;
	static float angle = 0;
	static float radius = 6*base_size;

	if (polygons.size() == 0) return;
	PushMovement push(btVector3(sin(angle)*radius,0,cos(angle)*radius), btVector3(0,0,0), pusher_size, pusher_speed);
	angle += 0.3;
  btTransform trans;
	
	for (unsigned int i=0; i<polygons.size(); i++) {
		PolygonObject po = *polygons[i];
		btRigidBody *b = adapter.to_bullet(po);
		bodyList.push_back(b);
	}

	bodyList[0]->getMotionState()->getWorldTransform(trans);
	cout << endl << "== Before ======" << endl;
	cout << "Polygon[0] vision transform: " << polygons[0]->getTransformation() << endl;
	cout << "Polygon[0] physics transform: (Origin: " << trans.getOrigin()
			 << " Rotation: " << trans.getRotation() << ")" << endl;
	cout << "simulating...";

	psim.simulate(push, bodyList);

	cout << "ready!" << endl;
	cout << "== After ======" << endl;
	bodyList[0]->getMotionState()->getWorldTransform(trans);
	cout << "Polygon[0] physics transform: (Origin: " << trans.getOrigin()
			 << " Rotation: " << trans.getRotation() << ")" << endl;
	cout << "Polygon[0] vision transform: " << adapter.to_vision(trans) << endl;
	
	// write the results to the polygon objects
	for (unsigned int i=0; i<polygons.size(); i++) {
		bodyList[i]->getMotionState()->getWorldTransform(trans);
		polygons[i]->setPredictedTransformation(adapter.to_vision(trans));
	}
	
	for (unsigned int i=0; i<bodyList.size(); i++) delete bodyList[i];
	bodyList.clear();
}

void vision_loop() {
	vision_gui.vision_loop();
	vector<PolygonObject*> actives = vision_gui.getActivePolygons();
	simulate_physics(actives);	
  Thread::msleep(50);
}

void show_physics_gui() {
	glutmain(0, NULL, 640, 480, "Minimal Visualization Example", &psim);
	//glutinit(0, NULL, 640, 480, "Minimal Visualization Example", &psim);
  //glutSetOption (GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
  //glutMainLoop();
	cout << "closed physic visulaization, terminating" << endl;
	QApplication::quit();	
}

void init() {
	GenericGrabber::resetBus();
	psim.init();
	psim.setCameraDistance(btScalar(4.));
	vision_gui.init();
	// load tangram shapes
	if (pa("-tangram-cfg")) {
		vision_gui.loadShapesFromXMLFile(pa("-tangram-cfg"));
	} else {
		vision_gui.loadStandardTangramShapes();
	}
	// load camera configuration
	if (pa("-cam-cfg")) {
		Camera cam(*pa("-cam-cfg"));
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

