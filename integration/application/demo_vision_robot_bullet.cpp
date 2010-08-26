// Copyright 2009 Erik Weitnauer
/// Detecting and tracking tangrams as well as moving the robot arm with the mouse.

#include "tangram_robot_gui.h"
#include "ICLQuick/Common.h"
#include "ICLUtils/XMLDocument.h"

#include "GlutStuff.h"
#include "PushingSimulatorGui.h"
#include <GL/freeglut_ext.h>

TangramRobotGui gui;
PushingSimulatorGui psimgui;

void vision_loop() {
	gui.vision_loop();
  Thread::msleep(100);
}

void show_physics_gui() {
	glutmain(0, NULL, 640, 480, "Minimal Visualization Example", &psimgui);
//	glutinit(0, NULL, 640, 480, "Minimal Visualization Example", &psim);
//  glutSetOption (GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
//  glutMainLoop();
	cout << "closed physic visualization, terminating" << endl;
	QApplication::quit();	
}

void init() {
	gui.init();
	psimgui.init();
	gui.setPushingSimulator((PushingSimulator*)&psimgui);
	// load tangram shapes
	if (pa("-tangram-cfg")) {
		gui.loadShapesFromXMLFile(pa("-tangram-cfg"));
	} else {
		gui.loadStandardTangramShapes();
	}
	// load camera configuration
	if (pa("-cam-cfg")) {
		Camera cam(*pa("-cam-cfg"));
		// the camera sees the top of the tangrams, so the intersection plane for the
		// viewrays must be at z=height_of_tangrams
		PlaneEquation z_plane(Vec(0,0,gui.getTangramHeight()),Vec(0,0,1));
	  gui.setCameraTransformer(CameraTransformer(cam, z_plane));
	}
	gui.connectToArm(pa("-mem-srv"),
									 pa("-robot-id"));
}

int main(int n, char **args) {
	paex("-input","define input grabber e.g. -input dc 0 or -input file images/*.ppm");
	paex("-size","image size of the camera, e.g. VGA or UXGA");
	paex("-mem-srv","name of the memory server, default is xcf:wb");
	paex("-cam-cfg","camera configuration file for screen to world transformation");
	paex("-tangram-cfg","xml file describing the polygon classes for classifying");
	paex("-robot-id","id of the robot to use, default: LeftArm");
  ICLApplication app(n, args, "-input|-i(2) -size|-s(1) -mem-srv(name=xcf:wb) -cam-cfg(1) -tangram-cfg(1) -robot-id(id=LeftArm)", init, vision_loop);
  
  ExecThread y(show_physics_gui);
	y.run(false); // no loop
	
  return app.exec();
}

