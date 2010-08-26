// Copyright 2009 Erik Weitnauer
/// Detecting and tracking tangrams as well as moving the robot arm with the mouse.

#include "tangram_robot_gui.h"
#include "ICLQuick/Common.h"
#include "ICLUtils/XMLDocument.h"

#include "GlutStuff.h"
#include "PushingSimulatorGui.h"
#include <GL/freeglut_ext.h>
#include <PushingSimulatorFast.h>

TangramRobotGui gui;
PushingSimulatorFast psim;

void vision_loop() {
	gui.vision_loop();
  Thread::msleep(100);
}

void init() {
	gui.init();
	psim.init();
	gui.setPushingSimulator((PushingSimulator*)&psim);
	if (pa("-grid")) {
		gui.enableGrid(pa("-grid"));
	}
	// load tangram shapes
	if (pa("-tangram-cfg")) {
		gui.loadShapesFromXMLFile(pa("-tangram-cfg"));
	} else {
		gui.loadStandardTangramShapes();
	}
	// load camera configuration
	if (pa("-cam-cfg")) {
		Camera cam(pa("-cam-cfg").as<string>());
		// the camera sees the top of the tangrams, so the intersection plane for the
		// viewrays must be at z=height_of_tangrams
		PlaneEquation z_plane(Vec(0,0,gui.getTangramHeight()),Vec(0,0,1));
	  gui.setCameraTransformer(CameraTransformer(cam, z_plane));
	}
	gui.connectToArm(pa("-mem-srv-arm"),pa("-robot-arm-id"));
        gui.connectToHand(pa("-mem-srv-hand"),pa("-robot-hand-id"));
}

int main(int n, char **args) {
	paex("-input","define input grabber e.g. -input dc 0 or -input file images/*.ppm");
	paex("-size","image size of the camera, e.g. VGA or UXGA");
	paex("-mem-srv-arm","name of the arm memory server, default is xcf:mem-arm");
        paex("-mem-srv-arm","name of the hand memory server, default is xcf:mem-hand");
        paex("-cam-cfg","camera configuration file for screen to world transformation");
	paex("-tangram-cfg","xml file describing the polygon classes for classifying");
	paex("-robot-arm-id","id of the robot arm to use, default: LeftArm");
        paex("-robot-hand-id","id of the robot hand to use, default: LeftHand");
	paex("-grid x","draw grid with a line every x mm");
	Camera c;
	cout << c;
        ICLApplication app(n, args, "-input|-i(2) -size|-s(Size=VGA) -mem-srv-arm(string=xcf:mem-arm) "
                "-mem-srv-hand(string=xcf:mem-hand) -cam-cfg(string) -tangram-cfg(string) "
                "-robot-arm-id(string=LeftArm) -robot-hand-id(string=LeftHand) -grid(float=-1)",
                init, vision_loop);
  
  return app.exec();
}

