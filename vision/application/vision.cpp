// Copyright 2009 Erik Weitnauer
/// Detecting and tracking tangrams.

#include "tangram_gui.h"
#include <ICLQuick/Common.h>
#include "ICLUtils/XMLDocument.h"

TangramGui gui;

void vision_loop() {
	gui.vision_loop();
  Thread::msleep(100);
}

void init() {
	gui.init();
	if (pa("-grid")) {
    gui.enableGrid(pa("-grid").as<float>());
	}
	// load tangram shapes
	if (pa("-tangram-cfg")) {
		gui.loadShapesFromXMLFile(*pa("-tangram-cfg"));
	} else {
		gui.loadStandardTangramShapes();
	}
	// load camera configuration
	if (pa("-cam-cfg")) {
		Camera cam(pa("-cam-cfg").as<std::string>());
		// the camera sees the top of the tangrams, so the intersection plane for the
		// viewrays must be at z=height_of_tangrams
		PlaneEquation z_plane(Vec(0,0,gui.getTangramHeight()),Vec(0,0,1));
	  gui.setCameraTransformer(CameraTransformer(cam, z_plane));
	}
}

int main(int n, char **args) {
	paex("-input","define input grabber e.g. -input dc 0 or -input file images/*.ppm");
	paex("-size","image size of the camera, e.g. VGA or UXGA");
	paex("-cam-cfg","camera configuration file for screen to world transformation");
	paex("-tangram-cfg","xml file describing the polygon classes for classifying");
	paex("-grid x","draw grid with a line every x mm");
  return ICLApplication(n, args, "-input|-i(device-type=dc,device-params=0) -size|-s(Size=VGA) -cam-cfg(1) -tangram-cfg(1) -grid(mm)", init, vision_loop).exec();
}

