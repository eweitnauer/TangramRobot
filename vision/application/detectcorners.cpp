// Copyright 2009 Erik Weitnauer
/// Example for corner detection of blobs with the CSS algorithm.

#include "basic_corner_detection_gui.h"
#include <ICLQuick/Common.h>

BasicCornerDetectionGui gui;

void vision_loop() {
	gui.vision_loop();
  Thread::msleep(10);
}

void init() {
	gui.init();
}

int main(int n, char **args) {
	paex("-input","define input device id and parameters");
  paex("-size","defines the input image size");
  return ICLApplication(n, args, "-size|-s(Size=VGA) -input|-i(device-type=dc,device-params=0)", init, vision_loop).exec();
}

