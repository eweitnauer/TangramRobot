// Copyright 2009 Erik Weitnauer
/// Test class for the PushingSimulatorGui.
#include "PushingSimulatorGui.h"
#include <iostream>

using namespace std;

int main(int argc, char **argv) {
	cout << "pushing test" << endl;
	PushingSimulatorGui psim;
	psim.init();
	return glutmain(argc, argv,640,480,"Minimal Visualization Example",&psim);
}
