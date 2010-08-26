// Copyright 2009 Erik Weitnauer
/// Test class for the PushingSimulator.
#include "PushingSimulatorFast.h"
#include <iostream>

using namespace std;

int main(int argc, char **argv) {
	cout << "pushing test" << endl;
	PushingSimulatorFast psim;
	psim.init();
	vector<btRigidBody*> nobody;
	psim.simulate(PushMovement(btVector3(0,1,40), btVector3(0,1,10)), nobody);
}
