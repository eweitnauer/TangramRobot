// Copyright 2009 Erik Weitnauer
/// Main method for the BasicDemo which comes with the bullet physics engine lib.
#include "BasicDemo.h"
#include "GlutStuff.h"
#include "btBulletDynamicsCommon.h"
	
int main(int argc,char** argv)
{
	BasicDemo ccdDemo;
	ccdDemo.initPhysics();

	return glutmain(argc, argv,640,480,"Bullet Physics Demo. http://bullet.sf.net",&ccdDemo);
	
	//default glut doesn't return from mainloop
	return 0;
}
