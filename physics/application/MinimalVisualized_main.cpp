// Copyright 2009 Erik Weitnauer
/// Test class for the MinimalVisualizer.
#include "MinimalVisualized.h"
#include "GlutStuff.h"
 
 int main(int argc,char** argv) {
  MinimalVisualizer mvis;
  mvis.init();
  return glutmain(argc, argv,640,480,"Minimal Visualization Example",&mvis);
 }
