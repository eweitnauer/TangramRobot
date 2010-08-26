// Copyright 2009 Erik Weitnauer
/** 
\mainpage Tangram Tracking

\section OV Overview

The Tangram Tracking package is the computer vision part of the diploma thesis
<em>Vision Based Prediction of Pushing Actions with State-of-the-Art Physics Engines</em>
of Erik Weitnauer at the University of Bielefeld.

The usage of the classes in the src/ directory is demonstrated by several example programs
in the /application directory. Most of them will need the ImageComponentLibrary (ICL)
version 4.3.2 installed.

\section EX Example of Usage

Here is a small code example, demonstration some features of the package:

\code
#include <ICLQuick/Common.h>
#include "polygon_mapper.h"
#include "tangram_classifier.h"

int main() {
	// defining some "observed" polygon
	PolygonShape data;
	data.addCorner(Point32f(   10,  10));
	data.addCorner(Point32f(   15,  10));
	data.addCorner(Point32f( 12.5,12.5));
	data.updateGeometry(); // calculates area, center

	// classify the polygon (get some candidates tangram tiles)
	TangramClassifier tc;
	tc.loadTangramShapes(10); // base length is 10
	vector<PolygonShape> candidates = tc.classify(data);
	cout << "Got " << candidates.size() << " candidate(s):" << endl;
	for (unsigned int i=0; i<candidates.size(); i++) cout << candidates[i] << endl;

	// map all candidate tangram tiles to the polygon
	vector<PolygonObject> result = PolygonMapper::mapPolygonShapes(data, candidates);
	cout << "Got " << result.size() << " mapping(s):" << endl;
	for (unsigned int i=0; i<result.size(); i++) cout << result[i] << endl;
}
\endcode

*/
