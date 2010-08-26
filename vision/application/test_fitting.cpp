// Copyright 2009 Erik Weitnauer
/// Not yet automated testing class for the tangram detection / tracking workflow.

#include <ICLQuick/Common.h>
#include "polygon_mapper.h"
#include "tangram_classifier.h"
#include "transformation.h"

int main() {

	cout << "Hello Tangram" << endl;
	
	Transformation t;
	cout << "Identity Transformation: [" << t << "]" << endl;

	// defining observed object data
	PolygonShape data("Data (4 corners)");
	data.addCorner(Point32f(11.5,12));
	data.addCorner(Point32f(  12, 8));
	data.addCorner(Point32f(   8, 8));
	data.addCorner(Point32f(   8,12), true);
	
	cout << "== Target Shape " << data << endl;
	//data.addCorner(Point32f(   10,  10));
	//data.addCorner(Point32f(   15,  10));
	//data.addCorner(Point32f( 12.5,12.5));
	//data.updateGeometry();
	
	// testing the TangramClassifier
	cout << "Now testing the TangramClassifier..." << endl;
	TangramClassifier tc;
	tc.loadStandardTangramShapes();
	
	vector<PolygonShape> candidates = tc.classify(data, 0.2, true); // tolerance 0.2 and size matters
	cout << "Got " << candidates.size() << " candidate(s):" << endl;
	for (unsigned int i=0; i<candidates.size(); i++) {
		cout << "== Candidate Model " << i << ": \n" << candidates[i] << endl;
	}

	// try to fit all tangram tile models to the data
	cout << endl << "Now mapping all candidates to target points..." << endl;	
	// dynThresh=0.1, sortResults=true, useScaling=false
	vector<PolygonObject> result = PolygonMapper::mapPolygonShapes(data, candidates, 0.1, true, false);
	cout << "Got " << result.size() << " mapping(s):" << endl;
	for (unsigned int i=0; i<result.size(); i++) {
		cout << "== PolygonObject " << i << ": \n" << result[i] << endl;
	}
	
	/*if (result.size() > 0) {
		cout << endl << "Moving all target points by (-1,2) and rotating by 1..." << endl;
		data.transformAroundCenter(LinAlg::createTransformationMatrix2d(1,-1,2,1));
		cout << "== New Target Shape " << data << endl;
		cout << "Now mapping the first of these mappings (which are PolygonObjects)";
		cout << " to the translated target points again..." << endl;
		bool success = PolygonMapper::mapPolygonObject(data, result[0], 1, 1000, 1000, 1000);
		cout << "Successful: " << (success ? "yes" : "no") << endl;
		if (success) {
			cout << "The changed PolygonObject: " << endl << result[0] << endl;
		}
	}*/
	
	if (result.size() > 0) {
		cout << endl << "Moving all target points by (-1,2) and rotating by 1..." << endl;
		data.transformAroundCenter(LinAlg::createTransformationMatrix2d(1,-1,2,1));
		cout << "== New Target Shape " << data << endl;
		cout << "Now mapping all these mappings (which are PolygonObjects)";
		cout << " to the translated target points again..." << endl;
		bool success = PolygonMapper::mapPolygonObjects(data, result, 1, 1000, 1000, 1000);
		cout << "Successful: " << (success ? "yes" : "no") << endl;
		if (success) {
			cout << "All PolygonObject again: " << endl;
			for (unsigned int i=0; i<result.size(); i++) {
				cout << "== PolygonObject " << i << ": \n" << result[i] << endl;
			}
		}
	}
	
  cout << "Seeya." << endl;
}
