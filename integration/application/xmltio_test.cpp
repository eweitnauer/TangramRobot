// Copyright 2009 Erik Weitnauer
/// Converting a PolygonShape to xml and back. It works.

#include "xmltio/xmltio.hpp"
#include <iostream>
#include "tangram_classifier.h"


namespace xmltio {
	template<>
	PolygonShape xmltio::extract<PolygonShape>::get() const {
		PolygonShape shape;
		shape.setName(extract<std::string>(loc_["name"]));
		int num = 0;
		for(XPathIterator i = XPath("/POLYGONSHAPE/CORNERS/CORNER").evaluate(loc_); i; ++i) {
    	Location l(*i);
    	if (extract<int>(l["n"]) != num) {
    		std::cout << "WRONG CORNER ORDER" << std::endl;
    	}
    	shape.addCorner(icl::Point32f(extract<float>(l["x"]), extract<float>(l["y"])));
    	num++;
   	}
		return shape;
	}
}

using namespace std;
using namespace xmltio;

void convert(const PolygonShape& shape, Location& l) {
	l["name"] = shape.getName();
	Location lCORNERS("<CORNERS/>", "/CORNERS");
	const vector<icl::Point32f> &corners = shape.getCorners();
  lCORNERS["count"] = corners.size();
  for (unsigned int i=0; i<corners.size(); i++) {
  	Location lC("<CORNER/>", "/CORNER");
  	lC["n"] = i; lC["x"] = corners[i].x; lC["y"] = corners[i].y;
  	lCORNERS.add(lC);
  }
  l.add(lCORNERS);
}

int main() {
	cout << "Xmltio test." << endl;
	TangramClassifier tc;
	tc.loadStandardTangramShapes();
	PolygonShape shape = tc.getAllShapes()[0];
	Location l("<POLYGONSHAPE/>", "/POLYGONSHAPE");
	l = shape;
	cout << l.getDocumentText(true) << "\n";
	PolygonShape shape2 = extract<PolygonShape>(l);
	cout << "The parsed shape is: " << shape2 << endl;
	return 0;
}
