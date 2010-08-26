// Copyright 2009 Erik Weitnauer
/// Example for detecting tangram tiles on a camera image.

#include "basic_corner_detection_gui.h"
#include <ICLQuick/Common.h>
#include "tangram_classifier.h"
#include "polygon_mapper.h"
#include "lin_alg.h"

using namespace icl;

TangramClassifier tc;

template <class T>
std::string to_string (const T &t) {
	stringstream ss;
	ss.precision(3);
	ss << t;
	return ss.str();
}

class TangramGui : public BasicCornerDetectionGui {
	virtual void draw(icl::ICLDrawWidget *w, const std::vector<icl::Region> &rs) {
	  static Color4D RED(255,50,50,255);
		static Point32f cog;
		static vector<PolygonObject> polygons;
		// iterate over detected regions
		for(unsigned int i=0;i<rs.size();++i) {
			// draw boundary
		  w->color(255,0,0,255); w->fill(255,0,0,255);
		  w->points(rs[i].getBoundary());
		  // do the corner detection
		  const vector<Point32f> &corners = detectCornersCSS(rs[i]);
		  // draw center of gravity
		  cog = rs[i].getCOG();
		  w->color(255,0,0,255); w->fill(255,0,0,255);
		  w->ellipse(cog.x-1, cog.y-1,2,2);
		  //w->text(to_string(rs[i].getSize()), cog.x, cog.y-10,-1,-1,10);
		  
		  drawCorners(w, corners, RED, RED);
		  // object classification, but only if polygon has less than 6 corners
		  if (corners.size() < 6) {
  		  PolygonShape shape(corners);
			  vector<PolygonShape> candidates = tc.classify(shape, true);
			  // find mappings, sortResults=true, useScaling=false
			  polygons = PolygonMapper::mapPolygonShapes(shape, candidates, 0.04, true, false);
			  w->color(255,0,0,255); w->fill(255,0,0,255);
			  for (unsigned int k=0; k<polygons.size(); k++) {
			  	PolygonShape poly = polygons[k].getTransformedShape();
			  	w->text(poly.getName(), poly.getCenter().x, poly.getCenter().y,-1,-1,10);
			  	w->linestrip(poly.getCorners());
					break; // only draw first (=best) mapping
			  }
	 	    polygons.clear();
			}
		}
	}
} gui;

void vision_loop() {
	gui.vision_loop();	
  Thread::msleep(10);
}

void init() {
	tc.loadStandardTangramShapes(63.2,0.2); // size in pixel
	gui.init();
}

int main(int n, char **args) {
	paex("-input","define input grabber e.g. -input dc 0 or -input file images/*.ppm");
  return ICLApplication(n, args, "-input(2) -size(1)", init, vision_loop).exec();
}

