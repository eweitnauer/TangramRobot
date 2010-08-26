#include "basic_corner_detection_gui.h"
#include <ICLQuick/Common.h>
#include <fstream>

class BlobGui : public BasicCornerDetectionGui {
	public:	
	virtual GUI &addControls(icl::GUI &gui) {
		GUI &tab = BasicCornerDetectionGui::addControls(gui);
		tab << "togglebutton(show regions,hide regions)[@out=show-regions]";
		return tab;
	}

	virtual void init() {
		BasicCornerDetectionGui::init();	
		m_grabber->setDesiredSize(Size(800,600));
	}
	
	virtual void draw(icl::ICLDrawWidget *w, const std::vector<icl::Region> &rs) {
		if (!m_gui.getValue<bool>("show-regions")) return;
	  
	  // iterate over detected regions
		for(unsigned int i=0;i<rs.size();++i) {
			// draw boundary
		  w->color(255,0,0,255); w->fill(255,0,0,255);
		  for (unsigned int j=0; j<rs[i].getBoundary().size(); ++j) {
			  w->rect(rs[i].getBoundary()[j].x,rs[i].getBoundary()[j].y,1,1);
		  }
		  // draw center of gravity
		  Point32f cog = rs[i].getCOG();
		  w->ellipse(cog.x-1, cog.y-1,2,2);
		  w->text(str(i),cog.x,cog.y);
		}
	}
} gui;

void vision_loop() {
	gui.vision_loop();
  Thread::msleep(100);
}

void init() {
	gui.init();
}

int main(int n, char **args) {
	paex("-input","define input grabber e.g. -input dc 0 or -input file images/*.ppm");
	return ICLApplication(n, args, "-input(2) -size(1)", init, vision_loop).exec();
}
