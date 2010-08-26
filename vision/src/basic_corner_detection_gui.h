// Copyright 2009 Erik Weitnauer
#ifndef __BASIC_CORNER_DETECTION_GUI_EWEITNAU_H__
#define __BASIC_CORNER_DETECTION_GUI_EWEITNAU_H__

#include <ICLQuick/Common.h>
#include <ICLFilter/LocalThresholdOp.h>
#include <ICLCC/CC.h>
#include <ICLCC/Color.h>
#include <ICLBlob/RegionDetector.h>

/// Class to make writing icl GUIs which use the CornerDetectionCSS class more convinient.
/**
You need to start a QApplication and then call the vision_loop() method inside
the ExecThread. You may inherit from the class and override the draw() method.
As an example for the latter, see vision application.

Most basic use case:
\code
#include "basic_corner_detection_gui.h"
#include <ICLQuick/Common.h>

BasicCornerDetectionGui gui;

void vision_loop() {
	gui.vision_loop();
  Thread::msleep(10);	
}

int main(int n, char **args) {
	GenericGrabber::resetBus();  
  QApplication app(n, args);
  gui.init(n, args);
  ExecThread x(vision_loop);
	x.run();
	return app.exec();
}
\endcode
 */
class BasicCornerDetectionGui : public icl::MouseHandler {
	public:
		BasicCornerDetectionGui(): m_gui(icl::GUI("vsplit")), m_refColor(3,255),
			m_h(NULL), m_grabber(NULL),	m_tab_names("Segmentation,CSS Corner Detection") {}
		~BasicCornerDetectionGui() { delete m_h; delete m_grabber; }

		/// Must be called to initialize the GUI.
		virtual void init();
		/// Override this method for custom drawing.
		virtual void draw(icl::ICLDrawWidget *w, const std::vector<icl::Region> &rs);
		/// The vision_loop method must be called in the main loop.
		void vision_loop();
		/// Convinience method for applying the CSS corner detector on a detected region.
		std::vector<icl::Point32f> detectCornersCSS(const icl::Region &r);
		
	protected:
		icl::GUI m_gui;
		icl::Mutex m_thresh_mutex;
		std::vector<double> m_refColor;
		icl::DrawHandle *m_h;
		icl::GenericGrabber *m_grabber;
		std::string m_tab_names;

		virtual GUI &addControls(icl::GUI &gui);
		
		void drawCorners(ICLDrawWidget *w,	const vector<Point32f> &corners,
			const Color4D &color, const Color4D &fill) const;
		
	  virtual void process(const icl::MouseEvent &event);
	  
	  int getSelectedTabIndex();

	private:
		icl::GUI m_tab;
		icl::TabHandle m_tab_handle;
};

#endif /* __BASIC_CORNER_DETECTION_GUI_EWEITNAU_H__ */

