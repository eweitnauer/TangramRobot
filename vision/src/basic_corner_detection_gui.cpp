// Copyright 2009 Erik Weitnauer
#include "basic_corner_detection_gui.h"
#include <ICLCore/Line.h>
#include <ICLQuick/Common.h>
#include <ICLUtils/StackTimer.h>
#include <ICLCore/CornerDetectorCSS.h>

using namespace std;
using namespace icl;


const vector< Point32f > &getThinnedBoundary(const vector< Point > &b) {
  static vector< Point32f > thinned;
  thinned.clear();
  if (b.size() < 2) return thinned;
  Point pre = b[b.size()-2];
  Point cur = b[b.size()-1];
  Point post; 
  for (unsigned i=0; i < b.size(); i++) {
    // search for the first point not in the 8 neighbourhood of current point
    if ((abs(b[i].x - cur.x) > 1) || (abs(b[i].y - cur.y) > 1)) {
      thinned.push_back((Point32f)cur);
      pre = cur;
      cur = post;
      post = b[i];
    } else post = b[i];
  }
  return thinned;
}

struct ColorDist{
  float r,g,b;
  ColorDist(const std::vector<double> &color):
    r(color.at(0)),g(color.at(1)),b(color.at(2)){}
  static inline float sqr(float x){ return x*x; }
  void operator()(const icl8u src[3], icl8u dst[1]) const{
    *dst = 255.0 - sqrt(sqr(r-src[0])+sqr(g-src[1])+sqr(b-src[2]))/sqrt(3);
  }
};

const Img8u &create_weight_image(const Img8u &image, const std::vector<double> &color){
  static Img8u wi(Size(1,1),1);
  wi.setSize(image.getSize());
  image.reduce_channels<icl8u,3,1,ColorDist>(wi,ColorDist(color));
  return wi;
}

void BasicCornerDetectionGui::vision_loop() {
  const Img8u *image = m_grabber->grab()->asImg<icl8u>();
	// get distance image to reference color
 	m_thresh_mutex.lock();
  static ImgBase *ltIm = 0;
  static Img8u wi;
  std::vector<icl::Region> rs;
  
  wi = create_weight_image(*image,m_refColor);
  m_thresh_mutex.unlock();
  {
  BENCHMARK_THIS_FUNCTION;
  // use the local threshold on that image
  static LocalThresholdOp lt(35,-10,0);
  static int &threshold = m_gui.getValue<int>("thresh");
  static int &maskSize = m_gui.getValue<int>("mask-size");
  lt.setGlobalThreshold(threshold);
  lt.setMaskSize(maskSize);
  lt.apply(&wi,&ltIm);

	static int &min_blob_size = m_gui.getValue<int>("min-blob-size");
	static int &max_blob_size = m_gui.getValue<int>("max-blob-size");
  static RegionDetector d(400,100000,255,255);
  d.setRestrictions(min_blob_size, max_blob_size, 255, 255);
  rs = d.detect(ltIm);
	}
	ICLDrawWidget *w = *(*m_h);
  w->lock();
  w->reset();
	
	draw(w, rs);

  w->unlock();

  string &vis =  m_gui.getValue<std::string>("vis");

  if (vis == "color image") *m_h = image;
  else if (vis == "color distance image") *m_h = &wi;
  else if (vis == "local threshold image") *m_h = ltIm;
  else ERROR_LOG("This combobox value is unknown: " + vis);
  m_h->update();
}

vector<Point32f> BasicCornerDetectionGui::detectCornersCSS(const icl::Region &r) {
	const vector<Point32f> &boundary = getThinnedBoundary(r.getBoundary());
	CornerDetectorCSS detector(
//	return r.getBoundaryCorners(
		m_gui.getValue<float>("max_angle"),
    m_gui.getValue<float>("rc_coeff"),
    m_gui.getValue<float>("sigma"),
    m_gui.getValue<float>("k_cutoff"),
    m_gui.getValue<float>("straight_line_thresh"));
		return detector.detectCorners(boundary);
}
    
void BasicCornerDetectionGui::draw(ICLDrawWidget *w, const vector<icl::Region> &rs) {
  // iterate over detected regions
  for(unsigned int i=0;i<rs.size();++i) {
  	// draw boundary
    w->color(0,255,0,255); w->fill(0,255,0,255);
    w->points(rs[i].getBoundary());
  	const vector<Point32f> &boundary = getThinnedBoundary(rs[i].getBoundary());
    w->color(255,0,0,255); w->fill(255,0,0,255);
    w->points(boundary);
    // do the corner detection
    vector<Point32f> corners = detectCornersCSS(rs[i]);
    //cout << "Contour pixels: " << boundary.size() << endl;
    //cout << "Corners: " << corners.size() << endl;
    // draw center of gravity
    Point32f cog = rs[i].getCOG();
    w->color(255,0,0,255); w->fill(255,0,0,255);
    w->ellipse(cog.x-1, cog.y-1,2,2);
		drawCorners(w, corners, Color4D(255,0,0,255), Color4D(255,0,0,150));    
  }
}

void BasicCornerDetectionGui::drawCorners(ICLDrawWidget *w,
		const vector<Point32f> &corners,
		const Color4D &color, const Color4D &fill) const {
	w->color(color[0],color[1],color[2],color[3]);
	w->fill(fill[0],fill[1],fill[2],fill[3]);
	for (unsigned int j=0; j<corners.size(); j++) {
  	// draw corner  
    w->ellipse(corners[j].x-2, corners[j].y-2,4,4);
	}
}

void BasicCornerDetectionGui::process(const MouseEvent &event) {
  if(event.isPressEvent()){
    if(event.getColor().size() == 3) {
      Mutex::Locker l(m_thresh_mutex);
      m_refColor = event.getColor();
      std::cout << "new Ref-Color: ("  << m_refColor[0] << ", "
      << m_refColor[1] << ", " << m_refColor[2] << ")" << std::endl;
    }
  }
}

void BasicCornerDetectionGui::init() {
	m_gui << "draw[@handle=img_input@minsize=32x24]";
	m_gui << addControls(m_gui);
	m_gui.show();
	m_tab_handle = m_gui.getValue<TabHandle>("tab");
	(*m_gui.getValue<icl::DrawHandle>("img_input"))->install(this);
	m_h = &m_gui.getValue<DrawHandle>("img_input");
	m_grabber = new GenericGrabber(FROM_PROGARG("-input"));
	if (pa("-size")) m_grabber->setIgnoreDesiredParams(false);
  m_grabber->setDesiredSize(pa("-size",0));
	m_grabber->setDesiredDepth(depth8u);
}

int BasicCornerDetectionGui::getSelectedTabIndex() {
	return (*m_tab_handle)->currentIndex();
}

std::string create_camcfg(const std::string&, const std::string &hint){
  return str("camcfg(")+hint+")[@maxsize=5x2]";
}

GUI &BasicCornerDetectionGui::addControls(icl::GUI &gui) {
	gui << (GUI("hbox") 
//     			<< create_camcfg(FROM_PROGARG("-input"))
     			<<  "combo(color image,color distance image,local threshold image)[@label=visualization@out=vis]"
         );
	m_tab = GUI("tab("+m_tab_names+")[@handle=tab]");
	
	// add segmentation controls
	GUI segmentation_controls("vbox");
	segmentation_controls
		<< (GUI("vbox[@label=blob detection]") 
    << "slider(1,100000,400)[@out=min-blob-size@handle=min-blob-size-handle@label=mininmal blob size]"
    << "slider(1,1000000,100000)[@out=max-blob-size@handle=max-blob-size-handle@label=maximal blob size]");
	segmentation_controls
		<< (GUI("vbox[@label=local threshold]") 
    << "slider(2,800,100)[@out=mask-size@handle=mask-size-handle@label=mask size]"
    << "slider(-200,200,50)[@out=thresh@handle=thresh-handle@label=threshold]");
  m_tab << segmentation_controls;
  
  // add CSS Corner Detection controls
 	GUI css_controls("vbox");
  css_controls << "fslider(1,20,3)[@out=sigma@label=gaussian sigma]";
  css_controls << "fslider(0,10,1.5)[@out=rc_coeff@label=round corner coefficient]";
  css_controls << "fslider(0,1000,100)[@out=k_cutoff@label=curvature cutoff]";
  css_controls << "fslider(0,180,162)[@out=max_angle@label=maximum corner angle]";
  css_controls << "fslider(0,180,10)[@out=straight_line_thresh@label=straight line threshold]";
  m_tab << css_controls;

  return m_tab;
}

