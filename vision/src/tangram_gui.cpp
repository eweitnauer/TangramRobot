#include "tangram_gui.h"
#include "tangram_classifier.h"
#include "polygon_mapper.h"
#include "lin_alg.h"

using namespace std;

vector<PolygonObject*> TangramGui::getActivePolygons() {
	icl::Mutex::Locker l(m_polygon_mutex);
	vector<PolygonObject*> actives;
	for (unsigned int i=0; i<m_polygon_pool.size(); i++) {
		if (m_polygon_pool[i].isActive(m_timestep)) actives.push_back(&(m_polygon_pool[i]));
	}
	return actives;
}

// Check the area of the smallest and the biggest polygon shape on the screen
// and adjust the max and min blob size of the RegionDetector.
void TangramGui::adjustRegionDetectorParams() {
	static bool &enabled = m_gui.getValue<bool>("blob-auto-adjust");
	if (!enabled) return;
	const std::vector<PolygonShape> &shapes = m_tangram_classifier.getAllShapes();
	if (shapes.empty()) return;
	int biggest_idx=0, smallest_idx=0;
	for (unsigned int i=1; i<shapes.size(); ++i) {
		if (shapes[i].getArea() > shapes[biggest_idx].getArea()) biggest_idx = i;
		else if (shapes[i].getArea() < shapes[smallest_idx].getArea()) smallest_idx = i;
	}
	float biggest_area, smallest_area;
	if (do_world_transformation) {
		PolygonShape bigshape = shapes[biggest_idx];
		PolygonShape smallshape = shapes[smallest_idx];
		m_cam_transformer.transformWorldToScreen2D(bigshape, bigshape.getHeight());
		m_cam_transformer.transformWorldToScreen2D(smallshape, smallshape.getHeight());
		biggest_area=bigshape.getArea();
		smallest_area=smallshape.getArea();
	} else {
		biggest_area=shapes[biggest_idx].getArea();
		smallest_area=shapes[smallest_idx].getArea();
	}
	m_gui.getValue<SliderHandle>("max-blob-size-handle") = biggest_area*1.2;
	m_gui.getValue<SliderHandle>("min-blob-size-handle") = smallest_area*0.8;
}

void TangramGui::init() {
	BasicCornerDetectionGui::init();
}

GUI &TangramGui::addControls(GUI &gui) {
	GUI &tab = BasicCornerDetectionGui::addControls(gui);
  
	GUI vbox("vbox");
	vbox << "togglebutton(no auto-adjust,!auto-adjust)[@out=blob-auto-adjust@handle=blob-auto-adjust-handle@label=region detector parameter]"
			<< "fslider(1,500,96)[@out=tile-size@label=tangram tile size]"
			<< "fslider(0,1,0.2)[@out=size-tolerance@label=classification shape tolerance]"
			<< "fslider(0,1,0.1)[@out=corner-tolerance@label=mapping corner distance tolerance]";
	tab << vbox;
	
	return tab;
}

string str_rounded(const Point32f p) {
	return str(0.1*round(p.x)) + ", " + str(0.1*round(p.y));
}

void TangramGui::loadShapesFromXMLFile(const std::string &file) {
	m_tangram_classifier.clearShapes();
	m_tangram_classifier.loadShapesFromXML(file);
	cout << "Loaded these polygon models:" << endl << m_tangram_classifier << endl;
	adjustRegionDetectorParams();
}

void TangramGui::loadStandardTangramShapes() {
	m_tangram_classifier.clearShapes();
	m_tangram_classifier.loadStandardTangramShapes(m_gui.getValue<float>("tile-size"),18);
	adjustRegionDetectorParams();
}

void TangramGui::drawPredictedShapes(ICLDrawWidget *w, const Color4D &color) {
	w->color(color[0],color[1],color[2],color[3]);
	for (unsigned int i=0; i<m_polygon_pool.size(); i++) {
		const PolygonShape &pshape = m_polygon_pool[i].getPredictedShape();
		if (!pshape.hasCorners()) continue;
		PolygonShape shape = pshape;
		if (do_world_transformation) 
			m_cam_transformer.transformWorldToScreen2D(shape, shape.getHeight());
		w->linestrip(shape.getCorners());
	}
}

void TangramGui::drawPolygons(ICLDrawWidget *w, const vector<PolygonObject> &polygons,
		const Color4D &color,	const Color4D &fill) {
	enum {INACTIVE, ACTIVE};
	for (int state=INACTIVE; state<=ACTIVE; ++state) {
	  for (unsigned int k=0; k<polygons.size(); k++) {
	   	if (!polygons[k].isActive(m_timestep) && state==INACTIVE) {
				w->color(100,100,100);
				w->fill(100,100,100);
			} else if (polygons[k].isActive(m_timestep) && state==ACTIVE) {
				w->color(color[0],color[1],color[2],color[3]);
				w->fill(fill[0],fill[1],fill[2],fill[3]);
			} else continue;
	  	PolygonShape shape = polygons[k].getTransformedShape();
	  	Point32f center = shape.getCenter();
	  	float rotation = polygons[k].getTransformation().getRotation()*180/M_PI;
			if (do_world_transformation) m_cam_transformer.transformWorldToScreen2D(shape, shape.getHeight());
	 		w->ellipse(shape.getCenter().x-1,shape.getCenter().y-1,2,2);
  		w->text("#" + str(polygons[k].getId()) + " " + shape.getName(), shape.getCenter().x, shape.getCenter().y,-1,-1,10);
  		if (do_world_transformation)
		 		w->text("Position [cm]: " + str_rounded(center), shape.getCenter().x, shape.getCenter().y+20,-1,-1,10);
	 		else w->text("Position [px]: " + str(round(shape.getCenter().x)) + ", "
	 																	 + str(round(shape.getCenter().y)),
	 								 shape.getCenter().x, shape.getCenter().y+20,-1,-1,10);

	 		w->text("Rotation [deg]: " + str(round(rotation)), shape.getCenter().x, shape.getCenter().y+40,-1,-1,10);
  		w->linestrip(shape.getCorners());
	  }
	}
}

void TangramGui::forgetOldPolygons(int max_age) {
	for (int i=m_polygon_pool.size()-1; i>=0; --i) {
		if (m_timestep - m_polygon_pool[i].getActiveTime() > max_age)
			m_polygon_pool.erase(m_polygon_pool.begin()+i);
	}
}

void TangramGui::drawGrid(ICLDrawWidget *w, float size_mm, const Color4D &color,
		float offset_x_mm, float offset_y_mm) {
	w->color(color[0],color[1],color[2],color[3]);
	if (size_mm < 40) size_mm = 40;
	for (float y=offset_y_mm; y<1000+offset_y_mm; y+= size_mm) {
		for (float x=-1000+offset_x_mm; x<1000+offset_x_mm; x+= size_mm) {
			// A -- B
			// |
			// C
			Point32f A = m_cam_transformer.transformedWorldToScreen(Vec(x,y,0,1));
			Point32f B = m_cam_transformer.transformedWorldToScreen(Vec(x+size_mm,y,0,1));
			Point32f C = m_cam_transformer.transformedWorldToScreen(Vec(x,y+size_mm,0,1));
			w->line(A.x,A.y,B.x,B.y);
			w->line(A.x,A.y,C.x,C.y);
		}
	}
}
		
void TangramGui::draw(ICLDrawWidget *w, const std::vector<icl::Region> &rs) {
	static const Color4D RED(255,50,50,255);
	static const Color4D YELLOW(255,255,0,255);
	static const Color4D GRAY(50,50,50,200);
	static float &tileSize = m_gui.getValue<float>("tile-size");
	static float &sizeTolerance = m_gui.getValue<float>("size-tolerance");
	static float &cornerTolerance = m_gui.getValue<float>("corner-tolerance");
	static ButtonHandle &auto_adjust_button = m_gui.getValue<ButtonHandle>("blob-auto-adjust-handle");
	
	Mutex::Locker l(m_polygon_mutex);
	m_polygon_shapes.clear();
	m_timestep++;
	if (m_tangram_classifier.getBaseLength() != tileSize) {
		m_tangram_classifier.setSize(tileSize);
		adjustRegionDetectorParams();
		m_polygon_pool.clear();
	}
	if (auto_adjust_button.wasTriggered()) adjustRegionDetectorParams();

	forgetOldPolygons(m_forget_after);

	// iterate over detected regions
	for(unsigned int i=0;i<rs.size();++i) {
	  // do the corner detection
	  vector<Point32f> corners = detectCornersCSS(rs[i]);
	  w->color(255,255,255,255);
    //w->text(str(rs[i].getSize()), rs[i].getCOG().x-20, rs[i].getCOG().y-20);
	  drawCorners(w, corners, RED, RED);
	  // transform to world coordinates
		if (do_world_transformation) m_cam_transformer.transformScreenToWorld2D(corners);

	  // object classification, but only if polygon has less than 6 corners
	  if (corners.size() > 5) continue;
	  PolygonShape observed_shape(corners);
	  m_polygon_shapes.push_back(observed_shape);
	  // try to match a polygon from the polygon pool on the observed shape
	  if (! PolygonMapper::mapPolygonObjects(observed_shape, m_polygon_pool, m_timestep, cornerTolerance)) {
	  	// it didn't work, so try to match any of the tangram shapes and create a new polygon object
	  	vector<PolygonShape> candidate_shapes = m_tangram_classifier.classify(observed_shape, sizeTolerance, true);
	  	vector<PolygonObject> polygons = PolygonMapper::mapPolygonShapes(observed_shape, candidate_shapes, cornerTolerance,true,false);
	  	// take the best match and copy it to the polygon pool
	  	if (polygons.size() > 0) {
	  		polygons[0].setActive(m_timestep);
	  		m_polygon_pool.push_back(polygons[0]);
	  	}
	  }
	}
	if (m_grid_size != -1) drawGrid(w,m_grid_size,Color4D(80,255,80,255));
	drawPolygons(w, m_polygon_pool, RED, RED);
	drawPredictedShapes(w, YELLOW);
}

void TangramGui::setCameraTransformer(CameraTransformer tc, bool use_it) {
	m_cam_transformer = tc;
	setDoWorldTransformation(use_it);
}

void TangramGui::setDoWorldTransformation(bool value) {
	do_world_transformation = value;
	if (value) adjustRegionDetectorParams();
}

