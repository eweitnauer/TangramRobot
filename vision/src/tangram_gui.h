#ifndef __TANGRAM_GUI_EWEITNAU_H__
#define __TANGRAM_GUI_EWEITNAU_H__

#include <ICLQt/DrawWidget.h>
#include <ICLCC/Color.h>
#include <ICLBlob/Region.h>
#include <ICLGeom/Camera.h>
#include <ICLUtils/XMLDocument.h>

#include "polygon_object.h"
#include "basic_corner_detection_gui.h"
#include "tangram_classifier.h"
#include "camera_transformer.h"

/// Class adding to the BasicsCornerDetectionGui the functionality of tracking tangrams.
/** In a vector, all currently detected (active) and formerly detected (inactive)
 * polygon objects are stored. Both types are shown in the GUI. Inactive polygons
 * are forgotten after some timesteps.
 * In the GUI are controls to dynamically adjust the tangram model size, and the
 * classification and mapping error threshholds.
 *
 * The tangram detection and mapping classes will work in world coordinates in [mm],
 * if a camera for performing the screen to world transformation is provided via
 * the setCamera(.) method and then calling the enableScreenToWorldTransformation()
 * method. When no camera is provides, all classes work with pixels as units.
 */
class TangramGui : public BasicCornerDetectionGui {
	public:
		TangramGui(): BasicCornerDetectionGui(), m_timestep(0),	do_world_transformation(false),
			m_grid_size(-1), m_forget_after(20)
			{ m_tab_names += ",Tangram Tracking"; }
		
		virtual void draw(icl::ICLDrawWidget *w, const std::vector<icl::Region> &rs);
		
		virtual void init();
		
		void loadShapesFromXMLFile(const std::string &file);
		void loadStandardTangramShapes();
		
		std::vector<PolygonObject*> getActivePolygons();
		std::vector<PolygonObject> &getPolygons() { return m_polygon_pool; }
		std::vector<PolygonShape> getCurrentPolygonShapes() const { return m_polygon_shapes; }
	
		void setCameraTransformer(CameraTransformer tc, bool use_it=true);
		const CameraTransformer &getCameraTransformer() { return m_cam_transformer; }
		virtual void setDoWorldTransformation(bool value);
		bool getDoWorldTransformation() { return do_world_transformation; }
		
		float getTangramHeight() const { return m_tangram_classifier.getHeight(); }
		float getTangramBaseLength() const { return m_tangram_classifier.getBaseLength(); }
		
		void enableGrid(float grid_size_mm) { m_grid_size = grid_size_mm; }
		void disableGrid() { m_grid_size = -1; }
		inline float getGridSize() const { return m_grid_size; }
				
		/// Set number of timesteps after which a polygon is removed, when it was not recognized anymore.
		void setForgetPolygonsAfter(int timesteps) { m_forget_after = timesteps; }
		/// Get number of timesteps after which a polygon is removed, when it was not recognized anymore.
		int getForgetPolygonsAfter() const { return m_forget_after; }
	protected:
		/// Gets called on init by the parent class
		virtual GUI &addControls(icl::GUI &gui);
		
		/// Automatically adjusts the parameters of the RegionDetector.
		/** Checks area of the smallest and biggest polygon shape model on screen and
		 * adjusts max and min blob size of the RegionDetector.*/
		void adjustRegionDetectorParams();

		void drawPolygons(icl::ICLDrawWidget *w, const std::vector<PolygonObject> &polygons,
				const icl::Color4D &color,	const icl::Color4D &fill);

		void drawPredictedShapes(icl::ICLDrawWidget *w, const icl::Color4D &color);

		/// Grid size must be 40 mm at least. A camera transformer must be set.
		void drawGrid(ICLDrawWidget *w, float size_mm, const Color4D &color,
			float offset_x_mm=0, float offset_y_mm=0);
		
	private:
		void forgetOldPolygons(int max_age);

		std::vector<PolygonObject> m_polygon_pool;
		std::vector<PolygonShape> m_polygon_shapes;
		int m_timestep;
		icl::Mutex m_polygon_mutex;
		TangramClassifier m_tangram_classifier;
		CameraTransformer m_cam_transformer;
		bool do_world_transformation;
		float m_grid_size;
		int m_forget_after;
};

#endif /* __TANGRAM_GUI_EWEITNAU_H__ */

