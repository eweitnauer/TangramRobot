#include <PushingSimulatorFast.h>
#include <PushingSimulatorGui.h>
#include <Shapes.h>
#include <transformation.h>
#include <vision_adapter.h>
#include <ICLUtils/ThreadUtils.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <PushedBody.h>
#include <PushingScene.h>
#include <PushingRecorder.h>
#include <string>
#include <ICLUtils/StringUtils.h>
#include <PushingSimulatorDebug.h>

using namespace std;

#define VISUALIZE 0

#if VISUALIZE
PushingSimulatorGui psim;
#else
PushingSimulatorFast psim;
#endif

PushingRecorder prec(&psim);

#define TANGRAM_LENGTH 0.096 // 9.6 cm
#define TANGRAM_HEIGHT 0.018 // 1.8 cm
#define SQ_MASS 0.138 // 138 g
#define LT_MASS 0.268 // 268 g
#define ST_MASS 0.072 // 72 g
#define PA_MASS 0.138 // 138 g
#define PUSHER_SPEED 0.05 // 5 cm / s
#define PUSHER_DIAMETER 0.022 // 2.2 cm

void record_multiple_datasets(const string &path_prefix,
		const PhysicsParameters &params, const vector<SimulationSettings> simsets,
		const vector<PushingSceneInfo> sceneInfos) {
	PushingScene scene;
	for (unsigned int j=0; j<sceneInfos.size(); ++j) {
		// use the first simulation settings to get a reference transformation of
		// the trangram piece
		cout << "=== Scene " << j << " ===" << endl;
		cout << "  Running first simulation to get reference tangram transformation..." << endl;
		prec.simulateSingleParameterSetting(scene, simsets[0], params, sceneInfos[j]);
		scene.calcStatistics();
		Transformation reference_t = scene.pbodies[0].getMeanDelta();
		cout << "  Using [" << reference_t << "] as reference transformation." << endl;
	
		for (unsigned int i=1; i<simsets.size(); ++i) {
			stringstream s;
			s << path_prefix << simsets[i].param_name << "_" << j << ".txt";
			string filename = s.str();
			cout << "   Simulating " << simsets[i].param_name << " and writing result to " << filename << endl;
			ofstream out(filename.c_str());
			prec.writeDataset(out, simsets[i], params, sceneInfos[j], reference_t);
			out.close();
		}		
	}
}

void loadData(const string &filename, vector<PushingSceneInfo> &data) {
  ifstream f(filename.c_str());
  string line;
  getline(f, line); // first line has column names
  while (f.good()) {
    getline(f, line);
    vector<float> v = icl::parseVecStr<float>(line, " ");
    if (v.size() != 10) continue;
    // file data columns: tx0 ty0 trot0 ax0 ay0 ax1 ay1 tx1 ty1 trot1
    PushingSceneInfo si;
    si.setTangramPos(v[0]/100,v[1]/100,v[2]/180*M_PI,v[7]/100,v[8]/100,v[9]/180*M_PI);
    si.setPusherPos(v[3]/100,v[4]/100,v[5]/100,v[6]/100);
    data.push_back(si);
  }
  f.close(); 
}

void loadSceneFromFile(const string &filename, vector<PushingSceneInfo> &data, float tangram_mass, Shapes::ShapeType shapeType) {
  cout << "loading data..." << endl;
  loadData(filename, data);
  cout << "loaded pushing trial data: " << endl;
  for (unsigned int i=0; i<data.size();i++) {
  	data[i].tmass = tangram_mass;
  	data[i].tlength = TANGRAM_LENGTH;
  	data[i].theight = TANGRAM_HEIGHT;
  	data[i].ttype = shapeType;
  	data[i].tcorners = Shapes::getCorners(shapeType, TANGRAM_LENGTH); 
  	data[i].pspeed = PUSHER_SPEED;
  	data[i].pdiam = PUSHER_DIAMETER;
  	cout << "  " << data[i] << endl;
  }
  cout << data.size() << " pushing trials loaded from file " << filename << "." << endl;
}

#if VISUALIZE
	void show_physics_gui() {
		glutmain(0, NULL, 640, 480, "Minimal Visualization Example", &psim);
	}
#endif

int main() {
	psim.init();
	#if VISUALIZE
		psim.setFastForward(4);
		icl::ExecThread y(show_physics_gui);
		y.run(false); // no loop
	#endif
	PhysicsParameters params;
	params["collision_margin"] = 0.005;
	vector<SimulationSettings> simsets;
	int rep = 2;
	simsets.push_back(SimulationSettings(rep*3));
	simsets.push_back(SimulationSettings(rep));
//	simsets.push_back(SimulationSettings("gravity", -20, -3, 17, rep));
//	simsets.push_back(SimulationSettings("sim_stepsize", 1./600, 1./30, 30, rep));
//	simsets.push_back(SimulationSettings("solver_iterations", 1, 15, 15, rep));
//	simsets.push_back(SimulationSettings("solver_mode_randomize", rep));
//	simsets.push_back(SimulationSettings("solver_mode_friction_separate", rep));
//	simsets.push_back(SimulationSettings("solver_mode_use_warmstarting", rep));
//	simsets.push_back(SimulationSettings("solver_mode_use_friction_warmstarting", rep));
//	simsets.push_back(SimulationSettings("solver_mode_use_2_friction_directions", rep));
//	simsets.push_back(SimulationSettings("solver_mode_enable_friction_direction_caching", rep));
//	simsets.push_back(SimulationSettings("solver_mode_disable_velocity_dependent_friction", rep));
//	simsets.push_back(SimulationSettings("world_scaling_factor", 1, 20, 20, rep));
//	simsets.push_back(SimulationSettings("split_impulse", rep));
//	simsets.push_back(SimulationSettings("erp", 0, 1, 15, rep));
//	simsets.push_back(SimulationSettings("tau", 0, 1, 15, rep));
//	simsets.push_back(SimulationSettings("friction_polygon", 0.05, 1, 15, rep));
//	simsets.push_back(SimulationSettings("friction_ground", 0.15, 3, 15, rep));
//	simsets.push_back(SimulationSettings("friction_pusher", 0.15, 3, 15, rep));
//	simsets.push_back(SimulationSettings("restitution_polygon", 0, 1, 15, rep));
//	simsets.push_back(SimulationSettings("restitution_ground", 0, 1, 15, rep));
//	simsets.push_back(SimulationSettings("restitution_pusher", 0, 1, 15, rep));
//	simsets.push_back(SimulationSettings("collision_margin", 0, params["world_scaling_factor"]*TANGRAM_HEIGHT, 15, rep));
//	simsets.push_back(SimulationSettings("inertia_scaling", 0.5, 5, 15, rep));
//	simsets.push_back(SimulationSettings("use_custom_inertia_tensor", rep));
//	simsets.push_back(SimulationSettings("use_modified_shape", rep));
//	simsets.push_back(SimulationSettings("fixed_shape_factor", 0.3, 1, 15, rep));
//	simsets.push_back(SimulationSettings("lin_damping", 0., 0.9, 15, rep));
//	simsets.push_back(SimulationSettings("lin_factor", 0.1, 1., 15, rep));
//	simsets.push_back(SimulationSettings("ang_damping", 0., 0.9, 15, rep));
//	simsets.push_back(SimulationSettings("ang_factor", 0.1, 1., 15, rep));


	vector<PushingSceneInfo> sceneInfos;
	string prefix="opt_";
	// small triangle
	if (0) {
		params["friction_polygon"] = 0.1914;  
		params["friction_pusher"] = 0.5226;
		params["fixed_shape_factor"] = 0.4702;
		sceneInfos.clear();
		loadSceneFromFile("./data/pushing_real_closed_loop/small_triangle/push_data_small.txt", sceneInfos, ST_MASS, Shapes::SMALL_TRIANGLE);
		record_multiple_datasets(string("./data/pushing_sim/small_triangle/") + prefix + "data_", params, simsets, sceneInfos);
	}
	
	// square
	if (0) {
	  params["friction_polygon"] = 0.1246;
		params["friction_pusher"] = 0.7372;
		params["fixed_shape_factor"] = 0.5274;
		sceneInfos.clear();
	  loadSceneFromFile("./data/pushing_real_closed_loop/square/push_data_small.txt", sceneInfos, SQ_MASS, Shapes::SQUARE);
		record_multiple_datasets(string("./data/pushing_sim/square/") + prefix + "data_", params, simsets, sceneInfos);
	}
	
	// parallelogram
	if (0) {
		params["friction_polygon"] = 0.120411;//0.0872;
		params["friction_pusher"] = 0.148613;//0.3458;
		params["fixed_shape_factor"] = 0.681792;//0.7086;
		sceneInfos.clear();
	  loadSceneFromFile("./data/pushing_real_closed_loop/parallelogram/push_data_first.txt", sceneInfos, PA_MASS, Shapes::PARALLELOGRAM);
		record_multiple_datasets(string("./data/pushing_sim/parallelogram/") + prefix + "data_", params, simsets, sceneInfos);
	}
	
	
	// large triangle
	if (1) {
		params["friction_polygon"] = 0.595058;
		params["friction_pusher"] = 0.714329;
		params["fixed_shape_factor"] = 0.565834;
		sceneInfos.clear();
		loadSceneFromFile("./data/pushing_real_closed_loop/large_triangle/push_data_small.txt", sceneInfos, LT_MASS, Shapes::LARGE_TRIANGLE);
		record_multiple_datasets(string("./data/pushing_sim/large_triangle/") + prefix + "data_", params, simsets, sceneInfos);
	}
	return 0;
}

