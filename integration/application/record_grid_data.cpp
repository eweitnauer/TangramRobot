#include <PushingScene.h>
#include <PushedBody.h>
#include <PushingSimulatorFast.h>
#include <PushingSimulatorGui.h>
#include <ICLUtils/ThreadUtils.h>
#include <Shapes.h>
#include <DESolver.h>
#include <PushingRecorder.h>

#include <ICLUtils/StringUtils.h>
#include <vector>
#include <iostream>
#include <fstream>


#define VISUALIZE 0

#if VISUALIZE
PushingSimulatorGui psim;
#else
PushingSimulatorFast psim;
#endif

#define TANGRAM_HEIGHT 0.018 // 1.8 cm
#define TANGRAM_LENGTH 0.096  // 9.6 cm
#define TANGRAM_MASS 0.180 // 180 g
#define PUSHER_SPEED 0.05 // 5 cm / s
#define PUSHER_DIAMETER 0.019 // 1.9 cm

PushingRecorder prec(&psim);

using namespace std;

/// Returns number of loaded entries
int loadData(const string &filename, vector<PushingSceneInfo> &data, Shapes::ShapeType shapeType) {
  ifstream f(filename.c_str());
  string line;
  getline(f, line); // first line has column names
	int counter = 0;
  while (f.good()) {
    getline(f, line);
    vector<float> v = icl::parseVecStr<float>(line, " ");
    if (v.size() != 10) continue;
    if (abs(v[0]-v[3]) < 0.5) continue; // pushing against center of object -> too chaotic
    // file data columns: tx0 ty0 trot0 ax0 ay0 ax1 ay1 tx1 ty1 trot1
    PushingSceneInfo si;
    si.setTangramPos(v[0]/100,v[1]/100,v[2]/180*M_PI,v[7]/100,v[8]/100,v[9]/180*M_PI);
    si.setPusherPos(v[3]/100,v[4]/100,v[5]/100,v[6]/100);
		si.tmass = TANGRAM_MASS;
		si.tlength = TANGRAM_LENGTH;
		si.theight = TANGRAM_HEIGHT;
		si.pspeed = PUSHER_SPEED;
		si.pdiam = PUSHER_DIAMETER;
		si.tcorners = Shapes::getCorners(shapeType, TANGRAM_LENGTH);
		si.ttype = shapeType;
		data.push_back(si);
		counter++;
  }
  f.close();
  return counter;
}

/// Returns mean corner distance to target in mm
float calculateError( PhysicsParameters &params,
		vector<PushingSceneInfo> &sceneInfos, float pusher_diam=-1) {
	float error = 0;
	int n = sceneInfos.size();
	static PushingScene scene;
  static SimulationSettings simsets(2); // 4 repetitions
	for (int i=0; i<n; i++) {
		PushingSceneInfo &sceneInfo = sceneInfos[i];
		float real_dx = (sceneInfo.tx1-sceneInfo.tx0);
		float real_dy = (sceneInfo.ty1-sceneInfo.ty0);
		float real_drot = (sceneInfo.trot1-sceneInfo.trot0);
		float old_diam = sceneInfo.pdiam;
		if (pusher_diam != -1) sceneInfo.pdiam = pusher_diam;
		prec.simulateSingleParameterSetting(scene, simsets, params, sceneInfo);
		if (pusher_diam != -1) sceneInfo.pdiam = old_diam;
		scene.calcStatistics(Transformation(real_drot, real_dx, real_dy));

		float dist = scene.pbodies[0].getDistanceToReference();
		error += dist;
	}
	return 1000. * error / n;
}

#if VISUALIZE
	void show_physics_gui() {
		glutmain(0, NULL, 640, 480, "Minimal Visualization Example", &psim);
	}
#endif

/** Simulates the scenes for all combinations of the two passed parameter ranges
  * and writes mean errors for each parameter combination to the passes vector
  * references. */
void record_grid(vector<float> &par1, vector<float> &par2, vector<float> &errors,
		const SimulationSettings &simset1, const SimulationSettings &simset2,
		const PhysicsParameters &params_const, vector<PushingSceneInfo> &sceneInfos) {
	PhysicsParameters params = params_const;
//	int N = simset1.steps * simset2.steps;
	for (int i=0; i<simset1.steps; i++) {
		cout << "[" << (i+1)*100/simset1.steps << "%]" << endl;
		for (int j=0; j<simset2.steps; j++) {
			float value1 = simset1.getValue(i);
			float value2 = simset2.getValue(j);
			params[simset1.param_name] = value1;
			params[simset2.param_name] = value2;
			float error = calculateError(params, sceneInfos);
			par1.push_back(value1);
			par2.push_back(value2);
			errors.push_back(error);
		}
	}
}

void write_grid(string filename, string parname1, const vector<float> &par1,
		string parname2, const vector<float> &par2, const vector<float> &errors) {
	ofstream out(filename.c_str());
	out << parname1 << " " << parname2 << " corner_dist" << endl;
	for (unsigned int i=0; i<par1.size(); i++) {
		out << par1[i] << " " << par2[i] << " " << errors[i] << endl;
	}
	out.close();
}

int main(int argc, char **argv) {
  if (argc != 2) {
    cout << "usage: " << argv[0] << " <shape-type>" << endl;
    cout << "  shape-types: all, sq, pa, st, mt, lt." << endl;
    return -1;
  }
  string name(argv[1]);

  // CAUTION: next two variables must stay consistent
  vector<string> data_files;
  vector<Shapes::ShapeType> shape_types;

	string filename = "push_data_small.txt";
  if (name == "all" || name == "sq") {
  	data_files.push_back(string("./data/pushing_real_closed_loop/square/")+filename);
  	shape_types.push_back(Shapes::SQUARE);
  }
  if (name == "all" || name == "pa") {
  	data_files.push_back(string("./data/pushing_real_closed_loop/parallelogram/")+filename);
  	shape_types.push_back(Shapes::PARALLELOGRAM);
  }
  if (name == "all" || name == "lt") {
  	data_files.push_back(string("./data/pushing_real_closed_loop/large_triangle/")+filename);
  	shape_types.push_back(Shapes::LARGE_TRIANGLE);
  }
  if (name == "all" || name == "mt") {
  	data_files.push_back(string("./data/pushing_real_closed_loop/medium_triangle/")+filename);
  	shape_types.push_back(Shapes::MEDIUM_TRIANGLE);
  }
  if (name == "all" || name == "st") {
  	data_files.push_back(string("./data/pushing_real_closed_loop/small_triangle/")+filename);
  	shape_types.push_back(Shapes::SMALL_TRIANGLE);
  }
  
	vector<PushingSceneInfo> data;
	for (unsigned int i=0; i<data_files.size(); ++i) {
		// read real pushing results from file
  	cout << "loading data from file " << data_files[i] << "...";
  	int count = loadData(data_files[i], data, shape_types[i]);
  	cout << "OK (" << count << ") trials loaded" << endl;
	}
	cout << "in total, " << data.size() << " trials were loaded." << endl;
  if (data.size() < 2) {
    cout << "Number of trials too small, exiting..." << endl;
    return 0;
  }
  cout << "starting to simulate..." << endl;
  vector<float> par1, par2, errors;
  SimulationSettings simset1("friction_polygon", 0.05, 1, 50, 4);
  SimulationSettings simset2("fixed_shape_factor", 0.3, 1, 50, 4);
//  SimulationSettings simset2("collision_margin", 0, 0.02, 20, 4);
  PhysicsParameters params;
  
  psim.init();
	#if VISUALIZE
		psim.setFastForward(2);
		icl::ExecThread y(show_physics_gui);
		y.run(false); // no loop
	#endif
	string outfile;
	
	par1.clear(); par2.clear(); errors.clear();
	params["friction_pusher"] = 0;
  record_grid(par1, par2, errors, simset1, simset2, params, data);
  outfile = string("./pf0_grid_error_50x50_") + name + ".txt";
  write_grid(outfile, simset1.param_name, par1, simset2.param_name, par2, errors);

	par1.clear(); par2.clear(); errors.clear();
	params["friction_pusher"] = 0.3;
  record_grid(par1, par2, errors, simset1, simset2, params, data);
  outfile = string("./pf3_grid_error_50x50_") + name + ".txt";
  write_grid(outfile, simset1.param_name, par1, simset2.param_name, par2, errors);

	par1.clear(); par2.clear(); errors.clear();
	params["friction_pusher"] = 0.8;
  record_grid(par1, par2, errors, simset1, simset2, params, data);
  outfile = string("./pf8_grid_error_50x50_") + name + ".txt";
  write_grid(outfile, simset1.param_name, par1, simset2.param_name, par2, errors);

  return 0;
}

