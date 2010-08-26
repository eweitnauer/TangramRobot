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
int loadTrainData(const string &filename, vector<PushingSceneInfo> &data, Shapes::ShapeType shapeType) {
  ifstream f(filename.c_str());
  string line;
  getline(f, line); // first line has column names
	int counter = 0;
  while (f.good()) {
    getline(f, line);
    vector<float> v = icl::parseVecStr<float>(line, " ");
    if (v.size() != 10) continue;
    if (abs(v[0]-v[3]) < 0.5) continue; // pushing against center of object -> too chaotic
    if (abs(v[0]-v[3]) > 2.5 && abs(v[0]-v[3]) < 3.5) continue;
    if (abs(v[0]-v[3]) > 7.5 && abs(v[0]-v[3]) < 8.5) continue;
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
  static SimulationSettings simsets(1); // 1 repetition
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

/// Returns mean corner distance to target in mm
float calculateError( PhysicsParameters &params,
		vector<PushingSceneInfo> &sceneInfos, double sf_sq, double sf_pa, double sf_st, double sf_lt, float pusher_diam=-1) {
	float error = 0;
	int n = sceneInfos.size();
	static PushingScene scene;
  static SimulationSettings simsets(1); // 1 repetition
	for (int i=0; i<n; i++) {
		PushingSceneInfo &sceneInfo = sceneInfos[i];
		switch (sceneInfo.ttype) {
			case Shapes::SQUARE: params["fixed_shape_factor"] = sf_sq; break;
			case Shapes::PARALLELOGRAM: params["fixed_shape_factor"] = sf_pa; break;
			case Shapes::SMALL_TRIANGLE: params["fixed_shape_factor"] = sf_st; break;
			case Shapes::LARGE_TRIANGLE: params["fixed_shape_factor"] = sf_lt; break;
			default: break;
		}
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

// Polynomial fitting problem
class ParamOptimizer : public DESolver
{
public:
	ParamOptimizer(int dim, int pop, const vector<PushingSceneInfo> &target_data) :
		DESolver(dim,pop), count(0), dim(dim), target_data(target_data) {;}
	double EnergyFunction(double trial[],bool &bAtSolution);
	
	double f1(double trial[], bool &bAtSolution);
	double f2(double trial[], bool &bAtSolution);

private:
	int count;
	int dim;
	vector<PushingSceneInfo> target_data;
};

double ParamOptimizer::EnergyFunction(double *trial,bool &bAtSolution)
{
	if (dim == 3) return f1(trial, bAtSolution);
	else return f2(trial, bAtSolution);
}

double ParamOptimizer::f2(double *trial,bool &bAtSolution) {
	static PhysicsParameters params;
	float frict_poly = trial[0];
	float frict_pusher = trial[1];
	float sq = trial[2];
	float pa = trial[3];
	float st = trial[4];
	float lt = trial[5];
	params["friction_polygon"] = frict_poly;
	params["friction_pusher"] = frict_pusher;
	float result;
	if (frict_poly >= 0 && frict_poly < 2 &&
			frict_pusher >= 0 && frict_pusher <= 2 &&
			sq > 0.3 && sq < 1 && pa > 0.3 && pa < 1 &&
			st > 0.3 && st < 1 &&	lt > 0.3 && lt < 1)
		result = calculateError(params, target_data, sq, pa, st, lt);
	else
		result = 1e20;

	if (count++ % nPop == 0) {
		double *s = Solution();
		printf("%5d frict-poly:%1.4f frict-push:%1.4f sq:%1.4f pa:%1.4f st:%1.4f lt:%1.4f err() = %.2f mm\n",
				count / nPop + 1,  s[0], s[1], s[2], s[3], s[4], s[5], Energy());
	}
	return(result);
}

double ParamOptimizer::f1(double *trial,bool &bAtSolution) {
	static PhysicsParameters params;
	float frict_poly = trial[0];
	float frict_pusher = trial[1];
	float shape_factor = trial[2];
	params["friction_polygon"] = frict_poly;
	params["friction_pusher"] = frict_pusher;
	params["fixed_shape_factor"] = shape_factor;
	float result;
	if (frict_poly >= 0 && frict_poly <= 2 &&
			frict_pusher >= 0 && frict_pusher <= 2 &&
			shape_factor >= 0.3 && shape_factor <= 1)
		result = calculateError(params, target_data);
	else
		result = 1e20;

	if (count++ % nPop == 0) {
		double *s = Solution();
		printf("%5d friction-polygon:%.4f friction-pusher:%.4f shape-factor:%.4f err() = %.2f mm\n",
				count / nPop + 1,  s[0], s[1], s[2], Energy());
	}
	return(result);
}

void optimizeParams(const vector<PushingSceneInfo> &data, bool optimize_shape_factors) {
	psim.init();
	#if VISUALIZE
		psim.setFastForward(2);
		icl::ExecThread y(show_physics_gui);
		y.run(false); // no loop
	#endif
	
	int N_DIM, N_POP, MAX_GENERATIONS;
	if (optimize_shape_factors)	N_DIM = 6;
	else N_DIM = 3;
	N_POP = N_DIM*10;
	MAX_GENERATIONS	= 1000;
	
	double min[N_DIM];
	double max[N_DIM];

	ParamOptimizer optimizer(N_DIM,N_POP,data);

	if (optimize_shape_factors) {
		min[0] = 0; max[0] = 2;	// friction polygon
		min[1] = 0; max[1] = 2;	// friction pusher
		min[2] = 0.3; max[2] = 1;   // shape factor sq
		min[3] = 0.3; max[3] = 1;   // shape factor pa
		min[4] = 0.3; max[4] = 1;   // shape factor st
		min[5] = 0.3; max[5] = 1;   // shape factor lt
	} else {
		min[0] = 0; max[0] = 2;	// friction polygon
		min[1] = 0; max[1] = 2;	// friction pusher
		min[2] = 0.3; max[2] = 1;   // shape factor
	}


	optimizer.Setup(min,max,2,0.8,0.9);
	
	printf("Calculating...\n\n");
	optimizer.Solve(MAX_GENERATIONS);

	double *solution = optimizer.Solution();

	printf("\n\nBest Coefficients:\n");
	for (int i=0;i<N_DIM;i++)
		printf("[%d]: %lf\n",i,solution[i]);
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

	string filename = "push_data.txt";
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
  	int count = loadTrainData(data_files[i], data, shape_types[i]);
  	cout << "OK (" << count << ") trials loaded" << endl;
	}
	cout << "in total, " << data.size() << " trials were loaded." << endl;
  if (data.size() < 2) {
    cout << "Number of trials too small, exiting..." << endl;
    return 0;
  }
  cout << "starting to optimize..." << endl;
  optimizeParams(data, name=="all");  
  return 0;
}

