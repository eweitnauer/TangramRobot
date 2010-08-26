#include <PushingScene.h>
#include <PushedBody.h>
#include <PushingSimulatorFast.h>
#include <PushingSimulatorGui.h>
#include <ICLUtils/ThreadUtils.h>
#include <Shapes.h>
#include <DESolver.h>
#include <PushingRecorder.h>
#include "gsl/gsl_multimin.h"

#include <ICLUtils/StringUtils.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <ICLUtils/StackTimer.h>

using namespace std;
using namespace icl;

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

struct ParamStruct {
	vector<PushingSceneInfo> sceneInfos;
	
	ParamStruct(vector<PushingSceneInfo> sceneInfos):
		sceneInfos(sceneInfos) {}
};

/// Returns number of loaded entries
int loadData(const string &filename, vector<PushingSceneInfo> &data, Shapes::ShapeType shapeType, vector<float> data_selector) {
  ifstream f(filename.c_str());
  string line;
  getline(f, line); // first line has column names
	int counter = 0;
  while (f.good()) {
    getline(f, line);
    vector<float> v = icl::parseVecStr<float>(line, " ");
    if (v.size() != 10) continue;
    bool use_this = false;
    for (unsigned int i=0; i<data_selector.size(); i++) {
    	if (data_selector[i]-0.5 < abs(v[0]-v[3]) && data_selector[i]+0.5 > abs(v[0]-v[3])) {
    		use_this = true; break;
    	}
    }
    if (!use_this) continue;
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
float calculateError(PhysicsParameters &params,
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
  static SimulationSettings simsets(5); // 5 repetition
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

double EnergyFunction(const gsl_vector *v, void *paramStruct)
{
	ParamStruct *ps = (ParamStruct*)paramStruct;
	static PhysicsParameters params;
	float p0 = gsl_vector_get(v, 0);
	float p1 = gsl_vector_get(v, 1);
	float p2 = gsl_vector_get(v, 2);
	params["friction_polygon"] = p0;
	params["friction_pusher"] = p1;
	params["fixed_shape_factor"] = p2;
	params["collision_margin"] = 0.005;
//	params["world_scaling_factor"] = 10;
	
	if (p0 > 0 && p0 < 2 && p1 > 0 && p1 < 2 && p2 >= 0.3 && p2 <= 1)
		return calculateError(params, ps->sceneInfos);
	else
		return 1e20;
}

double EnergyFunctionCustomShapeFactor(const gsl_vector *v, void *paramStruct)
{
	ParamStruct *ps = (ParamStruct*)paramStruct;
	static PhysicsParameters params;
	float p0 = gsl_vector_get(v, 0);
	float p1 = gsl_vector_get(v, 1);
	float sq = gsl_vector_get(v, 2);
	float pa = gsl_vector_get(v, 3);
	float st = gsl_vector_get(v, 4);
	float lt = gsl_vector_get(v, 5);
	params["friction_polygon"] = p0;
	params["friction_pusher"] = p1;
	params["collision_margin"] = 0.005;
	if (p0 > 0 && p0 < 2 && p1 > 0 && p1 < 2 &&
			sq > 0.3 && sq <= 1 && pa > 0.3 && pa <= 1 &&
			st > 0.3 && st <= 1 &&	lt > 0.3 && lt <= 1)
		return calculateError(params, ps->sceneInfos, sq, pa, st, lt);
	else
		return 1e20;
}

void optimizeParams(const vector<PushingSceneInfo> &train_data, const vector<PushingSceneInfo> &test_data, const vector<PushingSceneInfo> &all_data, bool optimize_shape_factors, ostream &out) {
	psim.init();
	#if VISUALIZE
		psim.setFastForward(2);
		icl::ExecThread y(show_physics_gui);
		y.run(false); // no loop
	#endif
	ParamStruct paramStructTrain(train_data);
	ParamStruct paramStructTest(test_data);
	ParamStruct paramStructAll(all_data);
	
	const gsl_multimin_fminimizer_type *T = gsl_multimin_fminimizer_nmsimplex;
	gsl_multimin_fminimizer *s = NULL;
	gsl_vector *ss, *x;
	gsl_multimin_function minex_func;

	size_t iter = 0;
	int status;
	double size;

	/* Starting point */
	if (optimize_shape_factors) {
		x = gsl_vector_alloc (6);
		gsl_vector_set(x, 0, 0.3);
		gsl_vector_set(x, 1, 0.7);
		gsl_vector_set(x, 2, 0.5);
		gsl_vector_set(x, 3, 0.7);
		gsl_vector_set(x, 4, 0.45);
		gsl_vector_set(x, 5, 0.56);
//		gsl_vector_set(x, 0, 0.547);
//		gsl_vector_set(x, 1, 0.7225);
//		gsl_vector_set(x, 2, 0.6166);
//		gsl_vector_set(x, 3, 0.7197);
//		gsl_vector_set(x, 4, 0.5388);
//		gsl_vector_set(x, 5, 0.5911);
	} else {
		x = gsl_vector_alloc (3);
	  gsl_vector_set(x, 0, 0.6213);
    gsl_vector_set(x, 1, 0.6932);
    gsl_vector_set(x, 2, 0.5754);
	}
	
	 /* Set initial step sizes */
 	if (optimize_shape_factors) {
		ss = gsl_vector_alloc (6);
		gsl_vector_set_all(ss, 0.2);
	} else {
		ss = gsl_vector_alloc (3);
		gsl_vector_set_all(ss, 0.2);
	}


	/* Initialize method and iterate */
 	if (optimize_shape_factors) {
 		minex_func.n = 6;
 		minex_func.f = EnergyFunctionCustomShapeFactor;
		minex_func.params = &paramStructTrain;
		s = gsl_multimin_fminimizer_alloc (T, 6);
 	} else { 
		minex_func.n = 3;
		minex_func.f = EnergyFunction;
		minex_func.params = &paramStructTrain;
		s = gsl_multimin_fminimizer_alloc (T, 3);
	}

	gsl_multimin_fminimizer_set (s, &minex_func, x, ss);

	if (optimize_shape_factors) {
		out << "frict_poly frict_pusher sf_sq sf_pa sf_st sf_lt train_error test_error total_error" << endl;
	} else {
		out << "frict_poly frict_pusher shape_factor train_error test_error total_error" << endl;
	}
	do {
     iter++;
     status = gsl_multimin_fminimizer_iterate(s);
     
     if (status) 
       break;

     size = gsl_multimin_fminimizer_size (s);
     status = gsl_multimin_test_size (size, 1e-4);

     if (status == GSL_SUCCESS)
       {
         printf ("converged to minimum at\n");
       }
     
     if (optimize_shape_factors) {
	     printf ("%5d frict-poly:%1.5f frict-pusher:%1.5f sq:%1.5f pa:%1.5f st:%1.5f lt:%1.5f err() = %.2f mm size = %.1e\n", 
             iter,
             gsl_vector_get (s->x, 0), 
             gsl_vector_get (s->x, 1), 
             gsl_vector_get (s->x, 2), 
             gsl_vector_get (s->x, 3), 
             gsl_vector_get (s->x, 4), 
             gsl_vector_get (s->x, 5), 
             s->fval, size);
       float error_train = EnergyFunctionCustomShapeFactor(s->x, (void*)&paramStructTrain);
       float error_test = EnergyFunctionCustomShapeFactor(s->x, (void*)&paramStructTest);
       float error_total = EnergyFunctionCustomShapeFactor(s->x, (void*)&paramStructAll);
       out << gsl_vector_get (s->x, 0) << " " << gsl_vector_get (s->x, 1) << " "
       		 << gsl_vector_get (s->x, 2) << " " << gsl_vector_get (s->x, 3) << " "
       		 << gsl_vector_get (s->x, 4) << " " << gsl_vector_get (s->x, 5) << " "
       		 << error_train << " " << error_test << " " << error_total << endl;       
     } else {
		   printf ("%5d frict-poly:%g frict-pusher: %g shape-factor:%g err() = %.2f mm size = %.1e\n", 
             iter,
             gsl_vector_get (s->x, 0), 
						 gsl_vector_get (s->x, 1), 
             gsl_vector_get (s->x, 2), 
             s->fval, size);
       float error_train = EnergyFunction(s->x, (void*)&paramStructTrain);
       float error_test = EnergyFunction(s->x, (void*)&paramStructTest);
       float error_total = EnergyFunction(s->x, (void*)&paramStructAll);
       out << gsl_vector_get (s->x, 0) << " " << gsl_vector_get (s->x, 1) << " "
       		 << gsl_vector_get (s->x, 2) << " " << error_train << " " << error_test << " "
       		 << error_total << endl;       
     }
	}
	while (status == GSL_CONTINUE && iter < 1000);
 
	gsl_vector_free(x);
	gsl_vector_free(ss);
	gsl_multimin_fminimizer_free (s);
}

int main(int argc, char **argv) {
  if (argc != 3) {
    cout << "usage: " << argv[0] << " <shape-type> <output-filename>" << endl;
    cout << "  shape-types: all, sq, pa, st, mt, lt." << endl;
    return -1;
  }
  string name(argv[1]);

  // CAUTION: next two variables must stay consistent
  vector<string> data_files;
  vector<Shapes::ShapeType> shape_types;

	string filename = "push_data.txt";
	if (name == "all") filename = "push_data_15.txt";
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
  
 	vector<float> all_data_selector;
  all_data_selector.push_back(1);all_data_selector.push_back(3);all_data_selector.push_back(5);all_data_selector.push_back(8);all_data_selector.push_back(11);
 	vector<float> train_data_selector;
  train_data_selector.push_back(1);train_data_selector.push_back(5);train_data_selector.push_back(11);
  vector<float> test_data_selector;
  test_data_selector.push_back(3);test_data_selector.push_back(8);
	vector<PushingSceneInfo> train_data, test_data, all_data;
	for (unsigned int i=0; i<data_files.size(); ++i) {
		// read real pushing results from file
  	loadData(data_files[i], train_data, shape_types[i], train_data_selector);
  	loadData(data_files[i], test_data, shape_types[i], test_data_selector);
  	loadData(data_files[i], all_data, shape_types[i], all_data_selector);
	}
	cout << "in total, " << train_data.size() << " training and " << test_data.size() << " testing trials were loaded." << endl;
  if (train_data.size() < 2 || test_data.size() < 2) {
    cout << "Number of trials too small, exiting..." << endl;
    return 0;
  }
  cout << "starting to optimize..." << endl;
  ofstream out(argv[2]);
  optimizeParams(train_data, test_data, all_data, name=="all", out);  
//  gsl_vector *x;
//  x = gsl_vector_alloc (3);
//  gsl_vector_set(x, 0, 0.124002);
//  gsl_vector_set(x, 1, 0.574365);
//  gsl_vector_set(x, 2, 0.678649);
//  ParamStruct paramStructTrain(train_data);
//  ParamStruct paramStructTest(test_data);
//  ParamStruct paramStructAll(all_data);
//  psim.init();
//  cout << "[pa] training error: " << EnergyFunction(x,(void*)&paramStructTrain) << endl;
//  cout << "[pa]     test error: " << EnergyFunction(x,(void*)&paramStructTest) << endl;
//  cout << "[pa]    total error: " << EnergyFunction(x,(void*)&paramStructAll) << endl;
  return 0;
}

