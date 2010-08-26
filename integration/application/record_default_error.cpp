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

// returns the stddev of tangram end positions in identical trials averaged over all groups of identical trials
float calculateStdDevReal(vector< vector<PushingSceneInfo> > &h_data) {
  float std_dev = 0;
  int total_n = 0;
	for (unsigned int dt=0; dt<h_data.size(); ++dt) {
	  vector<PushingSceneInfo> &data = h_data[dt];
	  if (data.size() == 0) continue;
		PolygonShape ps = PolygonShape(data[0].tcorners);
    float x=0, y=0, rot=0;
    int n = data.size();
    for (int i=0; i<n; ++i) {
	    rot += data[i].trot1-data[i].trot0;
	    x += data[i].tx1-data[i].tx0;
	    y += data[i].ty1-data[i].ty0;
    }
    x = x/n; y = y/n; rot = rot/n;
    Transformation t_mean(rot,x,y);
    PolygonShape ps_mean = t_mean*ps;

    float var = 0;
    for (int i=0; i<n; ++i) {
	    Transformation t_curr(data[i].trot1-data[i].trot0, data[i].tx1-data[i].tx0, data[i].ty1-data[i].ty0);
      float dist = ps_mean.getMeanCornerDistance(t_curr*ps);
	    var += dist*dist;
    }
    std_dev += sqrt(var/n); 
    total_n++;
  }
	return 1000.*std_dev/total_n;
}

// returns the mean corner distance of the 5 end positions in the identical trials
float calculateMinimalError(vector< vector<PushingSceneInfo> > &h_data) {
  float total_min_err = 0;
  int total_n = 0;
	for (unsigned int dt=0; dt<h_data.size(); ++dt) {
	  vector<PushingSceneInfo> &data = h_data[dt];
	  if (data.size() == 0) continue;
		PolygonShape ps = PolygonShape(data[0].tcorners);
    float x=0, y=0, rot=0;
    int n = data.size();
    for (int i=0; i<n; ++i) {
	    rot += data[i].trot1-data[i].trot0;
	    x += data[i].tx1-data[i].tx0;
	    y += data[i].ty1-data[i].ty0;
    }
    x = x/n; y = y/n; rot = rot/n;
    Transformation t_mean(rot,x,y);
    PolygonShape ps_mean = t_mean*ps;

    float min_err = 0;
    for (int i=0; i<n; ++i) {
	    Transformation t_curr(data[i].trot1-data[i].trot0, data[i].tx1-data[i].tx0, data[i].ty1-data[i].ty0);
      float dist = ps_mean.getMeanCornerDistance(t_curr*ps);
	    min_err += dist;
    }
    total_n += n;
    total_min_err += min_err;
  }
	return 1000.*total_min_err/total_n;
}

/// Returns corner distance from start to end position in mm
float calculateMaximalError(vector<PushingSceneInfo> &sceneInfos) {
	float error = 0;
	int n = sceneInfos.size();
	for (int i=0; i<n; i++) {
    PushingSceneInfo &data = sceneInfos[i];
		PolygonShape ps = PolygonShape(data.tcorners);
		Transformation t_curr(data.trot1-data.trot0, data.tx1-data.tx0, data.ty1-data.ty0);
		error += ps.getMeanCornerDistance(t_curr*ps);
	}
	return 1000. * error / n;
}

/// Returns mean corner distance to target in mm
float calculateError( PhysicsParameters &params,
		vector<PushingSceneInfo> &sceneInfos, float pusher_diam=-1) {
	float error = 0;
	int n = sceneInfos.size();
	static PushingScene scene;
  static SimulationSettings simsets(4); // 4 repetitions
	for (int i=0; i<n; i++) {
		PushingSceneInfo &sceneInfo = sceneInfos[i];
		float real_dx = (sceneInfo.tx1-sceneInfo.tx0);
		float real_dy = (sceneInfo.ty1-sceneInfo.ty0);
		float real_drot = (sceneInfo.trot1-sceneInfo.trot0);
		prec.simulateSingleParameterSetting(scene, simsets, params, sceneInfo);
		scene.calcStatistics(Transformation(real_drot, real_dx, real_dy));
		
		float dist = scene.pbodies[0].getDistanceToReference();
		error += dist;
		
		PolygonShape pstart(sceneInfo.tcorners);
		Transformation treal(real_drot,real_dx,real_dy);
	}
	return 1000. * error / n;
}

#if VISUALIZE
	void show_physics_gui() {
		glutmain(0, NULL, 640, 480, "Minimal Visualization Example", &psim);
	}
#endif

void init() {
  psim.init();
	#if VISUALIZE
		psim.setFastForward(2);
		icl::ExecThread y(show_physics_gui);
		y.run(false); // no loop
	#endif
}

void collectFilenames(string name, vector<string> &data_files, vector<Shapes::ShapeType> &shape_types) {
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
}

vector<PushingSceneInfo> loadScenesByName(string name, vector<float> data_selector) {
  vector<PushingSceneInfo> data;
  
  vector<string> data_files;
  vector<Shapes::ShapeType> shape_types;
  collectFilenames(name, data_files, shape_types);
	
  for (unsigned int i=0; i<data_files.size(); ++i) {
		// read real pushing results from file
  	//cout << "loading data from file " << data_files[i] << "...";
  	//int count = loadData(data_files[i], data, shape_types[i], data_selector);
  	//cout << "OK (" << count << ") trials loaded" << endl;
  	loadData(data_files[i], data, shape_types[i], data_selector);
	}
	//cout << "in total, " << data.size() << " trials were loaded." << endl;
  return data;
}

vector< vector<PushingSceneInfo> > loadScenesByNameHierachical(string name, vector<float> data_selector) {
  vector< vector<PushingSceneInfo> > data;
  vector<string> data_files;
  vector<Shapes::ShapeType> shape_types;
  collectFilenames(name, data_files, shape_types);
	
  for (unsigned int ds=0; ds<data_selector.size(); ++ds) {
    vector<float> single_data_selector;
    single_data_selector.push_back(data_selector[ds]);
    for (unsigned int i=0; i<data_files.size(); ++i) {
		  // read real pushing results from file
		  data.push_back(vector<PushingSceneInfo>());
  	  //int count = loadData(data_files[i], data.back(), shape_types[i], single_data_selector);
  	  //cout << "OK (" << count << ") trials loaded" << endl;
  	  loadData(data_files[i], data.back(), shape_types[i], single_data_selector);
	  }
  }
  //cout << "in total, " << data.size() << " trial types were loaded." << endl;
  return data;
}

int main(int argc, char **argv) {
  init();

	vector<float> train_data_selector;
  train_data_selector.push_back(1);train_data_selector.push_back(5);train_data_selector.push_back(11);
  vector<float> test_data_selector;
  test_data_selector.push_back(3);test_data_selector.push_back(8);
  vector<float> all_data_selector;
  all_data_selector.push_back(1); all_data_selector.push_back(3); all_data_selector.push_back(5); 
  all_data_selector.push_back(8); all_data_selector.push_back(11); 
  vector<float> x1_selector;
  x1_selector.push_back(1);
  PhysicsParameters params; params["collision_margin"] = 0.005;
  vector<PushingSceneInfo> scene_infos;
  vector< vector<PushingSceneInfo> > h_scene_infos;

  vector<string> types;
  types.push_back("sq"); types.push_back("pa"); types.push_back("st"); types.push_back("lt"); types.push_back("all");
  
  for (unsigned int i=0; i<types.size(); i++) {
    string type = types[i];
    
//    if (type=="lt") {
//      params["friction_polygon"] = 0.6213;//0.626651;
//      params["friction_pusher"] = 0.6932;//0.670126;
//      params["fixed_shape_factor"] = 0.5754;//0.573171;
//    } else if (type == "st") {
//      params["friction_polygon"] = 0.1914;
//      params["friction_pusher"] = 0.5226;
//      params["fixed_shape_factor"] = 0.4702;
//    } else if (type == "sq") {
//      params["friction_polygon"] = 0.1269;
//      params["friction_pusher"] = 0.6998;
//      params["fixed_shape_factor"] = 0.5437;
//    } else if (type == "pa") {
//      params["friction_polygon"] = 0.115235;//0.0872;
//      params["friction_pusher"] = 0.456686;//0.3458;
//      params["fixed_shape_factor"] = 0.681108;//0.7086;
//    } else if (type == "all") {
//      params["friction_polygon"] = 0.5470;
//      params["friction_pusher"] = 0.7225;
//      params["fixed_shape_factor"] = -1;
//      Shapes::setCorrectShapeFactors(0.6166,0.7197,0.5388,1,0.5911);
//    }
    
    scene_infos = loadScenesByName(type,train_data_selector);
    h_scene_infos = loadScenesByNameHierachical(type,train_data_selector);
    cout << "[" << type << "] training error: " << calculateError(params, scene_infos)*0.1 << " cm." << endl;
    cout << "            minimal: " << calculateMinimalError(h_scene_infos) *0.1 << " cm." << endl;
    cout << "            std dev: " << calculateStdDevReal(h_scene_infos) *0.1 << " cm." << endl;
    cout << "            maximal: " << calculateMaximalError(scene_infos) *0.1 << " cm." << endl;
    scene_infos = loadScenesByName(type,test_data_selector);
    h_scene_infos = loadScenesByNameHierachical(type,test_data_selector);
    cout << "[" << type << "]  testing error: " << calculateError(params, scene_infos)*0.1 << " cm." << endl;
    cout << "            minimal: " << calculateMinimalError(h_scene_infos) *0.1 << " cm." << endl;
    cout << "            std dev: " << calculateStdDevReal(h_scene_infos) *0.1 << " cm." << endl;
    cout << "            maximal: " << calculateMaximalError(scene_infos) *0.1 << " cm." << endl;
    scene_infos = loadScenesByName(type,all_data_selector);
    h_scene_infos = loadScenesByNameHierachical(type,all_data_selector);
    cout << "[" << type << "]    total error: " << calculateError(params, scene_infos)*0.1 << " cm." << endl;
    cout << "            minimal: " << calculateMinimalError(h_scene_infos) *0.1 << " cm." << endl;
    cout << "            std dev: " << calculateStdDevReal(h_scene_infos) *0.1 << " cm." << endl;
    cout << "            maximal: " << calculateMaximalError(scene_infos) *0.1 << " cm." << endl;
  }
 
  return 0;
}

