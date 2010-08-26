#include <PushingRecorder.h>
#include <PushingSimulatorDebug.h>
#include <LinearMath/btDefaultMotionState.h>
#include <vector>

using namespace std;

void PushingRecorder::simulate(PushingScene &scene,
		const PhysicsParameters &params, const SimulationSettings &simsets) {
	psim->setPhysicsParameters(params);
	vector<btRigidBody*> bodies = scene.getBodies();
	
	PushingSimulatorDebug *psim_debug = dynamic_cast<PushingSimulatorDebug*>(psim);
	for (int n=0; n<simsets.repetitions; n++) {
  	scene.init();
		if (psim_debug) psim_debug->openFileLogStream();
		psim->simulate(scene.push, bodies, simsets.before_time_s, simsets.after_time_s);
		if (psim_debug) psim_debug->closeFileLogStream();
		scene.record();
	}		
}

void PushingRecorder::simulateSingleParameterSetting(PushingScene &scene,
		btCollisionShape *scaled_shape, btVector3 localInertia, const SimulationSettings &simsets,
		const PhysicsParameters &params, const PushingSceneInfo &sceneInfo) {
	const float &scaling = params["world_scaling_factor"];
	float h = scaling*sceneInfo.theight;
	VisionAdapter adapter(scaling);
	btTransform trans = adapter.to_bullet(
		Transformation(sceneInfo.trot0, sceneInfo.tx0, sceneInfo.ty0), 
		sceneInfo.theight*0.5, params["collision_margin"]);
	btVector3 push_begin(sceneInfo.px0*scaling,h*0.25,sceneInfo.py0*scaling);
	btVector3 push_end(sceneInfo.px1*scaling,h*0.25,sceneInfo.py1*scaling);
	btVector3 pusher_dims(0.5*sceneInfo.pdiam*scaling,h*3.,0.5*sceneInfo.pdiam*scaling);
	
	btRigidBody *body = createDynamicRigidBody(trans, scaled_shape, sceneInfo.tmass, localInertia);
	PushedBody pbody(body, PolygonShape(sceneInfo.tcorners), scaling);
	scene.push = PushMovement(push_begin, push_end, pusher_dims, sceneInfo.pspeed*scaling);
	scene.clearBodies();
	scene.pbodies.push_back(pbody);
	
	scene.resetStatistics();
	simulate(scene, params, simsets);
}


void PushingRecorder::simulateSingleParameterSetting(PushingScene &scene,
		const SimulationSettings &simsets, const PhysicsParameters &params,
		const PushingSceneInfo &sceneInfo, float shapeFactor) {
	// init helper variables
	float scaling = params["world_scaling_factor"];
	float h = scaling*sceneInfo.theight;
	float l = scaling*sceneInfo.tlength;
	float margin = params["collision_margin"];

	// create collision shape
	btCollisionShape *shape = Shapes::createFlatObject(
		sceneInfo.tcorners, scaling, h, shapeFactor);
	
	// calculate inertia
	shape->setMargin(margin);
	btVector3 inertia(0,0,0);
	if (params["use_custom_inertia_tensor"]) {
		inertia = Shapes::getInertiaTensor(sceneInfo.ttype, l+2*margin, h+margin, sceneInfo.tmass);
	} else {	
		// Bullet's local inertia calculation depends on the collsion margin, which we
		// change so we also need to update the bounding box of the shape.
		if (btConvexHullShape* chshape = dynamic_cast<btConvexHullShape*>(shape)) {
			chshape->recalcLocalAabb();
		}
	  shape->calculateLocalInertia(sceneInfo.tmass, inertia);
	}
  inertia *= params["inertia_scaling"];
	
	// do the actual simulation
	simulateSingleParameterSetting(scene, shape, inertia, simsets, params, sceneInfo);
	delete shape;
}

void PushingRecorder::simulateSingleParameterSetting(
		PushingScene &scene, const SimulationSettings &simsets,
		const PhysicsParameters &params, const PushingSceneInfo &sceneInfo) {
	float shapeFactor = 1;
	if (params["use_modified_shape"]) {
		float sf = params["fixed_shape_factor"];
		if (sf > 0) shapeFactor = sf;
		else shapeFactor = Shapes::getCorrectShapeFactor(sceneInfo.ttype);
	}
	simulateSingleParameterSetting(scene, simsets, params, sceneInfo, shapeFactor);
}
		
void PushingRecorder::writeDataset(ostream &out,
		const SimulationSettings &simsets, const PhysicsParameters &params_const,
		const PushingSceneInfo &sceneInfo, Transformation reference_t, bool writeHeader) {
	PhysicsParameters params = params_const;
	if (writeHeader) PushingScene::writeStatisticsHeader(out, params);
	PushingScene scene;
	if (simsets.param_name == "") { // no parameter to vary
		simulateSingleParameterSetting(scene, simsets, params, sceneInfo);
		scene.calcStatistics(reference_t);
		scene.writeStatistics(out, params);
	} else { // a parameter to vary
		for (int i=0; i<simsets.steps; i++) {
			float value = simsets.getValue(i);
			params[simsets.param_name] = value;
			simulateSingleParameterSetting(scene, simsets, params, sceneInfo);
			scene.calcStatistics(reference_t);
			scene.writeStatistics(out, params);
		}
	}
}

btRigidBody* PushingRecorder::createDynamicRigidBody(btTransform transform,
		btCollisionShape* shape, float mass, btVector3 localInertia) {
  btDefaultMotionState* motionState = new btDefaultMotionState(transform);
  btRigidBody::btRigidBodyConstructionInfo bodyCI(mass, motionState, shape, localInertia);
  btRigidBody* body = new btRigidBody(bodyCI);
  body->setActivationState(DISABLE_DEACTIVATION);
  return body;
}

ostream &operator<<(ostream &out, const PushingSceneInfo &si) {
  out << "Tangram: (" << si.tx0 << ", " << si.ty0 << ", " << si.trot0 << ") ==> (";
  out << si.tx1 << ", " << si.ty1 << ", " << si.trot1 << ") Arm: (";
  out << si.px0 << ", " << si.py0 << ") ==> (";
  out << si.px1 << ", " << si.py1 << ")";
  out << " mass=" << si.tmass << " length=" << si.tlength << " height=" << si.theight;
  out << " pusher-diam=" << si.pdiam << " pusher-speed=" << si.pspeed;
  return out;
}

