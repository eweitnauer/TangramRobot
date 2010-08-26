#ifndef __PUHSING_RECORDER_WEITNAU_H__
#define __PUHSING_RECORDER_WEITNAU_H__

#include <iostream>
#include <Shapes.h>
#include <PhysicsParameters.h>
#include <PushMovement.h>
#include <PushingScene.h>
#include <PushingSimulator.h>
#include <stdexcept>

std::vector<float> &operator<<(std::vector<float>& vec, float value) {
	vec.push_back(value);
	return vec;
}

/// When changeType is set to NONE, no parameter is changed and only one simulation is neccessary.
struct SimulationSettings {
	enum ParamChangeType {
		NONE,
		RANGE,
		LIST,
		SWITCH
	};
	
	std::string param_name;
	float low;
	float high;
  std::vector<float> valueList;
	int steps;
	
	int repetitions;
	float before_time_s;
	float after_time_s;
	ParamChangeType changeType;

	SimulationSettings(std::string param_name, int repetitions=10,
		float before_time_s=0.2, float after_time_s=1.0):
		param_name(param_name), steps(2), repetitions(repetitions),
		before_time_s(before_time_s), after_time_s(after_time_s), changeType(SWITCH) {}

	SimulationSettings(std::string param_name, std::vector<float> valueList, int repetitions=10,
		float before_time_s=0.2, float after_time_s=1.0):
		param_name(param_name), valueList(valueList), steps(valueList.size()), repetitions(repetitions),
		before_time_s(before_time_s), after_time_s(after_time_s), changeType(LIST) {}
		
	SimulationSettings(std::string param_name, float low, float high, int steps, int repetitions=10,
		float before_time_s=0.2, float after_time_s=1.0):
		param_name(param_name), low(low), high(high), steps(steps), repetitions(repetitions),
		before_time_s(before_time_s), after_time_s(after_time_s), changeType(RANGE) {}

	SimulationSettings(int repetitions=10, float before_time_s=0.2, float after_time_s=1.0):
		param_name(""), repetitions(repetitions),
		before_time_s(before_time_s), after_time_s(after_time_s), changeType(NONE) {}
	
	/// step in {0...steps-1}
	float getValue(int step) const {
		switch(changeType) {
			case NONE: throw std::runtime_error("You called getValue() on a SimulationSetting object although its value-type is NONE!");
			case RANGE: return low+(high-low)/(steps-1)*step;
			case LIST: return valueList[step];
			case SWITCH: return step==1;
			default: throw std::runtime_error(std::string("Unknown ParamChangeType."));
		}
	}
};

/// All values in meter / radiants / kg
struct PushingSceneInfo {
  float tx0, ty0, trot0; // tangram start pos
  float tx1, ty1, trot1; // tangram end pos
  float tmass; // tragram mass
  float tlength; // base length of tangram
  float theight; // height of tangram
  Shapes::ShapeType ttype; // tangram shape type
  std::vector<float> tcorners; // tangram corners [x0,y0,x1,y1,...]
  float px0, py0; // pusher start pos
  float px1, py1; // pusher end pos
  float pdiam; // pusher diameter
  float pspeed; // pusher speed in m/s
  
  PushingSceneInfo(float tx0, float ty0, float trot0,
  								 float tmass, float tlength, float theight,
  								 Shapes::ShapeType ttype, std::vector<float> tcorners,
              		 float px0, float py0, float px1, float py1, float pdiam, float pspeed):
      tx0(tx0), ty0(ty0), trot0(trot0), tx1(tx0), ty1(ty0), trot1(trot0),
      tmass(tmass), tlength(tlength), theight(theight), ttype(ttype), tcorners(tcorners),
      px0(px0), py0(py0), px1(px1), py1(py1), pdiam(pdiam), pspeed(pspeed) {}
      
  PushingSceneInfo() {}
  
  void setTangramPos(float x0, float y0, float rot0, float x1, float y1, float rot1) {
  	tx0 = x0; ty0 = y0;
  	trot0 = rot0;
  	tx1 = x1; ty1 = y1;
  	trot1 = rot1;
  }
  
  void setPusherPos(float x0, float y0, float x1, float y1) {
	  px0 = x0; py0 = y0;
  	px1 = x1; py1 = y1;
  }
};

std::ostream &operator<<(std::ostream &out, const PushingSceneInfo &si);

class PushingRecorder {
	public:
		PushingRecorder(PushingSimulator *psim): psim(psim) {}
		
		void simulate(PushingScene &scene, const PhysicsParameters &params,
			const SimulationSettings &simsets);
		
		/// Runs a simulation for the passed physical parameters and scene settings.
		/** Removes all former bodies from the pushing scene. */
		void simulateSingleParameterSetting(PushingScene &scene,
			const SimulationSettings &simsets, const PhysicsParameters &params_const,
			const PushingSceneInfo &sceneInfo);

		/// Runs a simulation for the passed physical parameters and scene settings.
		/** Can be used to hard-code the shapeFactor independent from the PhysicsParameters. */
		void simulateSingleParameterSetting(PushingScene &scene,
			const SimulationSettings &simsets, const PhysicsParameters &params_const,
			const PushingSceneInfo &sceneInfo, float ShapeFactor);
		
		/// Runs simulations for all parameter settings described in simsets and writes the results to out.
		/** As reference transformation the resulting mean transformation of a trial
		 * with a default parameter setting can be passed. The written statistics
	 	 * include variance of distance and max distance of the current body
	 	 * transformations to the reference transformation. */
		void writeDataset(std::ostream &out,
			const SimulationSettings &simsets, const PhysicsParameters &params_const,
			const PushingSceneInfo &sceneInfo, Transformation reference_t, bool writeHeader=true);

	protected:
		void simulateSingleParameterSetting(PushingScene &scene, btCollisionShape *scaled_shape,
			btVector3 localInertia,	const SimulationSettings &simsets,
			const PhysicsParameters &params_const, const PushingSceneInfo &sceneInfo);
			
		btRigidBody* createDynamicRigidBody(btTransform transform,
			btCollisionShape* shape, float mass, btVector3 localInertia);

	private:
		PushingSimulator *psim;
};

#endif /* __PUHSING_RECORDER_WEITNAU_H__ */

