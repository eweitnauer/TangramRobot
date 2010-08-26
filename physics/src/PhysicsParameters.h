#ifndef __PHYSIC_PARAMETERS_EWEITNAU_H__
#define __PHYSIC_PARAMETERS_EWEITNAU_H__

#include <BulletDynamics/ConstraintSolver/btContactSolverInfo.h>
#include <LinearMath/btVector3.h>
#include <iostream>
#include <map>
#include <stdexcept>

/**
Extract from the BulletDynamics/btContactSolverInfo.h file.
enum    btSolverMode {
        SOLVER_RANDMIZE_ORDER = 1,
  SOLVER_FRICTION_SEPARATE = 2,
  SOLVER_USE_WARMSTARTING = 4,
  SOLVER_USE_FRICTION_WARMSTARTING = 8,
  SOLVER_USE_2_FRICTION_DIRECTIONS = 16,
  SOLVER_ENABLE_FRICTION_DIRECTION_CACHING = 32,
  SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION = 64,
  SOLVER_CACHE_FRIENDLY = 128,
  SOLVER_SIMD = 256,      //enabled for Windows, the solver innerloop is branchless SIMD, 40% faster than FPU/scalar version
  SOLVER_CUDA = 512       //will be open sourced during Game Developers Conference 2009. Much faster.
};
 */

/// Holds values for 30 of Bullet's simulation parameters.
/** Parameters include general ones affecting the whole simulation (e.g. gravity)
 * as well as object specific ones (e.g. polygon friction). All values can be
 * can be set and retrieved using their names as in a map.*/
class PhysicsParameters : public std::map<std::string, float> {
public:

    PhysicsParameters() {
        // world settings
        addParam("gravity", -9.81); // will NOT get scaled by world_scaling_factor
        addParam("sim_stepsize", 1. / 60);
        addParam("solver_iterations", 10);
        addParam("solver_mode_randomize", 0); // solver_mode = (btSolverMode) 1;
        addParam("solver_mode_friction_separate", 0); // solver_mode = (btSolverMode) 2;
        addParam("solver_mode_use_warmstarting", 0); // solver_mode = (btSolverMode) 4;
        addParam("solver_mode_use_friction_warmstarting", 0); // solver_mode = (btSolverMode) 8;
        addParam("solver_mode_use_2_friction_directions", 0); // solver_mode = (btSolverMode) 16;
        addParam("solver_mode_enable_friction_direction_caching", 0); // solver_mode = (btSolverMode) 32;
        addParam("solver_mode_disable_velocity_dependent_friction", 0); // solver_mode = (btSolverMode) 64;
        addParam("world_scaling_factor", 3);
        addParam("split_impulse", 0); // false
        addParam("split_impulse_penetration_threshold", -0.02f);
        addParam("erp", 0.2);
        addParam("tau", 0.6);

        // collision object settings
        addParam("restitution_polygon", 0.2);
        addParam("friction_polygon", 0.3);
        addParam("restitution_ground", 0.2);
        addParam("friction_ground", 1);
        addParam("restitution_pusher", 0);
        addParam("friction_pusher", 0.3);
        addParam("collision_margin", 0.04);
        addParam("inertia_scaling", 1);

        // use the correct inertia tensor instead of default if set to 1
        addParam("use_custom_inertia_tensor", 1);
        // attach shrinked version of object to its bottom for friction torque correction if set to 1
        addParam("use_modified_shape", 1);
        // with use_modified_shape set to one, instead of using the correct
        // shape-dependent value for modifying the shape, use this value, if it is set to >0
        addParam("fixed_shape_factor", -1);

        /// rigid body settings
        addParam("lin_damping", 0);
        addParam("ang_damping", 0);
        addParam("lin_factor", 1);
        addParam("ang_factor", 1);
    }

    btSolverMode getSolverMode() {
        int mode = 0;
        if (find("solver_mode_randomize")->second) mode += 1;
        if (find("solver_mode_friction_separate")->second) mode += 2;
        if (find("solver_mode_use_warmstarting")->second) mode += 4;
        if (find("solver_mode_use_friction_warmstarting")->second) mode += 8;
        if (find("solver_mode_use_2_friction_directions")->second) mode += 16;
        if (find("solver_mode_enable_friction_direction_caching")->second) mode += 32;
        if (find("solver_mode_disable_velocity_dependent_friction")->second) mode += 64;
        return (btSolverMode) mode;
    }

    /// Returns whether the collision shapes get affected by the parameter.

    bool isChangingCollisionShape(const std::string &param) {
        if (param == "world_scaling_factor") return true;
        if (param == "collision_margin") return true;
        if (param == "fixed_shape_factor") return true;
        if (param == "inertia_scaling") return true;
        if (param == "use_custom_inertia_tensor") return true;
        if (param == "use_modified_shape") return true;
        return false;
    }

    void addParam(const std::string &name, float value) {
        std::map<std::string, float>::operator[](name) = value;
    }

    float& operator[] (const std::string& x) {
        if (find(x) == end()) throw std::runtime_error(std::string("[PhysicsParameters] The parameter '") + x + "' does not exist!");
        return std::map<std::string, float>::operator[](x);
    }

    float operator[] (const std::string& x) const {
        if (find(x) == end()) throw std::runtime_error(std::string("[PhysicsParameters] The parameter '") + x + "' does not exist!");
        return find(x)->second;
    }

    btVector3 getLinearFactor() const {
        float value = find("lin_factor")->second;
        return btVector3(value, value, value);
    }

    btVector3 getAngularFactor() const {
        float value = find("ang_factor")->second;
        return btVector3(value, value, value);
    }

    btVector3 getGravity() const {
        return btVector3(0, find("gravity")->second, 0);
    }

    void setAllFriction(float value) {
        addParam("friction_polygon", value);
        addParam("friction_ground", value);
        addParam("friction_pusher", value);
    }

    void setAllRestitution(float value) {
        addParam("restitution_polygon", value);
        addParam("restitution_ground", value);
        addParam("restitution_pusher", value);
    }

    void writeHeader(std::ostream &out) const {
        for (std::map<std::string, float>::const_iterator it = begin(); it != end(); it++) {
            if (it != begin()) out << " ";
            out << (*it).first;
        }
    }

    void writeData(std::ostream &out) const {
        for (std::map<std::string, float>::const_iterator it = begin(); it != end(); it++) {
            if (it != begin()) out << " ";
            out << (*it).second;
        }
    }
};

#endif /* __PHYSIC_PARAMETERS_EWEITNAU_H__ */

