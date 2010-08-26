#ifndef __TANGRAM_ROBOT_GUI_EWEITNAU_H__
#define __TANGRAM_ROBOT_GUI_EWEITNAU_H__

#include <tangram_gui.h>
#include <riRobotInterface.h>
#include <riEventHandler.h>
#include <riWorldModelUpdateEvent.h>
#include <riFinishedEvent.h>
#include <PushingActionRecorder.h>
#include <PushingSimulator.h>
#include <vision_adapter.h>
#include <ICLIO/FilenameGenerator.h>

#define ROBOT_TO_BULLET_SCALING 0.05 // (everything is 5 times bigger in bullet)

/// Class adding to the TangramGui the functionality of moving the robot arm.

/** By clicking on the image in the case a CameraTransformer is set, the
coords are transformed into world coords and a pushing action with the arm
can be executed.
After calling the init() method, the connection to the ArmServer must be
established by calling the connectToArm() method with the name of the running
memory server.)

Coordinate Systems:
A) Vision-Screen: in pixel
B) Vision-World:  in mm
C) Robot:         in cm
D) Physic:        in units

Transformations:
A <==> B: Uses the CameraTransformer (contains calibrated camera and table plane)
B ==> C: *0.1
C ==> D: *0.05 (5 times bigger in bullet)

Below, if not stated otherwise, all positions are in the robot coord. system.
 */
class TangramRobotGui : public TangramGui, robotinterface::EventHandler, GUI::Callback {
public:
    TangramRobotGui(float low_z = 1, float high_z = 8) : TangramGui(),
            m_arm_pos(-1, -1, -1, 1), m_target_world(10, 20, 1, 1),
            m_posA_world(30, 60, 1, 1), m_posB_world(30, 10, 1, 1),
            m_low_z(low_z), m_high_z(high_z), m_psim(NULL),
            m_adapter(ROBOT_TO_BULLET_SCALING / 10), m_arm_moving(false),
            m_trajectory_filename_gen("./data/arm_trajectory##.txt"),
            m_tangram_filename_gen("./data/tangram_positions##.txt"),
            m_record_arm_position(false) {
        m_tab_names += ",Pushing Actions";
    }

    virtual void draw(icl::ICLDrawWidget *w, const std::vector<icl::Region> &rs);

    virtual void init();

    void connectToArm(const std::string &memName, const std::string &robotName);

    void connectToHand(const std::string &memName, const std::string &robotName);

    virtual void setDoWorldTransformation(bool value);

    /// Set the Pushing Simulator so the gui can predict the results of pushing actions.

    void setPushingSimulator(PushingSimulator *psim) {
        m_psim = psim;
    }

protected:
    static float vision2robot(float x);
    static Vec vision2robot(const Vec &vec);
    static Vec robot2vision(const Vec &vec);

    struct TangramInfo {
        float x,y,rot;
        string shape_name;
        TangramInfo(float x, float y, float rot, string shape_name) :
        x(x), y(y), rot(rot), shape_name(shape_name) {}

        TangramInfo(PolygonObject* p) {
            const Transformation &t = p->getTransformation();
            x = vision2robot(t.getTx());
            y = vision2robot(t.getTy());
            rot = t.getRotation()*180 / M_PI;
            shape_name = p->getShape().getName();
        }

        string toString() {
            stringstream s;
            s << x << "," << y << "," << rot;
            return s.str();
        }

        bool operator<(const TangramInfo &other) const {
            return shape_name<other.shape_name;
        }
    };
    
    /// Gets called on init by the parent class
    virtual GUI &addControls(icl::GUI &gui);

    /// Process mouse events
    virtual void process(const icl::MouseEvent &event);

    /// Process button events
    virtual void exec(const std::string &source);

    /// Method for handeling incomming events from the ArmServer (e.g. FinishedEvents).
    virtual void handle(const robotinterface::RobotEvent *event);

    /// Sends a notify tag to the arm server
    void askForNotify(string notify_id);

    /// Moves the tip of the index finger to the specified position.
    /** If notify=true is passed, a finish notification with attribute
     *  notify_id="end-of-move" is requested from the server. */
    void moveArm(float x, float y, float z, float yaw, float pitch, float roll, bool immediately, bool notify, bool integrate);

    /// Sets pitch and roll to 0 and the yaw between 150 and 170 depending on z.
    /** If notify=true is passed, a finish notification with attribute
     *  notify_id="end-of-move" is requested from the server. */
    void moveArm(float x, float y, float z, bool immediately, bool notify, bool integrate=true);

    /// Writes the previously recorded trajectory into a file and clears it.
    void writeTrajectoryToFile();
    
    /// Write positions of active tangrams to a file (in order of their shape name)
    void writeTangramDataToFile();

    /// Returns a string with comma separated x,y pos. and the rot. of the passed PolygonObject.
    /** Values are in robot coordinate system. */
    string getTangramPosition(const PolygonObject* p);

    /// Clears the vetor and stores currently active polygons in it.
    void saveCurrentPolygons(vector<TangramInfo> &tangrams);

    void addTrajectoryEntry(const robotinterface::PoseEvent *pe);
private:
    icl::Vec m_arm_pos; //<! curr. position of finger tip in arm coordinates
    icl::Vec m_target_world;
    icl::Point32f m_target_screen;
    icl::Vec m_posA_world;
    icl::Point32f m_posA_screen;
    icl::Vec m_posB_world;
    icl::Point32f m_posB_screen;
    robotinterface::RobotInterface m_robot_arm;
    robotinterface::RobotInterface m_robot_hand;
    robotinterface::RobotCommandSet m_robot_commands;
    icl::Mutex m_arm_position_mutex;
    float m_low_z; //! world z-coord. of finger tip (height over table) for pushing
    float m_high_z; //! world z-coord. of finger tip (height over table) for positioning
    PushingActionRecorder m_recorder;
    PushingSimulator *m_psim;
    VisionAdapter m_adapter;
    bool m_arm_moving;
    icl::FilenameGenerator m_trajectory_filename_gen, m_tangram_filename_gen;
    string m_tangram_filename;
    vector< std::string > m_arm_trajectory;
    bool m_record_arm_position;

    vector<TangramInfo> m_tangrams_start, m_tangrams_end;

    void updateScreenPositions();
    void setMoveTarget(float screen_x, float screen_y);
    void setPredefinedPosition(float screen_x, float screen_y);
    void setAllLabels();
    void predictPushingResults();
};

#endif /* __TANGRAM_ROBOT_GUI_EWEITNAU_H__ */

