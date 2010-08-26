#include <tangram_robot_gui.h>
#include <PushingSimulator.h>
#include <QtGui/qpushbutton.h>
#include <iostream>
#include <fstream>
#include "ICLUtils/StackTimer.h"
#include "ICLQt/LabelHandle.h"

using namespace std;
using namespace robotinterface;

Vec round(const Vec &vec) {
	Vec v(vec);
	for (int i=0; i<3; ++i) v[i] = float(round(v[i]*10))/10;
	return v;
}

string vec2str(const Vec &vec) {
	Vec v = round(vec);
	return str(v[0]) + ", " + str(v[1]) + ", " + str(v[2]);
}

float TangramRobotGui::vision2robot(float x) {
    return x/10.;
}

Vec TangramRobotGui::vision2robot(const Vec &vec) {
	Vec v(vec);
	v /= 10; // mm to cm
	v[3] = 1;
	return v;
}

Vec TangramRobotGui::robot2vision(const Vec &vec) {
	Vec v(vec);
	v *= 10; // cm to mm
	v[3] = 1;
	return v;
}

ostream& operator<<(std::ostream &out, const btVector3 &v) {
	return out << v.getX() << ", " << v.getY() << ", " << v.getZ();
}

void TangramRobotGui::predictPushingResults() {
    if (m_psim == NULL) return;
    //	static string &mouseMode = m_gui.getValue<string>("mouse-input-mode");
    static vector<btRigidBody*> bodyList;
    vector<PolygonObject*> polygons = getActivePolygons();
    if (polygons.empty()) return;
    if (m_arm_pos[2] == -1) return;
    // create pushing movement
    PushMovement push(m_adapter.to_bullet(robot2vision(m_arm_pos)),
            m_adapter.to_bullet(robot2vision(m_target_world)),
            btVector3(1. * ROBOT_TO_BULLET_SCALING, 5. * ROBOT_TO_BULLET_SCALING, 1. * ROBOT_TO_BULLET_SCALING),
            10. * ROBOT_TO_BULLET_SCALING);
    // convert and collect all active polygon objects
    for (unsigned int i = 0; i < polygons.size(); i++)
        bodyList.push_back(m_adapter.to_bullet(*polygons[i]));


    btTransform trans;
    //	bodyList[1]->getMotionState()->getWorldTransform(trans);
    //	cout << endl << "== Before ======" << endl;
    //	cout << "Polygon[1] vision transform: " << polygons[1]->getTransformation() << endl;
    //	cout << "Polygon[1] physics transform: (Origin: " << trans.getOrigin()
    //			 << " Rotation-Axis: " << trans.getRotation().getX() << ", " <<  trans.getRotation().getY() << ", " << trans.getRotation().getZ()
    //			 << " Rotation-Angle: " << trans.getRotation().getAngle() << ")" << endl;
    //	cout <<
    //	cout << "simulating push movement" << push << "...";

    m_psim->simulate(push, bodyList, 0.1, 0.5);

    //	cout << "ready!" << endl;
    //	cout << "== After ======" << endl;
    //	bodyList[1]->getMotionState()->getWorldTransform(trans);
    //	cout << "Polygon[1] physics transform: (Origin: " << trans.getOrigin()
    //			 << " Rotation-Axis: " << trans.getRotation().getX() << ", " <<  trans.getRotation().getY() << ", " << trans.getRotation().getZ()
    //			 << " Rotation-Angle: " << trans.getRotation().getAngle() << ")" << endl;
    //	cout << "Polygon[1] vision transform: " << m_adapter.to_vision(trans) << endl;

    // write the results to the polygon objects
    if (!m_arm_moving) {
        for (unsigned int i = 0; i < polygons.size(); i++) {
            bodyList[i]->getMotionState()->getWorldTransform(trans);
            polygons[i]->setPredictedTransformation(m_adapter.to_vision(trans));
        }
    }
    // clear the physic rigid bodies
    for (unsigned int i = 0; i < bodyList.size(); i++) delete bodyList[i];
    bodyList.clear();
}

void TangramRobotGui::connectToArm(const string &memName, const string &robotName) {
    try {
        m_robot_arm.connect(memName);
	m_robot_commands.setDefaultRobot(robotName);
 	cout << "Registering to WorldModelUpdate events..." << endl;
 	m_robot_arm.subscribe(RobotInterface::INSERT,"/EVENT[@name='WorldModelUpdate'][@sender='ArmServer']",this);
 	m_robot_arm.subscribe(RobotInterface::INSERT,"/EVENT[@name='finished'][@sender='ArmServer']",this);
        m_robot_arm.subscribe(RobotInterface::INSERT,"/EVENT[@name='error'][@sender='ArmServer']",this);
        askForNotify("update pos");
    } catch(runtime_error e) {
        cout << "Error on connecting to arm server: " << e.what() << endl;
    }
// 	m_robot.subscribe(RobotInterface::INSERT,"/EVENT[@name='finished' and @notify_id='posA']",boost::bind(this, &myfunc, _1));
}

void TangramRobotGui::connectToHand(const string &memName, const string &robotName) {
    try {
	m_robot_hand.connect(memName);
    } catch(std::runtime_error e) {
        cout << "Error on connecting to hand server: " << e.what() << endl;
    }
}

void TangramRobotGui::init() {
	TangramGui::init();
	
	setForgetPolygonsAfter(500);
	
	m_gui.registerCallback(this, "move-to-target-button,move-down-button,"
		"move-up-button,move-to-posA-button,interrupt,move-to-posB-button,"
		"update-arm-position,write-tangram-data-to-file,fingers-point,"
                "take-tangram-start-pos,take-tangram-end-pos");
}

// update the screen positions of the world coords.
void TangramRobotGui::setDoWorldTransformation(bool value) {
	TangramGui::setDoWorldTransformation(value);
	if (value) updateScreenPositions();
}

void TangramRobotGui::updateScreenPositions() {
	m_target_screen = getCameraTransformer().transformedWorldToScreen(robot2vision(m_target_world));
	m_posA_screen = getCameraTransformer().transformedWorldToScreen(robot2vision(m_posA_world));
	m_posB_screen = getCameraTransformer().transformedWorldToScreen(robot2vision(m_posB_world));
}

void TangramRobotGui::addTrajectoryEntry(const PoseEvent *pe) {
	stringstream s;
	s << pe->getTimeStamp() << " ";
	const vector<float> &v = pe->getPose(PoseEvent::POS);
	for (unsigned int i=0; i<v.size(); ++i) {
		s << v[i];
		if (i<v.size()-1) s << " ";
	}
	s << endl;
	m_arm_trajectory.push_back(s.str());
}

// process xcf event
void TangramRobotGui::handle(const RobotEvent *event) {
    Mutex::Locker l(m_arm_position_mutex);
    // only consider pose events, if the arm should be moving in order to
    // prevent updating the position with wrong data after the tempToolTM was reset
    if (m_arm_moving) {
        const PoseEvent *pe = (dynamic_cast<const PoseEvent*> (event));
        if (pe) {
            if (pe->hasPoseType(PoseEvent::POS)) {
                vector<float> v = pe->getPose(PoseEvent::POS);
                m_arm_pos = Vec(v[0], v[1], v[2], 1);
                v.insert(v.begin(), pe->getTimeStamp());
                if (m_record_arm_position) addTrajectoryEntry(pe);
                cout << "."; cout.flush();
            } else cout << "Error: Event has no position information! Add <update><POS/></update> tag to ArmServer config file." << endl;
        }
    }

    const FinishedEvent *fe = (dynamic_cast<const FinishedEvent*> (event));
    if (fe) {
        if (fe->hasPoseType(PoseEvent::POS)) {
            cout << "Finish event has position information." << endl;
            vector<float> v = fe->getPose(PoseEvent::POS);
            m_arm_pos = Vec(v[0], v[1], v[2], 1);
        }
        if (fe->hasAttrib("notify_id") && fe->getAttrib("notify_id") == "end-of-move") {
            cout << "Got end of move notification!" << endl;
            m_arm_moving = false;
        }
    }

    if (event->hasAttrib("name") && event->getAttrib("name") == "error") {
        cout << "Got an error from arm server: " << endl;
        cout << event->getText() << endl;
        m_arm_moving = false;
    }
}

void TangramRobotGui::askForNotify(string notify_id) {
    float dx = m_gui.getValue<float>("corr-x");
    float dy = m_gui.getValue<float>("corr-y");
    float dz = m_gui.getValue<float>("corr-z");
    m_robot_commands << cmd::clear() << att::immediately(false)
                     << cmd::tempToolTM(dx, dy, dz, -90, 0, 0, "workspace:Left.hand.ff1", "deg")
                     << cmd::notify("<POS/>")+att::custom<string>("notify_id", notify_id);
    m_robot_arm.send(m_robot_commands);
}

// Moves the tip of the index finger to the specified position.
void TangramRobotGui::moveArm(float x, float y, float z, float yaw, float pitch, float roll, bool immediately, bool notify, bool integrate) {
    m_arm_moving = true;
    float dx = m_gui.getValue<float>("corr-x");
    float dy = m_gui.getValue<float>("corr-y");
    float dz = m_gui.getValue<float>("corr-z");
    m_robot_commands << cmd::clear() << att::immediately(immediately)
            << cmd::maxSpeed("normal", 1.0) << cmd::moveMode("stp", "cartesian") + att::custom<int>("optimizePosture", 1)
            + att::custom<bool>("integrate", integrate) + att::custom<string > ("maskTS", "xyzabc") + att::custom<string > ("logfile", "/tmp/postureXXXXXX")
            << cmd::tempToolTM(dx, dy, dz, -90, 0, 0, "workspace:Left.hand.ff1", "deg")
            << cmd::moveTo(x, y, z, yaw, pitch, roll, "absolute", "deg");
    m_robot_arm.send(m_robot_commands);
    if (notify) askForNotify("end-of-move");
}

// Sets pitch and roll to 0 and the yaw between min-yaw from GUI and 170 depending on z.
void TangramRobotGui::moveArm(float x, float y, float z, bool immediately, bool notify, bool integrate) {
    float yaw = m_gui.getValue<float>("min-yaw") + z;
    if (yaw > 170) yaw = 170;
    cout << "Setting yaw to " << yaw << endl;
    moveArm(x,y,z,yaw,0,0,immediately, notify, integrate);
}

// Writes the previously recorded trajectory into a file and clears it.
void TangramRobotGui::writeTrajectoryToFile() {
    string filename = m_trajectory_filename_gen.next();
    cout << "writing to file " << filename << "...";
    ofstream out(filename.c_str());
    for (unsigned int i = 0; i < m_arm_trajectory.size(); ++i) {
        out << m_arm_trajectory[i];
    }
    out.close();
    m_arm_trajectory.clear();
    cout << "done" << endl;
}

string replaceSpaces(string name) {
    string::size_type pos = 0;
    while ( (pos = name.find(" ", pos)) != string::npos ) {
        name.replace(pos, 1, "_");
        pos += 1;
    }
    return name;
}

bool comparePolygonsByShapeName(const PolygonObject* a, const PolygonObject* b) {
    return a->getShape().getName() < b->getShape().getName();
}

// Write positions of active tangrams to a file (in order of their shape name)
void TangramRobotGui::writeTangramDataToFile() {
    if (m_tangrams_start.empty()) {
        cout << "Error: No tangram start positions set." << endl;
        return;
    }
    if (m_tangrams_end.empty()) {
        cout << "Error: No tangram end positions set." << endl;
        return;
    }
    if (m_tangrams_start.size() != m_tangrams_end.size()) {
        cout << "Error: Different number of tangram start and end positions." << endl;
        return;
    }
    sort(m_tangrams_start.begin(), m_tangrams_start.end());
    sort(m_tangrams_end.begin(), m_tangrams_end.end());
    for (unsigned int i=0; i<m_tangrams_start.size(); i++) {
        if (m_tangrams_start[i].shape_name != m_tangrams_end[i].shape_name) {
            cout << "Error: Shape names of tangram start and end don't match.";
            return;
        }
    }
    string filename = m_tangram_filename_gen.next();
    cout << "writing to file " << filename << "...";
    ofstream out(filename.c_str());
    for (unsigned int i=0; i<m_tangrams_start.size(); i++) {
        out << m_tangrams_start[i].toString() << ","
            << m_tangrams_end[i].toString() << ","
            << m_tangrams_start[i].shape_name << endl;
    }
    m_tangrams_start.clear();
    m_tangrams_end.clear();
    out.close();
    cout << "done" << endl;
}

void TangramRobotGui::saveCurrentPolygons(vector<TangramInfo> &tangrams) {
    cout << "saving polygons: " << endl;
    tangrams.clear();
    vector<PolygonObject*> polygons = getActivePolygons();
    for (unsigned int i=0; i<polygons.size(); i++) {
        tangrams.push_back(TangramInfo(polygons[i]));
        cout << tangrams.back().toString() << "," << tangrams.back().shape_name << endl;
    }
}

// process button event
void TangramRobotGui::exec(const string &source) {
    static bool &record_trajectory = m_gui.getValue<bool>("record-arm-trajectory");
    Mutex::Locker l(m_arm_position_mutex);
    m_record_arm_position = false; // only record when moving arm to position B
    float low_z = m_gui.getValue<float>("low-z");
    float high_z = m_gui.getValue<float>("high-z");
    if (source == "move-to-target-button") {
        moveArm(m_target_world[0], m_target_world[1], m_target_world[2], true, true);
    } else if (source == "move-down-button") {
        // was the arm position updated yet?
        if (m_arm_pos[2] >= 0) moveArm(m_arm_pos[0], m_arm_pos[1], low_z, true, true);
        else cout << "I did not receive a WorldModelUpdateEvent yet - cannot move down!" << endl;
    } else if (source == "move-up-button") {
        // was the arm position updated yet?
        if (m_arm_pos[2] >= 0) moveArm(m_arm_pos[0], m_arm_pos[1], high_z, true, true);
        else cout << "I did not receive a WorldModelUpdateEvent yet - cannot move up!" << endl;
    } else if (source == "move-to-posA-button") {
        // in case user forgot to press the write to file button, do it for him
        // before the next trajectory data is written
        if (record_trajectory && !m_arm_trajectory.empty())
            writeTrajectoryToFile();
        // first move upwards (imm., no notify)
        moveArm(m_arm_pos[0], m_arm_pos[1], high_z, true, false);
        // then back to A on same height (not imm., no notify)
        moveArm(m_posA_world[0], m_posA_world[1], high_z, false, false);
        // then down (not imm., with notify)
        moveArm(m_posA_world[0], m_posA_world[1], low_z, false, true);
    } else if (source == "move-to-posB-button") {
        m_record_arm_position = record_trajectory;
        // move to position B (imm., with notify, no integration)
        moveArm(m_posB_world[0], m_posB_world[1], low_z, true, true, false);
    } else if (source == "update-arm-position") {
        askForNotify("");
    } else if (source == "interrupt") {
        m_robot_commands << cmd::clear() << att::immediately(true) << cmd::custom("<interrupt></interrupt>");
        m_robot_hand.send(m_robot_commands);
    } else if (source == "take-tangram-start-pos") {
        saveCurrentPolygons(m_tangrams_start);
    } else if (source == "take-tangram-end-pos") {
        saveCurrentPolygons(m_tangrams_end);
    } else if (source == "write-tangram-data-to-file") {
        writeTangramDataToFile();
    } else if (source == "write-trajectory") {
        writeTrajectoryToFile();
    } else if (source == "fingers-point") {
        m_robot_commands << cmd::clear() << cmd::select("LeftHand")
                << cmd::custom("<posture units='deg'>0.0, 20.0, -20.0, 25.0, 0.0, 25.0, 75.0, 0.0, 0.0, 0.0, 0.0, 0.0, 85.0, 85.0, 85.0, 0.0, 85.0, 85.0, 85.0, 0.0, 0.0, 85.0, 85.0, 85.0"
                               "<controller>1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1</controller>"
                               "</posture>");
        m_robot_hand.send(m_robot_commands);
    } else {
        cout << "Unknown event source: " << source << "!" << endl;
    }
}

void TangramRobotGui::setMoveTarget(float screen_x, float screen_y) {
	static LabelHandle label = m_gui.getValue<LabelHandle>("target-pos-label");
	if (getDoWorldTransformation()) {
		m_target_screen = Point32f(screen_x, screen_y);
		m_target_world = getCameraTransformer().transformedScreenToWorld(m_target_screen);
		m_target_world = vision2robot(m_target_world);
		// stay at same height over table
		if (m_arm_pos[2] > 0)	m_target_world[2] = m_arm_pos[2];
		else m_target_world = m_gui.getValue<float>("high-z");
	} else {
		cout << "Error: Cannot transform to world coordinates." << endl
		     << "\n	Did you load a camera configuration?";
	}
}

// process mouse event
void TangramRobotGui::process(const icl::MouseEvent &event) {
	static string &mouseMode = m_gui.getValue<string>("mouse-input-mode");
	if (mouseMode == "select reference color") {
		TangramGui::process(event);
	} else {
		if (event.isPressEvent()) {
			if (mouseMode == "move to click position") setMoveTarget(event.getX(), event.getY());
		}
	}
}

GUI &TangramRobotGui::addControls(GUI &gui) {
	GUI &tab = TangramGui::addControls(gui);

	// Pushing Action  
	GUI vbox("vbox");
	vbox << "combo(select reference color,!move to click position)"
			    "[@label=mouse input mode@out=mouse-input-mode]"
 			 << (GUI("hbox") << "button(update finger position)[@handle=update-arm-position]"
         << "button(interrupt)[@handle=interrupt]"
         << "label(Curr. Pos: ? ? ?)[@handle=arm-pos-label]"
				 << "label(Target Pos: 0 0 1)[@handle=target-pos-label]")
			 << (GUI("hbox") << "button(move to high z)[@handle=move-up-button]"
			 	  << "button(move to low z)[@handle=move-down-button]"
			 	  << "button(move to target)[@handle=move-to-target-button]")
			 << (GUI("hbox") << "fslider(0,10,1)[@out=low-z@label=low z]"
			                 << "fslider(4,20,8)[@out=high-z@label=high z]"
			                 << "fslider(120,170,145)[@out=min-yaw@label=min. yaw]")
			 << (GUI("hbox") << "fslider(-5,5,-1.5)[@out=corr-x@label=fingertip correction x]"
			                 << "fslider(-5,5,-3)[@out=corr-y@label=fingertip correction y]"
			                 << "fslider(-5,5,1)[@out=corr-z@label=fingertip correction z]");
			 
	
	// Record Data
	GUI vbox2("vbox[@label=record data]");
	vbox2 << "label(Single tangram detected: Fail)[@handle=tangram-detected-label]"
				<< (GUI("hbox") << "button(move to A: ? ? ?)[@handle=move-to-posA-button]"
			 		 << "button(move to B: ? ? ?)[@handle=move-to-posB-button]"
					 << "togglebutton(Don't Record Arm Trajectory,Record Arm Trajectory)[@out=record-arm-trajectory]")
				<< (GUI("hbox") << "button(take tangram start pos)[@handle=take-tangram-start-pos]"
				   << "button(take tangram end pos)[@handle=take-tangram-end-pos]"
                                   << "button(Write Tangram Data To File)[@handle=write-tangram-data-to-file]"
                                   << "button(fingers point)[@handle=fingers-point]");
	return tab << (vbox << vbox2);
}

void TangramRobotGui::setAllLabels() {
	static LabelHandle labelArmPos = m_gui.getValue<LabelHandle>("arm-pos-label");
	static LabelHandle labelTarget = m_gui.getValue<LabelHandle>("target-pos-label");
	static ButtonHandle buttonA = m_gui.getValue<ButtonHandle>("move-to-posA-button");
	static ButtonHandle buttonB = m_gui.getValue<ButtonHandle>("move-to-posB-button");
	static LabelHandle labelTangramDetected = m_gui.getValue<LabelHandle>("tangram-detected-label");
	
	if (m_arm_pos[2]>=0) labelArmPos = "Curr. Pos: " + vec2str(m_arm_pos);
	labelTarget = "Target Pos: " + vec2str(m_target_world);
	(*buttonA)->setText(QString::fromStdString("move to A: " + str(m_posA_world[0]) + ", " + str(m_posA_world[1])));
	(*buttonB)->setText(QString::fromStdString("move to B: " + str(m_posB_world[0]) + ", " + str(m_posB_world[1])));
	if (getActivePolygons().size() == 1) labelTangramDetected = "Single tangram detected: OK";
	else labelTangramDetected = "Single tangram detected: Fail";
}

void TangramRobotGui::draw(ICLDrawWidget *w, const std::vector<icl::Region> &rs) {
	Mutex::Locker l(m_arm_position_mutex);
  predictPushingResults();

	TangramGui::draw(w, rs);

	setAllLabels();
	
	static string &mouseMode = m_gui.getValue<string>("mouse-input-mode");
	if (mouseMode == "move to click position") {
		w->color(0,0,255,255); w->symsize(10);
		w->sym(m_target_screen.x, m_target_screen.y, ICLDrawWidget::symCross);
		w->text("Target", m_target_screen.x, m_target_screen.y);
		w->color(100,100,200,255); w->symsize(10);
		//w->sym(m_posA_screen.x, m_posA_screen.y, ICLDrawWidget::symCross);
		//w->sym(m_posB_screen.x, m_posB_screen.y, ICLDrawWidget::symCross);
		//w->line(m_posA_screen.x, m_posA_screen.y, m_posB_screen.x, m_posB_screen.y);
		//w->text("A", m_posA_screen.x, m_posA_screen.y);
		//w->text("B", m_posB_screen.x, m_posB_screen.y);
	}
	
	if (m_arm_moving) w->color(200,0,0,255);
	else w->color(0,200,0,255);
	Point32f pos = getCameraTransformer().transformedWorldToScreen(robot2vision(m_arm_pos));
	w->symsize(20);
	w->sym(pos.x, pos.y, ICLDrawWidget::symPlus);
	Vec vpos = Vec(m_arm_pos[0], m_arm_pos[1], m_arm_pos[2], 1);
	w->text("Robot Arm World Position: " + vec2str(vpos), pos.x+5, pos.y+5, -1, -1, 8);
}

