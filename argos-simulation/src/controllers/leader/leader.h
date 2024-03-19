/*
 * AUTHOR: Genki Miyauchi <g.miyauchi@sheffield.ac.uk>
 *
 * An example controller for running SCT with the e-puck.
 *
 * The controller uses the supervisors generated in Nadzoru to 
 * determine its next action in each timestep.
 *
 * This controller is meant to be used with the XML files:
 *    experiments/leader_follower.argos
 * 
 * This example has been modified from the following examples provided in argos3-examples: https://github.com/ilpincy/argos3-examples/
 *   - argos3-examples/controllers/epuck_obstacleavoidance/
 *   - argos3-examples/controllers/footbot_manualcontrol/
 */

#ifndef LEADER_H
#define LEADER_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of proximity sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_proximity_sensor.h>
/* Definition of the range-and-bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range-and-bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definition of the positioning sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

/* PID controller */
#include <utility/pid.h>
/* SCT generator player */
#include <utility/sct.h>
/* Message structure */
#include <utility/robot_message.h>

#include <set>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CLeader : public CCI_Controller {

public:

    struct SWheelTurningParams {
        /*
        * The turning mechanism.
        * The robot can be in three different turning states.
        */
        enum ETurningMechanism
        {
            NO_TURN = 0, // go straight
            SOFT_TURN,   // both wheels are turning forwards, but at different speeds
            HARD_TURN    // wheels are turning with opposite speeds
        } TurningMechanism;
        /*
        * Angular thresholds to change turning state.
        */
        CRadians HardTurnOnAngleThreshold;
        CRadians SoftTurnOnAngleThreshold;
        CRadians NoTurnAngleThreshold;
        /* Maximum wheel speed */
        Real MaxSpeed;

        void Init(TConfigurationNode& t_tree);
    };

    /*
    * The following variables are used as parameters for
    * tracking waypoints. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><leader_controller><parameters><waypoint_tracking>
    * section.
    */
    struct SWaypointTrackingParams {
        /* Target angle to waypoint in radians */
        Real TargetAngle;
        /* Parameters to be used for PID */
        Real Kp;
        Real Ki;
        Real Kd;
        /* Consider it has arrived to a goal/waypoint if it is within a threshold */
        Real thresRange;
        
        void Init(TConfigurationNode& t_node);
    };

    /*
    * The following variables are used as parameters for
    * flocking interaction. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><leader_controller><parameters><team_flocking>
    * section.
    */
    struct SFlockingInteractionParams {
        /* Target robot-robot distance in cm */
        Real TargetDistance;
        /* Gain of the Lennard-Jones potential */
        Real Gain;
        /* Exponent of the Lennard-Jones potential */
        Real Exponent;

        void Init(TConfigurationNode& t_node);
        Real GeneralizedLennardJones(Real f_distance);
        Real GeneralizedLennardJonesRepulsion(Real f_distance);
    };

public:

    /* Class constructor. */
    CLeader();

    /* Class destructor. */
    virtual ~CLeader();

    /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><epuck_obstacleavoidance_controller> section.
    */
    virtual void Init(TConfigurationNode& t_node);

    /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
    virtual void ControlStep();

    /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    */
    virtual void Reset();

    /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
    virtual void Destroy() {}

    /*
    * Sets the selected flag on this robot.
    * When selected, a robot follows the control vector.
    */
    virtual void Select();

    /*
    * Unsets the selected flag on this robot.
    * When unselected, a robot stays still.
    */
    virtual void Deselect();

    /*
    * Gets the username of the operator that is currently controlling this robot.
    */
    virtual std::string GetUsername();

    /*
    * Sets the username of the operator that has selected this robot.
    */
    virtual void SetUsername(std::string username);

    /*
    * Sets the control vector.
    */
    virtual void SetControlVector(const CVector2& c_control);

    /*
    * Sets the start/stop signal.
    */
    virtual void SetSignal(const bool b_signal);

    /*
    * Sets the number of followers to request to the other team.
    */
    virtual void SetRobotsToRequest(const UInt32 un_robots);

    /*
    * Sets the number of followers to send to the other team.
    */
    virtual void SetRobotsToSend(const UInt32 un_robots);

    /*
    * Sets the number of followers to send to the specified team.
    */
    virtual void SetRobotsToSend(const UInt32 un_robots, const UInt8 un_team);

    /*
    * Get the next waypoint in the queue.
    */
    virtual CVector2 GetNextWaypoint();

    /*
    * Sets the list of waypoints to visit.
    */
    virtual void SetWaypoints(const std::queue<CVector2> waypts);

    /*
    * Get the current task's id.
    */
    virtual std::string GetTaskId();

    /*
    * Set the current task's id.
    */
    virtual void SetTaskId(const std::string str_id);

    /*
    * Get the current task's demand.
    */
    virtual UInt32 GetTaskDemand();

    /* 
    * Sets the current task's demand.
    */
    virtual void SetTaskDemand(const UInt32 un_demand);

    /*
    * Get the current task's initial demand.
    */
    virtual UInt32 GetInitTaskDemand();

    /* 
    * Sets the current task's initial demand.
    */
    virtual void SetInitTaskDemand(const UInt32 un_init_demand);

    /*
    * Get the minimum number of robots needed for the current task.
    */ 
    virtual UInt32 GetMinimumCount();

    /* 
    * Sets the minimum number of robots needed for the current task.
    */
    virtual void SetMinimumCount(const UInt32 un_min);

    /*
    * Get the minimum number of robots needed for the other leader's current task.
    */ 
    virtual UInt32 GetOtherMinimumCount();

    /*
    * Get the current number of followers in the team.
    */
    virtual UInt32 GetFollowerCount();

    /*
    * Sets the current number of followers in the team.
    */
    virtual void SetFollowerCount(const UInt32 un_count);

    /*
    * Get the current number of followers in the other leader's team.
    * negative value means there is no info of other leader's follower count.
    */
    virtual SInt32 GetOtherFollowerCount();

    /*
    * Get team ID.
    */
    virtual UInt8 GetTeamID();

    /*
    * Get hop count to teams.
    */
    virtual const std::map<UInt8, HopMsg>& GetHops() const;

    /*
    * Returns the timestep that this leader has sent a message to the other leader.
    * When it has not yet sent a time, it returns -1.
    */
    virtual Real GetLatestTimeSent();

    /*
    * Checks whether the leader has received a message from the other leader.
    * Returns the timestep that the message was sent, if message was received.
    */
    // virtual Real GetLatestTimeReceived();

    /*
    * Get the total number of messages received from the other leader.
    */
    virtual Real GetTotalSent();

    /*
    * Get the total number of messages received from the other leader.
    */
    // virtual Real GetTotalReceived();

    /*
    * Returns the last action made by the leader.
    */
    virtual std::string GetLastAction();

    /*
    * Get robot send delay
    */
    virtual UInt32 GetRobotSendDelay();

    /*
    * Get robot send delay
    */
    // virtual void SetEx1DeployAreaRadius(const double radius);

protected:

    /*
    * Reset variables
    */
    virtual void ResetVariables();

    /* 
    * Receive messages from neighboring robots.
    */
    virtual void GetMessages();

    /* 
    * Update sensor readings.
    */
    virtual void Update();

    /*
    * Returns whether it is near any other robot.
    */
    virtual bool IsNearRobot();

    /*
    * Upon receiving a Request message from follower, decide to send an Accept message (max 1).
    */
    virtual void ReplyToRequest();

    /*
    * Update the hop count to other teams.
    */
    virtual void UpdateHopCounts();

    /*
    * Find the closest connector info that needs to be shared within the team.
    */
    virtual void SetConnectorToRelay();

    /*
    * Check whether a heart beat message is received from the other leader.
    */
    virtual void CheckHeartBeat();

    /*
    * Calculates the vector to the next waypoint.
    */
    virtual CVector2 VectorToWaypoint();

    /* 
    * Get a flocking vector between itself and the other robots not in the same team.
    */
    virtual CVector2 GetRobotRepulsionVector(std::vector<Message>& msgs);

    /*
    * Get a repulsion vector from obstacles.
    */
    virtual CVector2 GetObstacleRepulsionVector();

    /*
    * Gets a direction vector as input and transforms it into wheel actuation.
    * Used in manual control.
    */
    void SetWheelSpeedsFromVector(const CVector2& c_heading);

    /*
    * Gets a direction vector as input and transforms it into eight different wheel actuation.
    * Used in manual control.
    */
    void SetWheelSpeedsFromVectorEightDirections(const CVector2& c_heading);

    /*
    * Gets a direction vector as input and transforms it into wheel actuation using a PID controller.
    * Used in autonomous control.
    */
    void SetWheelSpeedsFromVectorHoming(const CVector2& c_heading);

    /*
    * Print robot id.
    */
    void PrintName();

    /* Callback functions */
    virtual void Callback_Start(void* data);
    virtual void Callback_Stop(void* data);
    // virtual void Callback_Respond(void* data);
    virtual void Callback_Message(void* data);
    virtual void Callback_Exchange(void* data);

    // virtual unsigned char Check__RequestL(void* data);
    virtual unsigned char Check_PressStart(void* data);
    virtual unsigned char Check_PressStop(void* data);
    virtual unsigned char Check_InputMessage(void* data);
    virtual unsigned char Check_InputExchange(void* data);

private:

    /* Pointer to the differential steering actuator */
    CCI_DifferentialSteeringActuator* m_pcWheels;
    /* Pointer to the e-puck proximity sensor */
    CCI_ProximitySensor* m_pcProximity;
    /* Pointer to the range-and-bearing actuator */
    CCI_RangeAndBearingActuator* m_pcRABAct;
    /* Pointer to the range-and-bearing sensor */
    CCI_RangeAndBearingSensor* m_pcRABSens;
    /* Pointer to the LEDs actuator */
    CCI_LEDsActuator* m_pcLEDs;
    /* Pointer to the positioning sensor */
    CCI_PositioningSensor* m_pcPosSens;

    /* The turning parameters */
    SWheelTurningParams m_sWheelTurningParams;
    /* The waypoint tracking parameters */
    SWaypointTrackingParams m_sWaypointTrackingParams;
    /* The flocking interaction parameters between teammates. */
    SFlockingInteractionParams m_sTeamFlockingParams;

    /* SCT Controller */
    std::unique_ptr<SCT> sct;

    /* Last controllable action */
    std::string lastControllableAction;

    /* Current robot state */
    RobotState currentState;

    /* Flag to know whether this robot is selected */
    bool m_bSelected;

    /* Operator username */
    std::string m_strUsername;

    /* The control vector */
    CVector2 m_cControl;

    /* The task signal */
    bool m_bSignal;

    /* Current team ID, which is the number of the leader ID (e.g. L1 -> 1) */
    UInt8 teamID;

    /* Proximity sensor readings */
    std::vector<Real> proxReadings;

    /* Messages received from nearby robots */
    std::vector<Message> teamMsgs;
    std::vector<Message> connectorMsgs;
    std::vector<Message> otherLeaderMsgs;
    std::vector<Message> otherTeamMsgs;
    std::vector<Message> travelerMsgs;

    /* Outgoing message */
    CByteArray cbyte_msg;

    /* The number of hops to each team */
    std::map<UInt8, HopMsg> hopsDict;

    /* PID to control the heading angle */
    PID* PIDHeading;

    /* Ordered list of waypoints to visit */
    std::queue<CVector2> waypoints;
    bool inTask;

    /* Task demand of the current working task */
    std::string currentTaskId;
    UInt32 currentTaskDemand;
    UInt32 currentInitTaskDemand;
    UInt32 previousTaskDemand; // Task demand from 10 timesteps ago

    int robotsNeeded; // Minimum number of robots needed to perform the current task

    /* Current number of followers in the team */
    int currentFollowerCount;
    int previousFollowerCount;

    /* Flag to know whether there is a neighbor */
    bool nearRobot;

    /* Info of the closest non team member to broadcast */
    RobotState closeState;
    Real closeDist;
    std::string closeID;

    /* Incoming message buffer (occurances of public uncontrollable events) */
    // std::map<size_t, bool> pub_events;

    /* Connection related info to send in the current timestep */
    std::vector<ConnectionMsg> cmsgToSend; 
    std::vector<std::pair<size_t, ConnectionMsg>> cmsgToResend; // ConnectionMsg attached with a timer. Messages gets added into cmsgToSend while timer is running
    
    /* Team Shared Message */
    std::map<UInt8, TeamSharedMsg> teamSharedMsgDict;

    std::vector<RelayMsg> rmsgToSend;
    std::vector<std::pair<size_t, RelayMsg>> rmsgToResend;
    Real lastSent;
    std::map<UInt8, Real> lastBeatTime;
    bool isSendingRobots;
    std::map<UInt8, size_t> beatReceived;
    size_t beatSent;
    std::string switchCandidate; // Robot that the leader could choose to switch to the other team
    UInt8 switchCandidateDist; // Distance of the candidate to the non-team robot it received the message from
    size_t switchCandidateTimer; // Time it recorded the switchCandidate
    bool decremented;

    // std::map<std::string, std::map<std::string, UInt32>> otherLeaderInfo; // DELETE: Map to store information received from the other leader (followers, robotsNeeded)
    SInt8 numOtherFollower;
    UInt8 numOtherTaskRequire;

    /* Timer to count the timesteps for the initial communication to occur at the beginning of the simulation */
    size_t initStepTimer;

    /* Timer to count the number of messages to send before sending the next robot */
    size_t robotLastSentTime;
    std::string robotLastSent; // ID of robot last sent

    /* Team switch variables */
    int numRobotsToSend;
    int numRobotsRemainingToSend;
    int numRobotsToRequest;
    int numRobotsRemainingToRequest; // Used during the behavior analysis experiment
    bool allRequestsSatisfied; // Used during the behavior analysis experiment
    size_t allRequestsSatisfiedTime; // Used during the behavior analysis experiment
    std::map<UInt8, int> numRobotsRequested; // Number of robots requested by other teams
    std::string robotToSwitch;
    UInt8 teamToJoin;
    bool requestSent;
    bool acknowledgeSent;
    std::set<UInt8> otherTeams;
    std::set<UInt8> teamsToRequest; // Used to request to teams in order

    /* Flag to indicate trigerring of uncontrollable events */
    bool receivedMessage, receivedRelay, receivedRequest, inputStart, inputStop, inputMessage;

    /* Respond to follower message request */
    ConnectionMsg acceptCmsg;
    // std::map<UInt8, std::string> acceptIDs;

    /* Ex1L1StartCondition */
    // bool ex1L1StartCondition = false;
    // double ex1DeployAreaRadius = 0;

    /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><epuck_obstacleavoidance_controller> sectiong.
    */
    /* The leader can move as long as it is withthin the minimum distance threshold from the robots */
    Real minDistanceFromRobot;

    /* Chain formation threshold */
    Real separationThres;

    size_t sendDuration;
    size_t sendRobotDelay;

    /* SCT yaml path */
    std::string m_strSCTPath;

};

#endif
