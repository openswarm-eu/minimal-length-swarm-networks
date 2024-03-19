/*
 * AUTHOR: Genki Miyauchi <g.miyauchi@sheffield.ac.uk>
 *
 * An example controller for running SCT with the e-puck.
 *
 * The controller uses the supervisors generated in Nadzoru to 
 * determine its next action in each timestep.
 *
 * This controller is meant to be used with the XML files:
 *    experiments/leader_worker.argos
 * 
 * This example has been modified from the following examples provided in argos3-examples: https://github.com/ilpincy/argos3-examples/
 *   - argos3-examples/controllers/epuck_obstacleavoidance/
 *   - argos3-examples/controllers/footbot_flocking/
 */

#ifndef WORKER_H
#define WORKER_H

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
/* Definition of the ground sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_ground_sensor.h>

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
class CWorker : public CCI_Controller {

public:

    /*
    * The following variables are used as parameters for
    * turning during navigation. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><footbot_flocking_controller><parameters><wheel_turning>
    * section.
    */
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
    * flocking interaction. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><worker_controller><parameters><team_flocking>
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

    /*
    * Structure to store the set of conditions to determine whether it 
    * can switch its role (either to a follower or a connector).
    */
    struct RoleSwitchConditions {
        /* (F -> C) Whether distance exceeds the threshold */
        bool c1;
        /* (F -> C) Whether it is the closest among its teammates */
        bool c2;
        /* (F -> C) Whether the distance between the leader and connection candidate exceeds the separation threshold */
        bool c3;
        /* (C -> F) Whether it is a tail connector */
        bool f1;
        /* (C -> F) Whether the team is closer to its successor in the chain */
        bool f2;

        /* Whether the conditions to become a connector are satisfied for this team */
        bool ConnectorConditions() const;
        // /* Whether the conditions to become a follower are satisfied for this team */
        // bool FollowerConditions() const;
    };

    /* List of move types available to the robot */
    enum class MoveType {
        ADJUST = 0,         // Adjust its position in the chain to maintain a straight line (used by connector)
        FLOCK,              // Flock with the team (used by follower)
        TRAVEL_TO_TEAM,     // Move along the chain to a team (used by traveler)
    } currentMoveType;

public:

    /* Class constructor. */
    CWorker();

    /* Class destructor. */
    virtual ~CWorker();

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
    * Get team ID.
    */
    virtual UInt8 GetTeamID() const;

    /*
    * Set team ID.
    */
    virtual void SetTeamID(const UInt8 id);

    /*
    * Get robot state.
    */
    virtual RobotState GetRobotState() const;

    /*
    * Get move type.
    */
    virtual std::string GetMoveType() const;

    /*
    * Get hop count to teams.
    */
    virtual const std::map<UInt8, HopMsg>& GetHops() const;

    /*
    * Get team hop count (hop count within the team).
    */
    virtual const UInt8 GetTeamHopCount() const;

    /*
    * Return whether the robot is working on a task.
    */
    virtual bool IsWorking();

    /*
    * Returns the last action.
    */
    virtual std::string GetLastAction() const;

    /*
    * Return the previous hop count.
    */
    virtual const std::map<UInt8, HopMsg>& GetPrevHops() const;

    /*
    * Set worker to travel to another team.
    */
    virtual void SetTravelToTeam(UInt8 team);

    /*
    * Infrom the range of the range-and-bearing sensor
    */
    virtual void SetRABRange(Real fRABRange);

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
    * Extract information about the leader from the team messages received.
    */
    virtual void GetLeaderInfo();

    /*
    * Return the closest non-team robot within its range.
    *
    * Connectors have higher priority than followers from another team.
    * @return Map of Messages received from the closest robots to each team.
    */
    virtual std::map<UInt8, Message> GetClosestNonTeam();

    /*
    * Check whether it satisfies the conditions to become a connector.
    * 
    * @param msgs Map of Messages of the robot to check the distance with other team members.
    */
    virtual void SwitchToConnectorConditions(const std::map<UInt8, Message>& msgs);

    /*
    * Check whether it satisfies the conditions to become a follower.
    */
    virtual void SwitchToFollowerConditions();

    /*
    * Check whether it has received an accept message.
    */
    virtual void CheckAccept();

    /*
    * Relay Request and Accept messages.
    *
    * Messages are relayed both upstream (to leader) and downstream (to the team).
    */
    virtual void SetCMsgsToRelay();

    /* 
    * Relay leader message to the other team.
    */
    virtual void SetLeaderMsgToRelay(const RobotState state);

    /*
    * Find the closest connector info that needs to be shared within the team.
    */
    virtual void SetConnectorToRelay();

    /*
    * When no connector has been established between a team, find the shortest distance to that team. Distance will be shared within the team.
    */
    virtual void SetDistanceToRelay();

    /*
    * Update the hop count when forming part of the chain network (connector).
    */
    virtual void UpdateHopCountsConnector();

    /*
    * Check whether it satisfies the condition to modify its connections
    */
    virtual void CheckNetworkModification();

    /*
    * Store messages that has network modification messages
    */
    virtual void StoreNetworkModification();

    /*
    * Update the hop count when in a team (follower).
    */
    virtual void UpdateHopCountsFollower();

    /*
    * Check whether it has received any request messages and decide which to accept.
    *
    * Check all requests sent to itself and choose one to respond for each team.
    */
    virtual void CheckRequests();

    /*
    * Move wheels according to flocking vector
    */
    virtual void Flock();

    /* 
    * Get a flocking vector between itself and team members with the smallest hop count.
    */
    virtual CVector2 GetTeamFlockingVector();

    /* 
    * Get a repulsion vector between itself and all other robots.
    */
    virtual CVector2 GetRobotRepulsionVector(std::vector<Message>& msgs);

    /*
    * Get a repulsion vector from obstacles.
    */
    virtual CVector2 GetObstacleRepulsionVector();

    /*
    * Move wheels according to chain traversal behavior
    */
    virtual void Travel();

    /*
    * Get a vector to travel along the chain.
    */
    virtual CVector2 GetChainTravelVector();

    /*
    * Move wheels according to the adjacent connectors to make the chain straight.
    */
    virtual void AdjustPosition();

    /*
    * Get a list of messages that this robot is connected to (Used in the CONNECTOR state).
    */
    virtual std::map<UInt8, Message> GetNeighbors();

    /*
    * Get a vector that attracts itself to neighbor robots.
    */
    virtual CVector2 GetConnectorAttractNeighborVector(std::map<UInt8, Message>& neighbor_msgs);

    /*
    * Get a vector that attracts itself to the team.
    */
    virtual CVector2 GetConnectorAttractTeamVector(std::map<UInt8, Message>& neighbor_msgs);

    /*
    * Gets a direction vector as input and transforms it into wheel actuation.
    */
    virtual void SetWheelSpeedsFromVector(const CVector2& c_heading);

    /*
    * Print robot id.
    */
    virtual void PrintName();

    /* Callback functions */
    virtual void Callback_MoveFlock(void* data);
    virtual void Callback_MoveToTeam(void* data);
    virtual void Callback_MoveAdjust(void* data);
    virtual void Callback_SwitchF(void* data);
    virtual void Callback_SwitchC(void* data);
    virtual void Callback_SwitchT(void* data);
    virtual void Callback_RequestC(void* data);
    virtual void Callback_Respond(void* data);
    virtual void Callback_Relay(void* data);
    virtual void Callback_NotifyNM(void* data);
    virtual void Callback_ApplyNM(void* data);

    virtual unsigned char Check_CondC(void* data);
    virtual unsigned char Check_NotCondC(void* data);
    virtual unsigned char Check_CondF(void* data);
    virtual unsigned char Check_NotCondF(void* data);
    virtual unsigned char Check__RequestC(void* data);
    virtual unsigned char Check__Respond(void* data);
    virtual unsigned char Check_Accept(void* data);
    virtual unsigned char Check_Reject(void* data);
    virtual unsigned char Check__Message(void* data);
    virtual unsigned char Check__Relay(void* data);
    virtual unsigned char Check__Exchange(void* data);
    virtual unsigned char Check_Chosen(void* data);
    virtual unsigned char Check_NotChosen(void* data);
    virtual unsigned char Check_NearToLF(void* data);
    virtual unsigned char Check_NotNearToLF(void* data);
    virtual unsigned char Check_CondNM(void* data);
    virtual unsigned char Check_NotCondNM(void* data);
    virtual unsigned char Check__NotifyNM(void* data);
    virtual unsigned char Check_InitC(void* data);

protected:

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
    /* Pointer to the ground sensor */
    CCI_GroundSensor* m_pcGround;

    /* The turning parameters. */
    SWheelTurningParams m_sWheelTurningParams;
    /* The flocking interaction parameters between teammates. */
    SFlockingInteractionParams m_sTeamFlockingParams;
    /* The connector's target distance */
    Real m_unConnectorTargetDistance;
    /* The traveler's joining threshold */
    Real m_unTravelerJoiningThres;

    /* Weights using for the motion in each role */
    Real followerAttraction, followerRepulsion, followerObstacle;
    Real connectorAttractionToConnector, connectorAttractionToTeam, connectorRepulsion, connectorObstacle;
    Real travelerAttraction, travelerRepulsion, travelerObstacle;

    /* Controller */
    std::unique_ptr<SCT> sct;

    /* Last controllable action */
    std::string lastControllableAction;

    /* Current robot state */
    RobotState currentState;

    /* Current team ID, which is the number of the leader ID (e.g. L1 -> 1) */
    UInt8 teamID;
    UInt8 prevTeamID;

    /* Outgoing message */
    CByteArray cbyte_msg;

    /* Messages received from nearby robots */
    Message leaderMsg;
    std::vector<Message> teamMsgs;
    std::vector<Message> connectorMsgs;
    std::vector<Message> otherLeaderMsgs;
    std::vector<Message> otherTeamMsgs;
    std::vector<Message> travelerMsgs;

    /* The number of hops from the leader to itself */
    UInt8 hopCountToLeader;  // default to 255 if unknown // (used in the FOLLOWER state)

    /* The number of hops to each team (used in the CONNECTOR state) */
    std::map<UInt8, HopMsg> hopsDict;
    // The hop count info of the connector this robot will connect with (used in the FOLLOWER state)
    std::map<UInt8, HopMsg> hopsCopy;
    /* The previous connection if hop is still new (i.e. connection in hopsDict still appears in cmsgToResend) */
    std::map<UInt8, HopMsg> prevHops;

    /* Sensor reading results */
    std::map<UInt8, Message> connectionCandidates; // (used in the FOLLOWER state)
    Message firstConnector; // The connector that a follower in the team should connect to next

    bool nearToLF; // (used in the TRAVELER state)
    std::map<UInt8, RoleSwitchConditions> roleSwitchConditionsPerTeam;
    bool receivedRequest, receivedAccept, receivedReject, receivedInwardRelayMsg, receivedOutwardRelayMsg, receivedInwardSendMsg, receivedOutwardSendMsg;
    bool responseToProcess;
    bool waitingResponse;

    ConnectionMsg currentRequest, currentResponse;
    int requestTimer; // Remaining timesteps to wait since a request was made

    /* Task and team switch signals from leader */
    UInt8 leaderSignal = 1; // DEFAULT TO WORK ON TASK : 0 = stop working on task, 1 = start working on task (used in the FOLLOWER state)
    std::string robotToSwitch; // (used in the FOLLOWER state)
    UInt8 teamToMove; // Direction to move along the network
    UInt8 teamToJoin; // Team to join

    std::map<UInt8, Message> robotsToAccept; // List of robots to accept as connectors (used in the CONNECTOR state)

    /* Connection related info to send in the current timestep */
    std::vector<ConnectionMsg> cmsgToSend;
    std::vector<std::pair<size_t, ConnectionMsg>> cmsgToResend; // ConnectionMsg attached with a timer. Messages gets added into cmsgToSend while timer is running
    
    /* Team Shared Message */
    std::map<UInt8, TeamSharedMsg> teamSharedMsgDict;

    std::vector<RelayMsg> rmsgToSend;
    std::vector<std::pair<size_t, RelayMsg>> rmsgToResend;
    std::map<UInt8, std::pair<RelayMsg, char>> lastBeat; // RelayMsg indexed with teamID and attached with a boolean to determine whether a new message was received in this timestep
    // (used in the CONNECTOR state)

    /* Flag to indicate whether this robot is working on a task */
    bool bPerformingTask;

    /* Teams that are within the safety range and the average distance to the team */
    std::map<UInt8, int> nearbyTeams;

    /* Timer to count the timesteps for the initial communication to occur at the beginning of the simulation */
    size_t initStepTimer;
    int tailSwitchTimer; // Timer that decrements. Prevent connectors from letting a neighbor become the new tail connector when it has just become one
    int connectorSwitchTimer; // Timer that decrements. Prevent connectors from triggering network change when it has just become a connector
    int networkChangeTimer; // Timer that decrements. Prevent connectors from triggering network change when it has just applied another network change

    /* Network change messages to broadcast */
    std::map<UInt8, std::pair<std::string, Message>> networkChanges; // key: team, pair<prevRobotID, newRobotMessage>
    bool notifySent;
    std::vector<Message> nmsgsReceived;
    std::vector<NetworkChangeMsg> nmsgToSend;
    std::vector<std::pair<size_t, NetworkChangeMsg>> nmsgToResend;
    std::vector<std::pair<size_t, std::string>> newConnections; // robot ids attached with a timer. Any robot ids here will be ignored even if the other connector doesn't have its id in their hopsDict

    /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><epuck_obstacleavoidance_controller> section.
    */
    /* Chain formation threshold */
    Real separationThres;
    Real joiningThres;

    size_t sendDuration;
    size_t sendRespondDuration;
    size_t waitRequestDuration;

    /* SCT yaml path */
    std::string m_strSCTPath;

    /* Experiment to change rab_range */
    static constexpr Real fDefaultRABRange = 0.8;
    Real m_fRABRange;
    Real m_fRABFactor;

};

#endif
