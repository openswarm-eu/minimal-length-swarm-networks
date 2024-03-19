/* Include the controller definition */
#include "leader.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <utility/team_color.h>
#include <algorithm>

/****************************************/
/****************************************/

static const std::vector<CRadians> PROX_ANGLE {
                                                CRadians::PI / 10.5884f,
                                                CRadians::PI / 3.5999f,
                                                CRadians::PI_OVER_TWO,  // side sensor
                                                CRadians::PI / 1.2f,    // back sensor
                                                CRadians::PI / 0.8571f, // back sensor
                                                CRadians::PI / 0.6667f, // side sensor
                                                CRadians::PI / 0.5806f,
                                                CRadians::PI / 0.5247f
                                              };

/****************************************/
/****************************************/

void CLeader::SWheelTurningParams::Init(TConfigurationNode& t_node) {
    try {
        TurningMechanism = NO_TURN;
        CDegrees cAngle;
        GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
        HardTurnOnAngleThreshold = ToRadians(cAngle);
        GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
        SoftTurnOnAngleThreshold = ToRadians(cAngle);
        GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
        NoTurnAngleThreshold = ToRadians(cAngle);
        GetNodeAttribute(t_node, "max_speed", MaxSpeed);
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
    }
}

/****************************************/
/****************************************/

void CLeader::SWaypointTrackingParams::Init(TConfigurationNode& t_node) {
   try {
      GetNodeAttribute(t_node, "target_angle", TargetAngle);
      GetNodeAttribute(t_node, "kp", Kp);
      GetNodeAttribute(t_node, "ki", Ki);
      GetNodeAttribute(t_node, "kd", Kd);
      GetNodeAttribute(t_node, "thres_range", thresRange);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller waypoint tracking parameters.", ex);
   }
}

/****************************************/
/****************************************/
void CLeader::SFlockingInteractionParams::Init(TConfigurationNode& t_node) {
   try {
      GetNodeAttribute(t_node, "target_distance", TargetDistance);
      GetNodeAttribute(t_node, "gain", Gain);
      GetNodeAttribute(t_node, "exponent", Exponent);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller flocking parameters.", ex);
   }
}

/****************************************/
/****************************************/

/*
 * This function is a generalization of the Lennard-Jones potential
 */
Real CLeader::SFlockingInteractionParams::GeneralizedLennardJones(Real f_distance) {
   Real fNormDistExp = ::pow(TargetDistance / f_distance, Exponent);
   return -Gain / f_distance * (fNormDistExp * fNormDistExp - fNormDistExp);
}

/****************************************/
/****************************************/

/*
 * This function is a generalization of the Lennard-Jones potential for repulsion only 
 */
Real CLeader::SFlockingInteractionParams::GeneralizedLennardJonesRepulsion(Real f_distance) {
   Real fNormDistExp = ::pow(TargetDistance / f_distance, Exponent);
   return -Gain / f_distance * (fNormDistExp * fNormDistExp);
}

/****************************************/
/****************************************/

CLeader::CLeader() :
    m_pcWheels(NULL),
    m_pcProximity(NULL),
    m_pcRABAct(NULL),
    m_pcRABSens(NULL),
    m_pcLEDs(NULL),
    m_pcPosSens(NULL),
    m_bSelected(false),
    m_strUsername(""),
    m_bSignal(false),
    PIDHeading(NULL),
    inTask(false),
    nearRobot(false) {}

/****************************************/
/****************************************/

void CLeader::Init(TConfigurationNode& t_node) {

    /* Get sensor/actuator handles */
    m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor  <CCI_ProximitySensor             >("proximity"            );
    m_pcRABAct    = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
    m_pcRABSens   = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );
    m_pcLEDs      = GetActuator<CCI_LEDsActuator                >("leds"                 );
    m_pcPosSens   = GetSensor  <CCI_PositioningSensor           >("positioning"          );

    /*
    * Parse the config file
    */
    try {
        /* Wheel turning */
        m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
        /* Waypoint tracking */
        m_sWaypointTrackingParams.Init(GetNode(t_node, "waypoint_tracking"));
        /* Flocking-related */
        m_sTeamFlockingParams.Init(GetNode(t_node, "team_flocking"));
        /* Minimum distance from robot */
        GetNodeAttribute(GetNode(t_node, "team_distance"), "min_leader_robot_distance", minDistanceFromRobot);
        GetNodeAttribute(GetNode(t_node, "team_distance"), "separation_threshold", separationThres);
        /* Minimum duration the accept message will be sent for */
        GetNodeAttribute(GetNode(t_node, "timeout"), "send_message", sendDuration);
        /* Time to wait between sending each robot */
        GetNodeAttribute(GetNode(t_node, "timeout"), "send_robot_delay", sendRobotDelay);
        /* SCT Model */
        GetNodeAttribute(GetNode(t_node, "SCT"), "path", m_strSCTPath);
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
    }
    // std::cout << m_strSCTPath << std::endl;

    /* Get team ID from leader ID */
    teamID = stoi(GetId().substr(1));

    float timeInSeconds = sendRobotDelay / 10.0;
    sendRobotDelay = (size_t)ceil(timeInSeconds) * 10;

    /* Initialization */
    currentState = RobotState::LEADER;
    inputStart = false;
    inputStop = true;
    
    currentTaskId = "";
    currentTaskDemand = 0;
    currentInitTaskDemand = 0;

    numOtherTaskRequire = 0;
    numOtherFollower = -1;

    initStepTimer = 0;
    robotLastSentTime = 0;
    robotLastSent = "";
    acceptCmsg = ConnectionMsg();

    lastSent = -1;
    beatSent = 0;

    numRobotsToSend = 0;
    numRobotsRemainingToSend = 0;
    numRobotsToRequest = 0;
    numRobotsRemainingToRequest = 0;
    isSendingRobots = false;
    switchCandidate = "";
    switchCandidateDist = 255;
    robotToSwitch = "";
    
    decremented = false;
    robotsNeeded = 0;
    requestSent = false;
    acknowledgeSent = false;
    // requestReceived = false;

    /* Behavior analysis */
    allRequestsSatisfied = false;
    allRequestsSatisfiedTime = 0;

    previousFollowerCount = 0;

    /*
    * Init SCT Controller
    */
    sct = std::make_unique<SCTPub>(m_strSCTPath);

    /* Register controllable events */
    sct->add_callback(this, std::string("EV_start"),    &CLeader::Callback_Start,    NULL, NULL);
    sct->add_callback(this, std::string("EV_stop"),     &CLeader::Callback_Stop,     NULL, NULL);
    // sct->add_callback(this, std::string("EV_respond"],  &CLeader::Callback_Respond,  NULL, NULL);
    sct->add_callback(this, std::string("EV_message"),  &CLeader::Callback_Message,  NULL, NULL);
    sct->add_callback(this, std::string("EV_exchange"), &CLeader::Callback_Exchange, NULL, NULL);

    /* Register uncontrollable events */
    // sct->add_callback(this, std::string("EV__requestL"],     NULL, &CLeader::Check__RequestL,     NULL);
    sct->add_callback(this, std::string("EV_pressStart"),    NULL, &CLeader::Check_PressStart,    NULL);
    sct->add_callback(this, std::string("EV_pressStop"),     NULL, &CLeader::Check_PressStop,     NULL);
    sct->add_callback(this, std::string("EV_inputMessage"),  NULL, &CLeader::Check_InputMessage,  NULL);
    sct->add_callback(this, std::string("EV_inputExchange"), NULL, &CLeader::Check_InputExchange, NULL);

    /* Set LED color */
    // m_pcLEDs->SetAllColors(teamColor[teamID]);
    m_pcLEDs->SetAllColors(CColor::RED);

    /* Init PID Controller */
    PIDHeading = new PID(0.1,                             // dt  (loop interval time)
                         m_sWheelTurningParams.MaxSpeed,  // max
                         -m_sWheelTurningParams.MaxSpeed, // min
                         m_sWaypointTrackingParams.Kp,    // Kp
                         m_sWaypointTrackingParams.Ki,    // Ki
                         m_sWaypointTrackingParams.Kd);   // Kd

    Reset();
}

/****************************************/
/****************************************/

CLeader::~CLeader() {
    delete PIDHeading;
}

/****************************************/
/****************************************/

void CLeader::Reset() {

    /* Initialize the msg contents to 255 (Reserved for "no event has happened") */
    m_pcRABAct->ClearData();
    cbyte_msg = CByteArray(MESSAGE_BYTE_SIZE, 255);
    m_pcRABAct->SetData(cbyte_msg);

    /* Reset the incoming public events */
    // pub_events.clear();
    // pub_events[EV_b] = false;
}

/****************************************/
/****************************************/

void CLeader::ControlStep() {

    // LOG << "---------- " << this->GetId() << " ----------" << std::endl;

    initStepTimer++;
    // std::cout << "TIME: " << initStepTimer << std::endl;

    // for(int i = 0; i < waypoints.size(); i++) {
    //     //std::cout << waypoints[i].GetX() << "," << waypoints[i].GetY() << std::endl;
    // }

    /*-----------------*/
    /* Reset variables */
    /*-----------------*/
    ResetVariables();

    /*----------------------*/
    /* Receive new messages */
    /*----------------------*/
    GetMessages();
    
    /*------------------------*/
    /* Update sensor readings */
    /*------------------------*/
    Update();

    /*--------------------*/
    /* Run SCT controller */
    /*--------------------*/
    //std::cout << "--- Supervisors ---" << std::endl;

    if(initStepTimer > 4)
        sct->run_step();    // Run the supervisor to get the next action
    
    // RLOG << "" << sct->get_current_state_string() << std::endl;
    // RLOG << "Action: " << lastControllableAction << std::endl;

    /*-----------------------------*/
    /* Implement action to perform */
    /*-----------------------------*/

    /*---------*/
    /* Control */
    /*---------*/

    /* Store previous task demand */
    previousTaskDemand = currentTaskDemand;

    /* Set ConnectionMsg to send during this timestep */
    //std::cout << "resend size: " << cmsgToResend.size() << std::endl;
    for(auto it = cmsgToResend.begin(); it != cmsgToResend.end();) {
        if(it->first > 0) {
            cmsgToSend.push_back(it->second);
            it->first--; // Decrement timer
            ++it;
        } else {
            it = cmsgToResend.erase(it);
            acceptCmsg = ConnectionMsg();
        }
    }

    /* Set RelayMsg to send during this timestep */
    //std::cout << "resend size: " << rmsgToResend.size() << std::endl;
    for(auto it = rmsgToResend.begin(); it != rmsgToResend.end();) {
        if(it->first > 0) {
            rmsgToSend.push_back(it->second);
            it->first--; // Decrement timer
            ++it;
        } else {
            it = rmsgToResend.erase(it);
        }
    }

    /* Set LED */
    if(m_bSignal)
        m_pcLEDs->SetAllColors(CColor::WHITE);
    else {
        // m_pcLEDs->SetAllColors(teamColor[teamID]);
        m_pcLEDs->SetAllColors(CColor::RED);
    }
    // if(m_bSelected)
    //     m_pcLEDs->SetAllColors(CColor::RED);
    // else
    //     m_pcLEDs->SetAllColors(CColor::BLACK);

    /* Set Motion */
    if(initStepTimer > 4) {

        /* Is the robot selected? */
        if(m_bSelected) {

            /* Follow the control vector */
            SetWheelSpeedsFromVectorEightDirections(m_cControl);
        }
        else {
            if( !nearRobot ) {
                /* Stop if other robots are too far from itself */
                m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
            }
            else if( !waypoints.empty() ) {
                /* Check if it is near the waypoint */
                CVector3 pos3d = m_pcPosSens->GetReading().Position;
                CVector2 pos2d = CVector2(pos3d.GetX(), pos3d.GetY());
                Real dist = (waypoints.front() - pos2d).Length();
                inTask = (dist <= m_sWaypointTrackingParams.thresRange);
                // RLOG << "dist: " << dist << ", inTask: " << inTask << " waypoint: " << waypoints.front().Length() << std::endl;

                /* If current task is completed, move to the next one */
                // if( !inTask || currentTaskDemand == 0) {
                if( !inTask ) { //############### TEMP FOR CIRCLE_TASK_NO_DEMAND ################

                    //std::cout << "[LOG] Moving to next task" << std::endl;

                    std::vector<Message> repulseMsgs;

                    /* Add robots to repel from */
                    repulseMsgs.insert(std::end(repulseMsgs), std::begin(otherLeaderMsgs), std::end(otherLeaderMsgs));
                    repulseMsgs.insert(std::end(repulseMsgs), std::begin(otherTeamMsgs), std::end(otherTeamMsgs));
                    repulseMsgs.insert(std::end(repulseMsgs), std::begin(connectorMsgs), std::end(connectorMsgs));
                    repulseMsgs.insert(std::end(repulseMsgs), std::begin(travelerMsgs), std::end(travelerMsgs));

                    /* Calculate overall force applied to the robot */
                    /* ########################### */
                    CVector2 waypointForce = VectorToWaypoint();           // Attraction to waypoint
                    CVector2 robotForce    = GetRobotRepulsionVector(repulseMsgs);    // Repulsion from other robots
                    CVector2 obstacleForce = GetObstacleRepulsionVector(); // Repulsion from obstacles

                    CVector2 sumForce      = waypointForce + robotForce + obstacleForce;
                    /* ########################### */

                    // RLOG << "waypointForce: " << waypointForce << std::endl;
                    //std::cout << "robotForce: " << robotForce << std::endl;
                    //std::cout << "obstacleForce: " << obstacleForce << std::endl;
                    //std::cout << "sumForce: " << sumForce << std::endl;

                    SetWheelSpeedsFromVectorHoming(sumForce);
                } 
                else {
                    m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
                }
            }
            else {
                m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
                inTask = true;

                // FOR TRAINING

                /* Send the requested number of robots to the other leader */
                // if( !isSendingRobots ) {

                //     if(numRobotsRequested > 0) {
                //         SetRobotsToSend(numRobotsRequested);
                //         isSendingRobots = true;
                //         numRobotsRequested = 0; // reset
                //     }
                // }

                // if(numRobotsToSend == 0) {
                //     isSendingRobots = false;
                // }
            }
        }
    }
    
    /* Create new message to send */
    Message msg = Message();

    /* Set message content */
    msg.state = currentState;
    msg.ID = this->GetId();
    msg.teamID = teamID;
    msg.leaderSignal = m_bSignal;
    if( !robotToSwitch.empty() ) {
        msg.robotToSwitch = robotToSwitch;
        msg.teamToJoin = teamToJoin;
    }
    
    /* Team Hop Count */
    msg.teamHopCount = 0;
    // HopMsg hop;
    // hop.count = 0;
    // // Skip ID

    // msg.hops[teamID] = hop;

    /* Team Shared Message */
    for(const auto& it : teamSharedMsgDict) {
        msg.tmsg[it.first] = it.second;
    }

    /* Network Hop Count */
    for(const auto& it : hopsDict) {

        HopMsg hop = HopMsg();

        hop.count = it.second.count;

        if( !it.second.ID.empty() )
            hop.ID = it.second.ID;

        hop.resendCount = it.second.resendCount + 1; // Increment because it is increasing the number of times this message is being resent

        msg.hops[it.first] = hop;
    }

    /* Connection Message */
    for(const auto& conMsg : cmsgToSend) {
        msg.cmsg.push_back(conMsg);
    }

    // Skip Teams Nearby

    /* Relay Message */
    for(const auto& relayMsg : rmsgToSend) {
        msg.rmsg.push_back(relayMsg);
    }

    /* Set ID of all connections to msg (only those involved in modifying the network) */
    std::vector<Message> allMsgs(teamMsgs);
    allMsgs.insert(std::end(allMsgs), std::begin(connectorMsgs), std::end(connectorMsgs));
    allMsgs.insert(std::end(allMsgs), std::begin(otherLeaderMsgs), std::end(otherLeaderMsgs));
    allMsgs.insert(std::end(allMsgs), std::begin(otherTeamMsgs), std::end(otherTeamMsgs));

    for(size_t i = 0; i < allMsgs.size(); i++) {
        msg.connections.push_back(allMsgs[i].ID);
    }

    // msg.Print();

    /* Convert message into CByteArray */
    cbyte_msg = msg.GetCByteArray();

    /*--------------*/
    /* Send message */
    /*--------------*/
    m_pcRABAct->SetData(cbyte_msg);

}

/****************************************/
/****************************************/

void CLeader::Select() {
   m_bSelected = true;
//    m_pcLEDs->SetAllColors(CColor::RED);
}

/****************************************/
/****************************************/

void CLeader::Deselect() {
   m_bSelected = false;
//    m_pcLEDs->SetAllColors(CColor::BLACK);
}

/****************************************/
/****************************************/

std::string CLeader::GetUsername() {
    return m_strUsername;
}

/****************************************/
/****************************************/

void CLeader::SetUsername(std::string username) {
    m_strUsername = username;
}

/****************************************/
/****************************************/

void CLeader::SetControlVector(const CVector2& c_control) {
   m_cControl = c_control;
}

/****************************************/
/****************************************/

void CLeader::SetSignal(const bool b_signal) {
    if(b_signal) {
        inputStart = true;
        inputStop = false;
    } else {
        inputStop = true;
        inputStart = false;
    }
}

/****************************************/
/****************************************/

void CLeader::SetRobotsToRequest(const UInt32 un_robots) {

    RLOG << "Received " << un_robots << " robots to request from user" << std::endl;

    numRobotsToRequest = un_robots;
}

/****************************************/
/****************************************/

void CLeader::SetRobotsToSend(const UInt32 un_robots) {

    RLOG << "Received " << un_robots << " robots to send from user" << std::endl;

    if(currentFollowerCount <= 1) {
        LOG << "{" << this->GetId() << "}[LOG] Cannot send if robots <= 1 " << std::endl;
        return;
    } else if(currentFollowerCount <= un_robots) { // If robots to send exceed current team size, send all followers
        numRobotsToSend = currentFollowerCount - 1;
    } else {
        numRobotsToSend = un_robots;
    }

    numRobotsRemainingToSend = numRobotsToSend;
}

/****************************************/
/****************************************/

void CLeader::SetRobotsToSend(const UInt32 un_robots, const UInt8 un_team) {
    teamToJoin = un_team;
    SetRobotsToSend(un_robots);
    RLOG << "Send to team " << teamToJoin << std::endl;
}

/****************************************/
/****************************************/

CVector2 CLeader::GetNextWaypoint() {
    return waypoints.front();
}

/****************************************/
/****************************************/

void CLeader::SetWaypoints(const std::queue<CVector2> waypts) {
    waypoints = waypts;
}

/****************************************/
/****************************************/

std::string CLeader::GetTaskId() {
    return currentTaskId;
}

/****************************************/
/****************************************/

void CLeader::SetTaskId(const std::string str_id) {
    currentTaskId = str_id;
}

/****************************************/
/****************************************/

UInt32 CLeader::GetTaskDemand() {
    return currentTaskDemand;
}

/****************************************/
/****************************************/

void CLeader::SetTaskDemand(const UInt32 un_demand) {
    currentTaskDemand = un_demand;
}

/****************************************/
/****************************************/

UInt32 CLeader::GetInitTaskDemand() {
    return currentInitTaskDemand;
}

/****************************************/
/****************************************/

void CLeader::SetInitTaskDemand(const UInt32 un_init_demand) {
    currentInitTaskDemand = un_init_demand;
}

/****************************************/
/****************************************/

UInt32 CLeader::GetMinimumCount() {
    return robotsNeeded;
}

/****************************************/
/****************************************/

void CLeader::SetMinimumCount(const UInt32 un_min) {
    robotsNeeded = un_min;
}

/****************************************/
/****************************************/

UInt32 CLeader::GetOtherMinimumCount() {
    return numOtherTaskRequire;
}

/****************************************/
/****************************************/

UInt32 CLeader::GetFollowerCount() {
    return currentFollowerCount;
}

/****************************************/
/****************************************/

void CLeader::SetFollowerCount(const UInt32 un_count) {
    currentFollowerCount = un_count;
}

/****************************************/
/****************************************/

SInt32 CLeader::GetOtherFollowerCount() {
    return numOtherFollower;
}

/****************************************/
/****************************************/

UInt8 CLeader::GetTeamID() {
    return teamID;
}

/****************************************/
/****************************************/

const std::map<UInt8, HopMsg>& CLeader::GetHops() const {
    return hopsDict;
}

/****************************************/
/****************************************/

Real CLeader::GetLatestTimeSent() {
    return lastSent;
}

/****************************************/
/****************************************/

// Real CLeader::GetLatestTimeReceived() {
//     return lastBeatTime;
// }

/****************************************/
/****************************************/

Real CLeader::GetTotalSent() {
    return beatSent;
}

/****************************************/
/****************************************/

// Real CLeader::GetTotalReceived() {
//     return beatReceived;
// }

/****************************************/
/****************************************/

std::string CLeader::GetLastAction() {
    return lastControllableAction;
}

/****************************************/
/****************************************/

UInt32 CLeader::GetRobotSendDelay() {
    return sendRobotDelay;
}

/****************************************/
/****************************************/

void CLeader::ResetVariables() {
    /* Clear messages received */
    teamMsgs.clear();
    connectorMsgs.clear();
    otherLeaderMsgs.clear();
    otherTeamMsgs.clear();
    travelerMsgs.clear();

    cmsgToSend.clear();
    rmsgToSend.clear();

    teamSharedMsgDict.clear();

    nearRobot = false;
    inTask = false;

    receivedMessage = false;
    receivedRelay = false;
    receivedRequest = false;
    inputMessage = false;

    lastControllableAction = "";

    robotToSwitch = "";

}

/****************************************/
/****************************************/

void CLeader::GetMessages() {
    
    /* Get RAB messages from nearby e-pucks */
    const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();

    if( !tMsgs.empty() ) {
        for(int i = 0; i < tMsgs.size(); i++) {

            Message msg = Message(tMsgs[i]);

            /* Store message */
            if(msg.state == RobotState::LEADER) {
                msg.ID = 'L' + msg.ID;
                otherLeaderMsgs.push_back(msg);

            } else if(msg.state == RobotState::FOLLOWER) {
                msg.ID = 'F' + msg.ID;

                if(msg.teamID == teamID)
                    teamMsgs.push_back(msg);
                else
                    otherTeamMsgs.push_back(msg);

            } else if(msg.state == RobotState::CONNECTOR) {
                msg.ID = 'F' + msg.ID;
                connectorMsgs.push_back(msg);
            } else if(msg.state == RobotState::TRAVELER) {
                msg.ID = 'F' + msg.ID;
                travelerMsgs.push_back(msg);
            }
        }
    }
}

/****************************************/
/****************************************/

void CLeader::Update() {

    nearRobot = IsNearRobot();

    /* Update knowledge of all teams */
    for(const auto& [team, hop] : hopsDict)
        otherTeams.insert(team);
    /* Refill teams that it can send a request to */
    if(teamsToRequest.empty())
        teamsToRequest = otherTeams;

    // if(GetId() == "L4") {
    //     for(const auto& team : teamsToRequest) {
    //         RLOG << team << std::endl;
    //     }
    // }

    UpdateHopCounts();

    SetConnectorToRelay();

    // ReplyToRequest();

    CheckHeartBeat();

    /* If there are no followers in the team, cancel sending the */
    /* remaining number of robots                                */
    if(numRobotsToSend > 0 && currentFollowerCount == 0) {
        numRobotsToSend = 0;
        numRobotsRemainingToSend = 0;
    }

    /* Only for simulated users */
    /* Check if task is completed or not to set signal to send */
    if( !m_bSelected ) {

        /* Simulated user signal */
        bool signal = m_bSignal;

        if( !waypoints.empty() ) {

            /* Check if it is near the waypoint */
            CVector3 pos3d = m_pcPosSens->GetReading().Position;
            CVector2 pos2d = CVector2(pos3d.GetX(), pos3d.GetY());
            Real dist = (waypoints.front() - pos2d).Length();

            if(dist < m_sWaypointTrackingParams.thresRange) {
                // FOR CIRCLE_TASK
                /* Check if task is completed */
                if(currentTaskDemand == 0) {
                    signal = false;
                    waypoints.pop(); // Delete waypoint from queue
                    requestSent = false; // Set to false since it has finished the task.
                } else
                    signal = true;
            }
        }

        if(signal && !m_bSignal)
            inputStart = true;
        else if(!signal && m_bSignal)
            inputStop = true;   
    }
}

/****************************************/
/****************************************/

bool CLeader::IsNearRobot() {
    
    /* Combine messages received */
    std::vector<Message> combinedMsgs(connectorMsgs);
    combinedMsgs.insert(std::end(combinedMsgs),
                        std::begin(teamMsgs),
                        std::end(teamMsgs));
    combinedMsgs.insert(std::end(combinedMsgs),
                        std::begin(otherLeaderMsgs),
                        std::end(otherLeaderMsgs));
    combinedMsgs.insert(std::end(combinedMsgs),
                        std::begin(otherTeamMsgs),
                        std::end(otherTeamMsgs));

    /* Check whether there is a neighbor (within threshold) */
    for(int i = 0 ; i < combinedMsgs.size(); i++) {
        Real dist = combinedMsgs[i].direction.Length();
        if(dist < minDistanceFromRobot)
            return true;
    }
    return false;
}

/****************************************/
/****************************************/

void CLeader::ReplyToRequest() {

    // Try printing all the requests received

    // for(const auto& teamMsg : teamMsgs) {
    //     // RLOG << "ID: " << teamMsg.ID << std::endl; 

    //     for(const auto& cmsg : teamMsg.cmsg) {
    //         // RLOG << "something cmsg " << std::endl; 
    //         if(cmsg.to == this->GetId() && cmsg.type == 'R') {
    //             RLOG << "got request from " << cmsg.from << " for team " << cmsg.otherTeam << std::endl; 
    //         }
    //     }
    // }

    /* Check if it has not recently sent an accept message */
    if( !cmsgToResend.empty() )
        return;

    /* For each team, find the shortest distance */
    std::map<UInt8, Real> teamShortestDistance;
    for(const auto& [team, hop] : hopsDict) {
        UInt8 teamMinDist = 255;
        for(auto& msg : teamMsgs) {
            if(msg.tmsg[team].dist < teamMinDist) {
                teamMinDist = msg.tmsg[team].dist;
                // RLOG << this->GetId() << ": dist is " << msg.tmsg[team].dist << " from " << msg.ID << " for team " << team << std::endl;
            }
        }
        teamShortestDistance[team] = teamMinDist;
    }

    /* Find the longest "shortest distamce" found in the previous step */
    UInt8 furthestTeam = 0;
    Real furthestTeamDist = 0;
    for(const auto& [team, dist] : teamShortestDistance) {
        if(dist > furthestTeamDist) {
            furthestTeam = team;
            furthestTeamDist = dist;
        }
    }

    // RLOG << "Furthest team: " << furthestTeam << ", dist: " << furthestTeamDist << std::endl;

    if(furthestTeam != 0 && teamSharedMsgDict[furthestTeam].connectorIdDownstream.empty() && furthestTeamDist > separationThres) {

        /* Upon receiving a request message, send an accept message to the follower with the smallest ID */
        for(const auto& teamMsg : teamMsgs) {
            for(const auto& cmsg : teamMsg.cmsg) {

                if(cmsg.to == this->GetId() && cmsg.type == 'R') {
                    receivedRequest = true;

                    /* Set the ID of the first follower request seen */
                    if(acceptCmsg.from.empty()) {
                        acceptCmsg = cmsg;
                        // RLOG << "Potential accept " << acceptCmsg.from << std::endl;
                        continue;
                    }

                    UInt8 currentFID = stoi(acceptCmsg.from.substr(1));
                    UInt8 newFID = stoi(cmsg.from.substr(1));

                    /* Send an accept message to the follower with the smallest ID */
                    if(newFID < currentFID) {
                        acceptCmsg = cmsg;
                    }
                }
            }
        }
    }
}

/****************************************/
/****************************************/

void CLeader::UpdateHopCounts() {

    /* Get connector messages that has connection to this team */
    std::vector<Message> adjacentConnectorMsgs;
    for(const auto& msg : connectorMsgs) {
        for(const auto& hop : msg.hops) {
            if(hop.first == teamID) {

                if(hop.second.ID == "" || hop.second.ID[0] != 'F') {
                    /* Connector is a tail connector or adjacent to this team */
                    adjacentConnectorMsgs.push_back(msg);
                    break;
                }
            }
        }
    }

    /* Get team IDs from team and adjacent connectors */
    std::vector<Message> combinedMsgs(adjacentConnectorMsgs);
    combinedMsgs.insert(std::end(combinedMsgs), std::begin(teamMsgs), std::end(teamMsgs));

    std::set<UInt8> teamIDs;
    for(const auto& msg : combinedMsgs) {
        for(const auto& hop : msg.hops) {
            teamIDs.insert(hop.first);
        }
    }
    teamIDs.erase(teamID); // Delete its own teamID

    for(auto& otherTeamID : teamIDs) {

        /* Loop connector (that has connection to this team) */

        bool connectorFound = false;

        for(auto& msg : adjacentConnectorMsgs) {

            /* If connector is a predecessor, use it to update its own hop count */
            if(msg.hops[otherTeamID].ID.empty() || msg.hops[otherTeamID].ID[0] == 'F') {

                if(hopsDict.count(otherTeamID) == 0)
                    hopsDict[otherTeamID] = HopMsg();
                
                hopsDict[otherTeamID].count = msg.hops[otherTeamID].count + 1;
                hopsDict[otherTeamID].ID = msg.ID;
                hopsDict[otherTeamID].resendCount = 0;
                connectorFound = true;
                break;
            }
        }

        if( !connectorFound ) {
            /* Loop teammates */
            std::vector<Message> combinedTeamMsgs(teamMsgs);

            HopMsg minHop;
            UInt8 minResendCount = 255;

            for(auto& msg : combinedTeamMsgs) {
                if(msg.hops[otherTeamID].resendCount < minResendCount) {
                    minHop = msg.hops[otherTeamID];
                    minResendCount = msg.hops[otherTeamID].resendCount;
                }
            }

            if(minResendCount < 255) {
                hopsDict[otherTeamID] = minHop;
            }
        }
    }
}

/****************************************/
/****************************************/

void CLeader::SetConnectorToRelay() {

    /* Get all visible teams from connectors and followers */
    std::vector<Message> combinedMsgs(connectorMsgs);
    combinedMsgs.insert(std::end(combinedMsgs), std::begin(teamMsgs), std::end(teamMsgs));

    std::set<UInt8> teamIDs;
    for(const auto& msg : combinedMsgs) {
        for(const auto& hop : msg.hops) {
            teamIDs.insert(hop.first);
        }
    }
    teamIDs.erase(teamID); // Delete its own teamID

    /* Add new entries to teamSharedMsgDict if it doesn't exist */
    for(const auto& otherTeamID : teamIDs) {
        if(teamSharedMsgDict.count(otherTeamID) == 0) {
            teamSharedMsgDict[otherTeamID] = TeamSharedMsg();
        }
    }

    // RLOG << "size: " << teamSharedMsgDict.size();

    /* Update connector to relay to the team */
    for(auto& teamMsg : teamSharedMsgDict) {

        UInt8 otherTeamID = teamMsg.first;

        // /* Check if it detects the tail connector directly */
        // bool connectorNearby = false;

        // UInt8 minHopCount = 255;
        // std::string minHopCountRobot = "";

        // for(const auto& msg : connectorMsgs) {
        //     /* Find connector with the smallest hop count to the team and record it */
        //     for (const auto&  hop : msg.hops) {
        //         if(hop.first == otherTeamID) {
        //             if(minHopCountRobot.empty() || hop.second.count < minHopCount) {
        //                 minHopCount = hop.second.count;
        //                 minHopCountRobot = msg.ID;
        //                 connectorNearby = true;
        //             }
        //         }
        //     }
        // }

        // if(connectorNearby) {
        //     teamMsg.second.connectorIdDownstream = minHopCountRobot; // Update ID to send downstream to team
        // } else {

            if( !teamMsgs.empty() ) {
                bool previousSeen = false;
                bool newValue = false;

                for(const auto& msg : teamMsgs) {
                    bool newInfoFound = false;
                    for(const auto& followerTeamMsg : msg.tmsg) {

                        if(followerTeamMsg.first == otherTeamID) {
                            if(followerTeamMsg.second.connectorIdUpstream == teamSharedMsgDict[otherTeamID].connectorIdDownstream) {

                                previousSeen = true; // Follower indicating the same connector as leader. Confirms leader's current data is correct

                            } else if( !followerTeamMsg.second.connectorIdUpstream.empty() ) {
                                
                                /* Update connector info */
                                teamMsg.second.connectorIdDownstream = followerTeamMsg.second.connectorIdUpstream;
                                newValue = true;
                                newInfoFound = true;
                                break;
                            }
                        }
                    }
                    if(newInfoFound)
                        break;
                }

                if( !previousSeen && !newValue ) // If previous info not received and no new info, reset downstream
                    teamMsg.second.connectorIdDownstream = "";

            } else {
                /* Found and received no info about this team, so reset downstream */
                teamMsg.second.connectorIdDownstream = "";
            }

        // }
        // LOG << "("  << teamMsg.first << "," << teamMsg.second.connectorIdDownstream << "), ";
    }
    // LOG << std::endl;
}

/****************************************/
/****************************************/

void CLeader::CheckHeartBeat() {

    std::vector<Message> combinedMsgs(otherLeaderMsgs);
    combinedMsgs.insert(std::end(combinedMsgs), std::begin(teamMsgs), std::end(teamMsgs));
    combinedMsgs.insert(std::end(combinedMsgs), std::begin(otherTeamMsgs), std::end(otherTeamMsgs));
    combinedMsgs.insert(std::end(combinedMsgs), std::begin(connectorMsgs), std::end(connectorMsgs));

    for(const auto& msg : combinedMsgs) {
        for(const auto& beat : msg.rmsg) {
            if(beat.from != this->GetId()) { 
                // RLOGERR << "received " << beat.time << " from " << beat.from << "! (" << beat.type << ", " << beat.firstFollower << ")" << std::endl;

                /* Get team ID number */
                UInt8 receivedTeamID = stoi(beat.from.substr(1));

                /* If receiving this team for the first time, initialize */
                if(lastBeatTime.count(receivedTeamID) == 0) {
                    lastBeatTime[receivedTeamID] = 0;
                    beatReceived[receivedTeamID] = 0;
                }

                if(beat.time > lastBeatTime[receivedTeamID]) {

                    /* Update */
                    lastBeatTime[receivedTeamID] = beat.time;
                    beatReceived[receivedTeamID]++;
                    // RLOGERR << " Received " << lastBeatTime[receivedTeamID] << " from " << beat.from << "! (" << beatReceived[receivedTeamID] << ")" << std::endl;
                    // if(this->GetId() == "L2")
                    //     std::cerr << this->GetId() << " received " << lastBeatTime << "! (" << beatReceived << ")" << std::endl;

                    if(msg.state == RobotState::LEADER)
                        receivedMessage = true;
                    else
                        receivedRelay = true;

                    decremented = false;

                    /* Store message info from other leader */
                    numOtherFollower = beat.follower_num;
                    numOtherTaskRequire = beat.task_min_num;

                    if(beat.type == 'R') {
                        // if( !requestReceived ) {
                        //     if(beat.robot_num != numPreviousRequest) {
                        //         numRobotsToSend += beat.robot_num - numPreviousRequest;
                        //         if(numRobotsToSend < 0)
                        //             numRobotsToSend = 0;
                        //         numPreviousRequest = beat.robot_num;
                        //         std::cout << this->GetId() << ": request from " << beat.from << " to send " << numRobotsToSend << " robots" << std::endl;

                        //     }
                        // }

                        // std::cout << "{" << this->GetId() << "} [REQUEST] Received request from " << beat.from << " to send " << numRobotsRequested << " robots" << std::endl;
                        if(beat.request_to_team == teamID) {
                            numRobotsRequested[receivedTeamID] = beat.robot_num;
                            teamToJoin = receivedTeamID; // TEMP: Assuming only one team ever requests (behaviour analysis)  
                            std::cout << "{" << this->GetId() << "}[REQUEST] Received request to send " << beat.robot_num << " robots" << std::endl;
                        }

                        // DEBUG
                        // if( !m_bSelected ) {
                        //     if(currentFollowerCount <= numRobotsRequested) {
                        //         numRobotsToSend = numRobotsRequested - 1; // Keep one follower and send the rest
                        //     } else {
                        //         numRobotsToSend = numRobotsRequested;
                        //     }
                        //     numRobotsRemainingToSend = numRobotsToSend;
                        //     // std::cout << "[LOG] " << numRobotsToSend << std::endl;
                        // }

                    } else if(beat.type == 'A' && beat.accept_to_team == teamID) {
                        std::cout << this->GetId() << " Received Acknowledge from " << beat.from << " who is sending " << beat.robot_num << std::endl;
                        if(beat.robot_num > 0) {
                            std::cout << "{" << this->GetId() << "}[SEND] " << beat.robot_num << " robots are heading this way!" << std::endl;
                            if(numRobotsRemainingToRequest > 0) {
                                if(beat.robot_num >= numRobotsRemainingToRequest) {
                                    numRobotsRemainingToRequest = 0; // No more robots to request
                                } else {
                                    numRobotsRemainingToRequest -= beat.robot_num; // Update remaining robots to request to other teams
                                }
                                allRequestsSatisfied = true;
                                allRequestsSatisfiedTime = initStepTimer;
                                // RLOGERR << allRequestsSatisfiedTime << std::endl;
                            }
                            requestSent = false;
                        } else {
                            std::cout << "{" << this->GetId() << "}[SEND] Request to team " << receivedTeamID << " was rejected" << std::endl;
                            requestSent = false;
                        }
                    }

                    // switchCandidate = ""; // Reset candidate follower to switch
                } 
                
                /* Set a follower that received the leader message from a non-team robot as a candidate to switch teams */
                if(!beat.firstFollower.empty() && msg.teamID == teamID && msg.ID != robotLastSent) {
                    bool update = false;
                    if(switchCandidate.empty()) {
                        update = true;
                        // RLOG << this->GetId() << ": switch candidate (Empty) " << beat.firstFollower << std::endl;
                    } else if(beat.firstFollowerDist < switchCandidateDist) {
                        update = true;
                        // RLOG << this->GetId() << ": switch candidate (Dist) " << beat.firstFollower << ", " << beat.firstFollowerDist << std::endl;
                    } else if(switchCandidateTimer + 10 < initStepTimer) {
                        update = true;
                        // RLOG << this->GetId() << ": switch candidate (Timer) " << beat.firstFollower << std::endl;
                    }

                    if(update) {
                        switchCandidate = beat.firstFollower;
                        switchCandidateDist = beat.firstFollowerDist;
                        switchCandidateTimer = initStepTimer;
                    }
                }
            }
        }
    }

    //std::cout << "lastBeatTime: " << lastBeatTime << std::endl;
    //std::cout << "beatReceived: " << beatReceived << std::endl;
}

/****************************************/
/****************************************/

CVector2 CLeader::VectorToWaypoint() {   
     /* Get current position */
    CVector3 pos3d = m_pcPosSens->GetReading().Position;
    CVector2 pos2d = CVector2(pos3d.GetX(), pos3d.GetY());
    CRadians cZAngle, cYAngle, cXAngle;
    m_pcPosSens->GetReading().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
    //std::cout << "pos2d " << pos2d << std::endl;

    /* Calculate a normalized vector that points to the next waypoint */
    CVector2 cAccum = waypoints.front() - pos2d;

    //std::cout << "cAccum: " << cAccum << std::endl;
    //std::cout << "angle: " << cAccum.Angle() << std::endl;

    cAccum.Rotate((-cZAngle).SignedNormalize());
    
    //std::cout << "cAccum: " << cAccum << std::endl;
    //std::cout << "angle: " << cAccum.Angle() << std::endl;

    if(cAccum.Length() > 0.0f) {
        /* Make the vector as long as the max speed */
        cAccum.Normalize();
        cAccum *= m_sWheelTurningParams.MaxSpeed;
    }
    //std::cout << "cAccum: " << cAccum << std::endl;
    return cAccum;
}

/****************************************/
/****************************************/

CVector2 CLeader::GetRobotRepulsionVector(std::vector<Message>& msgs) {
    
    CVector2 resVec = CVector2();

    for(size_t i = 0; i < msgs.size(); i++) {
        /* Calculate LJ */
        Real fLJ = m_sTeamFlockingParams.GeneralizedLennardJonesRepulsion(msgs[i].direction.Length());
        /* Sum to accumulator */
        resVec += CVector2(fLJ,
                           msgs[i].direction.Angle());
    }

    /* Calculate the average vector */
    if( !msgs.empty() )
        resVec /= msgs.size();

    /* Limit the length of the vector to the max speed */
    if(resVec.Length() > m_sWheelTurningParams.MaxSpeed) {
        resVec.Normalize();
        resVec *= m_sWheelTurningParams.MaxSpeed * 0.5;
    }

    return resVec;
}

/****************************************/
/****************************************/

CVector2 CLeader::GetObstacleRepulsionVector() {
    /* Get proximity sensor readings */
    std::vector<Real> fProxReads = m_pcProximity->GetReadings();

    CVector2 resVec = CVector2();

    for(size_t i = 0; i < fProxReads.size(); i++) {
        CVector2 vec = CVector2();
        if(fProxReads[i] > 0.0f) {
            Real length = (fProxReads[i] - 0.9) * m_sWheelTurningParams.MaxSpeed * 10; // Map length to 0 ~ max_speed
            vec = CVector2(length, PROX_ANGLE[i]);

            resVec -= vec; // Subtract because we want the vector to repulse from the obstacle
        }
        // //std::cout << "sensor " << i << ": " << vec.Length() << std::endl;
    }

    /* Clamp the length of the vector to the max speed */
    if(resVec.Length() > m_sWheelTurningParams.MaxSpeed) {
        resVec.Normalize();
        resVec *= m_sWheelTurningParams.MaxSpeed;
    }

    return resVec;
}

/****************************************/
/****************************************/

void CLeader::SetWheelSpeedsFromVector(const CVector2& c_heading) {
    /* Get the heading angle */
    CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
    /* Get the length of the heading vector */
    Real fHeadingLength = c_heading.Length();
    /* Clamp the speed so that it's not greater than MaxSpeed */
    Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);

    /* Wheel speeds based on current turning state */
    Real fSpeed1, fSpeed2;

    if(c_heading.GetX() > 0) {
        /* Go straight */
        fSpeed1 = fBaseAngularWheelSpeed;
        fSpeed2 = fBaseAngularWheelSpeed;
    } else if(c_heading.GetX() < 0) {
        /* Go back */
        fSpeed1 = -fBaseAngularWheelSpeed;
        fSpeed2 = -fBaseAngularWheelSpeed;
    } else {
        /* Rotate */
        fSpeed1 = -fBaseAngularWheelSpeed;
        fSpeed2 = fBaseAngularWheelSpeed;
    }

    /* Apply the calculated speeds to the appropriate wheels */
    Real fLeftWheelSpeed, fRightWheelSpeed;
    if(cHeadingAngle > CRadians::ZERO) {
        /* Turn Left */
        fLeftWheelSpeed  = fSpeed1;
        fRightWheelSpeed = fSpeed2;
    }
    else {
        /* Turn Right */
        fLeftWheelSpeed  = fSpeed2;
        fRightWheelSpeed = fSpeed1;
    }
    /* Finally, set the wheel speeds */
    m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

/****************************************/
/****************************************/

void CLeader::SetWheelSpeedsFromVectorEightDirections(const CVector2& c_heading) {
    /* Get the heading angle */
    CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
    /* Get the length of the heading vector */
    Real fHeadingLength = c_heading.Length();
    /* Clamp the speed so that it's not greater than MaxSpeed */
    Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);

    /* Wheel speeds based on current turning state */
    Real fLeftWheelSpeed = 0;
    Real fRightWheelSpeed = 0;

    if(c_heading.GetX() > 0) {
        if(c_heading.GetY() > 0) {
            fLeftWheelSpeed  = fBaseAngularWheelSpeed / 2;
            fRightWheelSpeed = fBaseAngularWheelSpeed;
        } else if(c_heading.GetY() < 0) {
            fLeftWheelSpeed  = fBaseAngularWheelSpeed;
            fRightWheelSpeed = fBaseAngularWheelSpeed / 2;
        } else {
            fLeftWheelSpeed  = fBaseAngularWheelSpeed;
            fRightWheelSpeed = fBaseAngularWheelSpeed;
        }
    } else if(c_heading.GetX() < 0) {
        if(c_heading.GetY() > 0) {
            fLeftWheelSpeed  = -fBaseAngularWheelSpeed / 2;
            fRightWheelSpeed = -fBaseAngularWheelSpeed;
        } else if(c_heading.GetY() < 0) {
            fLeftWheelSpeed  = -fBaseAngularWheelSpeed;
            fRightWheelSpeed = -fBaseAngularWheelSpeed / 2;
        } else {
            fLeftWheelSpeed  = -fBaseAngularWheelSpeed;
            fRightWheelSpeed = -fBaseAngularWheelSpeed;
        }
    } else if(c_heading.GetX() == 0) {
        if(c_heading.GetY() > 0) {
            fLeftWheelSpeed  = -fBaseAngularWheelSpeed;
            fRightWheelSpeed = fBaseAngularWheelSpeed;
        } else if(c_heading.GetY() < 0) {
            fLeftWheelSpeed  = fBaseAngularWheelSpeed;
            fRightWheelSpeed = -fBaseAngularWheelSpeed;
        }
    }

    /* Finally, set the wheel speeds */
    m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

/****************************************/
/****************************************/

void CLeader::SetWheelSpeedsFromVectorHoming(const CVector2& c_heading) {

    /* Get the heading angle */
    CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
    /* Get the length of the heading vector */
    Real fHeadingLength = c_heading.Length();
    
    /* Calculate the amount to adjust the wheel speeds */
    Real fSpeed = PIDHeading->calculate(0,cHeadingAngle.GetValue());
    //std::cout << fSpeed << std::endl;

    /* Apply the calculated speeds to the appropriate wheels */
    Real fLeftWheelSpeed, fRightWheelSpeed;
    fLeftWheelSpeed  = m_sWheelTurningParams.MaxSpeed+fSpeed;
    fRightWheelSpeed = m_sWheelTurningParams.MaxSpeed-fSpeed;

    /* Clamp the speed so that it's not greater than MaxSpeed */
    fLeftWheelSpeed = Min<Real>(fLeftWheelSpeed, m_sWheelTurningParams.MaxSpeed);
    fRightWheelSpeed = Min<Real>(fRightWheelSpeed, m_sWheelTurningParams.MaxSpeed);

    /* Finally, set the wheel speeds */
    m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

/****************************************/
/****************************************/

void CLeader::PrintName() {
    //RLOG << "";
}

/****************************************/
/****************************************/

/* Callback functions (Controllable events) */

void CLeader::Callback_Start(void* data) {
    lastControllableAction = "start";
    m_bSignal = true;
    inputStart = false;
}

void CLeader::Callback_Stop(void* data) {
    lastControllableAction = "stop";
    m_bSignal = false;
    inputStop = false;
}

void CLeader::Callback_Message(void* data) {
    lastControllableAction = "message";

    /* Send a heart-beat message to the other leader every 10 timesteps */
    RelayMsg beat;
    beat.type = 'H';
    beat.from = this->GetId();
    beat.time = initStepTimer;
    beat.follower_num = (UInt8)currentFollowerCount;
    beat.task_min_num = (UInt8)robotsNeeded;

    /* For every 10 timesteps, check if the demand is not decreasing to request robots from the other team */
    // if(exchangeUsed) {
    //     if(initStepTimer > 0 /* && initStepTimer % 10 == 0 */) {
    //         if(m_bSignal) { // Sending start signal to robots
    //             if(robotsNeeded - currentFollowerCount > 0) {
    //                 beat.type = 'R';
    //                 // beat.robot_num = 100; // TEMP: Large number to send all robots
    //                 beat.robot_num = robotsNeeded - currentFollowerCount + 1;
    //                 std::cout << this->GetId() << ": Sending request for " << beat.robot_num << " robots" << std::endl;
    //             }
    //         }
    //     }
    // }

    /* ############################################################ */
    // // DEBUG (Auto request) Ex0. Not for Ex1 or Ex2
    // if( !m_bSelected ) {
    //     /* Check if it is near the waypoint */
    //     if( !waypoints.empty() ) {
    //         CVector3 pos3d = m_pcPosSens->GetReading().Position;
    //         CVector2 pos2d = CVector2(pos3d.GetX(), pos3d.GetY());
    //         Real dist = (waypoints.front() - pos2d).Length();
    //         inTask = (dist <= m_sWaypointTrackingParams.thresRange);
    //     } else
    //         inTask = true;

    //     // Try requesting again if requested robots did not arrive
    //     if(initStepTimer > allRequestsSatisfiedTime + 700 && robotsNeeded - currentFollowerCount > 0) { // TEMP: hard-coded wait value
    //         allRequestsSatisfiedTime = initStepTimer;
    //         allRequestsSatisfied = false;
    //         requestSent = false;
    //         RLOG << "Still insufficient robots. Resetting request process" << std::endl;
    //     }

    //     if(inTask && robotsNeeded - currentFollowerCount > 0 && numRobotsRemainingToRequest == 0 && !allRequestsSatisfied) {
    //         numRobotsRemainingToRequest = robotsNeeded - currentFollowerCount;
    //     }

    //     // RLOG << "to request: " << numRobotsRemainingToRequest << ", " << "requestSent: " << requestSent << std::endl;

    //     if(numRobotsRemainingToRequest > 0 && !requestSent) {

    //         /* Request the number of robots needed to the leader with the smallest hop count to itself */
    //         std::vector<UInt8> closestTeams;
    //         UInt8 smallestHop = 255;
    //         for(const auto& [team, hop] : hopsDict) {
    //             if(teamsToRequest.count(team)) {
    //                 if(hop.count < smallestHop) {
    //                     closestTeams.clear();
    //                     closestTeams.push_back(team);
    //                     smallestHop = hop.count;
    //                 } else if(hop.count == smallestHop) {
    //                     closestTeams.push_back(team);
    //                 }
    //             }
    //         }

    //         std::sort(closestTeams.begin(), closestTeams.end());

    //         beat.type = 'R';
    //         beat.robot_num = numRobotsRemainingToRequest;
    //         beat.request_to_team = closestTeams[0];
    //         RLOG << "need: " << numRobotsRemainingToRequest << " currentTeam: " << currentFollowerCount << std::endl;
    //         std::cout << "{" << this->GetId() << "}[REQUEST] Requesting " << beat.robot_num << " robots to team " << beat.request_to_team << "..." << std::endl;
    //         requestSent = true;
    //         teamsToRequest.erase(beat.request_to_team);
    //     }
    // }
    /* ############################################################ */

    // DEBUG (Auto send/reject)
    if( !m_bSelected ) {
        // /* Check if it is near the waypoint */
        // if( !waypoints.empty() ) {
        //     CVector3 pos3d = m_pcPosSens->GetReading().Position;
        //     CVector2 pos2d = CVector2(pos3d.GetX(), pos3d.GetY());
        //     Real dist = (waypoints.front() - pos2d).Length();
        //     inTask = (dist <= m_sWaypointTrackingParams.thresRange);
        // } else
        //     inTask = true;

        // /* Get number of robots requested */
        // // TEMP: Assuming only one team ever requests (behaviour analysis)   
        // for(const auto& [team, num] : numRobotsRequested) {
        //     if(currentFollowerCount > 2 && !acknowledgeSent && inTask ) {
        //         if(currentFollowerCount > num + 2) {
        //             numRobotsToSend = num;
        //         } else {
        //             numRobotsToSend = currentFollowerCount - 2; // Keep 2 robots in team
        //         }
        //         // numRobotsToSend = currentFollowerCount - 1;
        //         numRobotsRemainingToSend = numRobotsToSend;
        //         teamToJoin = team;
        //         RLOG << "ACCEPT request from team " << team << ", current: " << currentFollowerCount << ", num: " << numRobotsToSend << std::endl;
        //     } else {
        //         beat.type = 'A';
        //         beat.robot_num = 0;
        //         beat.accept_to_team = teamToJoin;
        //         acknowledgeSent = true;
        //         RLOG << "REJECT request from team " << team << std::endl;
        //     }
        //     break;
        // }
    }
    numRobotsRequested.clear();

    /* User Signal */
    if(numRobotsToRequest > 0) {
        beat.type = 'R';
        beat.robot_num = numRobotsToRequest;
        std::cout << "{" << this->GetId() << "}[REQUEST] Requesting " << beat.robot_num << " robots..." << std::endl;
        // RLOG << "Requested for " << beat.robot_num << " robots" << std::endl;
        numRobotsToRequest = 0;
    }

    /* Acknowledge message */
    // std::cout << this->GetId() << " requested: " << numRobotsRequested << ", to send: " << numRobotsToSend << std::endl;
    // if(numRobotsRequested == numRobotsToSend && numRobotsToSend > 0) {
    if(!acknowledgeSent && numRobotsToSend > 0) {
        beat.type = 'A';
        beat.robot_num = numRobotsToSend;
        beat.accept_to_team = teamToJoin;
        std::cout << "{" << this->GetId() << "}[SEND] Sending " << numRobotsToSend << " robots to team " << beat.accept_to_team  << "!" << std::endl;
        acknowledgeSent = true;
    } else if(numRobotsRemainingToSend == 0) {
        numRobotsToSend = 0;
        acknowledgeSent = false;
    }
    
    // std::cout << this->GetId() << " remaining to send " << numRobotsRemainingToSend << std::endl;
    // RLOG << "type: " << beat.type << std::endl;

    rmsgToResend.push_back({sendDuration,beat});
    lastSent = initStepTimer;

    if(beat.type == 'R')
        lastControllableAction += "_" + std::to_string(beat.robot_num);

    beatSent++;
}

// void CLeader::Callback_Respond(void* data) {
//     lastControllableAction = "respond";

//     ConnectionMsg response;

//     if(teamID < acceptCmsg.otherTeam) { // Leader accepts its follower to become a connector if its teamID is smaller than the other team the follower is trying to join

//         /* Upon receiving a request message, send an accept message to the follower with the smallest ID */
//         response.type   = 'A';
//         response.from   = this->GetId();
//         response.to     = acceptCmsg.from;
//         response.toTeam = teamID;
//         // response.otherTeam = key;
//         // RLOG << "Accept " << acceptIDs[key] << " for team " << key << std::endl;
//     } else {
        
//         response.type   = 'N';
//         response.from   = this->GetId();
//         response.to     = "F0"; // DUMMY ID
//         response.toTeam = teamID;
//         // response.otherTeam = key;

//     }

//     cmsgToResend.push_back({sendDuration,response});
    
//     RLOG << "ACTION: respond(accept), ";
//     for(const auto& [t,cmsg] : cmsgToResend) {
//         LOG << "(to: " << cmsg.to << ", in: " << cmsg.toTeam << ")";
//     }
//     LOG << std::endl;
// }

void CLeader::Callback_Exchange(void* data) {
    lastControllableAction = "exchange";

    if( !switchCandidate.empty() ) {
        
        // ########### CONDITION FOR ENERG-AWARE EXPERIMENT ###########
        // if(GetId() != "L1") {
        // ############################################################
            /* Signal a follower to switch to the other team */
            if(!decremented) {
                if(initStepTimer - robotLastSentTime > sendRobotDelay) {
                    robotToSwitch = switchCandidate;
                    numRobotsRemainingToSend--;
                    decremented = true;
                    robotLastSentTime = initStepTimer;
                    robotLastSent = robotToSwitch;
                    RLOG << "Send " << robotToSwitch << " to team " << teamToJoin << std::endl;
                    // reset
                    switchCandidate = "";
                    switchCandidateDist = 255;
                }
            }
        // } else {
        //     // ########### CONDITION FOR ENERG-AWARE EXPERIMENT ###########

        //     if(!decremented) {
        //         if(initStepTimer - robotLastSentTime > sendRobotDelay) {
        //             // robotToSwitch = switchCandidate;
        //             numRobotsRemainingToSend--;
        //             decremented = true;
        //             robotLastSentTime = initStepTimer;
        //             robotLastSent = robotToSwitch;
        //             RLOG << "Send " << robotToSwitch << " to team " << teamToJoin << std::endl;
        //             // reset
        //             switchCandidate = "";
        //             switchCandidateDist = 255;
        //         }
        //     }

        //     // ############################################################
        // }

    } else {
        RLOGERR << "switchCandidate is empty" << std::endl;
    }
}

/****************************************/
/****************************************/

/* Callback functions (Uncontrollable events) */

// unsigned char CLeader::Check__RequestL(void* data) {
//     // std::cout << "Event: " << receivedRequest << " - _requestL" << std::endl;
//     return receivedRequest;
// }

unsigned char CLeader::Check_PressStart(void* data) {
    // std::cout << "Event: " << pressStart << " - pressStart" << std::endl;
    return inputStart;
}

unsigned char CLeader::Check_PressStop(void* data) {
    // std::cout << "Event: " << pressStop << " - inputStop" << std::endl;
    return inputStop;
}

unsigned char CLeader::Check_InputMessage(void* data) {
    bool timeToSend = (initStepTimer > 0 && initStepTimer % 10 == 0);
    // std::cout << "Event: " << timeToSend << " - inputMessage" << std::endl;
    return timeToSend;
}

unsigned char CLeader::Check_InputExchange(void* data) {
    bool exchangeRobot = (numRobotsRemainingToSend > 0 && !switchCandidate.empty());

    if(exchangeRobot) {
        // RLOG << "Event: " << 1 << " - inputExchange" << std::endl;
        return 1;
    }
    // RLOG << "Event: " << 0 << " - inputExchange" << std::endl;
    return 0;
}

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CLeader, "leader_controller")
