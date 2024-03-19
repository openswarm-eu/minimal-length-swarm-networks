/* Include the controller definition */
#include "worker.h"
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

void CWorker::SWheelTurningParams::Init(TConfigurationNode& t_node) {
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

void CWorker::SFlockingInteractionParams::Init(TConfigurationNode& t_node) {
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
Real CWorker::SFlockingInteractionParams::GeneralizedLennardJones(Real f_distance) {
   Real fNormDistExp = ::pow(TargetDistance / f_distance, Exponent);
   return -Gain / f_distance * (fNormDistExp * fNormDistExp - fNormDistExp);
}

/****************************************/
/****************************************/

/*
 * This function is a generalization of the Lennard-Jones potential for repulsion only 
 */
Real CWorker::SFlockingInteractionParams::GeneralizedLennardJonesRepulsion(Real f_distance) {
   Real fNormDistExp = ::pow(TargetDistance / f_distance, Exponent);
   return -Gain / f_distance * (fNormDistExp * fNormDistExp);
}

/****************************************/
/****************************************/

bool CWorker::RoleSwitchConditions::ConnectorConditions() const {
    return c1 && c2 && c3;
}

/****************************************/
/****************************************/

// bool CWorker::RoleSwitchConditions::FollowerConditions() {
//     return f1 && f2;
// }

/****************************************/
/****************************************/

CWorker::CWorker() :
    m_pcWheels(NULL),
    m_pcProximity(NULL),
    m_pcRABAct(NULL),
    m_pcRABSens(NULL),
    m_pcLEDs(NULL) {}

/****************************************/
/****************************************/

CWorker::~CWorker() {
    
}

/****************************************/
/****************************************/

void CWorker::Init(TConfigurationNode& t_node) {

    /* Get sensor/actuator handles */
    m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor  <CCI_ProximitySensor             >("proximity"            );
    m_pcRABAct    = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
    m_pcRABSens   = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );
    m_pcLEDs      = GetActuator<CCI_LEDsActuator                >("leds"                 );
    m_pcGround    = GetSensor  <CCI_GroundSensor                >("ground"               );

    /*
    * Parse the config file
    */
    try {
        /* Wheel turning */
        m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));

        /* Flocking-related */
        // m_sLeaderFlockingParams.Init(GetNode(t_node, "leader_flocking"));
        m_sTeamFlockingParams.Init(GetNode(t_node, "team_flocking"));

        /* Chain formation threshold */
        GetNodeAttribute(GetNode(t_node, "team_distance"), "separation_threshold", separationThres);
        GetNodeAttribute(GetNode(t_node, "team_distance"), "joining_threshold", joiningThres);

        /* Weights using for the motion in each role */
        GetNodeAttribute(GetNode(t_node, "follower"), "attract",  followerAttraction);
        GetNodeAttribute(GetNode(t_node, "follower"), "repulse",  followerRepulsion);
        GetNodeAttribute(GetNode(t_node, "follower"), "obstacle", followerObstacle);

        GetNodeAttribute(GetNode(t_node, "connector"), "attract_to_connector", connectorAttractionToConnector);
        GetNodeAttribute(GetNode(t_node, "connector"), "attract_to_team", connectorAttractionToTeam);
        GetNodeAttribute(GetNode(t_node, "connector"), "repulse",  connectorRepulsion);
        GetNodeAttribute(GetNode(t_node, "connector"), "obstacle", connectorObstacle);

        GetNodeAttribute(GetNode(t_node, "traveler"), "attract",  travelerAttraction);
        GetNodeAttribute(GetNode(t_node, "traveler"), "repulse",  travelerRepulsion);
        GetNodeAttribute(GetNode(t_node, "traveler"), "obstacle", travelerObstacle);

        /* Connector motion */
        GetNodeAttribute(GetNode(t_node, "connector"), "target_distance", m_unConnectorTargetDistance);

        /* Traveler */
        GetNodeAttribute(GetNode(t_node, "traveler"), "joining_threshold", m_unTravelerJoiningThres);

        /* Timeout duration */
        GetNodeAttribute(GetNode(t_node, "timeout"), "send_message", sendDuration);
        GetNodeAttribute(GetNode(t_node, "timeout"), "send_respond", sendRespondDuration);
        GetNodeAttribute(GetNode(t_node, "timeout"), "wait_request", waitRequestDuration);

        /* SCT Model */
        GetNodeAttribute(GetNode(t_node, "SCT"), "path", m_strSCTPath);

    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
    }
    // RLOG << m_strSCTPath << std::endl;

    /* Initialization */
    currentState = RobotState::FOLLOWER; // Set initial state to follower
    currentMoveType = MoveType::ADJUST; // Set initial move type of not moving
    requestTimer = 0;
    bPerformingTask = true;
    hopCountToLeader = 255; // Default (max) value as hop count is unknown
    teamToMove = 255; // Default for no team
    teamToJoin = 255; // Default for no team
    initStepTimer = 0;
    tailSwitchTimer = 0;
    connectorSwitchTimer = 0;
    networkChangeTimer = 0;

    notifySent = false; // flag to indicate whether notifyNM was triggered

    responseToProcess = false;
    waitingResponse = false;

    /*
    * Init SCT Controller
    */
    sct = std::make_unique<SCTPub>(m_strSCTPath);

    /* Register controllable events */
    sct->add_callback(this, std::string("EV_moveFlock"),  &CWorker::Callback_MoveFlock,  NULL, NULL);
    sct->add_callback(this, std::string("EV_moveToTeam"), &CWorker::Callback_MoveToTeam, NULL, NULL);
    sct->add_callback(this, std::string("EV_moveAdjust"), &CWorker::Callback_MoveAdjust, NULL, NULL);
    sct->add_callback(this, std::string("EV_switchF"),    &CWorker::Callback_SwitchF,    NULL, NULL);
    sct->add_callback(this, std::string("EV_switchC"),    &CWorker::Callback_SwitchC,    NULL, NULL);
    sct->add_callback(this, std::string("EV_switchT"),    &CWorker::Callback_SwitchT,    NULL, NULL);
    sct->add_callback(this, std::string("EV_requestC"),   &CWorker::Callback_RequestC,   NULL, NULL);
    sct->add_callback(this, std::string("EV_respond"),    &CWorker::Callback_Respond,    NULL, NULL);
    sct->add_callback(this, std::string("EV_relay"),      &CWorker::Callback_Relay,      NULL, NULL);
    sct->add_callback(this, std::string("EV_notifyNM"),   &CWorker::Callback_NotifyNM,   NULL, NULL);
    sct->add_callback(this, std::string("EV_applyNM"),    &CWorker::Callback_ApplyNM,    NULL, NULL);

    /* Register uncontrollable events */
    sct->add_callback(this, std::string("EV_condC"),        NULL, &CWorker::Check_CondC,        NULL);
    sct->add_callback(this, std::string("EV_notCondC"),     NULL, &CWorker::Check_NotCondC,     NULL);
    sct->add_callback(this, std::string("EV_condF"),        NULL, &CWorker::Check_CondF,        NULL);
    sct->add_callback(this, std::string("EV_notCondF"),     NULL, &CWorker::Check_NotCondF,     NULL);
    sct->add_callback(this, std::string("EV__respond"),     NULL, &CWorker::Check__Respond,     NULL);
    sct->add_callback(this, std::string("EV_accept"),       NULL, &CWorker::Check_Accept,       NULL);
    sct->add_callback(this, std::string("EV_reject"),       NULL, &CWorker::Check_Reject,       NULL);
    sct->add_callback(this, std::string("EV__requestC"),    NULL, &CWorker::Check__RequestC,    NULL);
    sct->add_callback(this, std::string("EV__message"),     NULL, &CWorker::Check__Message,     NULL);
    sct->add_callback(this, std::string("EV__relay"),       NULL, &CWorker::Check__Relay,       NULL);
    sct->add_callback(this, std::string("EV__exchange"),    NULL, &CWorker::Check__Exchange,    NULL);
    sct->add_callback(this, std::string("EV_chosen"),       NULL, &CWorker::Check_Chosen,       NULL);
    sct->add_callback(this, std::string("EV_notChosen"),    NULL, &CWorker::Check_NotChosen,    NULL);
    sct->add_callback(this, std::string("EV_nearToLF"),     NULL, &CWorker::Check_NearToLF,     NULL);
    sct->add_callback(this, std::string("EV_notNearToLF"),  NULL, &CWorker::Check_NotNearToLF,  NULL);
    sct->add_callback(this, std::string("EV_condNM"),       NULL, &CWorker::Check_CondNM,       NULL);
    sct->add_callback(this, std::string("EV_notCondNM"),    NULL, &CWorker::Check_NotCondNM,    NULL);
    sct->add_callback(this, std::string("EV__notifyNM"),    NULL, &CWorker::Check__NotifyNM,    NULL);
    sct->add_callback(this, std::string("EV_initC"),        NULL, &CWorker::Check_InitC,        NULL);

    Reset();
}

/****************************************/
/****************************************/

void CWorker::Reset() {

    /* Initialize the msg contents to 255 (Reserved for "no event has happened") */
    m_pcRABAct->ClearData();
    cbyte_msg = CByteArray(MESSAGE_BYTE_SIZE, 255);
    m_pcRABAct->SetData(cbyte_msg);
}

/****************************************/
/****************************************/

UInt8 CWorker::GetTeamID() const {
    return teamID;
}

/****************************************/
/****************************************/

void CWorker::SetTeamID(const UInt8 id) {
    teamID = id;
    prevTeamID = id;
    currentState = RobotState::FOLLOWER;
}

/****************************************/
/****************************************/

RobotState CWorker::GetRobotState() const {
    return currentState;
}

/****************************************/
/****************************************/

std::string CWorker::GetMoveType() const {
    switch(currentMoveType) {
        case MoveType::FLOCK:
            return "FLOCK";
        case MoveType::ADJUST:
            return "ADJUST";
        case MoveType::TRAVEL_TO_TEAM:
            return "TRAVEL_TO_TEAM";
    }
    return "INVALID";
}

/****************************************/
/****************************************/

const std::map<UInt8, HopMsg>& CWorker::GetHops() const {
    return hopsDict;
}

/****************************************/
/****************************************/

const UInt8 CWorker::GetTeamHopCount() const {
    return hopCountToLeader;
}

/****************************************/
/****************************************/

bool CWorker::IsWorking() {
    return bPerformingTask;
}

/****************************************/
/****************************************/

std::string CWorker::GetLastAction() const {
    return lastControllableAction;
}

/****************************************/
/****************************************/

const std::map<UInt8,HopMsg>& CWorker::GetPrevHops() const {
    return prevHops;
}

/****************************************/
/****************************************/

void CWorker::SetTravelToTeam(UInt8 team) {
    teamToMove = team;
    teamToJoin = team;
    robotToSwitch = this->GetId();
}

/****************************************/
/****************************************/

void CWorker::SetRABRange(Real fRABRange) {
    m_fRABRange = fRABRange;
    m_fRABFactor = m_fRABRange / fDefaultRABRange;
}

/****************************************/
/****************************************/

void CWorker::ControlStep() {

    std::string id = this->GetId();
    // LOG << "---------- " << id << " ----------" << std::endl;

    initStepTimer++;

    // TEMP: force movetype to default to Adjust
    if(initStepTimer == 1) {
        currentMoveType = MoveType::ADJUST;
    }

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
    // RLOG << "--- Supervisors ---" << std::endl;

    if(initStepTimer > 4)
        sct->run_step();    // Run the supervisor to get the next action

    // RLOG << "Action: " << lastControllableAction << std::endl;
    // RLOG << ", " << sct->get_current_state_string() << std::endl;

    /*-----------------------------*/
    /* Implement action to perform */
    /*-----------------------------*/

    Message msg = Message();

    msg.state = currentState;
    msg.ID = id;
    msg.teamID = teamID;
    msg.prevTeamID = prevTeamID;

    // Decide what to communicate depending on current state (switch between follower and connector)
    switch(currentState) {
        case RobotState::FOLLOWER: {
            //std::cout << "State: FOLLOWER" << std::endl;

            m_pcLEDs->SetAllColors(teamColor[teamID]);

            /* Team Hop Count */
            /* Set its hop count to the leader */
            msg.teamHopCount = hopCountToLeader;

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

            break;
        }
        case RobotState::CONNECTOR: {
            // RLOG << "State: CONNECTOR" << std::endl;

            m_pcLEDs->SetAllColors(CColor::CYAN);

            /* Hop count */
            for(const auto& it : hopsDict) {

                HopMsg hop = HopMsg();

                hop.count = it.second.count;

                if( !it.second.ID.empty() )
                    hop.ID = it.second.ID;

                msg.hops[it.first] = hop;
            }

            /* PrevHops */
            for(const auto& pair : cmsgToResend) {
                ConnectionMsg cmsg = pair.second;
                if(cmsg.type == 'A') {
                    HopMsg hop;
                    hop.ID = cmsg.to;
                    hop.count = 1;
                    prevHops[cmsg.toTeam] = hop;
                }
            }

            break;
        }
        case RobotState::TRAVELER: {
            // RLOG << "State: TRAVELER" << std::endl;
            // RLOG << "MoveState: " << (UInt8)currentMoveType << std::endl;

            m_pcLEDs->SetAllColors(CColor::MAGENTA);
            break;
        }
        case RobotState::LEADER: {
            RLOG << "State: LEADER for " << this->GetId() << ". Something went wrong." << std::endl;
            break;
        }
    }

    /* Movement */
    switch(currentMoveType) {
        case MoveType::FLOCK: {
            Flock();
            break;
        }
        case MoveType::ADJUST: {
            AdjustPosition();
            break;
        }
        case MoveType::TRAVEL_TO_TEAM: {
            Travel();
            break;
        }
    }

    /* Decrement timer for newConnections */
    for(auto it = newConnections.begin(); it != newConnections.end();) {
        // RLOG << "newConnections size: " << newConnections.size() << std::endl;
        // RLOG << "timer: " << it->first << std::endl;
        if(it->first > 0) {
            it->first--;
            ++it;
        } else {
            it = newConnections.erase(it);
        }
    }

    /* Network Change Message */
    /* Set NetworkChangeMsg to send during this timestep */
    // RLOG << "nmsgToResend size: " << nmsgToResend.size() << std::endl;
    for(auto it = nmsgToResend.begin(); it != nmsgToResend.end();) {
        if(it->first > 0) {
            nmsgToSend.push_back(it->second);
            it->first--; // Decrement timer
            ++it;
        } else {
            it = nmsgToResend.erase(it);
        }
    }

    for(const auto& changeMsg : nmsgToSend) {
        msg.nmsg.push_back(changeMsg);
    }

    /* Connection Message */
    /* Set ConnectionMsg to send during this timestep */
    for(auto it = cmsgToResend.begin(); it != cmsgToResend.end();) {
        if(it->first > 0) {
            if( !waitingResponse && (receivedAccept || receivedReject)) {
                it = cmsgToResend.erase(it); // Stop resending when a response is received
            } else {
                cmsgToSend.push_back(it->second);
                it->first--; // Decrement timer
                ++it;
            }
        } else {
            it = cmsgToResend.erase(it);
        }
    }

    /* Connection Message */
    for(const auto& conMsg : cmsgToSend) {
        msg.cmsg.push_back(conMsg);
    }

    /* Teams Nearby */
    for(const auto& [team, dist] : nearbyTeams) {
        msg.nearbyTeams[team] = dist;
        // RLOG << "TEAM: " << team << ", DIST: " << dist << std::endl;
    }

    /* Relay Message */
    /* Set RelayMsg to send during this timestep */
    // RLOG << "resend size: " << rmsgToResend.size() << std::endl;
    for(auto it = rmsgToResend.begin(); it != rmsgToResend.end();) {
        if(it->first > 0) {
            rmsgToSend.push_back(it->second);
            it->first--; // Decrement timer
            ++it;
        } else {
            it = rmsgToResend.erase(it);
            //std::cout << "STOP RESENDING, TIMEOUT HAS BEEN REACHED" << std::endl;
        }
    }

    /* Relay Message */
    for(auto& relayMsg : rmsgToSend) {
        msg.rmsg.push_back(relayMsg);
    }

    /* Set ID of all connections to msg (only those involved in modifying the network) */
    std::vector<Message> allMsgs(teamMsgs);
    allMsgs.insert(std::end(allMsgs), std::begin(connectorMsgs), std::end(connectorMsgs));
    allMsgs.insert(std::end(allMsgs), std::begin(otherLeaderMsgs), std::end(otherLeaderMsgs));
    allMsgs.insert(std::end(allMsgs), std::begin(otherTeamMsgs), std::end(otherTeamMsgs));

    if( !leaderMsg.Empty() ) {
        allMsgs.push_back(leaderMsg);
    }

    for(size_t i = 0; i < allMsgs.size(); i++) {
        msg.connections.push_back(allMsgs[i].ID);
    }

    cbyte_msg = msg.GetCByteArray();

    /*--------------*/
    /* Send message */
    /*--------------*/
    // if(GetId() == "F6")
    //     msg.Print();
    m_pcRABAct->SetData(cbyte_msg);

}

/****************************************/
/****************************************/

void CWorker::ResetVariables() {
    /* Clear messages received */
    leaderMsg = Message();
    teamMsgs.clear();
    connectorMsgs.clear();
    otherLeaderMsgs.clear();
    otherTeamMsgs.clear();
    travelerMsgs.clear();

    cmsgToSend.clear();
    rmsgToSend.clear();
    nmsgToSend.clear();

    prevHops.clear();

    teamSharedMsgDict.clear();
    connectionCandidates.clear();
    roleSwitchConditionsPerTeam.clear();

    hopCountToLeader = 255; // Default value for not known hop count to the leader

    /* Check the connector matches its original request and is directed to its current team */
    if(currentResponse.to != this->GetId()) {
        currentResponse = ConnectionMsg();
    }

    /* Reset sensor reading results */
    nearToLF = false;
    receivedRequest   = false;
    receivedAccept = false;
    receivedReject = false;
    receivedInwardRelayMsg = false;
    receivedOutwardRelayMsg = false;
    receivedInwardSendMsg = false;
    receivedOutwardSendMsg = false;
    for(auto& info : lastBeat)
        info.second.second = 'N'; // Reset received flag to N (None)

    robotsToAccept.clear();
    nearbyTeams.clear();

    if(!notifySent)
        networkChanges.clear();
    nmsgsReceived.clear();

    lastControllableAction = "";
}

/****************************************/
/****************************************/

void CWorker::GetMessages() {

    /* Get RAB messages from nearby e-pucks */
    const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();

    if( !tMsgs.empty() ) {
        for(int i = 0; i < tMsgs.size(); i++) {

            Message msg = Message(tMsgs[i]);

            /* Store message */
            if(msg.state == RobotState::LEADER) {
                msg.ID = 'L' + msg.ID;

                if(msg.teamID == teamID)
                    leaderMsg = msg;
                else
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

void CWorker::Update() {

    if(currentState == RobotState::FOLLOWER) {
        UpdateHopCountsFollower();

        GetLeaderInfo();
        SetConnectorToRelay();
        SetDistanceToRelay();

        connectionCandidates = GetClosestNonTeam();

        if( !connectionCandidates.empty() )
            SwitchToConnectorConditions(connectionCandidates);

        /* Check whether it has received an accept message */
        if(currentRequest.type == 'R') {
            CheckAccept();
            
            /* Decrement timer */
            requestTimer--;
            
            /* Check whether an Accept message was not received before the timeout */
            if(requestTimer <= 0)
                receivedReject = true;
        }

        SetCMsgsToRelay();
        SetLeaderMsgToRelay(currentState);

    } else if(currentState == RobotState::CONNECTOR) {

        if(tailSwitchTimer > 0)
            tailSwitchTimer--;
        if(connectorSwitchTimer > 0)
            connectorSwitchTimer--;
        if(networkChangeTimer > 0)
            networkChangeTimer--;

        CheckRequests();

        UpdateHopCountsConnector();

        StoreNetworkModification();
        CheckNetworkModification();

        SetLeaderMsgToRelay(currentState);

        /* Check if it is allowed to switch to a follower */
        SwitchToFollowerConditions();
            
    } else if(currentState == RobotState::TRAVELER) {

        // RLOG << "teamToMove = " << teamToMove << ", prevTeamID = " << prevTeamID << std::endl;

        std::vector<Message> combinedMsgs(otherTeamMsgs);
        combinedMsgs.insert(combinedMsgs.end(), otherLeaderMsgs.begin(), otherLeaderMsgs.end());

        /* Check whether it has reached the other team */
        for(const auto& msg : combinedMsgs) {
            // RLOG << "teamToJoin = " << teamToJoin << ", msg.teamID = " << msg.teamID << std::endl;
            // RLOG << "teamHopCount = " << msg.teamHopCount << std::endl;
            if(msg.teamID == teamToJoin && msg.teamID == teamToMove) {
                if(msg.direction.Length() < m_unTravelerJoiningThres && msg.teamHopCount <= 1) {
                    nearToLF = true;
                    // RLOG << "TEAM FOUND!" << std::endl;
                    break;
                }
            }
        }
    }
}

/****************************************/
/****************************************/

void CWorker::GetLeaderInfo() {

    /* Find the hop count to and signal from the leader */
    if( !leaderMsg.Empty() ) { // Leader is in range

        hopCountToLeader = 1;
        // leaderSignal = leaderMsg.leaderSignal;

        // Comment out below for Ex2 with 0 second delay
        if(robotToSwitch != this->GetId()) {
            robotToSwitch = leaderMsg.robotToSwitch;
            teamToMove = leaderMsg.teamToJoin;
            teamToJoin = leaderMsg.teamToJoin;
        }

    } else { // Leader is not in range. Relay leader signal

        UInt8 minCount = 255;

        /* Find the smallest hop count among team members */
        for(auto& msg : teamMsgs) {
            if(msg.teamHopCount < minCount)
                minCount = msg.teamHopCount;
        }

        /* Record its own hop count */
        if(minCount < 255)
            hopCountToLeader = minCount + 1; // Set its count to +1 the smallest value

        for(const auto& msg : teamMsgs) {
            if(msg.teamHopCount < minCount) { // Get info from team member with smaller hop count to leader
                // leaderSignal = msg.leaderSignal;
                robotToSwitch = msg.robotToSwitch;
                teamToJoin = msg.teamToJoin;
                teamToMove = msg.teamToJoin;
                break;
            }
        }
    }
}

/****************************************/
/****************************************/

std::map<UInt8, Message> CWorker::GetClosestNonTeam() {
    
    std::map<UInt8, Message> closestRobots;

    /* Return a connection candidate for each team */
    for(auto& tmsg : teamSharedMsgDict) {

        /* Check for the robot that this robot can connect */
        Real minDist = 100000; // TEMP very large value
        std::vector<Message> candidateMsgs;
        Message closestRobot;

        // /* Prioritize connectors over other team members */
        // if( !connectorMsgs.empty() ) {
        //     candidateMsgs = connectorMsgs;
        // } else {
        //     candidateMsgs = otherLeaderMsgs;
        //     candidateMsgs.insert(std::end(candidateMsgs), std::begin(otherTeamMsgs), std::end(otherTeamMsgs));
        // }
        candidateMsgs = connectorMsgs;

        /* Find the closest non team robot from the matching team */
        for(size_t i = 0; i < candidateMsgs.size(); i++) {
            Real dist = candidateMsgs[i].direction.Length();
            if(dist < minDist) {
                if((candidateMsgs[i].state == RobotState::FOLLOWER && candidateMsgs[i].teamID == tmsg.first) ||
                   (candidateMsgs[i].state == RobotState::CONNECTOR && candidateMsgs[i].hops[teamID].count == 1)) {

                    minDist = dist;
                    closestRobot = candidateMsgs[i];
                }
            }
        }

        if( !closestRobot.Empty() )
            closestRobots[tmsg.first] = closestRobot;
    }
    
    return closestRobots;
}

/****************************************/
/****************************************/

void CWorker::SwitchToConnectorConditions(const std::map<UInt8, Message>& msgs) {
    
    /* Find whether this robot is the closest to any of the connection candidates among its teammates */

    // For each team in map
        // find if true
        // return if true
        // else false

    for(const auto& msg : msgs) {

        bool isClosest = true;
        Real myDist = msg.second.direction.Length();

        /* condC1: shortest distance to the team */

        if(myDist >= separationThres) {
            roleSwitchConditionsPerTeam[msg.first].c1 = true;
        } else {
            roleSwitchConditionsPerTeam[msg.first].c1 = false;
        }

        /* condC2: whether it is the closest among teammates */

        /* If the team has identified the next connector to connect to, check if it is the same */
        if( !teamSharedMsgDict[msg.first].connectorIdDownstream.empty() ) {
            if(msg.second.ID != teamSharedMsgDict[msg.first].connectorIdDownstream) {
                roleSwitchConditionsPerTeam[msg.first].c2 = false;
                continue;
            }
        }

        /* Check whether it is the closest to the candidate among other followers in the team that sees it (condC2) */
        for(size_t i = 0; i < teamMsgs.size(); i++) {

            std::vector<std::string> connections = teamMsgs[i].connections;

            // //std::cout << teamMsgs[i].ID << ": ";
            // for(size_t j = 0; j < teamMsgs[i].connections.size(); j++)
            //     //std::cout << teamMsgs[i].connections[j] << ", ";
            // //std::cout << std::endl;

            // Check if the team robot has seen the non-team robot 
            if (std::find(connections.begin(), connections.end(), msg.second.ID) != connections.end()) {

                /* Check the distance between its candidate and nearby team robots */
                CVector2 diff = msg.second.direction - teamMsgs[i].direction;
                Real dist = diff.Length();

                if(dist + 2 < myDist) { // TEMP: Fixed extra buffer value
                    isClosest = false; // Not the closest to the candidate robot
                    roleSwitchConditionsPerTeam[msg.first].c2 = false;
                    break;
                }
            }
        }
        if(isClosest)
            roleSwitchConditionsPerTeam[msg.first].c2 = true; // It is the closest

        /* condC3: whether the distance between the leader and connection candidate exceeds the separation threshold */

        if(leaderMsg.Empty())
            roleSwitchConditionsPerTeam[msg.first].c3 = true; // true, if leader is not in range
        else {
            CVector2 diff = msg.second.direction - leaderMsg.direction;
            Real dist = diff.Length();

            if(dist > separationThres)
                roleSwitchConditionsPerTeam[msg.first].c3 = true;
            else
                roleSwitchConditionsPerTeam[msg.first].c3 = false; // If leader is close, no need to become a connector
        }
    }
}

/****************************************/
/****************************************/

void CWorker::SwitchToFollowerConditions() {

    /* Check if it is allowed to switch to a follower */

    /* Check if it has just became a connector, whether there are any connectors that are claiming to be the tail connector for the team it just left from */
    if(connectorSwitchTimer > 0) {
        for(const auto& msg : connectorMsgs) {
            for(const auto& hop : msg.hops) {
                if(hop.first == prevTeamID && hop.second.count == 1) {
                    RLOG << msg.ID << " is claiming tail for team " << hop.first << ", return to follower" << std::endl;
                    roleSwitchConditionsPerTeam[prevTeamID].f1 = true;
                    roleSwitchConditionsPerTeam[prevTeamID].f2 = true;
                    return;
                }
            }
        }
    }

    /* Extract adjacent connector IDs to check */
    std::set<std::string> robotIDs;
    for(const auto& [key,hop] : hopsDict) {
        if( !hop.ID.empty() )
            robotIDs.insert(hop.ID);
    }

    for(const auto& [key, val] : hopsDict) {
        // RLOG << "team: " << int(key) << ", val: " << val.ID << std::endl;
        /* condF1: Is it a tail connector for this team? */

        if(val.count == 1)
            roleSwitchConditionsPerTeam[key].f1 = true;

        /* condF2: Are the successors all closer to the team than itself? */

        if( !robotIDs.empty() ) {
            /* If it is connected to at least one connector */

            bool neededAsConnector = false;

            /* Find the other connector message */
            for(const auto& id : robotIDs) {
                for(const auto& msg : connectorMsgs) {
                    if(msg.ID == id) {
                        
                        // If it appears in adjacent connector's hopsDict, check if the team is also close.
                        // If adjacent connector is near team, it is not needed as connector for this team.

                        bool isNearTeam = false;

                        // Are the other hop counts smaller?
                        bool hasSmallerHopCounts = false;
                        std::map<UInt8, bool> isHopCountSmallerMap;

                        for(const auto& hop : msg.hops) {
                            if(hop.second.ID == this->GetId() && hop.first == key) {

                                for(const auto& [team, dist] : msg.nearbyTeams) {
                                    if(key == team && dist < joiningThres) {
                                        isNearTeam = true;
                                        break;
                                    }
                                }
                            }

                            /* Check if any other hop count is larger or not */
                            if(hop.first != key) {
                                // RLOG << "otherHop: " << hop.second.count << ", myHop: " << hopsDict[hop.first].count;

                                if(hop.second.count < hopsDict[hop.first].count) {
                                    isHopCountSmallerMap[hop.first] = true;
                                }
                                else {
                                    isHopCountSmallerMap[hop.first] = false;
                                }
                            }
                        }

                        /* Check if all hop counts are smaller */
                        bool largerHopExists = false;
                        for(const auto& [key,val] : isHopCountSmallerMap) {
                            if( !val )
                                largerHopExists = true;
                        }
                        if( !largerHopExists )
                            hasSmallerHopCounts = true;

                        // RLOG << "isNearTeam: " << isNearTeam << ", hasSmaller: " << hasSmallerHopCounts << std::endl;
                        if(!isNearTeam || !hasSmallerHopCounts) {
                            neededAsConnector = true;
                            // RLOG << "neededAsConnector = true, ID: " << id << std::endl;
                        }

                        roleSwitchConditionsPerTeam[key].f2 = !neededAsConnector; // If needed, set false

                        break;
                    }
                }
            }

        } else {
            // TODO: when connecting two teams.
        }   

        // DEBUG
        // RLOG << "team: " << key << " condF1: " << roleSwitchConditionsPerTeam[key].f1 << ", condF2: " << roleSwitchConditionsPerTeam[key].f2 << std::endl;
    }
}

/****************************************/
/****************************************/

void CWorker::CheckAccept() {

    // /* Request sent to leader */
    // if(currentRequest.to[0] == 'L') {

    //     std::vector<Message> combinedTeamMsgs(teamMsgs);
    //     if( !leaderMsg.Empty() )
    //         combinedTeamMsgs.push_back(leaderMsg);

    //     for(const auto& teamMsg : combinedTeamMsgs) {
    //         for(const auto& cmsg : teamMsg.cmsg) {
    //             if(cmsg.type == 'A'){

    //                 if(cmsg.to == this->GetId()) {    // Request approved for this follower
    //                     receivedAccept = true;
    //                     currentResponse = cmsg;
    //                 } else                            // Request approved for another follower
    //                     receivedReject = true;

    //             } else if(cmsg.type == 'N') {         // No request has been approved
    //                 receivedReject = true;
    //             }
    //         }
    //     }
    // }
    /* Request sent to connector */
    // else {
        for(const auto& msg : connectorMsgs) {
            for(const auto& cmsg : msg.cmsg) {
                if(cmsg.type == 'A') {  // TEMP: Connector always sends accept messagesGetHops
                    // std::cout << "scan accept, from: " << cmsg.from << ", to: " << cmsg.to << ", toTeam: " << cmsg.toTeam << std::endl;
                    // std::cout << "currentRequestTo: " << req.second.to << std::endl;
                    /* Check the connector matches its original request and is directed to its current team */
                    if(cmsg.from == currentRequest.to && cmsg.toTeam == teamID) {
                        currentResponse = cmsg;
                        if(cmsg.to == this->GetId()) {    // Request approved for this follower
                            receivedAccept = true;
                            hopsCopy = msg.hops;
                            // std::cout << "received accept from connector!" << std::endl;
                            // msg.Print();
                            // std::cout << "team: " << req.first << std::endl;
                            // for(const auto& [key, val] : hopsCopy[req.first]) {
                            //     std::cout << "key: " << key << ", val: " << val.count << ", " << val.ID << ", " << val.resendCount << std::endl;
                            // }
                            // std::cout << "currentResponses below" << std::endl; 
                            // for(const auto& [key, val] : currentResponses) {
                            //     std::cout << "key: " << key << ", val: from: " << val.from << ", to: " << val.to << std::endl;
                            // }
                        } else                            // Request approved for another follower
                            receivedReject = true;
                    }
                }
            }
        }
    // }

    // /* Request sent to leader */
    // if(currentRequest.to[0] == 'L') {

    //     std::vector<Message> combinedTeamMsgs(teamMsgs);
    //     if( !leaderMsg.Empty() )
    //         combinedTeamMsgs.push_back(leaderMsg);

    //     for(const auto& teamMsg : combinedTeamMsgs) {
    //         for(const auto& cmsg : teamMsg.cmsg) {
    //             if(cmsg.type == 'A'){

    //                 if(cmsg.to == this->GetId()) {    // Request approved for this follower
    //                     receivedAccept = true;
    //                     currentResponse = cmsg;
    //                 } else                            // Request approved for another follower
    //                     receivedReject = true;

    //             } else if(cmsg.type == 'N') {         // No request has been approved
    //                 receivedReject = true;
    //             }
    //         }
    //     }
    // } 
    // /* Request sent to connector */
    // else {
    //     for(const auto& msg : connectorMsgs) {
    //         for(const auto& cmsg : msg.cmsg) {
    //             if(cmsg.type == 'A') {  // TEMP: Connector always sends accept messages

    //                 /* Check the connector matches its original request and is directed to its current team */
    //                 if(cmsg.from == currentRequest.to && cmsg.toTeam == teamID) {

    //                     if(cmsg.to == this->GetId()) {    // Request approved for this follower
    //                         receivedAccept = true;
    //                         currentResponse = cmsg;
    //                         hopsCopy = msg.hops;
    //                     } else                            // Request approved for another follower
    //                         receivedReject = true;
    //                 }
    //             }
    //         }
    //     }
    // }
}

/****************************************/
/****************************************/

void CWorker::CheckRequests() {

    /* Check the shortest distance to each team */
    std::map<UInt8,Real> teamDistances;

    for(const auto& hop : hopsDict) {
        for(const auto& msg : otherTeamMsgs) {
            if(msg.teamID == hop.first && hop.second.ID.empty()) { // If hop count == 1 for a given team
                if(teamDistances.find(hop.first) == teamDistances.end() && msg.teamHopCount != 255) {
                    teamDistances[hop.first] = msg.direction.Length();
                } else {
                    if(msg.direction.Length() < teamDistances[hop.first] && msg.teamHopCount != 255) {
                        teamDistances[hop.first] = msg.direction.Length();
                    }
                }
            }
        }
    }

    // if(GetId() == "F12") {
    //     RLOG << "dist 4: " << teamDistances[4] << std::endl;
    // }

    /* Check all requests sent to itself and choose one to respond to each team */
    for(const auto& msg : otherTeamMsgs) {
        // if(GetId() == "F12")
        //     RLOG << "msg.ID: " << msg.ID << " cmsg.size " << msg.cmsg.size() << std::endl;

        for(const auto& cmsg : msg.cmsg) {
            if(cmsg.to == this->GetId() && cmsg.type == 'R') {

                receivedRequest = true;

                // if(GetId() == "F12")
                //     RLOG << "received request from " << cmsg.from << std::endl;

                /* Accept if it does not have a fixed connector (ID field is empty) */
                if(hopsDict[msg.teamID].ID.empty()) {

                    /* Accept if the distance to all robots from that team is far */
                    if(teamDistances[msg.teamID] > separationThres - 2) { // NOTE: Modified with small buffer value, more likely for connector to accept a message from a follower
                        // if(GetId() == "F12")
                        //     RLOG << "ready to accept " << std::endl;

                        /* Accept first request seen for a team */
                        if(robotsToAccept.find(msg.teamID) == robotsToAccept.end()) {
                            robotsToAccept[msg.teamID] = msg;
                            // if(GetId() == "F12")
                            //     RLOG << "added to accept " << std::endl;

                            continue;
                        }

                        Real currentDist = robotsToAccept[msg.teamID].direction.Length();
                        Real newDist = msg.direction.Length();

                        /* Send an accept message to the closest follower */
                        if(newDist < currentDist)
                            robotsToAccept[msg.teamID] = msg;
                    }
                }
            }
        }
    }
}

/****************************************/
/****************************************/

void CWorker::SetCMsgsToRelay() {

    /* Combine messages from the leader and other followers that belong in the same team */
    std::vector<Message> combinedTeamMsgs(teamMsgs);
    combinedTeamMsgs.push_back(leaderMsg);

    // Relay any number of request or accept messages

    // for every team msg
        // for every cmsg
            // if message type = R & hop count larger
                // relay
            // if message type = A & hop count smaller
                //relay

    for(const auto& msg : combinedTeamMsgs) {
        for(const auto& cmsg : msg.cmsg) {
            
            /* Relay Request messages received from robots with greater hop count */
            if(cmsg.type == 'R' && msg.teamHopCount > hopCountToLeader) {
                if(currentRequest.type != 'R') // Check if it is currently not requesting
                    cmsgToSend.push_back(cmsg);
            }
            /* Relay Accept messages received from robots with smaller hop count */
            if(cmsg.type == 'A' && msg.teamHopCount < hopCountToLeader)
                cmsgToSend.push_back(cmsg);
        }
    }

    // /* Booleans to only relay up to 1 request and accept messages each */
    // bool receivedRequestuest = false;
    // bool receivedAccept = false;

    // for(const auto& msg : combinedTeamMsgs) {
    //     for(const auto& cmsg : msg.cmsg) {
    //         auto teamHops = msg.hops;

    //         /* Relay Request message received from robots with greater hop count */
    //         if( !receivedRequestuest ) {
    //             if(cmsg.type == 'R' && teamHops[teamID].count > hopCountToLeader) {
    //                 if(currentRequest.type != 'R') { // Check if it is currently not requesting
    //                     cmsgToSend.push_back(cmsg);
    //                     receivedRequestuest = true;
    //                     //std::cout << "Relay Request, from: " << cmsg.from << " to: " << cmsg.to << std::endl;
    //                 }
    //             }
    //         }
            
    //         /* Relay Accept message received from robots with smaller hop count */
    //         if( !receivedAccept ) {
    //             if(cmsg.type == 'A' && teamHops[teamID].count < hopCountToLeader) {
    //                 cmsgToSend.push_back(cmsg);
    //                 receivedAccept = true;
    //                 //std::cout << "Relay Accept, from: " << cmsg.from << " to: " << cmsg.to << std::endl;
    //             }
    //         }
    //     }
    // }
}

/****************************************/
/****************************************/

void CWorker::SetLeaderMsgToRelay(const RobotState state) {
    if(state == RobotState::FOLLOWER) {

        std::vector<Message> inwardMsgs;
        std::vector<Message> outwardMsgs;
        
        // Add connectors and leaders/followers from other teams to check for inward message relay
        inwardMsgs.insert(std::end(inwardMsgs), std::begin(otherLeaderMsgs), std::end(otherLeaderMsgs));
        inwardMsgs.insert(std::end(inwardMsgs), std::begin(otherTeamMsgs), std::end(otherTeamMsgs));
        inwardMsgs.insert(std::end(inwardMsgs), std::begin(connectorMsgs), std::end(connectorMsgs));

        // Add the leader to check for outward message relay
        if( !leaderMsg.Empty() )
            outwardMsgs.push_back(leaderMsg);

        // Split messages from team followers into two groups.
        for(const auto& msg : teamMsgs) {
            auto teamHops = msg.hops;
            if( !msg.rmsg.empty() ) {
                if(teamHops[teamID].count > hopCountToLeader)
                    inwardMsgs.push_back(msg);
                if(teamHops[teamID].count < hopCountToLeader)
                    outwardMsgs.push_back(msg);
            }
        }

        // For inward message, find all that's not in resend
        for(auto& msg : inwardMsgs) {
            for(auto& relayMsg : msg.rmsg) {
                UInt8 receivedTeamID = stoi(relayMsg.from.substr(1));
                if(receivedTeamID != teamID) {
                    if(lastBeat.find(receivedTeamID) == lastBeat.end()) { // If its the first time receiving, add it to lastBeat received
                        if(msg.state == RobotState::LEADER) {
                            relayMsg.firstFollower = this->GetId();
                            relayMsg.firstFollowerDist = msg.direction.Length();
                            lastBeat[receivedTeamID] = {relayMsg,'L'};
                        } else if(msg.teamID != teamID) {
                            relayMsg.firstFollower = this->GetId();
                            relayMsg.firstFollowerDist = msg.direction.Length();
                            lastBeat[receivedTeamID] = {relayMsg,'F'};
                        } else
                            lastBeat[receivedTeamID] = {relayMsg,'F'};
                    } else {
                        if(relayMsg.time > lastBeat[receivedTeamID].first.time) { // Else update it only if the timestep is newer
                            if(msg.state == RobotState::LEADER) {
                                relayMsg.firstFollower = this->GetId();
                                relayMsg.firstFollowerDist = (UInt8)msg.direction.Length();
                                lastBeat[receivedTeamID] = {relayMsg,'L'};
                            } else if(msg.teamID != teamID) {
                                relayMsg.firstFollower = this->GetId();
                                relayMsg.firstFollowerDist = (UInt8)msg.direction.Length();
                                lastBeat[receivedTeamID] = {relayMsg,'F'};
                            } else
                                lastBeat[receivedTeamID] = {relayMsg,'F'};
                        }
                    }
                }
            }
        }

        // For outward message, only find one
        for(const auto& msg : outwardMsgs) {
            for(const auto& relayMsg : msg.rmsg) {
                if(stoi(relayMsg.from.substr(1)) == teamID) {
                    if(lastBeat.find(teamID) == lastBeat.end()) { // If its the first time receiving, add it to lastBeat received
                        if(msg.state == RobotState::LEADER)
                            lastBeat[teamID] = {relayMsg,'L'};
                        else
                            lastBeat[teamID] = {relayMsg,'F'};
                    } else {
                        if(relayMsg.time > lastBeat[teamID].first.time) { // Else update it only if the timestep is newer
                            if(msg.state == RobotState::LEADER)
                                lastBeat[teamID] = {relayMsg,'L'};
                            else
                                lastBeat[teamID] = {relayMsg,'F'};
                        }
                    }
                }
            }
        }

    } else if(state == RobotState::CONNECTOR) {

        /* Check whether new relayMsg is received */
        for(const auto& hop : hopsDict) {
            /* Check the team */
            if(hop.second.count == 1 || hop.second.count == 2) {

                std::vector<Message> combinedMsgs(otherLeaderMsgs);
                combinedMsgs.insert(std::end(combinedMsgs), std::begin(otherTeamMsgs), std::end(otherTeamMsgs));

                for(const auto& msg : combinedMsgs) {
                    if(msg.teamID == hop.first) {
                        for(const auto& relayMsg : msg.rmsg) {
                            if(stoi(relayMsg.from.substr(1)) == hop.first) {
                                if(lastBeat.find(hop.first) == lastBeat.end()) { // If its the first time receiving, add it to lastBeat received
                                    if(msg.state == RobotState::LEADER)
                                        lastBeat[hop.first] = {relayMsg,'L'};
                                    else
                                        lastBeat[hop.first] = {relayMsg,'F'};
                                } else {
                                    if(relayMsg.time > lastBeat[hop.first].first.time) { // Else update it only if the timestep is newer
                                        if(msg.state == RobotState::LEADER)
                                            lastBeat[hop.first] = {relayMsg,'L'};
                                        else
                                            lastBeat[hop.first] = {relayMsg,'F'};
                                    }
                                }
                            }
                        }
                    }
                }
            }

            /* Check the connectors */
            if(hop.second.count > 1) {
                for(const auto& msg : connectorMsgs) {
                    if(hop.second.ID == msg.ID) {
                        //std::cerr << "---------" << this->GetId() << "---------" << std::endl;
                        for(const auto& relayMsg : msg.rmsg) {
                            if(stoi(relayMsg.from.substr(1)) == hop.first) {
                                //std::cerr << msg.ID << "(" << hop.first << ") -> " << this->GetId() << std::endl;
                                if(lastBeat.find(hop.first) == lastBeat.end()) { // If its the first time receiving, add it to lastBeat received
                                    lastBeat[hop.first] = {relayMsg,'F'};
                                    //std::cerr << "Relaying this " << relayMsg.time << std::endl;
                                } else {
                                    if(relayMsg.time > lastBeat[hop.first].first.time) { // Else update it only if the timestep is newer
                                        lastBeat[hop.first] = {relayMsg,'F'};
                                        //std::cerr << " Relaying this " << relayMsg.time << std::endl;
                                    }
                                }
                            }
                        }
                    }
                }
            }       
        }
    }
}

/****************************************/
/****************************************/

void CWorker::SetConnectorToRelay() {

    /* Get all visible teams from connectors and followers */
    std::vector<Message> combinedMsgs(connectorMsgs);
    combinedMsgs.insert(std::end(combinedMsgs), std::begin(teamMsgs), std::end(teamMsgs));

    std::set<UInt8> teamIDs;
    for(const auto& msg : combinedMsgs) {
        for(const auto& hop : msg.hops) {
            if(hop.first != 0) { // TEMP solution from accidentally creating a new entry for team 0
                /* Connector */
                if(msg.state == RobotState::CONNECTOR)
                    teamIDs.insert(hop.first);

                /* Follower that is in the same team (and is further away from the leader?) */
                if(msg.state == RobotState::FOLLOWER)
                    teamIDs.insert(hop.first);
            }
        }
    }
    teamIDs.erase(teamID); // Delete its own teamID

    /* Add new entries to teamSharedMsgDict if it doesn't exist */
    for(const auto& otherTeamID : teamIDs) {
        if(teamSharedMsgDict.count(otherTeamID) == 0) {
            teamSharedMsgDict[otherTeamID] = TeamSharedMsg();
        }
    }

    /* Update connector to relay upstream to the leader */
    for(auto& teamMsg : teamSharedMsgDict) {

        UInt8 otherTeamID = teamMsg.first;

        /* Check if it detects the tail connector directly */
        bool connectorNearby = false;

        UInt8 minHopCount = 255;
        std::string minHopCountRobot = "";

        for(auto& msg : connectorMsgs) {
            /* Find connector with the smallest hop count to the team and record it */
            for (const auto& hop : msg.hops) {
                if(hop.first == otherTeamID && msg.hops[teamID].count == 1) {
                    if(minHopCountRobot.empty() || hop.second.count < minHopCount) {
                        minHopCount = hop.second.count;
                        minHopCountRobot = msg.ID;
                        connectorNearby = true;
                    }
                }
            }
        }

        if(connectorNearby) {
            teamMsg.second.connectorIdUpstream = minHopCountRobot; // Update ID to send upstream to team
        } else {

            if( !teamMsgs.empty() ) {
                bool previousSeen = false;
                bool newValue = false;

                for(const auto& msg : teamMsgs) {
                    if(msg.teamHopCount > hopCountToLeader) { // Only check if it is further away from the leader
                        bool newInfoFound = false;
                        for(const auto& followerTeamMsg : msg.tmsg) {
                            if(followerTeamMsg.first == otherTeamID) {

                                if(followerTeamMsg.second.connectorIdUpstream == teamSharedMsgDict[otherTeamID].connectorIdUpstream) {

                                    previousSeen = true; // Received the same connector as before. Confirms current data is correct

                                } else if( !followerTeamMsg.second.connectorIdUpstream.empty() ) {
                                    
                                    /* Update connector info */
                                    teamMsg.second.connectorIdUpstream = followerTeamMsg.second.connectorIdUpstream;
                                    newValue = true;
                                    newInfoFound = true;
                                    break;
                                }
                            }
                        }
                        if(newInfoFound)
                            break;
                    }
                    
                }

                if( !previousSeen && !newValue ) // If previous info not received and no new info, reset upstream
                    teamMsg.second.connectorIdUpstream = "";

            } else {
                /* Found and received no info about this team, so reset upstream */
                teamMsg.second.connectorIdUpstream = "";
            }

        }

        /* Update connector to relay downstream to the team */
        combinedMsgs = teamMsgs;
        if( !leaderMsg.Empty() )
            combinedMsgs.push_back(leaderMsg);

        if( !combinedMsgs.empty() ) {

            for(const auto& msg : combinedMsgs) {
                // // DEBUG
                // if(this->GetId() == "F33" || this->GetId() == "F34" || this->GetId() == "F35" || this->GetId() == "F32") {
                //     RLOG << msg.ID << ": otherHopLeader: " << msg.teamHopCount << ", hopToLeader: " << hopCountToLeader << std::endl;
                // }

                if(msg.teamHopCount < hopCountToLeader) {

                    bool newInfoFound = false;
                    for(const auto& followerTeamMsg : msg.tmsg) {
                        if(followerTeamMsg.first == otherTeamID) {
                            teamMsg.second.connectorIdDownstream = followerTeamMsg.second.connectorIdDownstream;
                            newInfoFound = true;
                            break;
                        }
                    }

                    if(newInfoFound)
                        break;
                }
            }
        } else {
            /* Received no info about this team, so reset downstream */
            teamMsg.second.connectorIdDownstream = "";
        }
    }
}

/****************************************/
/****************************************/

void CWorker::SetDistanceToRelay() {
    /* Calculate the shortest distance to share to the rest of the team */

    // For every team in shared msg dict
        // if connector is not set
            // Declare my min var (255)
            // For otherTeamMsg
                // if matches with team to check
                    // if smaller than current min
                        // Update my min
            // For teamMsgs
                // if in dict < my min
                    // update my min
            // if my min not default
                // set dist for that team (UInt8)

    for(auto& tmsg : teamSharedMsgDict) {
        if(tmsg.second.connectorIdUpstream.empty()) {
            Real minDist = 255; // TEMP for converting to UInt8

            /* Find the shortest distance to a team (if directly visible) */
            for(const auto& msg : otherTeamMsgs) {
                if(msg.teamID == tmsg.first) {
                    if(msg.direction.Length() < minDist)
                        minDist = msg.direction.Length();
                }
            }

            /* Find the shortest distance to a team (received from team) */
            for(auto& msg : teamMsgs) {
                if(msg.tmsg.count(tmsg.first)) { // If dict contains the team ID
                    if(msg.teamHopCount > hopCountToLeader) {
                        if(msg.tmsg[tmsg.first].dist < minDist)
                            minDist = msg.tmsg[tmsg.first].dist;
                    }
                }
            }
            tmsg.second.dist = (UInt8)minDist;
        }
    }
}

/****************************************/
/****************************************/

void CWorker::UpdateHopCountsConnector() {

    /*
     *  Init hopsDict
     */

    /* Add every other visible team to hop map */
    std::vector<Message> otherTeamsAndLeadersMsgs;
    otherTeamsAndLeadersMsgs.insert(std::end(otherTeamsAndLeadersMsgs), std::begin(otherTeamMsgs), std::end(otherTeamMsgs));
    otherTeamsAndLeadersMsgs.insert(std::end(otherTeamsAndLeadersMsgs), std::begin(otherLeaderMsgs), std::end(otherLeaderMsgs));
    for(const auto& msg : otherTeamsAndLeadersMsgs) {

        /* Add hop count entry if not yet registered */
        if(hopsDict.find(msg.teamID) == hopsDict.end()) {
            HopMsg hop;
            hop.count = 1;
            hopsDict[msg.teamID] = hop;
            // RLOGERR << "Adding " << msg.teamID << " to hopsDict with count " << hop.count << std::endl;
        }
    }

    /* Extract neighboring connector IDs to check */
    std::set<std::string> robotIDs;
    for(const auto& hop : hopsDict) {
        if( !hop.second.ID.empty() )
            robotIDs.insert(hop.second.ID);
    }

    /* Extract Messages from connectors that have the IDs found previously */
    std::map<std::string, Message> robotMessages;
    for(const auto& msg : connectorMsgs) {

        if(robotIDs.empty())
            break;

        /* Find the next connector */
        if(robotIDs.count(msg.ID)) { // Should always return 0 or 1 as it is a set
            robotMessages[msg.ID] = msg;
            robotIDs.erase(msg.ID);
        }
    }

    // if(GetId() == "F1") {
    //     for(const auto& [team, hop] : hopsDict) {
    //         RLOG << "team: " << team << ", id: " << hop.ID << ", count: " << hop.count << std::endl;
    //     }
    // }

    // if(GetId() == "F1") {
    //     for(const auto& [team, hop] : hopsDict) {
    //         RLOG << "team: " << team << ", id: " << hop.ID << ", count: " << hop.count << std::endl;
    //     }
    // }

    // if(GetId() == "F34") {
    //     for(const auto& id : robotIDs) {
    //         RLOG << "id " << id;
    //     }
    //     LOG << std::endl;
    // }

    /* If a connector was not found, update hop count if it has become a follower */
    if( !robotIDs.empty() ) {
        for(const auto& msg : otherTeamMsgs) {

            /* Robot is found to be a follower so delete entries from hopsDict with the robot's id */
            if(robotIDs.count(msg.ID)) { // Should always return 0 or 1 as it is a set
                // if(GetId() == "F34") {
                //     RLOG << "msg.id " << msg.ID << std::endl;
                // }
                /* Check if it is not a robot that it has just sent an accept message to */
                bool sentAccept = false;
                for(const auto& sendMsg : cmsgToResend) {
                    if(sendMsg.second.to == msg.ID) {
                        sentAccept = true;
                        robotIDs.erase(msg.ID);
                        break;
                    }
                }

                if( !sentAccept ) {
                    // if(GetId() == "F34") {
                    //     RLOG << "!sentAccept " << msg.ID << std::endl;
                    // }
                    /* Find all keys that this robot appears in */
                    std::vector<UInt8> teamKeys;
                    for(const auto& hop : hopsDict) {
                        if(hop.second.ID == msg.ID)
                            teamKeys.push_back(hop.first);
                    }

                    /* Delete the robot's ID and update hop count to 1 */
                    for(const auto& key : teamKeys) {
                        hopsDict[key].ID = "";
                        hopsDict[key].count = 1;
                    }

                    // tailSwitchTimer = 2; // TEMP hard-coded duration

                    robotIDs.erase(msg.ID);
                    // if(GetId() == "F34") {
                    //     for(const auto& hop : hopsDict) {
                    //         RLOG << hop.first << ", " << hop.second.count << std::endl;
                    //     }
                    // }
                }
            }
        }
    }

    if( !robotIDs.empty() ) {
        RLOGERR << "robotIDs not empty for robot: " << this->GetId() << ", ";
        for(const auto& id : robotIDs) {
            LOGERR << id << ", ";
        }
        LOGERR << std::endl;
    }

    /*
     * Update hop count
     */
    std::set<std::string> connectorsToUpdate;
    std::set<std::string> newConnectorsPointing;
    for(auto& hop : hopsDict) {
        std::string previousRobotID = hop.second.ID;

        if( !previousRobotID.empty() ) {

            if(robotIDs.count(previousRobotID)) {
                hop.second.count = 255; // Could not be found, so set it to max value
            } else {
                // If it has just sent an accept, don't update count
                UInt8 teamToCheck = hop.first;
                HopMsg previousHop = robotMessages[previousRobotID].hops[teamToCheck];

                bool sendingAccept = false;
                for(const auto& pair : cmsgToResend) {
                    if(pair.second.type == 'A' && pair.second.to == previousRobotID) {
                        sendingAccept = true;
                    }
                }

                if( !sendingAccept ) {
                    hop.second.count = previousHop.count + 1; // Increment by 1
                }
            }
        }
    }

    /* Extract updated connector IDs to check */
    robotIDs.clear();
    for(const auto& hop : hopsDict) {
        if( !hop.second.ID.empty() )
            robotIDs.insert(hop.second.ID);
    }

    /* Add hop information about teams that it hasn't seen before */

    /* Extract all teams it hasn't seen before i.e. not in hopsDict */
    std::set<UInt8> newTeamIDs;
    std::vector<Message> combinedAdjacentMsg;
    for(const auto& [key,val] : robotMessages)
        combinedAdjacentMsg.push_back(val);

    combinedAdjacentMsg.insert(std::end(combinedAdjacentMsg), std::begin(otherTeamMsgs), std::end(otherTeamMsgs));
    for(auto& msg : combinedAdjacentMsg) {
        for(auto& hop : msg.hops) {
            /* Check if teamID exists in its hopsDict */
            if( !hopsDict.count(hop.first) ) {
                /* Add teamID if it doesn't exist */
                newTeamIDs.insert(hop.first);
            }
        }
    }

    /*
     * Add hop count to unseen team via adjacent connection
     */

    /* For each new ID, find the adjacent connector with the smallest hop count and register it to hopsDict */
    for(auto& newTeamID : newTeamIDs) {
        UInt8 minHopCount = 255;
        std::string minHopCountRobot = "";
        for(auto& msg : combinedAdjacentMsg) {
            for(auto& hop : msg.hops) {

                if(hop.first == newTeamID) {

                    if(minHopCountRobot.empty() || hop.second.count < minHopCount) {

                        minHopCount = hop.second.count;
                        minHopCountRobot = msg.ID;
                        std::cerr << "[" << this->GetId() << "] adding new entry to hopsDict for team " << newTeamID << " with hop " << minHopCount << " by " << minHopCountRobot << std::endl;
                    }
                }
            }
        }

        /* Add newTeamID to hopsDict */
        HopMsg hop;
        hop.count = minHopCount + 1;
        hop.ID = minHopCountRobot;
        hopsDict[newTeamID] = hop;
    }
}

/****************************************/
/****************************************/

void CWorker::CheckNetworkModification() {

    /* Check whether its ID appears in any other connectors not currently in its hopsDict */
    /* Add it to hopsDict by overwriting the entries with the smallest hop counts */

    /* Find whether there is a potential connector that is closer to itself than the current neighbors */

    // Get connector messages that are not currently in hopsDict
    // for each connector message
        // for each entry in hopsDict
            // if dist connector < dist entry
                // for each hop count in hopsDict, check whether the two in question have counts that are larger AND smaller than its own
                    // if true
                        // cannot swap
                    // if false
                        // can swap, set it as its new connector by updating the hop count and id for the old connector in hopsDict
                        // Update any other entries in hopsDict if it provides the smallest hop count

    /* Extract neighboring connector IDs to check */
    std::set<std::string> robotIDs;
    for(const auto& hop : hopsDict) {
        if( !hop.second.ID.empty() )
            robotIDs.insert(hop.second.ID);
    }

    /* Extract Messages from connectors that have the IDs found previously */
    std::map<std::string, Message> robotMessages, neighborMessages, otherConnectorMessages;
    for(const auto& msg : connectorMsgs) {

        /* Find the next connector */
        if(robotIDs.count(msg.ID)) { // Should always return 0 or 1 as it is a set
            robotMessages[msg.ID] = msg;
            neighborMessages[msg.ID] = msg;
            robotIDs.erase(msg.ID);
        } else {
            otherConnectorMessages[msg.ID] = msg;
        }
    }

    // RLOG << "neighbor: ";
    // for(const auto& [key,val] : neighborMessages) {
    //     LOG << key << ",";
    // }
    // LOG << "\n";

    // RLOG << "other: ";
    // for(const auto& [key,val] : otherConnectorMessages) {
    //     LOG << key << ",";
    // }
    // LOG << "\n";

    /* Find the number of unique robots/teams this robot is connected with */
    std::set<std::string> neighborIDs;
    for(const auto& [id, msg] : neighborMessages) {
        neighborIDs.insert(id);
    }

    // RLOG << "unique neighbors: ";
    // for(const auto& id : neighborIDs) {
    //     LOG << id << ",";
    // }
    // LOG << "\n";

    /* Check if it should modify its connection to optimize the network */
    if(connectorSwitchTimer == 0 && networkChangeTimer == 0 && !notifySent) { // Only check if it does not have outstanding network modifications to apply

        bool changeConnection = false;

        /* Check neighbor teams */
        /* Find the teamIDs of followers that are within its safety range */
        std::vector<Message> nearbyMsgs(otherTeamMsgs);
        nearbyMsgs.insert(std::end(nearbyMsgs), std::begin(otherLeaderMsgs), std::end(otherLeaderMsgs));

        for(const auto& msg : nearbyMsgs) {
            if(msg.teamHopCount != 255) {
                Real dist = msg.direction.Length();
                if(nearbyTeams.count(msg.teamID)) {
                    if(dist < nearbyTeams[msg.teamID])
                        nearbyTeams[msg.teamID] = dist; // Update dist if a shorter dist is found
                } else {
                    nearbyTeams[msg.teamID] = dist; // Store team id and dist
                }
            }
        }

        // For every nearbyTeam
            // For every neighbor connector
                // if neighbor hop count to team = 1 && teamDist < neighborDist
                    // changeConnection = True
                    // networkChanges[team] = {old,new(team)}

        if(travelerMsgs.empty()) { // Don't switch tail connectors while there are travelers nearby
            for(const auto [team, dist] : nearbyTeams) { // Non-adjacent team
                for(auto& neighbor : neighborMessages) { // Adjacent connector

                    /* Count the number of connections the neighbor has */
                    std::set<std::string> uniqueConnections;
                    for(const auto& [otherTeam,otherHop] : neighbor.second.hops) {
                        if(otherHop.count == 1) {
                            uniqueConnections.insert(std::to_string(otherTeam));
                        } else if(uniqueConnections.count(otherHop.ID) == 0) {
                            uniqueConnections.insert(otherHop.ID);
                        }
                    }

                    if(uniqueConnections.size() > 2) { // condNO3
                        Real neighborDistToTeam = neighbor.second.nearbyTeams[team];

                        if(neighbor.second.hops[team].count == 1 && dist < neighborDistToTeam) { // condNO2 & condNO1
                            changeConnection = true;
                            Message teamPlaceHolder = Message();
                            std::ostringstream oss;
                            oss.str("");
                            oss << "T" << team;
                            teamPlaceHolder.ID = oss.str();
                            networkChanges[team] = {neighbor.second.ID, teamPlaceHolder};
                            break;
                        }
                    }
                }
                if(changeConnection)
                    break;
            }
        }

        /* Check neighbor connectors */
        if( !changeConnection ) {
            for(auto& connector : otherConnectorMessages) { // Non-adjacent connector
                for(auto& neighbor : neighborMessages) {    // Adjacent connector

                    Real connectorDist = connector.second.direction.Length();
                    Real neighborDist = neighbor.second.direction.Length();

                    if(connectorDist < neighborDist) { // condNO1
                        // Check whether the connector and neighbor are overlapping itself in the network
                        
                        bool changeConnector = false;

                        for(const auto& [team,hop] : hopsDict) {

                            bool connectorExists = false;
                            std::set<std::string> uniqueConnections;
                            for(const auto& [otherTeam,otherHop]: neighbor.second.hops) {
                                /* Count the number of connections the neighbor has */
                                if(otherHop.count == 1) {
                                    uniqueConnections.insert(std::to_string(otherTeam));
                                } else if(uniqueConnections.count(otherHop.ID) == 0) {
                                    uniqueConnections.insert(otherHop.ID);
                                }
                                
                                if(otherHop.ID == connector.second.ID) { // condNO2
                                    connectorExists = true;
                                    // break;
                                }
                            }
                            if(uniqueConnections.size() > 2 && connectorExists){ // condNO3
                                // RLOG << connector.second.ID << " for " << neighbor.second.ID << ", CAN SWAP " << std::endl;
                                changeConnector = true;
                            } else {
                                // RLOG << connector.second.ID << " for " << neighbor.second.ID << ", OVERLAPS" << std::endl;
                                changeConnector = false;
                                break;
                            }
                        }

                        if(changeConnector) {

                            /* Store potential network changes */
                            for(auto& [team,hop] : hopsDict) {
                                if(hop.ID == neighbor.second.ID) {
                                    /* To avoid connectors simultaneously changing the network, only allow connectors with larger IDs to make the change */
                                    if(stoi(this->GetId().substr(1)) > stoi(connector.second.ID.substr(1))) {
                                        networkChanges[team] = {hop.ID, connector.second};
                                    }
                                }
                            }                    
                        }
                    }
                }
            }
        }
    }
}

/****************************************/
/****************************************/

void CWorker::StoreNetworkModification() {

    /* Check for network change messages and update accordingly */
    for(auto& msg : connectorMsgs) {
        // RLOG << msg.ID << ": nmsg.size: " << msg.nmsg.size() << std::endl;

        /* Store messages with nmsg */
        if( !msg.nmsg.empty() ) {
            nmsgsReceived.push_back(msg);
        }
    }
}

/****************************************/
/****************************************/

void CWorker::UpdateHopCountsFollower() {

    // Extract connector messages that has connection to this team

    // Extract all other team IDs from leader, team and connector (has connection to its team, either tail or has team in at least one hop)

    // For each team ID
        // Loop connector (that has connection to this team)
            // If ID matches
                // If connector's hop message comes from elsewhere (not this team ID, F or none), add it or update hopsDict
                // break

        // Loop teammate and leader
            // Find matching team ID with the smallest resendCount
            // If ID matches
                // If resendCount < currentFound
                    // Add it or update HopsDict

    // Extract other team IDs that did not appear from followers in other teams
    // Add them to HopsDict

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
    combinedMsgs.push_back(leaderMsg);
    combinedMsgs.insert(std::end(combinedMsgs), std::begin(teamMsgs), std::end(teamMsgs));

    std::set<UInt8> teamIDs;
    for(const auto& msg : combinedMsgs) {
        for(const auto& hop : msg.hops) {
            teamIDs.insert(hop.first);
        }
    }
    teamIDs.erase(teamID); // Delete its own teamID

    // RLOG << "";
    // for(auto& id : teamIDs) {
    //     std::cout << id << " ";
    // }
    // std::cout << std::endl;

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
            /* Loop teammate and leader */
            std::vector<Message> combinedTeamMsgs(teamMsgs);
            combinedTeamMsgs.push_back(leaderMsg);

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

void CWorker::Flock() {
    
    std::vector<Message> repulseMsgs;

    /* Add robots to repel from */
    if( !leaderMsg.Empty() )
        repulseMsgs.push_back(leaderMsg);
    repulseMsgs.insert(std::end(repulseMsgs), std::begin(teamMsgs), std::end(teamMsgs));
    repulseMsgs.insert(std::end(repulseMsgs), std::begin(otherLeaderMsgs), std::end(otherLeaderMsgs));
    repulseMsgs.insert(std::end(repulseMsgs), std::begin(otherTeamMsgs), std::end(otherTeamMsgs));
    repulseMsgs.insert(std::end(repulseMsgs), std::begin(connectorMsgs), std::end(connectorMsgs));
    repulseMsgs.insert(std::end(repulseMsgs), std::begin(travelerMsgs), std::end(travelerMsgs));
    
    /* Calculate overall force applied to the robot */
    CVector2 teamForce     = GetTeamFlockingVector();
    CVector2 robotForce    = GetRobotRepulsionVector(repulseMsgs);
    CVector2 obstacleForce = GetObstacleRepulsionVector();
    CVector2 sumForce = followerAttraction*teamForce + followerRepulsion*robotForce + followerObstacle*obstacleForce;

    /* Set Wheel Speed */
    if(sumForce.Length() > 1.0f) {
        SetWheelSpeedsFromVector(sumForce);
    } else {
        m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
    }
}

/****************************************/
/****************************************/

CVector2 CWorker::GetTeamFlockingVector() {

    CVector2 resVec = CVector2();

    if( !leaderMsg.Empty() ) {

        resVec = leaderMsg.direction;

    } else {
        
        if(hopCountToLeader == 255)
            return CVector2();  // No attraction
        
        size_t numAttract = 0;

        /* Calculate attractive force towards team members with a smaller hop count */
        for(size_t i = 0; i < teamMsgs.size(); i++) {
            if(teamMsgs[i].hops[teamID].count < hopCountToLeader) {
                resVec += teamMsgs[i].direction;
                numAttract++;
            }
        }
        resVec /= numAttract;
    }

    /* Limit the length of the vector to the max speed */
    if(resVec.Length() > m_sWheelTurningParams.MaxSpeed) {
        resVec.Normalize();
        resVec *= m_sWheelTurningParams.MaxSpeed;
    }

    return resVec;
}

/****************************************/
/****************************************/

CVector2 CWorker::GetRobotRepulsionVector(std::vector<Message>& msgs) {
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
        resVec *= m_sWheelTurningParams.MaxSpeed;
    }

    return resVec;
}

/****************************************/
/****************************************/

CVector2 CWorker::GetObstacleRepulsionVector() {
    /* Get proximity sensor readings */
    std::vector<Real> fProxReads = m_pcProximity->GetReadings();

    CVector2 resVec = CVector2();

    for(size_t i = 0; i < fProxReads.size(); i++) {
        CVector2 vec = CVector2();
        if(fProxReads[i] > 0.0f) {
            Real distance = -( log(fProxReads[i]) / log(exp(1)) );
            Real length = (0.1 - distance) / 0.1 * m_sWheelTurningParams.MaxSpeed;
            vec = CVector2(length, PROX_ANGLE[i]);
            
            resVec -= vec; // Subtract because we want the vector to repulse from the obstacle
        }
    }

    resVec /= 8;

    /* Limit the length of the vector to the max speed */
    if(resVec.Length() > m_sWheelTurningParams.MaxSpeed) {
        resVec.Normalize();
        resVec *= m_sWheelTurningParams.MaxSpeed;
    }

    return resVec;
}

/****************************************/
/****************************************/

void CWorker::Travel() {

    /* Add robots to repel from */
    std::vector<Message> repulseMsgs;
    repulseMsgs.insert(std::end(repulseMsgs), std::begin(otherLeaderMsgs), std::end(otherLeaderMsgs));
    repulseMsgs.insert(std::end(repulseMsgs), std::begin(otherTeamMsgs), std::end(otherTeamMsgs));
    repulseMsgs.insert(std::end(repulseMsgs), std::begin(connectorMsgs), std::end(connectorMsgs));
    repulseMsgs.insert(std::end(repulseMsgs), std::begin(travelerMsgs), std::end(travelerMsgs));

    /* Calculate overall force applied to the robot */
    CVector2 travelForce   = GetChainTravelVector();
    CVector2 robotForce    = GetRobotRepulsionVector(repulseMsgs);
    CVector2 obstacleForce = GetObstacleRepulsionVector();

    CVector2 sumForce = travelerAttraction*travelForce + travelerRepulsion*robotForce + travelerObstacle*obstacleForce;

    // /* Set Wheel Speed */
    // if(sumForce.Length() > 0.5f) {
    //     SetWheelSpeedsFromVector(sumForce);
    //     bMoving = true;
    // } else {
    //     m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
    //     bMoving = false;
    // }

    /* Set Wheel Speed */
    if(sumForce.Length() > 0.5f) {
        SetWheelSpeedsFromVector(sumForce);
    } else {
        m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
    }

}

/****************************************/
/****************************************/

CVector2 CWorker::GetChainTravelVector() {

    CVector2 resVec = CVector2();

    /* If a follower in the target team is visible, move towards them directly */
    std::vector<Message> combinedMsgs(otherTeamMsgs);
    combinedMsgs.insert(combinedMsgs.end(), otherLeaderMsgs.begin(), otherLeaderMsgs.end());

    UInt8 numTeamMembers = 0;
    for(const auto& msg : combinedMsgs) {
        if(msg.teamID == teamToMove) {
            resVec += msg.direction;
            numTeamMembers++;
        }
    }

    if(numTeamMembers) {
        resVec /= numTeamMembers;
        /* Limit the length of the vector to the max speed */
        if(resVec.Length() > m_sWheelTurningParams.MaxSpeed) {
            resVec.Normalize();
            resVec *= m_sWheelTurningParams.MaxSpeed;
        }
        return resVec;
    }

    /* Sort connectors according to the hop count towards the target team (large -> small) */
    auto property = teamToMove;
    auto sortRuleLambda = [property] (Message& m1, Message& m2) -> bool
    {
        return m1.hops[property].count > m2.hops[property].count;
    };

    std::vector<Message> sortedConnectorMsgs(connectorMsgs);
    std::sort(sortedConnectorMsgs.begin(), sortedConnectorMsgs.end(), sortRuleLambda);

    if(sortedConnectorMsgs.empty()) {
        for(const auto& msg : otherLeaderMsgs) { // If no connector and leader is visible, move towards previous leader
            if(msg.teamID == prevTeamID)
                return msg.direction;
        }
    }

    // for(auto& msg : sortedConnectorMsgs) {
    //     RLOG << msg.ID << ", count: " << msg.hops[teamToMove].count << std::endl;
    // }

    /* Find the next connector to move towards */
    Message nextConnector;
    for(auto& msg : sortedConnectorMsgs) {
        if(nextConnector.Empty())
            nextConnector = msg;
        else {
            /* Calculate target vector */
            // CVector2 margin = msg.direction;
            // margin.Rotate(CRadians::PI_OVER_TWO);
            // margin.Normalize();
            // margin *= 20;
            // CVector2 target = msg.direction + margin; // Not needed up to here ?
            nextConnector = msg;
        }
    }

    // RLOG << "Next connector: " << nextConnector.ID << std::endl;

    /* Calculate the position of the left side of the connector */

    // std::cout << "direction: " << nextConnector.direction << std::endl;

    CVector2 margin = nextConnector.direction;
    margin.Rotate(CRadians::PI_OVER_TWO);

    // std::cout << "rotated: " << margin << std::endl;

    if(margin.Length() > 0) {
        margin.Normalize();
        margin *= 20; // TEMP hard-coded margin from connector
    }

    resVec = nextConnector.direction + margin;

    // RLOG << "margin.Length: " << margin.Length() << std::endl;
    // RLOG << "direction: " << nextConnector.direction << std::endl;
    // std::cout << "margin: " << margin << std::endl;
    // RLOG << "target: " << resVec << std::endl;

    /* Limit the length of the vector to the max speed */
    if(resVec.Length() > m_sWheelTurningParams.MaxSpeed) {
        resVec.Normalize();
        resVec *= m_sWheelTurningParams.MaxSpeed;
    }

    return resVec;
}

/****************************************/
/****************************************/

void CWorker::AdjustPosition() {

    if(currentState == RobotState::FOLLOWER) {
        m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
        return;
    }

    /* Get robot messages that this robot is directly connected with */
    std::map<UInt8, Message> neighborMsgs = GetNeighbors();

    /* Add robots to repel from */
    std::set<std::string> neighborIDs;
    for(const auto& [key, msg] : neighborMsgs)
        neighborIDs.insert(msg.ID);

    std::vector<Message> repulseMsgs;
    for(const auto& msg : connectorMsgs) {
        if(neighborIDs.count(msg.ID))
            repulseMsgs.push_back(msg);
    }
    // repulseMsgs.insert(std::end(repulseMsgs), std::begin(connectorMsgs), std::end(connectorMsgs));

    /* Calculate overall force applied to the robot */
    CVector2 attractNeighborForce = GetConnectorAttractNeighborVector(neighborMsgs);
    CVector2 attractTeamForce = GetConnectorAttractTeamVector(neighborMsgs);
    CVector2 robotForce    = GetRobotRepulsionVector(repulseMsgs);
    CVector2 obstacleForce = GetObstacleRepulsionVector();
    CVector2 sumForce      = connectorAttractionToConnector*attractNeighborForce + connectorAttractionToTeam*attractTeamForce + connectorRepulsion*robotForce + connectorObstacle*obstacleForce;

    // RLOG << "sumForce: " << sumForce << std::endl;

    /* If it is a tail connector AND it loses connection with the team, stop */
    bool teamLost = false;
    for(const auto& [key,val] : hopsDict) {
        if(val.count == 1) {
            bool teamFound = false;
            for(const auto& [team,msg] : neighborMsgs) {
                if(key == team) {
                    teamFound = true;
                    break;
                }
            }

            if( !teamFound ) {
                teamLost = true;
                break;
            }
        }
    }

    if(teamLost)
        sumForce = CVector2();

    /* Set Wheel Speed */
    if(sumForce.Length() > 1.5f) {
        SetWheelSpeedsFromVector(sumForce);
    } else {
        m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
    }

}

/****************************************/
/****************************************/

std::map<UInt8, Message> CWorker::GetNeighbors() {

    std::map<UInt8, Message> neighborMsgs;

    /* Get messages to check */
    std::vector<Message> otherMsgs = otherTeamMsgs;
    otherMsgs.insert(std::end(otherMsgs), std::begin(otherLeaderMsgs), std::end(otherLeaderMsgs));
    otherMsgs.insert(std::end(otherMsgs), std::begin(connectorMsgs), std::end(connectorMsgs));

    for(const auto& hop : hopsDict) {

        UInt8 teamToCheck = hop.first;
        UInt8 myHopCount = hop.second.count;
        std::string robotToCheck = hop.second.ID;

        std::vector<Message> neighborTeamMsgs;

        for(const auto& msg : otherMsgs) {

            if((msg.state == RobotState::LEADER || msg.state == RobotState::FOLLOWER) && myHopCount == 1) {

                /* For the team that it is a tail connector for */

                if(msg.teamID == teamToCheck) {

                    Real dist = msg.direction.Length();

                    /* Store the message with the shortest distance */
                    if(!neighborMsgs.count(teamToCheck) || dist < neighborMsgs[teamToCheck].direction.Length()) {
                        neighborMsgs[teamToCheck] = msg;
                    }

                    neighborTeamMsgs.push_back(msg);
                }
            } else if(msg.state == RobotState::CONNECTOR) {

                /* For the team that it is NOT a tail connector for */

                /* Is this connector my adjacent connector? If yes, record vector towards it */
                if(msg.ID == robotToCheck) {
                    neighborMsgs[teamToCheck] = msg;
                }
            }
        }

        if(myHopCount == 1) {
            /* If it is a tail connector, use the average position of the team members */
            if( !neighborTeamMsgs.empty() ) {
                CVector2 resVec;

                /* Get average position of the team */
                for(const auto& msg : neighborTeamMsgs) {
                    resVec += msg.direction;
                }
                resVec /= neighborTeamMsgs.size();

                /* Replace the vector; use the average position of the team */
                neighborMsgs[teamToCheck].direction = resVec;
            }
        }
    }

    // RLOG << "Neighbors: ";
    // for(const auto& [team,msg] : neighborMsgs) {
    //     LOG << msg.ID << ", ";
    // }
    // LOG << "\n";

    return neighborMsgs;
}

/****************************************/
/****************************************/

CVector2 CWorker::GetConnectorAttractNeighborVector(std::map<UInt8, Message>& neighbor_msgs) {
    CVector2 resVec;

    std::set<std::string> checkedNeighbors;

    /* Attraction to neighbors */
    for(const auto& [team, msg] : neighbor_msgs) {
        if(checkedNeighbors.count(msg.ID) == 0) {

            /* If the distance to neighbors is critical, reduce the attraction towards team */
            // TODO: change param definition in config file
            Real target = m_unConnectorTargetDistance;
            Real modifier = 1;

            if(msg.direction.Length() > target) {
                modifier = (target+20*m_fRABFactor - target) / (target+20*m_fRABFactor - msg.direction.Length());
            }

            /* Add extra vector if it is too far */
            resVec += CVector2( modifier * msg.direction.Length(),
                                msg.direction.Angle());

            // resVec += msg.direction;
            checkedNeighbors.insert(msg.ID);
        }
    }

    resVec /= neighbor_msgs.size();

    return resVec;
}

/****************************************/
/****************************************/

CVector2 CWorker::GetConnectorAttractTeamVector(std::map<UInt8, Message>& neighbor_msgs) {

    /* Attraction to team */
    CVector2 resVec;

    /* Calculate the furthest neighbor distance */
    Real furthestNeighborDist = 0;
    for(const auto& [team, msg] : neighbor_msgs) {
        if(msg.state == RobotState::CONNECTOR && msg.direction.Length() > furthestNeighborDist)
            furthestNeighborDist = msg.direction.Length();
    }

    /* If the distance to neighbors is critical, reduce the attraction towards team */
    // TODO: change param definition in config file
    Real target = m_unConnectorTargetDistance;
    Real modifier = 1;

    if(furthestNeighborDist > target) {
        modifier = (target+10*m_fRABFactor - furthestNeighborDist) / (target+10*m_fRABFactor - target);
    }

    /* If smaller than zero, set to zero */
    if(modifier < 0)
        modifier = 0;

    /* Check whether it is connected to two neighbors */
    std::set<std::string> neighborIDs;
    for(const auto& [team, hop] : hopsDict) {
        neighborIDs.insert(hop.ID);
    }

    if(neighborIDs.size() < 3) {
        UInt8 numTeamAttract = 0;

        /* Add attraction toward teams with hop count 1 */
        for(const auto& [team, hop] : hopsDict) {
            if(hop.count == 1) {
                // RLOG << "Attracted to team " << team << std::endl;
                /* Add the vector to the team */
                /* For larger hop counts, make the attraction smaller */
                resVec += CVector2( (1.0/hop.count) * modifier * neighbor_msgs[team].direction.Length(),
                                    neighbor_msgs[team].direction.Angle());
                numTeamAttract++;
            }
        }

        if(numTeamAttract)
            resVec /= numTeamAttract;
    }

    return resVec;
}

/****************************************/
/****************************************/

void CWorker::SetWheelSpeedsFromVector(const CVector2& c_heading) {
    /* Get the heading angle */
    CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
    /* Get the length of the heading vector */
    Real fHeadingLength = c_heading.Length();
    /* Clamp the speed so that it's not greater than MaxSpeed */
    Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);
    /* State transition logic */
    if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::HARD_TURN) {
        if(Abs(cHeadingAngle) <= m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
            m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
        }
    }
    if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::SOFT_TURN) {
        if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
            m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
        }
        else if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
            m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
        }
    }
    if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN) {
        if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
            m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
        }
        else if(Abs(cHeadingAngle) > m_sWheelTurningParams.NoTurnAngleThreshold) {
            m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
        }
    }
    /* Wheel speeds based on current turning state */
    Real fSpeed1, fSpeed2;
    switch(m_sWheelTurningParams.TurningMechanism) {
        case SWheelTurningParams::NO_TURN: {
            /* Just go straight */
            fSpeed1 = fBaseAngularWheelSpeed;
            fSpeed2 = fBaseAngularWheelSpeed;
            break;
        }
        case SWheelTurningParams::SOFT_TURN: {
            /* Both wheels go straight, but one is faster than the other */
            Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
            fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
            fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
            break;
        }
        case SWheelTurningParams::HARD_TURN: {
            /* Opposite wheel speeds */
            fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
            fSpeed2 =  m_sWheelTurningParams.MaxSpeed;
            break;
        }
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

void CWorker::PrintName() {
    //RLOG << "";
}

/****************************************/
/****************************************/

/* Callback functions (Controllable events) */

void CWorker::Callback_MoveFlock(void* data) {
    lastControllableAction = "moveFlock";
    currentMoveType = MoveType::FLOCK;
    RLOG << "ACTION: MoveFlock" << std::endl;
}

void CWorker::Callback_MoveToTeam(void* data) {
    lastControllableAction = "moveToTeam";
    currentMoveType = MoveType::TRAVEL_TO_TEAM;
    teamToMove = teamToJoin;
    RLOG << "ACTION: MoveToTeam " << ", teamToMove = " << teamToMove << std::endl;
}

void CWorker::Callback_MoveAdjust(void* data) {
    lastControllableAction = "moveAdjust";
    currentMoveType = MoveType::ADJUST;
    RLOG << "ACTION: MoveAdjust" << std::endl;
}

void CWorker::Callback_SwitchF(void* data) {
    lastControllableAction = "switchF";

    /* Set new teamID */
    if(currentState == RobotState::CONNECTOR) {
        for(const auto& hop : hopsDict) {
            if(hop.second.count == 1) {
                teamID = hop.first;
                break;
            }
        }
        hopsDict.clear();

    } else if(currentState == RobotState::TRAVELER) {
        teamID = teamToJoin; // Update its own team ID
        hopsDict.erase(teamID); // Delete entry of new team from hopsDict
    }

    robotToSwitch = "";

    currentState = RobotState::FOLLOWER;

    connectorSwitchTimer = 0;
    networkChangeTimer = 0;

    RLOG << "ACTION: switchF, joining team " << int(teamID) << std::endl;
}

void CWorker::Callback_SwitchC(void* data) {
    lastControllableAction = "switchC";

    // TODO: Decide how it should respond when it receives multiple accepts (e.g. from leader & connector)

    // // Loop accept messages to see if it has received a message from a leader
    // bool acceptFromLeader = false;
    // for(const auto& accept : currentResponses) {
    //     if(accept.second.from[0] == 'L') {
    //         acceptFromLeader = true;
    //         break;
    //     }
    // }

    // if(currentResponse.from[0] == 'L') {  // Accept received from the leader

    //     /* Add every other visible team to hop map */
    //     for(const auto& msg : otherTeamMsgs) {

    //         /* Add hop count entry if not yet registered */
    //         if(hopsDict.find(msg.teamID) == hopsDict.end()) {
    //             HopMsg hop;
    //             hop.count = 1;
    //             hopsDict[msg.teamID] = hop;
    //         }
    //     }
    // } else {    
        
        // Accept received from a connector
        
        // // TODO: Currently copies hopsDict of first received accept from a connector
        // //       but this should be combined from multiple connectors who have accepted.
        // //       e.g. Compare and take the smaller hop count towards each team
        // UInt8 team;
        // for(const auto& [key,val] : currentResponses) {
        //     team = key;
        //     break;
        // }
        // std::map<UInt8, HopMsg> newHopsDict = hopsCopy[team];

        // std::cout << "team: " << team << std::endl;
        // for(const auto& [key, val] : hopsCopy[team]) {
        //     std::cout << "key: " << key << ", val: " << val.count << ", " << val.ID << ", " << val.resendCount << std::endl;
        // }

        /* Use the connector to generate its hop count to other teams */
        hopsCopy.erase(teamID);            // Delete entry of its own team

        for(auto& it : hopsCopy) {         // Loop to add hop count of 1 to each item
            it.second.count++;
            it.second.ID = currentResponse.from;
        }
        hopsDict = hopsCopy;                   // Set to its hops
    // }

    /* Set hop count to the team it is leaving to 1 */
    HopMsg hop;
    hop.count = 1;
    hopsDict[teamID] = hop;

    /* Reset variables */
    currentResponse = ConnectionMsg(); 
    hopsCopy.clear();
    for(auto& teamMsg : teamSharedMsgDict) {
        teamMsg.second = TeamSharedMsg();
    }

    currentState = RobotState::CONNECTOR;
    prevTeamID = teamID;
    teamID = 255;

    connectorSwitchTimer = 4; // TODO: Hard-coded timesteps to wait until it can trigger a networkchange
    networkChangeTimer = 0;

    RLOG << "ACTION: switchC, joining network" << std::endl;
}

void CWorker::Callback_SwitchT(void* data) {
    lastControllableAction = "switchT";

    /* Reset variables */
    for(auto& teamMsg : teamSharedMsgDict) {
        teamMsg.second = TeamSharedMsg();
    }

    currentState = RobotState::TRAVELER;
    prevTeamID = teamID;
    teamID = 255;
    connectorSwitchTimer = 0;
    networkChangeTimer = 0;

    RLOG << "ACTION: switchT" << std::endl;
}

void CWorker::Callback_RequestC(void* data) {
    lastControllableAction = "requestC";

    /* Set request to send */
    for(auto& cond : roleSwitchConditionsPerTeam) {
        if(cond.second.ConnectorConditions()) {
            ConnectionMsg cmsg;
            cmsg.type = 'R';
            cmsg.from = this->GetId();
            cmsg.otherTeam = cond.first;
            cmsg.to = teamSharedMsgDict[cond.first].connectorIdDownstream;
            if(cmsg.to.empty()) {
                cmsg.to = "F1"; // If the team has not identified the adjacent connector, send the message to F1, which is the default connector
            }
            cmsg.toTeam = 255;
            cmsgToResend.push_back({sendDuration,cmsg}); // Transmit public event
            currentRequest = cmsg;
            // RLOG << "Request to: " << cmsg.to << " for: " << cmsg.otherTeam << " duration: " << sendDuration << std::endl;
            break; // request for the first satisying team
        }
    }    

    requestTimer = waitRequestDuration;
    waitingResponse = true;

    RLOG << "ACTION: requestC, (to: " << currentRequest.to << ")" << std::endl;
}

void CWorker::Callback_Respond(void* data) {
    lastControllableAction = "respond";

    for(const auto& it : robotsToAccept) {
        ConnectionMsg cmsg;
        cmsg.type   = 'A';
        cmsg.from   = this->GetId();
        cmsg.to     = it.second.ID;
        cmsg.toTeam = it.first;
        cmsgToResend.push_back({sendRespondDuration,cmsg}); // Transmit public event

        /* Update hop count to the team using the new connector */
        hopsDict[it.first].count++; // 1 -> 2
        hopsDict[it.first].ID = it.second.ID;

        RLOG << "Accept " << it.second.ID << std::endl;
    }

    RLOG << "ACTION: respond(accept), ";
    for(const auto& [t,cmsg] : cmsgToResend) {
        LOG << "(to: " << cmsg.to << ", in: " << cmsg.toTeam << ")";
    }
    LOG << std::endl;
}

void CWorker::Callback_Relay(void* data) {
    lastControllableAction = "relay";

    for(const auto& info : lastBeat) {
        if(info.second.second != 'N')
            rmsgToResend.push_back({sendDuration,info.second.first});
    }
}

void CWorker::Callback_NotifyNM(void* data) {
    RLOG << "ACTION: notifyNM" << std::endl;

    /* Set network change to broadcast */
    for(const auto& [team, pair] : networkChanges) {
        NetworkChangeMsg nmsg;
        nmsg.teamID = team;
        nmsg.prevID = pair.first;
        nmsg.newID = pair.second.ID;
        nmsgToResend.push_back({sendDuration+2,nmsg}); // TEMP: added extra timestep to send
        RLOG << "NOTIFY CHANGE FROM " << nmsg.prevID << " TO " << nmsg.newID << " FOR TEAM " << team << std::endl;
    }

    notifySent = true;
}

void CWorker::Callback_ApplyNM(void* data) {

    if( notifySent ) {

        RLOG << "ACTION: applyNM (sent)" << std::endl;

        /* Network change was found by this robot */

        for(auto& [team, hop] : hopsDict) {
            for (auto& [team_nmsg, pair] : networkChanges) {
                if(team == team_nmsg && hop.ID == pair.first) {

                    size_t ignoreDuration = 4; // TEMP: hard-coded duration

                    if(pair.second.ID[0] == 'T') {
                        /* Become a tail connector */
                        hop.ID = "";
                        hop.count = 1;
                        newConnections.push_back({ignoreDuration,hop.ID});
                        RLOG << "UPDATED CONNECTION(SENT) FOR TEAM " << team << " TO (tail)" << std::endl; 
                    } else {
                        /* Change connections between connectors */
                        hop.ID = pair.second.ID;
                        hop.count = pair.second.hops[team].count + 1;
                        RLOG << "UPDATED CONNECTION(SENT) FOR TEAM " << team << " TO " << hop.ID << std::endl; 
                    }
                }
            }
        }

        notifySent = false; // reset

    } else {

        RLOG << "ACTION: applyNM (received)" << std::endl;

        /* Network change was received from another robot */

        for(const auto& msg : nmsgsReceived) {
            /* Get set of teams the other robot is changing connections for */
            std::set<UInt8> teamIDs;
            for(const auto& nmsg : msg.nmsg) {
                teamIDs.insert(nmsg.teamID);
            }

            for(const auto& nmsg : msg.nmsg) {

                /* Received connection removal; connect to new target */
                if(nmsg.prevID == this->GetId()) {
                    if(nmsg.newID[0] == 'F') {
                        for(auto& [team,hop] : hopsDict) {
                            if(hop.ID == msg.ID) { 
                                hop.ID = nmsg.newID; // Change team source to the new robot
                                RLOG << "UPDATED CONNECTION(PREV) FOR TEAM " << team << " TO " << hop.ID << std::endl; 
                            }
                        }
                    } else {
                        UInt8 team = stoi(nmsg.newID.substr(1));
                        if(hopsDict[team].ID != msg.ID) {
                            hopsDict[team].ID = msg.ID;
                            RLOG << "UPDATED CONNECTION(PREV) FOR TEAM " << team << " TO " << msg.ID << std::endl; 
                        }
                    }
                }
                /* Received connection addition; connect to sender */
                else if(nmsg.newID == this->GetId()) {
                    for(auto& [team,hop] : hopsDict) {
                        if( !teamIDs.count(team) ) {
                            if(hop.ID != msg.ID) {
                                hop.ID = msg.ID; // Change team source to sender for connections not modified by sender
                                RLOG << "UPDATED CONNECTION(NEW) FOR TEAM " << team << " TO " << hop.ID << std::endl; 
                            }
                        }
                    }
                }
            }
        }

        nmsgsReceived.clear(); // empty the vector
    }

    networkChangeTimer = 4; // TODO: Hard-coded timesteps to wait until it can trigger a networkchange
}

/****************************************/
/****************************************/

/* Callback functions (Uncontrollable events) */

unsigned char CWorker::Check_CondC(void* data) {
    /* If it satisfies the connector condition for any team, return true */
    for(const auto& cond : roleSwitchConditionsPerTeam) {
        if(cond.second.ConnectorConditions()) {
            // std::cout << "Event: " << 1 << " - condC" << std::endl;
            return true;
        }
    }
    // std::cout << "Event: " << 0 << " - condC" << std::endl;
    return false;
}

unsigned char CWorker::Check_NotCondC(void* data) {
    /* TEMP: If it does not satisfy the connector condition for any team, return true */
    for(const auto& cond : roleSwitchConditionsPerTeam) {
        if(cond.second.ConnectorConditions()) {
            // std::cout << "Event: " << 0 << " - notCondC" << std::endl;
            return false;
        }
    }
    // std::cout << "Event: " << 1 << " - notCondC" << std::endl;
    return true;
}

// unsigned char CWorker::Check_NearC(void* data) {
//     // bool connectorSeen = !connectorMsgs.empty();
//     // if(connectorSeen) {
//         for(const auto& [team, tmsg] : teamSharedMsgDict) {
//             if( !tmsg.connectorIdDownstream.empty() ) {
//                 // RLOG << "Event: " << 1 << " - nearC" << std::endl;
//                 return true;
//             }
//         }
//     // }
//     // RLOG << "Event: " << 0 << " - nearC" << std::endl;
//     return false;
// }

// unsigned char CWorker::Check_NotNearC(void* data) {
//     // bool connectorSeen = !connectorMsgs.empty();
//     // if(connectorSeen) {
//         for(const auto& [team, tmsg] : teamSharedMsgDict) {
//             if( !tmsg.connectorIdDownstream.empty() ) {
//                 // RLOG << "Event: " << 0 << " - notNearC" << std::endl;
//                 return false;
//             }
//         }
//     // }
//     // RLOG << "Event: " << 1 << " - notNearC" << std::endl;
//     return true;
// }

unsigned char CWorker::Check_CondF(void* data) {
    // RLOG << "Event: " << condF << " - condF" << std::endl;

    /* Get all conditions per team where f1 is true */
    std::set<UInt8> tailTeams;
    for(const auto& [key,cond] : roleSwitchConditionsPerTeam) {
        if(cond.f1)
            tailTeams.insert(key);
    }

    if(!tailTeams.empty()) {
        for(const auto& key : tailTeams) {
            if(!roleSwitchConditionsPerTeam[key].f2) {
                /* If it is needed by at least one adjacent connector, it cannot become a follower */
                return false;
            }
        }
        /* It was not needed by any connector or team, so it can be a follower */
        return true; 
    }

    return false;
}

unsigned char CWorker::Check_NotCondF(void* data) {
    // RLOG << "Event: " << !condF << " - notCondF" << std::endl;

    /* Get all conditions per team where f1 is true */
    std::set<UInt8> tailTeams;
    for(const auto& [key,cond] : roleSwitchConditionsPerTeam) {
        if(cond.f1)
            tailTeams.insert(key);
    }

    if(!tailTeams.empty()) {
        for(const auto& key : tailTeams) {
            if(!roleSwitchConditionsPerTeam[key].f2) {
                /* If it is needed by at least one adjacent connector, it cannot become a follower */
                return true;
            }
        }
        /* It was not needed by any connector or team, so it can be a follower */
        return false; 
    }

    return true;
}

unsigned char CWorker::Check__RequestC(void* data) {
    // RLOG << "Event: " << receivedRequest << " - _requestC" << std::endl;
    return receivedRequest;
}

// unsigned char CWorker::Check__Respond(void* data) {
//     // RLOG << "Event: " << !currentResponse.from.empty() << " - _respond" << std::endl;
//     return !currentResponse.from.empty();
// }

// unsigned char CWorker::Check_Accept(void* data) {
//     // RLOG << "Event: " << receivedAccept << " - accept" << std::endl;
//     if( !currentResponse.from.empty() ) // Clear existing request
//         currentRequest = ConnectionMsg();

//     return receivedAccept;
// }

// unsigned char CWorker::Check_Reject(void* data) {
//     // RLOG << "Event: " << receivedReject << " - reject" << std::endl;
//     if( !currentResponse.from.empty() )
//         currentRequest = ConnectionMsg(); // Clear existing request

//     return receivedReject;
// }

unsigned char CWorker::Check__Respond(void* data) {
    // RLOG << "Event: " << !currentResponse.from.empty() << " - _respond" << std::endl;
    // return !currentResponse.from.empty();
    if ( !responseToProcess && (receivedAccept || receivedReject) ) {
        responseToProcess = true;
        return true;
    }
    return false;
}

unsigned char CWorker::Check_Accept(void* data) {
    // RLOG << "Event: " << receivedAccept << " - accept" << std::endl;
    // if( !currentResponse.from.empty() ) // Clear existing request
    //     currentRequest = ConnectionMsg();

    // return receivedAccept;
    if(responseToProcess && receivedAccept) {
        responseToProcess = false;
        waitingResponse = false;
        return true;
    }
    return false;
}

unsigned char CWorker::Check_Reject(void* data) {
    // RLOG << "Event: " << receivedReject << " - reject" << std::endl;
    // if( !currentResponse.from.empty() )
    //     currentRequest = ConnectionMsg(); // Clear existing request

    // return receivedReject;
    if(responseToProcess && receivedReject) {
        responseToProcess = false;
        waitingResponse = false;
        return true;
    }
    return false;
}

unsigned char CWorker::Check__Message(void* data) {
    for(const auto& info : lastBeat) {
        if(info.second.second == 'L') {
            // RLOG << "Event: " << 1 << " - _message" << std::endl;
            return 1;
        }
    }
    // RLOG << "Event: " << 0 << " - _message" << std::endl;
    return 0;
}

unsigned char CWorker::Check__Relay(void* data) {
    for(const auto& info : lastBeat) {
        if(info.second.second == 'F') {
            // RLOG << "Event: " << 1 << " - _relay" << std::endl;
            return 1;
        }
    }
    // RLOG << "Event: " << 0 << " - _relay" << std::endl;
    return 0;
}

unsigned char CWorker::Check__Exchange(void* data) {
    bool receivedExchange = !robotToSwitch.empty();
    // RLOG << "Event: " << receivedExchange << " - _exchange" << std::endl;
    return receivedExchange;
}

unsigned char CWorker::Check_Chosen(void* data) {
    bool chosen = robotToSwitch == this->GetId();
    // RLOG << "Event: " << chosen << " - chosen" << std::endl;
    return chosen;
}

unsigned char CWorker::Check_NotChosen(void* data) {
    bool chosen = robotToSwitch == this->GetId();
    // RLOG << "Event: " << !chosen << " - notChosen" << std::endl;
    return !chosen;
}

unsigned char CWorker::Check_NearToLF(void* data) {
    // RLOG << "Event: " << nearToLF << " - nearToLF" << std::endl;
    return nearToLF;
}

unsigned char CWorker::Check_NotNearToLF(void* data) {
    // RLOG << "Event: " << !nearToLF << " - notnearToLF" << std::endl;
    return !nearToLF;
}

unsigned char CWorker::Check_CondNM(void* data) {
    return !networkChanges.empty();
}

unsigned char CWorker::Check_NotCondNM(void* data) {
    return networkChanges.empty();
}

unsigned char CWorker::Check__NotifyNM(void* data) {
    // RLOG << "Event: " << !nmsgsReceived.empty() << " - notifyNM" << std::endl;

    // if(this->GetId() == "F34") {
    //     RLOG << "size: " << nmsgsReceived.size() << std::endl;
    //     for(const auto& msg : nmsgsReceived) {
    //         for(const auto& nmsg : msg.nmsg) {
    //             LOG << " " << nmsg.teamID << ", prev: " << nmsg.prevID << ", new: " << nmsg.newID << std::endl;
    //         }
    //         // LOG << "team: " << nmsg.nmsg << " pair: " << pair.first;
    //     }
    // }

    for(const auto& msg : nmsgsReceived) {

        /* Get set of teams the other robot is changing connections for */
        std::set<UInt8> teamIDs;
        for(const auto& nmsg : msg.nmsg) {
            teamIDs.insert(nmsg.teamID);
        }

        for(const auto& nmsg : msg.nmsg) {

            /* Received connection removal; connect to new target */
            if(nmsg.prevID == this->GetId()) {
                for(auto& [team,hop] : hopsDict) {
                    if(nmsg.newID[0] == 'F') {
                        if(hop.ID == msg.ID) { 
                            /* There is a NetworkChangeMsg that alters the current connection */
                            return true; 
                        }
                    } else {
                        // Assuming a message to switch the tail connector role
                        if(team == stoi(nmsg.newID.substr(1)) && hop.ID.empty()) {
                            /* There is a NetworkChangeMsg that alters the current connection */
                            return true;
                        }
                    }
                }
            }
            /* Received connection addition; connect to sender */
            else if(nmsg.newID == this->GetId()) {
                for(auto& [team,hop] : hopsDict) {
                    if( !teamIDs.count(team) ) {
                        if(hop.ID != msg.ID) {
                            /* There is a NetworkChangeMsg that alters the current connection */
                            return true; 
                        }
                    }
                }
            }
        }
    }

    return false;
}

unsigned char CWorker::Check_InitC(void* data) {
    return this->GetId() == "F1" && initStepTimer == 5;
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
REGISTER_CONTROLLER(CWorker, "worker_controller")
