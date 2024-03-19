#include "robot_message.h"

/****************************************/
/****************************************/

Message::Message() {

}

/****************************************/
/****************************************/

Message::Message(CCI_RangeAndBearingSensor::SPacket packet) {

    size_t index = 0;

    /* Core */
    direction = CVector2(packet.Range, packet.HorizontalBearing);
    state = static_cast<RobotState>(packet.Data[index++]);
    ID = std::to_string(packet.Data[index++]); // Only stores number part of the id here
    teamID = packet.Data[index++];
    prevTeamID = packet.Data[index++];

    /* Leader Signal */
    leaderSignal = packet.Data[index++]; // Task signal

    /* Leader Team Switch Signal */
    if(packet.Data[index] != 255) {
        std::string switchID;
        switchID += (char)packet.Data[index++];            // First char of ID
        switchID += std::to_string(packet.Data[index++]);  // ID number
        robotToSwitch = switchID;
        teamToJoin = packet.Data[index++];
    } else
        index += 3;

    /* Team Hop Count */
    teamHopCount = packet.Data[index++];

    /* Team Shared Message */
    size_t msg_num = packet.Data[index++];

    if(msg_num == 255) // Safety check value
        msg_num = 0;

    for(size_t j = 0; j < msg_num; j++) {

        TeamSharedMsg teamMsg;

        UInt8 tmpTeamID = packet.Data[index++];
        teamMsg.dist = packet.Data[index++];

        std::string robotID;
        if(packet.Data[index] != 255) {
            robotID += (char)packet.Data[index++];            // First char of ID
            robotID += std::to_string(packet.Data[index++]);  // ID number
            teamMsg.connectorIdUpstream = robotID;
        } else
            index += 2;

        if(packet.Data[index] != 255) {
            robotID = "";
            robotID += (char)packet.Data[index++];            // First char of ID
            robotID += std::to_string(packet.Data[index++]);  // ID number
            teamMsg.connectorIdDownstream = robotID;
        } else
            index += 2;

        tmsg[tmpTeamID] = teamMsg;
    }

    /* Network Hops */
    msg_num = packet.Data[index++];

    if(msg_num == 255) // Safety check value
        msg_num = 0;

    for(size_t j = 0; j < msg_num; j++) {

        HopMsg hop;

        UInt8 tmpTeamID = packet.Data[index++];
        hop.count = packet.Data[index++];

        if(hop.count > 1) {
            std::string robotID;
            robotID += (char)packet.Data[index++];            // First char of ID
            robotID += std::to_string(packet.Data[index++]);  // ID number
            hop.ID = robotID;
        } else
            index += 2;

        hop.resendCount = packet.Data[index++];
        
        hops[tmpTeamID] = hop;
    }

    /* Network Change Message */
    msg_num = packet.Data[index++];

    if(msg_num == 255)
        msg_num = 0;

    for(size_t j = 0; j < msg_num; j++) {

        NetworkChangeMsg changeMsg;
        
        UInt8 teamID = packet.Data[index++];
        changeMsg.teamID = teamID;

        std::string robotID;
        robotID = "";
        robotID += (char)packet.Data[index++];            // First char of ID
        robotID += std::to_string(packet.Data[index++]);  // ID number
        changeMsg.prevID = robotID;

        robotID = "";
        robotID += (char)packet.Data[index++];            // First char of ID
        robotID += std::to_string(packet.Data[index++]);  // ID number
        changeMsg.newID = robotID;

        nmsg.push_back(changeMsg);
    }

    /* Connection Message */
    msg_num = packet.Data[index++];

    if(msg_num == 255)
        msg_num = 0;

    for(size_t j = 0; j < msg_num; j++) {

        ConnectionMsg conMsg;

        conMsg.type = (char)packet.Data[index++];

        std::string robotID;
        robotID += (char)packet.Data[index++];            // First char of ID
        robotID += std::to_string(packet.Data[index++]);  // ID number
        conMsg.from = robotID;

        //std::cout << "FROM: " << conMsg.from << std::endl;

        robotID = "";
        robotID += (char)packet.Data[index++];            // First char of ID
        robotID += std::to_string(packet.Data[index++]);  // ID number
        conMsg.to = robotID;
        
        //std::cout << "TO: " << conMsg.to << std::endl;
        
        conMsg.toTeam = packet.Data[index++]; 

        conMsg.otherTeam = packet.Data[index++];

        cmsg.push_back(conMsg);
    }

    /* Nearby Teams */
    msg_num = packet.Data[index++];

    if(msg_num == 255)
        msg_num = 0;

    for(size_t j = 0; j < msg_num; j++) {
        UInt8 team = packet.Data[index++];
        int dist = packet.Data[index++];
        nearbyTeams[team] = dist;
    }

    /* Relay Message */
    msg_num = packet.Data[index++];

    if(msg_num == 255)
        msg_num = 0;

    for(size_t j = 0; j < msg_num; j++) {

        RelayMsg relayMsg;

        relayMsg.type = (char)packet.Data[index++];

        std::string robotID;
        robotID += (char)packet.Data[index++];            // First char of ID
        robotID += std::to_string(packet.Data[index++]);  // ID number
        relayMsg.from = robotID;

        // //std::cout << "FROM: " << relayMsg.from << std::endl;
        
        relayMsg.time = packet.Data[index++]*256 + packet.Data[index++]; 
        if(relayMsg.time > 65535) {
            std::cerr << "INVALID TIME RECEIVED from " << ID << std::endl;
            std::cerr << packet.Data << std::endl;
        }

        if(packet.Data[index] != 255) {
            robotID = "";
            robotID += (char)packet.Data[index++];            // First char of ID
            robotID += std::to_string(packet.Data[index++]);  // ID number
            relayMsg.firstFollower = robotID;
            relayMsg.firstFollowerDist = packet.Data[index++];
        } else
            index += 3;

        relayMsg.follower_num = packet.Data[index++];
        relayMsg.task_min_num = packet.Data[index++];
        relayMsg.robot_num = packet.Data[index++];
        relayMsg.request_to_team = packet.Data[index++];
        relayMsg.accept_to_team = packet.Data[index++];

        rmsg.push_back(relayMsg);
    }

    /* Connections */
    while(packet.Data[index] != 255) {    // Check if data exists
        std::string robotID;
        robotID += (char)packet.Data[index++];            // First char of ID
        robotID += std::to_string(packet.Data[index++]);  // ID number
        connections.push_back(robotID);
    }

    // this->Print();

}

/****************************************/
/****************************************/

Message::~Message() {

}

/****************************************/
/****************************************/

CByteArray Message::GetCByteArray() {
    // std::cout << "---start--- \t\t\tstate: " << int(state) << " ID: " << ID << std::endl;

    CByteArray arr = CByteArray(MESSAGE_BYTE_SIZE, 255);
    size_t index = 0;

    /* Sender State */
    arr[index++] = static_cast<UInt8>(state);

    /* Sender ID */
    arr[index++] = stoi(ID.substr(1));

    /* Sender TeamID */
    arr[index++] = teamID;
    arr[index++] = prevTeamID;

    /* Leader Signal */
    arr[index++] = leaderSignal;

    /* Team Switch */
    if( !robotToSwitch.empty() ) {
        arr[index++] = robotToSwitch[0];
        arr[index++] = stoi(robotToSwitch.substr(1));
        arr[index++] = teamToJoin;
    } else
        index += 3;

    /* Team Hop Count */
    arr[index++] = teamHopCount;

    /* Team Shared Message */
    arr[index++] = tmsg.size(); // Set the number of TeamSharedMsg
    for(const auto& teamMsg : tmsg) {
        arr[index++] = teamMsg.first;
        arr[index++] = teamMsg.second.dist;
        if( !teamMsg.second.connectorIdUpstream.empty() ) {
            arr[index++] = teamMsg.second.connectorIdUpstream[0];
            arr[index++] = stoi(teamMsg.second.connectorIdUpstream.substr(1));
        } else {
            index += 2;
        }

        if( !teamMsg.second.connectorIdDownstream.empty() ) {
            arr[index++] = teamMsg.second.connectorIdDownstream[0];
            arr[index++] = stoi(teamMsg.second.connectorIdDownstream.substr(1));
        } else {
            index += 2;
        }
    }

    /* Network Hop Count */
    arr[index++] = hops.size(); // Set the number of HopMsg
    for(const auto& it : hops) {

        arr[index++] = it.first;                     // Team ID
        arr[index++] = it.second.count;              // Count

        if( it.second.ID.empty() )
            index += 2; // Skip
        else {
            arr[index++] = it.second.ID[0];              // ID
            arr[index++] = stoi(it.second.ID.substr(1)); // ID
        }

        arr[index++] = it.second.resendCount;
    }

    /* Network Change  */
    arr[index++] = nmsg.size(); // Set the number of NetworkChangeMsg
    for(const auto& changeMsg : nmsg) {
        arr[index++] = changeMsg.teamID;
        arr[index++] = changeMsg.prevID[0];
        arr[index++] = stoi(changeMsg.prevID.substr(1));
        arr[index++] = changeMsg.newID[0];
        arr[index++] = stoi(changeMsg.newID.substr(1));
    }

    /* Connection Message */
    arr[index++] = cmsg.size(); // Set the number of ConnectionMsg
    for(const auto& conMsg : cmsg) {
        arr[index++] = (UInt8)conMsg.type;
        arr[index++] = conMsg.from[0];
        arr[index++] = stoi(conMsg.from.substr(1));
        arr[index++] = conMsg.to[0];
        arr[index++] = stoi(conMsg.to.substr(1));
        arr[index++] = conMsg.toTeam;
        arr[index++] = conMsg.otherTeam;
    }

    /* Teams Nearby */
    arr[index++] = nearbyTeams.size(); // Set the number of nearby teams
    for(const auto& [team, dist] : nearbyTeams) {
        arr[index++] = team;
        if(dist > 0 && dist < 255)
            arr[index++] = (UInt8)dist;
        else
            arr[index++] = 255;
    }

    /* Relay Message */
    arr[index++] = rmsg.size(); // Set the number of RelayMsg
    for(const auto& relayMsg : rmsg) {
        arr[index++] = (UInt8)relayMsg.type;
        arr[index++] = relayMsg.from[0];
        arr[index++] = stoi(relayMsg.from.substr(1));
        arr[index++] = (UInt8)(relayMsg.time / 256.0);
        arr[index++] = (UInt8)(relayMsg.time % 256);

        if( !relayMsg.firstFollower.empty() ) {
            arr[index++] = relayMsg.firstFollower[0];
            arr[index++] = stoi(relayMsg.firstFollower.substr(1));
            arr[index++] = relayMsg.firstFollowerDist;
        } else
            index += 3;

        arr[index++] = relayMsg.follower_num;
        arr[index++] = relayMsg.task_min_num;
        arr[index++] = relayMsg.robot_num;
        arr[index++] = relayMsg.request_to_team;
        arr[index++] = relayMsg.accept_to_team;
    }

    /* Connections */
    for(size_t i = 0; i < connections.size(); i++) {
    
        //std::cout << allMsgs[i].ID << ", ";

        arr[index++] = connections[i][0];    // First character of ID
        arr[index++] = stoi(connections[i].substr(1));    // ID number

        if(i >= 399){
            std::cerr << "[" << ID << "] max connections reached" << std::endl;
            break;
        }
    }
    // std::cout << "---end--- \t\t\tindex: " << index << std::endl;

    return arr;
}

/****************************************/
/****************************************/

/* 
* Checks whether the Message is empty or not by checking the direction it was received from
*/
bool Message::Empty() {
    return direction.Length() == 0.0f;
}

/****************************************/
/****************************************/

void Message::Print() const {

    std::cout << "\n##########" << std::endl;
    
    switch(state) {
        case RobotState::LEADER:
            std::cout << "state: LEADER" << std::endl;
            break;
        case RobotState::FOLLOWER:
            std::cout << "state: FOLLOWER" << std::endl;
            break;
        case RobotState::CONNECTOR:
            std::cout << "state: CONNECTOR" << std::endl;
            break;
        case RobotState::TRAVELER:
            std::cout << "state: TRAVELER" << std::endl;
            break;
        default:
            /* The message is not initialised */
            if(int(state) != 255) {
                std::cerr << "Unknown state " << int(state) << " in " << ID << std::endl;
            }
            break;
    }

    std::cout << "ID: " << ID << std::endl;

    std::cout << "teamID: " << teamID << std::endl;

    std::cout << "prevTeamID: " << prevTeamID << std::endl;

    std::cout << "leaderSignal: " << leaderSignal << std::endl;

    std::cout << "robotToSwitch: " << robotToSwitch << std::endl;

    std::cout << "teamToJoin: " << teamToJoin << std::endl;

    std::cout << "teamHopCount: " << teamHopCount << std::endl;

    std::cout << "tmsg:" << std::endl;
    for(const auto& teamMsg : tmsg) {
        std::cout << "--- team: " << teamMsg.first
                  << ", dist: " << teamMsg.second.dist
                  << ", up: " << teamMsg.second.connectorIdUpstream
                  << ", down: " << teamMsg.second.connectorIdDownstream
                  << std::endl;
    }

    std::cout << "hops:" << std::endl;
    for(const auto& hop : hops) {
        std::cout << "--- team: " << hop.first
                  << ", count: " << hop.second.count
                  << ", ID: " << hop.second.ID
                  << ", resendCount: " << hop.second.resendCount
                  << std::endl;
    }

    std::cout << "nmsg:" << std::endl;
    for(const auto& changeMsg : nmsg) {
        std::cout << "--- teamID: " << changeMsg.teamID
                  << ", prevID: " << changeMsg.prevID
                  << ", newID: " << changeMsg.newID
                  << std::endl;
    }

    std::cout << "cmsg:" << std::endl;
    for(const auto& conMsg : cmsg) {
        std::cout << "--- type: " << conMsg.type
                  << ", from: " << conMsg.from
                  << ", to: " << conMsg.to
                  << ", toTeam: " << conMsg.toTeam
                  << ", otherTeam: " << conMsg.otherTeam
                  << std::endl;
    }

    std::cout << "nearbyTeams:" << std::endl;
    for(const auto& [team, dist] : nearbyTeams) {
        std::cout << "--- team: " << team
                  << ", dist: " << dist << std::endl;
    }

    std::cout << "rmsg:" << std::endl;
    for(const auto& relayMsg : rmsg) {
        std::cout << "--- type: " << relayMsg.type
                  << ", from: " << relayMsg.from
                  << ", time: " << relayMsg.time
                  << ", firstFollower: " << relayMsg.firstFollower
                  << ", firstFollowerDist: " << relayMsg.firstFollowerDist
                  << ", follower_num: " << relayMsg.follower_num
                  << ", task_min_num: " << relayMsg.task_min_num
                  << ", robot_num: " << relayMsg.robot_num
                  << ", request_to_team: " << relayMsg.request_to_team
                  << ", accept_to_team: " << relayMsg.accept_to_team;
        std::cout << "]" << std::endl;
    }

    std::cout << "connections: ";
    for(const auto& connection : connections) {
        std::cout << connection << ",";
    }
    std::cout << std::endl;


}
