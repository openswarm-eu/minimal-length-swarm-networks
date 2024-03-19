/*
* AUTHOR: Genki Miyauchi <g.miyauchi@sheffield.ac.uk>
* 
* Define the message structure used to communicate between the robots.
*/

#ifndef ROBOT_MESSAGE_H
#define ROBOT_MESSAGE_H

/*
 * Include some necessary headers.
 */

/* Definition of the CVector2 datatype */
#include <argos3/core/utility/math/vector2.h>

/* Definition of the range-and-bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range-and-bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

#include <map>
#include <unordered_map>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/* List of states */
enum class RobotState {
    LEADER = 0,
    FOLLOWER,
    CONNECTOR,
    TRAVELER
};

/* Structure to share the closest connector or distance to a team */
struct TeamSharedMsg {
    UInt8 dist; // Shortest distance to another team (when there is no connector to that team)
    std::string connectorIdUpstream;   // ID of the tail connector to join (send to leader)
    std::string connectorIdDownstream; // ID of the tail connector to join (received from leader)
};

/* Structure to store the connection to the team */
struct HopMsg {
    UInt8 count;
    std::string ID; // Robot with lower hop value
    UInt8 resendCount = 255; // Number of times it was sent by a follower (used by followers within their team)
};

/* Structure to store the network change message */
struct NetworkChangeMsg {
    UInt8 teamID;       // The team this connection is affecting
    std::string prevID; // Connection to previous robot
    std::string newID;  // Connection to new robot
};

/*
* Structure to store request/approval messages for extending the chain.
* 
*   - R (Request) : Follower sends to leader or connector 
*   - A (Accept)  : Leader or connector sends to follower
* 
*       Structure: Type [1], sender ID [2], recipient ID [2], recipient team ID [1] (for Connector -> Follower accepts) 
*/
struct ConnectionMsg {
    char type = 'N'; // R or A or N (none)
    std::string from;
    std::string to;
    UInt8 toTeam; // its current team
    UInt8 otherTeam; // the other team it intends to connect for
};

/* Message sent by a leader to other leaders */
struct RelayMsg {
    char type = 'H'; // H (heart-beat) or R (request-robot) or A (acknowledge)
    std::string from;
    UInt16 time;
    std::string firstFollower; // First follower that received this message from a non-team robot
    UInt8 firstFollowerDist = 0; // Distance of the first follower to from the non-team robot it received the message from
    UInt8 follower_num; // Used in a heart-beat message. The number of followers the leader has.
    UInt8 task_min_num = 0; // Used in a heart-beat message. The minimum number of robots required to perform the task that the leader is on.
    UInt8 robot_num = 0; // Used in request-robot and acknowledge message. The number of robots the leader is requesting or sending.
    UInt8 request_to_team = 255; // Used in request-robot message. The team to request the robots from.
    UInt8 accept_to_team = 255; // Used in acknowledge message. The team to send the robots to.
};

/*
* Communication buffer size
*
* for n = 10: // team
*
* size = fixed data + TeamSharedMsg + Network Hop Count + Network Change + ConnectionMsg + Teams Nearby + Relay Msg + Visible Robots + End;
*      = 9          + (1 + 6n)      + (1 + 5n)          + (1 + 5n)       + (1 + 7n)      + (1 + n)      + (1 + 12n) + 400*2          + 1
*      = 9          + 61            + 51                + 51             + 71            + 11           + 121       + 800            + 1
*      = 1176
*/
static const UInt8 MAX_TEAM = 10;
static const UInt32 MESSAGE_BYTE_SIZE = 
    /* fixed data */
    9 + 
    /* TeamSharedMsg */
    (1 + 6*MAX_TEAM) + 
    /* Network Hop Count */
    (1 + 5*MAX_TEAM) + 
    /* Network Change */
    (1 + 5*MAX_TEAM) + 
    /* ConnectionMsg */
    (1 + 7*MAX_TEAM) + 
    /* Teams Nearby */
    (1 + MAX_TEAM) + 
    /* Relay Msg */
    (1 + 12*MAX_TEAM) + 
    /* Visible Robots */
    400*2 + 
    /* End */
    1;


/* 
* Structure to store incoming data received from other robots 
* 
* - Leader Signal
*   - Leader    : task signal [1]
* 
* - Team Switch Signal
*   - Leader informs a follower to join another team
*       - robotID [2]
*       - teamID [1]
* 
* - Hop count
*   Prefix with number of messages (max 2) [1]
*   - HopMsg [4] (teamID [1], count [1], ID [2])
* 
* - Connection Message
*   Prefix with number of messages (max 2) [1]
*   - ConnectionMsg [6]
* 
*       - Exchanging ConnectionMsg within a team:
*           - If message destination is to leader, relay upstream
*           - If message sender is the leader, relay downstream
*
*       - Exchanging ConnectionMsg between follower and connector:
*           - Follower will send up to one request message (R)
*           - Connector will send up to two approval messages (A)
* 
* - Shared Message
* 
*       - Share information about the closest connector to the team
*           - shareToLeader: Upstream (Follower to Leader)
*           - shareToTeam  : Downstream (Leader to Follower)
*       - Share information about the shortest distance to the other team (only when no connector is detected)
*           - shareDist    : Upstream (Follower to Leader) 
* 
* - Teams Nearby
*   Prefix with number of teams nearby (max 2) [1]
*   - teamID [1]
* 
*       - Used by connectors to determine whether other connectors can switch to a follower
* 
* - Relay Message
*   Prefix with number of messages (max 2) [1]
*   - RelayMsg [10] (Type [1], Leader ID [2], time sent [2], first follower [2], follower_num [1], task_min_num [1]. robot_num [1])
* 
*       - Message sent by a leader to other leaders
* 
*/
class Message {

    public:

        /* Class constructor */
        Message();

        Message(CCI_RangeAndBearingSensor::SPacket packet);

        virtual ~Message();

        virtual CByteArray GetCByteArray();

        virtual bool Empty();

        virtual void Print() const;

    public:

        /* Core */
        CVector2 direction = CVector2();
        RobotState state;
        std::string ID; // Only store numberhere. Together with robot type, it must have the structure with a char, follower by a number (e.g. F1, L2)
        UInt8 teamID = 255;
        UInt8 prevTeamID = 255;

        /* Leader Signal */
        UInt8 leaderSignal;

        /* Team Switch */
        std::string robotToSwitch = "";
        UInt8 teamToJoin = 255; // new team to join

        /* Team Hop Count */
        UInt8 teamHopCount = 255;

        /* Team Shared Message */
        std::map<UInt8, TeamSharedMsg> tmsg;

        /* Network Hop Count */
        std::map<UInt8, HopMsg> hops; // Key is teamID

        /* Network Change Message */
        std::vector<NetworkChangeMsg> nmsg;

        /* Connection Message*/
        std::vector<ConnectionMsg> cmsg;

        /* Teams Nearby */
        std::map<UInt8, int> nearbyTeams;

        /* Relay Message */
        std::vector<RelayMsg> rmsg;

        /* Connections */
        std::vector<std::string> connections;

};

#endif