#include "experiment_loop_functions_nop.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/plugins/robots/e-puck_leader/simulator/epuckleader_entity.h>
#include <controllers/leader/leader.h>
#include <controllers/worker/worker.h>
#include <argos3/plugins/simulator/entities/circle_task_entity.h>
#include <argos3/plugins/simulator/entities/rectangle_task_entity.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_render.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_widget.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

#include <utility/robot_message.h>

#include <filesystem>
#include <fstream>
#include <google/protobuf/util/json_util.h>
#include <google/protobuf/util/delimited_message_util.h>
#include <protos/generated/time_step.pb.h>

namespace fs = std::filesystem;

/****************************************/
/****************************************/

static const Real        EP_RADIUS        = 0.035f;
static const Real        EP_AREA          = ARGOS_PI * Square(0.035f);
static Real        EP_RAB_RANGE     = 0.8f;
static const std::string LE_CONTROLLER    = "leader";
static const std::string WO_CONTROLLER    = "worker";
static const UInt32      MAX_PLACE_TRIALS = 1000;
static const UInt32      MAX_ROBOT_TRIALS = 20;

static const std::string BINARY_FILENAME   = "log_data.pb";
static const std::string SUMMARY_FILENAME  = "summary.csv";
static const std::string COMMAND_FILENAME  = "commands.csv";

/****************************************/
/****************************************/

CExperimentLoopFunctionsNop::CExperimentLoopFunctionsNop() :
    m_pcFloor(NULL),
    m_pcRNG(NULL),
    m_bTaskExists(false),
    m_bTaskComplete(true),
    finishDelay(0) {
}

/****************************************/
/****************************************/

void CExperimentLoopFunctionsNop::Init(TConfigurationNode& t_node) {
    
    LOG << "[LOG] Init experiment loop function" << std::endl;

    LOG << "[LOG] SEED: " << (int)CSimulator::GetInstance().GetRandomSeed() << std::endl;

    config = t_node;

    try {
        /*
        * Parse the configuration file
        */
        TConfigurationNode& tChainFormation = GetNode(config, "output");
        /* Get a pointer to the floor entity */
        m_pcFloor = &GetSpace().GetFloorEntity();
        /* Create a new RNG */
        m_pcRNG = CRandom::CreateRNG("argos");
        /* Get the output file name from XML */
        GetNodeAttributeOrDefault(tChainFormation, "logging", m_bLogging, false);
        GetNodeAttributeOrDefault(tChainFormation, "out_path", m_strOutput, std::string("results/default/"));
        GetNodeAttributeOrDefault(tChainFormation, "run_number", m_strRunNumber, std::string(""));
        /* Set the frame grabbing settings */
        GetNodeAttributeOrDefault(tChainFormation, "frame_grabbing", m_bFrameGrabbing, false);
        GetNodeAttributeOrDefault(tChainFormation, "camera_index", m_unCameraIndex, (UInt32)0);

        TConfigurationNode& tDraw = GetNode(config, "draw");
        GetNodeAttributeOrDefault(tDraw, "robot_label", m_bDrawRobotLabel, true);

        TConfigurationNode& tEpuck = GetNode(config, "epuck");
        GetNodeAttributeOrDefault(tEpuck, "rab_range", EP_RAB_RANGE, 0.8);

        /* ############################# */
        m_bNoDemandTasks = false;
        /* Network maintenance */
        InitRobots();
        InitTasksCircular();
        AssignTasks();
        CheckInitialConnectorInRange();
        if(m_bLogging) {
            InitLogging();
        }
        /* ############################# */

        // InitRobots();
        // m_bNoDemandTasks = false;
        // InitTasks();

        if(m_bLogging) {
            /* Log arena information */
            m_cOutput.open(m_strSummaryFilePath.c_str(), std::ios_base::app);
            m_cOutput << "ARENA_RADIUS," << m_fArenaRadius << "\n";
            m_cOutput << "DEPLOY_RADIUS," << m_fDeploymentRadius << "\n";
            m_cOutput.close();
        }
    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing loop functions!", ex);
    }
}

/****************************************/
/****************************************/

void CExperimentLoopFunctionsNop::Reset() {
    std::cout << "RESET called" << std::endl;
    
    /* Delete existing robot and task entities from the simulation */
    for(const auto& id : m_vecEntityID) {
        RemoveEntity(id);
    }

    m_vecEntityID.clear();

    InitRobots();
    InitTasksCircular();
    AssignTasks();
}

/****************************************/
/****************************************/

void CExperimentLoopFunctionsNop::Destroy() {
    int final_time = GetSpace().GetSimulationClock();
    LOG << "[LOG] Final Timestep: " << final_time << std::endl;
    
    // if(m_bLogging) {
    //     m_cOutput.open(m_strSummaryFilePath.c_str(), std::ios_base::app);
    //     m_cOutput << "\n";
    //     m_cOutput << "FINISH_TIME," << final_time << "\n";
    //     if(m_bTaskComplete) {
    //         m_cOutput << "TASK_STATUS,FINISHED" << "\n";
    //         std::cout << "[LOG] Task Status: FINISHED" << std::endl;
    //     } else {
    //         m_cOutput << "TASK_STATUS,UNFINISHED" << "\n";
    //         std::cout << "[LOG] Task Status: UNFINISHED" << std::endl;
    //     }
    //     m_cOutput.close();
    // }
    
    LOG << "[LOG] DESTROY called" << std::endl;
    CSimulator::GetInstance().Terminate();

}

/****************************************/
/****************************************/

CColor CExperimentLoopFunctionsNop::GetFloorColor(const CVector2& c_position_on_plane) {

    CSpace::TMapPerType* cCTasks;
    if(m_bNoDemandTasks) {
        cCTasks = &GetSpace().GetEntitiesByType("circle_task_no_demand");
    } else {
        cCTasks = &GetSpace().GetEntitiesByType("circle_task");
    }

    for(CSpace::TMapPerType::iterator it = cCTasks->begin();
       it != cCTasks->end();
       ++it) {

        if(m_bNoDemandTasks) {
            CCircleTaskNoDemandEntity& cCTask = *any_cast<CCircleTaskNoDemandEntity*>(it->second);
            if(cCTask.InArea(c_position_on_plane) and Distance(c_position_on_plane, cCTask.GetPosition()) > cCTask.GetRadius()*0.9) {
                return CColor(255,191,191);
            }
        } else {
            CCircleTaskEntity& cCTask = *any_cast<CCircleTaskEntity*>(it->second);
            if(cCTask.InArea(c_position_on_plane) and Distance(c_position_on_plane, cCTask.GetPosition()) > cCTask.GetRadius()*0.9) {
                if(cCTask.GetDemand() > 0) {
                    return CColor(255,191,191);
                } else {
                    return CColor(255,250,250);
                }
            }
        }
    }
    return CColor::WHITE;
}

/****************************************/
/****************************************/

void CExperimentLoopFunctionsNop::PreStep() {

    LOG << "TIME: " << GetSpace().GetSimulationClock() << std::endl;

    std::unordered_map<UInt8, UInt32> unFollowers;
    UInt32 unConnectors = 0;
    std::unordered_map<std::string,CVector2> leaderPos;
    m_mapRobotPerTask.clear(); // Used to store the number of e-pucks that have worked on each task in the previous timestep

    /* Add existing task id to the map */
    CSpace::TMapPerType* cCTasks;

    if(m_bTaskExists) {
        if(m_bNoDemandTasks) {
            cCTasks = &GetSpace().GetEntitiesByType("circle_task_no_demand");
        } else {
            cCTasks = &GetSpace().GetEntitiesByType("circle_task");
            // cCTasks = &GetSpace().GetEntitiesByType("rectangle_task");
        }

        for(CSpace::TMapPerType::iterator itTask = cCTasks->begin();
            itTask != cCTasks->end();
            ++itTask) {
            
            /* Initialize each task with zero e-pucks working on it */
            if(m_bNoDemandTasks) {
                CCircleTaskNoDemandEntity& cCTask = *any_cast<CCircleTaskNoDemandEntity*>(itTask->second);
                m_mapRobotPerTask[cCTask.GetId()] = 0;
            } else {
                CCircleTaskEntity& cCTask = *any_cast<CCircleTaskEntity*>(itTask->second);
                m_mapRobotPerTask[cCTask.GetId()] = 0;
            }

        }
    }

    /* Loop leaders */
    CSpace::TMapPerType& m_cEPuckLeaders = GetSpace().GetEntitiesByType("e-puck_leader");
    for(CSpace::TMapPerType::iterator it = m_cEPuckLeaders.begin();
        it != m_cEPuckLeaders.end();
        ++it) {

        /* Get handle to e-puck_leader entity and controller */
        CEPuckLeaderEntity& cEPuckLeader = *any_cast<CEPuckLeaderEntity*>(it->second);
        CLeader& cController = dynamic_cast<CLeader&>(cEPuckLeader.GetControllableEntity().GetController());

        /* Get the position of the leader on the ground as a CVector2 */
        CVector2 cPos = CVector2(cEPuckLeader.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                                 cEPuckLeader.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
        leaderPos[cEPuckLeader.GetId()] = cPos;

        bool leaderAtTask = false;

        /* Give the leader its next task info if it is within the task range */        
        if(m_bTaskExists) {

            for(CSpace::TMapPerType::iterator itTask = cCTasks->begin();
                itTask != cCTasks->end();
                ++itTask) {

                /* Task location */
                if(m_bNoDemandTasks) {
                    CCircleTaskNoDemandEntity& cCTask = *any_cast<CCircleTaskNoDemandEntity*>(itTask->second);

                    CVector2 cTaskPos = cCTask.GetPosition();

                    /* If there is a task with the given task position AND leader is within the task range, return task demand */
                    if(cCTask.InArea(cPos)) {
                        cController.SetTaskId(cCTask.GetId());
                        UInt32 demand = cCTask.GetWorkPerformed();
                        cController.SetTaskDemand(demand);
                        cController.SetInitTaskDemand(cCTask.GetInitDemand());
                        if(demand > 0) 
                            cController.SetMinimumCount(cCTask.GetMinRobotNum());
                        else
                            cController.SetMinimumCount(1); // to prevent leader from moving away from the task
                        leaderAtTask = true;
                        break;
                    }
                } else {
                    CCircleTaskEntity& cCTask = *any_cast<CCircleTaskEntity*>(itTask->second);
                    // CRectangleTaskEntity& cCTask = *any_cast<CRectangleTaskEntity*>(itTask->second);

                    CVector2 cTaskPos = cCTask.GetPosition();

                    // /* If there is a task with the given task position AND leader is within the task range, return task demand */
                    // if((cPos - cTaskPos).SquareLength() < pow(cCTask.GetRadius(),2)) {
                    //     cController.SetTaskDemand(cCTask.GetDemand());
                    //     break;
                    // }

                    /* If there is a task with the given task position AND leader is within the task range, return task demand */
                    if(cCTask.InArea(cPos)) {
                        cController.SetTaskId(cCTask.GetId());
                        UInt32 demand = cCTask.GetDemand();
                        cController.SetTaskDemand(demand);
                        cController.SetInitTaskDemand(cCTask.GetInitDemand());
                        if(demand > 0) 
                            cController.SetMinimumCount(cCTask.GetMinRobotNum());
                        else
                            cController.SetMinimumCount(0);
                        leaderAtTask = true;
                        break;
                    }
                }
            }
        }

        /* Reset task info if the leader is not at any task */
        if( !leaderAtTask ) {
            cController.SetTaskId("");
            cController.SetTaskDemand(0);
            cController.SetInitTaskDemand(0);
            cController.SetMinimumCount(0);
        }
    }

    /* Loop workers */
    CSpace::TMapPerType& m_cEPucks = GetSpace().GetEntitiesByType("e-puck");
    for(CSpace::TMapPerType::iterator itEpuck = m_cEPucks.begin();
        itEpuck != m_cEPucks.end();
        ++itEpuck) {

        /* Get handle to e-puck entity and controller */
        CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(itEpuck->second);
        try {
            CWorker& cController = dynamic_cast<CWorker&>(cEPuck.GetControllableEntity().GetController());
            UInt8 unTeamId = cController.GetTeamID();

            /* Count how many e-pucks are in each state */
            if( cController.GetRobotState() == RobotState::FOLLOWER ) {
                // Count flock state

                if(unFollowers.count(unTeamId) == 0) {
                    unFollowers[unTeamId] = 1;
                } else {
                    unFollowers[unTeamId]++;
                }

                /* 
                * Check whether the e-puck is working on a task
                */            

                /* Current location */
                CVector2 cPos = CVector2(cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                                         cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

                /* Leaeder's location */
                std::ostringstream cLeaderId;
                cLeaderId.str("");
                cLeaderId << "L" << unTeamId;
                CVector2 cLeaderPos = leaderPos[cLeaderId.str()];

                if(m_bTaskExists) {

                    for(CSpace::TMapPerType::iterator itTask = cCTasks->begin();
                        itTask != cCTasks->end();
                        ++itTask) {

                        /* Task location */
                        if(m_bNoDemandTasks) {
                            CCircleTaskNoDemandEntity& cCTask = *any_cast<CCircleTaskNoDemandEntity*>(itTask->second);

                            CVector2 cTaskPos = cCTask.GetPosition();

                            /* Check if robot is working on a task */
                            if(cController.IsWorking()) {
                                if(cCTask.InArea(cPos) && cCTask.InArea(cLeaderPos)) {
                                    
                                    m_mapRobotPerTask[cCTask.GetId()]++; // Increment robot working on this task
                                    m_mapRobotTaskStatus[cEPuck.GetId()] = true;
                                    break;
                                }
                            }
                        } else {
                            CCircleTaskEntity& cCTask = *any_cast<CCircleTaskEntity*>(itTask->second);
                            // CRectangleTaskEntity& cCTask = *any_cast<CRectangleTaskEntity*>(itTask->second);

                            CVector2 cTaskPos = cCTask.GetPosition();

                            /* Check if robot is working on a task */
                            if(cController.IsWorking()) {
                                /* Check e-puck and its leader is within the range of a task */
                                // if((cPos - cTaskPos).SquareLength() < pow(cCTask.GetRadius(),2) &&
                                // (cLeaderPos - cTaskPos).SquareLength() < pow(cCTask.GetRadius(),2)) {
                                    
                                //     m_mapRobotPerTask[cCTask.GetId()]++; // Increment robot working on this task
                                //     break;
                                // }
                                if(cCTask.InArea(cPos) && cCTask.InArea(cLeaderPos)) {
                                    
                                    m_mapRobotPerTask[cCTask.GetId()]++; // Increment robot working on this task
                                    m_mapRobotTaskStatus[cEPuck.GetId()] = true;
                                    break;
                                }
                            }
                        }
                    }
                }

            }
            else if( cController.GetRobotState() == RobotState::CONNECTOR )
                ++unConnectors; // Count the number of connectors

        } catch(CARGoSException& ex) {
            THROW_ARGOSEXCEPTION_NESTED("While casting robot as a worker", ex);
            
        } catch(const std::bad_cast& e) {
            std::cout << e.what() << " in PreStep" << '\n';

        }
    }

    /* Loop leaders to inform */
    for(CSpace::TMapPerType::iterator it = m_cEPuckLeaders.begin();
        it != m_cEPuckLeaders.end();
        ++it) {

        /* Get handle to e-puck_leader entity and controller */
        CEPuckLeaderEntity& cEPuckLeader = *any_cast<CEPuckLeaderEntity*>(it->second);
        CLeader& cController = dynamic_cast<CLeader&>(cEPuckLeader.GetControllableEntity().GetController());

        /* Inform each leader the number of followers in its team */
        if(unFollowers.count(cController.GetTeamID()) == 0)
            cController.SetFollowerCount(0);
        else
            cController.SetFollowerCount(unFollowers[cController.GetTeamID()]);

    }

    // loop print m_mapRobotPerTask
    // for(const auto& [key, value] : m_mapRobotPerTask) {
    //     LOG << "Task: " << key << " Robots: " << value << std::endl;
    // }
}

/****************************************/
/****************************************/

void CExperimentLoopFunctionsNop::PostStep() {
    Real total_demand = 0;

    /* Add existing task id to the map */
    CSpace::TMapPerType* cCTasks;

    if(m_bTaskExists) {
        if(m_bNoDemandTasks) {
            cCTasks = &GetSpace().GetEntitiesByType("circle_task_no_demand");
        } else {
            cCTasks = &GetSpace().GetEntitiesByType("circle_task");
            // cCTasks = &GetSpace().GetEntitiesByType("rectangle_task");
        }

        for(CSpace::TMapPerType::iterator itTask = cCTasks->begin();
            itTask != cCTasks->end();
            ++itTask) {
            
            if(m_bNoDemandTasks) {
                CCircleTaskNoDemandEntity& cCTask = *any_cast<CCircleTaskNoDemandEntity*>(itTask->second);
                total_demand += (int)cCTask.GetWorkPerformed();
            } else {
                CCircleTaskEntity& cCTask = *any_cast<CCircleTaskEntity*>(itTask->second);
                // CRectangleTaskEntity& cCTask = *any_cast<CRectangleTaskEntity*>(itTask->second);
                total_demand += (int)cCTask.GetDemand();
            }
        }
    }

    /* Update task demands */
    if(m_bTaskExists) {

        for(CSpace::TMapPerType::iterator itTask = cCTasks->begin();
            itTask != cCTasks->end();
            ++itTask) {

            if(m_bNoDemandTasks) {
                CCircleTaskNoDemandEntity& cCTask = *any_cast<CCircleTaskNoDemandEntity*>(itTask->second);
                UInt32 currentWorkPerformed = cCTask.GetWorkPerformed();

                cCTask.SetCurrentRobotNum(m_mapRobotPerTask[cCTask.GetId()]);

                /* Check if there is enough robots working on the task */
                if(m_mapRobotPerTask[cCTask.GetId()] >= cCTask.GetMinRobotNum()) {

                    /* Update task demand */
                    cCTask.SetWorkPerformed(currentWorkPerformed + m_mapRobotPerTask[cCTask.GetId()]);
                    m_unPointsObtained += m_mapRobotPerTask[cCTask.GetId()];
                }
            } else {
                CCircleTaskEntity& cCTask = *any_cast<CCircleTaskEntity*>(itTask->second);
                UInt32 currentDemand = cCTask.GetDemand();

                cCTask.SetCurrentRobotNum(m_mapRobotPerTask[cCTask.GetId()]);

                if(currentDemand == 0)
                    continue; // Skip completed tasks

                /* Check if there is enough robots working on the task */
                if(m_mapRobotPerTask[cCTask.GetId()] >= cCTask.GetMinRobotNum()) {

                    // /* Update task demand (max robot number unit per step) */
                    // if(currentDemand <= m_mapRobotPerTask[cCTask.GetId()]) {
                    //     cCTask.SetDemand(0);
                    //     /* The floor texture must be updated */
                    //     m_pcFloor->SetChanged();
                    // } else {
                    //     cCTask.SetDemand(currentDemand - m_mapRobotPerTask[cCTask.GetId()]);
                    // }

                    /* Update task demand (max one unit per step) */
                    if(currentDemand <= m_mapRobotPerTask[cCTask.GetId()]) {
                        cCTask.SetDemand(0);
                        /* The floor texture must be updated */
                        m_pcFloor->SetChanged();
                    } else {
                        cCTask.SetDemand(currentDemand - 1);
                    }
                }
            }
        }
    }

    /* 
    * Output stuff to file 
    */

    if(m_bLogging) {
        /* Create new node for this timestep */
        TimeStep tData;
        tData.set_time(GetSpace().GetSimulationClock());
        tData.set_points(m_unPointsObtained);

        /* Output leader info */
        CSpace::TMapPerType& m_cEPuckLeaders = GetSpace().GetEntitiesByType("e-puck_leader");
        for(CSpace::TMapPerType::iterator it = m_cEPuckLeaders.begin();
            it != m_cEPuckLeaders.end();
            ++it) {

            CEPuckLeaderEntity& cEPuckLeader = *any_cast<CEPuckLeaderEntity*>(it->second);
            CLeader& cController = dynamic_cast<CLeader&>(cEPuckLeader.GetControllableEntity().GetController());

            Robot* robot = tData.add_robots();
            robot->set_name(cEPuckLeader.GetId());
            robot->set_teamid(cController.GetTeamID());
            robot->set_state(Robot_State_LEADER);
            if( !cController.GetLastAction().empty() )
                robot->set_action(cController.GetLastAction());
            robot->mutable_position()->set_x(cEPuckLeader.GetEmbodiedEntity().GetOriginAnchor().Position.GetX());
            robot->mutable_position()->set_y(cEPuckLeader.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
            robot->mutable_orientation()->set_w(cEPuckLeader.GetEmbodiedEntity().GetOriginAnchor().Orientation.GetW());
            robot->mutable_orientation()->set_x(cEPuckLeader.GetEmbodiedEntity().GetOriginAnchor().Orientation.GetX());
            robot->mutable_orientation()->set_y(cEPuckLeader.GetEmbodiedEntity().GetOriginAnchor().Orientation.GetY());
            robot->mutable_orientation()->set_z(cEPuckLeader.GetEmbodiedEntity().GetOriginAnchor().Orientation.GetZ());
            robot->set_totalsent((int)cController.GetTotalSent());
            // robot->set_totalreceived((int)cController.GetTotalReceived());
        }

        /* Output worker info */
        CSpace::TMapPerType& m_cEPucks = GetSpace().GetEntitiesByType("e-puck");
        for(CSpace::TMapPerType::iterator itEpuck = m_cEPucks.begin();
            itEpuck != m_cEPucks.end();
            ++itEpuck) {

            CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(itEpuck->second);
            CWorker& cController = dynamic_cast<CWorker&>(cEPuck.GetControllableEntity().GetController());

            Robot* robot = tData.add_robots();
            robot->set_name(cEPuck.GetId());
            robot->set_teamid(cController.GetTeamID());
            switch(cController.GetRobotState()) {
                case RobotState::FOLLOWER:
                    robot->set_state(Robot_State_FOLLOWER);
                    robot->set_hopcountteam(cController.GetTeamHopCount());
                    break;
                case RobotState::CONNECTOR:
                    robot->set_state(Robot_State_CONNECTOR);
                    break;
                case RobotState::TRAVELER:
                    robot->set_state(Robot_State_TRAVELER);
                    break;
                default:
                    std::cerr << "Tried to log unknown state " << (int)cController.GetRobotState() << std::endl;
                    break;
            }
            if( !cController.GetLastAction().empty() )
                robot->set_action(cController.GetLastAction());
            robot->mutable_position()->set_x(cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetX());
            robot->mutable_position()->set_y(cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
            robot->mutable_orientation()->set_w(cEPuck.GetEmbodiedEntity().GetOriginAnchor().Orientation.GetW());
            robot->mutable_orientation()->set_x(cEPuck.GetEmbodiedEntity().GetOriginAnchor().Orientation.GetX());
            robot->mutable_orientation()->set_y(cEPuck.GetEmbodiedEntity().GetOriginAnchor().Orientation.GetY());
            robot->mutable_orientation()->set_z(cEPuck.GetEmbodiedEntity().GetOriginAnchor().Orientation.GetZ());

            if(cController.GetRobotState() == RobotState::CONNECTOR) {
                for(const auto& [team, hop] : cController.GetHops()) {
                    HopCount* hopCount = robot->add_hopcount();
                    hopCount->set_teamid(team);
                    hopCount->set_count(hop.count);
                    hopCount->set_neighbor(hop.ID);
                }
                for(const auto& [team, hop] : cController.GetPrevHops()) {
                    HopCount* prevHop = robot->add_prevhops();
                    prevHop->set_teamid(team);
                    prevHop->set_count(hop.count);
                    prevHop->set_neighbor(hop.ID);
                }
            }
        }

        if(m_bTaskExists) {

            /* Output task info */
            for(CSpace::TMapPerType::iterator itTask = cCTasks->begin();
                itTask != cCTasks->end();
                ++itTask) {

                if(m_bNoDemandTasks) {
                    CCircleTaskNoDemandEntity& cCTask = *any_cast<CCircleTaskNoDemandEntity*>(itTask->second);

                    /* Log tasks that are inside the arena */
                    if(cCTask.GetPosition().GetX() < 500) {
                        Task* task = tData.add_tasks();
                        task->set_name(cCTask.GetId());
                        task->set_demand(cCTask.GetWorkPerformed());
                        task->set_requiredrobots(cCTask.GetMinRobotNum());
                        task->set_currentrobots(cCTask.GetCurrentRobotNum());
                        task->mutable_position()->set_x(cCTask.GetPosition().GetX());
                        task->mutable_position()->set_y(cCTask.GetPosition().GetY());
                        task->set_radius(cCTask.GetRadius());
                    }
                } else {
                    CCircleTaskEntity& cCTask = *any_cast<CCircleTaskEntity*>(itTask->second);
                    // CRectangleTaskEntity& cCTask = *any_cast<CRectangleTaskEntity*>(itTask->second);

                    /* Log tasks that are inside the arena */
                    if(cCTask.GetPosition().GetX() < 500) {
                        Task* task = tData.add_tasks();
                        task->set_name(cCTask.GetId());
                        task->set_demand(cCTask.GetDemand());
                        task->set_requiredrobots(cCTask.GetMinRobotNum());
                        task->set_currentrobots(cCTask.GetCurrentRobotNum());
                        task->mutable_position()->set_x(cCTask.GetPosition().GetX());
                        task->mutable_position()->set_y(cCTask.GetPosition().GetY());
                        task->set_radius(cCTask.GetRadius());
                    }
                }
            }
        }

        /* Write to file */
        m_cOutput.open(m_strBinaryFilePath.c_str(), std::ios::app | std::ios::binary);
        google::protobuf::util::SerializeDelimitedToOstream(tData, &m_cOutput);
        m_cOutput.close();
    }

    /* Grab frame */
    if(m_bFrameGrabbing) {
        CQTOpenGLRender& render = dynamic_cast<CQTOpenGLRender&>(GetSimulator().GetVisualization());
        CQTOpenGLWidget& widget = render.GetMainWindow().GetOpenGLWidget();
        widget.SetCamera(m_unCameraIndex);
        widget.SetGrabFrame(m_bFrameGrabbing);
    }

    /* Terminate simulation time limit is reached */
    if(m_bTaskExists) {
        if(m_bNoDemandTasks) {
            if (GetSpace().GetSimulationClock() == CSimulator::GetInstance().GetMaxSimulationClock()) {
                int final_time = GetSpace().GetSimulationClock();
                m_cOutput.open(m_strSummaryFilePath.c_str(), std::ios_base::app);
                m_cOutput << "\n";
                m_cOutput << "FINISH_TIME," << final_time << "\n";
                m_cOutput << "POINTS SCORED," << (int)m_unPointsObtained << "\n";
                // m_cOutput << "TASK_STATUS,FINISHED" << "\n";
                m_cOutput.close();
                std::cout << "[LOG] Reached time limit!" << std::endl;
                std::cout << "[LOG] Score: " << (int)m_unPointsObtained << std::endl;
                // std::cout << "[LOG] Mission time: " << final_time << std::endl;
                // std::cout << "[LOG] All tasks completed" << std::endl;
                std::cout << "[LOG] TERMINATING SIMULATION ..." << std::endl;
                this->Destroy();
            }
        } else {
            if(total_demand == 0) {
                int final_time = GetSpace().GetSimulationClock();
                m_cOutput.open(m_strSummaryFilePath.c_str(), std::ios_base::app);
                m_cOutput << "\n";
                m_cOutput << "FINISH_TIME," << final_time << "\n";
                m_cOutput << "POINTS SCORED," << (int)m_unPointsObtained << "\n";
                // m_cOutput << "TASK_STATUS,FINISHED" << "\n";
                m_cOutput.close();
                std::cout << "[LOG] Tasks completed!" << std::endl;
                std::cout << "[LOG] Score: " << (int)m_unPointsObtained << std::endl;
                // std::cout << "[LOG] Mission time: " << final_time << std::endl;
                // std::cout << "[LOG] All tasks completed" << std::endl;
                std::cout << "[LOG] TERMINATING SIMULATION ..." << std::endl;
                this->Destroy();
            }
        }
    }
    else if (GetSpace().GetSimulationClock() == CSimulator::GetInstance().GetMaxSimulationClock()) {

        int final_time = GetSpace().GetSimulationClock();
        m_cOutput.open(m_strSummaryFilePath.c_str(), std::ios_base::app);
        m_cOutput << "\n";
        m_cOutput << "FINISH_TIME," << final_time << "\n";
        m_cOutput << "POINTS SCORED," << (int)m_unPointsObtained << "\n";
        // m_cOutput << "TASK_STATUS,FINISHED" << "\n";
        m_cOutput.close();
        std::cout << "[LOG] Reached time limit!" << std::endl;
        std::cout << "[LOG] Score: " << (int)m_unPointsObtained << std::endl;
        // std::cout << "[LOG] Mission time: " << final_time << std::endl;
        // std::cout << "[LOG] All tasks completed" << std::endl;
        std::cout << "[LOG] TERMINATING SIMULATION ..." << std::endl;
        this->Destroy();
    }
}

Real CExperimentLoopFunctionsNop::GetArenaRadius() const {
    return m_fArenaRadius;
}

Real CExperimentLoopFunctionsNop::GetDeploymentRadius() const {
    return m_fDeploymentRadius;
}

// std::vector<CVector2> CExperimentLoopFunctionsNop::GetArenaSize() const {
//     return m_vecArenaSize;
// }

bool CExperimentLoopFunctionsNop::IsDrawRobotLabel() const {
    return m_bDrawRobotLabel;
}

/****************************************/
/****************************************/

bool CExperimentLoopFunctionsNop::IsLogging() {
    return m_bLogging;
}

/****************************************/
/****************************************/

void CExperimentLoopFunctionsNop::InitLogging() {
    
    /* 
    * Create new directory to store the logs
    */ 

    std::string dir_name = m_strOutput;

    /* Get the experiment directory name */
    if(dir_name[dir_name.size() - 1] == '/') {
        dir_name.pop_back(); // If last char is /, drop it
    }

    std::stringstream ss(dir_name);
    std::string segment;
    std::vector<std::string> dir_path;

    while(std::getline(ss, segment, '/')) {
        dir_path.push_back(segment);
    }
    dir_name = dir_path[dir_path.size() - 1];

    /* Get the parent directory name */
    dir_path.pop_back();
    std::ostringstream oss;
    oss.str("");
    for(auto& segment : dir_path) {
        oss << segment << "/";
    }
    std::string dir_parent_name = oss.str();
    
    /* Loop directory to see what experiment number to append to dir_name */
    oss.str("");
    oss << dir_parent_name << dir_name << "/";
    m_strDirPath = oss.str();
    std::vector<std::string> r;

    if(fs::exists(m_strDirPath)) {
        for(auto& p : fs::recursive_directory_iterator(m_strDirPath)) {
            if (p.is_directory()) {
                if(p.path().string().find(dir_name) != std::string::npos)
                    r.push_back(p.path().string()); // Count
            }
        }
    }
    
    std::string new_dir_name = dir_name;

    /* Append experiment config (team_num, team_size) */
    oss.str("");
    oss << "_" << m_unTeams << "T_" << m_unWorkerPerTeam << "R_" << EP_RAB_RANGE << "RAB";
    new_dir_name.append(oss.str());

    /* Append experiment number */
    int run_number;
    if( !m_strRunNumber.empty() ) {
        run_number = stoi(m_strRunNumber);
    } else {
        run_number = r.size() + 1;
    }

    if(run_number < 10) {
        new_dir_name.append("_00");
    } else if(run_number < 100) {
        new_dir_name.append("_0");
    } else {
        new_dir_name.append("_");
    }
    new_dir_name.append(std::to_string(run_number));

    /* Create directory */
    oss.str("");
    oss << dir_parent_name << dir_name << "/" << new_dir_name << "/";
    m_strDirPath = oss.str();
    fs::create_directories(m_strDirPath);
    LOG << "Created " << m_strDirPath << std::endl;

    /* Set output file names */
    oss.str("");
    oss << m_strDirPath << BINARY_FILENAME;
    m_strBinaryFilePath = oss.str();

    oss.str("");
    oss << m_strDirPath << SUMMARY_FILENAME;
    m_strSummaryFilePath = oss.str();

    oss.str("");
    oss << m_strDirPath << COMMAND_FILENAME;
    m_strCommandFilePath = oss.str();

    LOG << m_strBinaryFilePath << std::endl;
    LOG << m_strCommandFilePath << std::endl;

    /* 
    * Log experiment summary data
    */

    /* Write to file */
    m_cOutput.open(m_strSummaryFilePath.c_str(), std::ios_base::app);
    m_cOutput << "SCENARIO_NAME," << new_dir_name << "\n";
    m_cOutput << "SEED," << (int)CSimulator::GetInstance().GetRandomSeed() << "\n";
    m_cOutput << "MAX_SIMULATION_CLOCK," << (int)CSimulator::GetInstance().GetMaxSimulationClock() << "\n";

    m_cOutput << "TOTAL_LEADERS," << (int)m_unTeams << "\n";
    m_cOutput << "TOTAL_WORKERS," << (int)m_unTotalWorkers << "\n";
    // m_cOutput << "TOTAL_ROBOTS," << (int)(m_unTeams + m_unWorkerPerTeam) << "\n";

    // m_cOutput << "TOTAL_TASKS," << (int)m_unTotalTasks << "\n";
    // m_cOutput << "TASK_DEMAND," << (int)m_unTaskDemand << "\n";

    m_cOutput.close();
}

/****************************************/
/****************************************/

std::string CExperimentLoopFunctionsNop::GetCommandFilePath() {
    return m_strCommandFilePath;
}

/****************************************/
/****************************************/

std::unordered_map<std::string, UInt32> CExperimentLoopFunctionsNop::GetRobotPerTask() {
    return m_mapRobotPerTask;
}

/****************************************/
/****************************************/

UInt32 CExperimentLoopFunctionsNop::GetCurrentPoints() {
    return m_unPointsObtained;
}

/****************************************/
/****************************************/

std::string CExperimentLoopFunctionsNop::GetWorkerType() const {
    return m_strWorkerType;
}

std::string CExperimentLoopFunctionsNop::GetTaskType() const {
    if(m_bNoDemandTasks) {
        return "circle_task_no_demand";
    } else {
        return "circle_task";
    }
}

/****************************************/
/****************************************/

void CExperimentLoopFunctionsNop::InitRobots() {
    /*
    * Distribute leaders and robots
    */

    LOG << "[LOG] Adding robots..." << std::endl;

    /* ID counts */
    UInt32 unNextLeaderId = 1;
    UInt32 unNextRobotId = 1;
    /* Get the teams node */
    TConfigurationNode& et_tree = GetNode(config, "teams");
    /* Go through the nodes (teams) */
    TConfigurationNodeIterator itDistr;
    /* Number of teams */
    size_t unTeamCount = 0;
    m_unTotalLeaders = 0;
    // size_t unTotalWorkers = 0;
    m_unTotalWorkers = 0;

    for(itDistr = itDistr.begin(&et_tree);
        itDistr != itDistr.end();
        ++itDistr) {
        
        /* Get current node (team/custom_team) */
        TConfigurationNode& tDistr = *itDistr;

        /* Number of leaders to place */
        UInt32 unLeaders = 0;

        /* Number of robots to place */
        UInt32 unRobots;

        if(itDistr->Value() == "team") {
            /* Distribution center */
            CVector2 cCenter;
            GetNodeAttributeOrDefault(tDistr, "center", cCenter, CVector2());
            GetNodeAttributeOrDefault(tDistr, "leader_num", unLeaders, (UInt32)1);
            GetNodeAttributeOrDefault(tDistr, "robot_num", unRobots, (UInt32)0);
            /* Density of the robots */
            Real fDensity;
            GetNodeAttributeOrDefault(tDistr, "density", fDensity, 0.0);
            /* Worker type */
            GetNodeAttributeOrDefault(tDistr, "worker_type", m_strWorkerType, std::string("worker"));
            /* Place robots */
            PlaceCluster(cCenter, unLeaders, unRobots, fDensity, m_strWorkerType, unNextLeaderId, unNextRobotId);

            /* Get the waypoints node */
            std::queue<CVector2> waypoints; // Queue to provide to the robot
            /* Go through the nodes (waypoints) */
            TConfigurationNodeIterator itWaypt;
            for(itWaypt = itWaypt.begin(&tDistr);
                itWaypt != itWaypt.end();
                ++itWaypt) {

                /* Get current node (waypoint) */
                TConfigurationNode& tWaypt = *itWaypt;
                /* Coordinate of waypoint */
                CVector2 coord;
                GetNodeAttribute(tWaypt, "coord", coord);
                m_vecWaypointPos.push_back(coord);
                waypoints.push(coord);
            }

            /* Get the newly created leader */
            std::ostringstream cEPId;
            cEPId.str("");
            cEPId << "L" << unNextLeaderId;
            CEPuckLeaderEntity& cEPuckLeader = dynamic_cast<CEPuckLeaderEntity&>(GetSpace().GetEntity(cEPId.str()));
            CLeader& cController = dynamic_cast<CLeader&>(cEPuckLeader.GetControllableEntity().GetController());
            /* Set list of waypoints to leader */
            cController.SetWaypoints(waypoints);

            // LOG << "Set waypoints for " << cEPId.str() << std::endl;

            /* Update robot count */

            /* Update the number of teams in the experiment */
            unTeamCount++;

            /* Update robot count */
            m_unTotalLeaders += unLeaders;
            unNextLeaderId += unLeaders;
            m_unTotalWorkers += unRobots;
            unNextRobotId += unRobots;

        }
        else if(itDistr->Value() == "distribute_teams") {

            /* Load parameters */
            CVector2 cCenter;
            GetNodeAttributeOrDefault(tDistr, "center", cCenter, CVector2());
            GetNodeAttributeOrDefault(tDistr, "team_num", unLeaders, (UInt32)2);
            GetNodeAttributeOrDefault(tDistr, "max_per_team", m_unMaxPerTeam, (UInt32)10);
            GetNodeAttributeOrDefault(tDistr, "robot_per_team", unRobots, (UInt32)6);
            Real fDensity;
            GetNodeAttributeOrDefault(tDistr, "density", fDensity, 0.2);
            GetNodeAttributeOrDefault(tDistr, "worker_type", m_strWorkerType, std::string("worker"));

            Real fRadius = Sqrt((EP_AREA * (1 + m_unMaxPerTeam)) / (fDensity * ARGOS_PI));
            m_fTeamDeployRadius = fRadius;

            for(size_t i = 0; i < unLeaders; i++) {

                Real deployRange = 0;
                CRadians deployAngle = CRadians();

                // if(i > 0) {
                    deployRange = 2 * m_fTeamDeployRadius;
                // }
                // if(i > 1) {
                    deployAngle.SetValue(2 * ARGOS_PI / unLeaders * i);
                // }

                LOG << "radius: " << deployRange << std::endl;
                LOG << "angle: " << deployAngle << std::endl;

                cCenter = CVector2(deployRange, deployAngle);

                LOG << "cCenter: " << cCenter << std::endl;

                /* Update robot count */
                m_unTotalLeaders++;
                m_unTotalWorkers += unRobots;

                /* Place robots */
                PlaceCluster(cCenter, 1, unRobots, fDensity, m_strWorkerType, unNextLeaderId, unNextRobotId);
                
                unNextLeaderId++;
                unNextRobotId += unRobots;
            }

            /* Set the number of teams in the experiment */
            unTeamCount = unLeaders;
        } 
        // LOG << "m_unTotalWorkers: " << m_unTotalWorkers << std::endl;
    }

    m_unTeams = unTeamCount; // leader with the highest id - 1 = teams available to work
    m_unWorkerPerTeam = m_unTotalWorkers / m_unTeams;
    //##########################################
    m_fArenaRadius = 2.4; // TEMP n-team varying rab_range
    //##########################################
    m_fDeploymentRadius = 3 * m_fTeamDeployRadius;
    LOG << "arena radius " << m_fArenaRadius << ", deployment radius " << m_fDeploymentRadius << std::endl;

    LOG << "[LOG] Added robots" << std::endl;

}

/****************************************/
/****************************************/

void CExperimentLoopFunctionsNop::InitTasks() {
    /*
    * Initialize tasks
    */

    LOG << "[LOG] Adding tasks..." << std::endl;

    /* ID counts */
    UInt32 m_unNextTaskId = 1;
    /* Meta data */
    size_t m_unTotalTasks = 0;
    UInt32 m_unTaskDemand = 0; 
    /* Get the teams node */
    TConfigurationNode& ts_tree = GetNode(config, "tasks");
    /* Go through the nodes (tasks) */
    TConfigurationNodeIterator itDistr;
    for(itDistr = itDistr.begin(&ts_tree);
        itDistr != itDistr.end();
        ++itDistr) {

        m_bTaskExists = true;
        m_bTaskComplete = false;

        /* Get current node (task) */
        TConfigurationNode& tDistr = *itDistr;
        /* Task center */
        CVector2 cCenter;
        GetNodeAttribute(tDistr, "position", cCenter);
        /* Task radius */
        Real fRadius;
        GetNodeAttribute(tDistr, "radius", fRadius);
        /* Task Height */
        Real fHeight;
        GetNodeAttribute(tDistr, "height", fHeight);
        /* Task demand */
        UInt32 unDemand;
        GetNodeAttribute(tDistr, "task_demand", unDemand);
        /* Minimum robot constraint */
        UInt32 unMinRobotNum;
        GetNodeAttribute(tDistr, "minimum_robot_num", unMinRobotNum);
        /* Maximum robot constraint */
        UInt32 unMaxRobotNum;
        GetNodeAttribute(tDistr, "maximum_robot_num", unMaxRobotNum);
        
        /* Place Tasks */
        // PlaceTask(cCenter, fRadius, unDemand, unMinRobotNum, unMaxRobotNum, m_unNextTaskId);
        PlaceCircleTask(cCenter, fRadius, fHeight, unDemand, unMinRobotNum, unMaxRobotNum, m_unNextTaskId);

        /* Update task count */
        m_unNextTaskId++;

        m_unTotalTasks++;
        m_unTaskDemand += unDemand;
    }

    if(m_bLogging) {
        /* Write to file */
        m_cOutput.open(m_strSummaryFilePath.c_str(), std::ios_base::app);
        m_cOutput << "TOTAL_TASKS," << (int)m_unTotalTasks << "\n";
        m_cOutput << "TASK_DEMAND," << (int)m_unTaskDemand << "\n";
        m_cOutput.close();
    }

    LOG << "[LOG] Added tasks" << std::endl;
}

/****************************************/
/****************************************/

void CExperimentLoopFunctionsNop::InitTasksCircular() {
    /*
    * Initialize tasks
    */

    LOG << "[LOG] Adding tasks..." << std::endl;

    /* Set the type of the task */
    // m_bNoDemandTasks = true;
    // m_bNoDemandTasks = false;

    /* ID counts */
    m_unNextTaskId = 1;
    /* Meta data */
    UInt32 unTotalTasks = m_unTeams;
    UInt32 unInitTasks = m_unTeams;
    m_unTotalTasks = 0;
    m_unTaskDemand = 0; 
    m_unPointsObtained = 0;

    m_bTaskExists = true;
    m_bTaskComplete = false;

    // TODO: Check if task exists at all in argos file

    for(UInt32 i = 1; i <= unTotalTasks; ++i) {

        /* Task dimensions */
        Real density = 0.05;
        // Real fRadius = Sqrt(EP_AREA * (m_unMaxPerTeam + 1) / (density * ARGOS_PI));
        Real fRadius = 0.25; // TEMP FIXED
        Real fHeight = 0.3;
        /* Task demand */
        UInt32 unDemand = 400; // FIXED
        if(m_bNoDemandTasks)
            unDemand = 400000;
        /* Min and Max robot constraint */
        UInt32 unMinRobotNum = 1; // m_unWorkerPerTeam / 2;
        UInt32 unMaxRobotNum = 100;

        CVector2 cCenter = CVector2();
        if(i <= unInitTasks) {

            /* Check whether this position overlaps with existing tasks */
            for(UInt32 j = 0; j < 100; ++j) {

                LOG << "--------------" << std::endl;

                // // Pick random length
                // // Make sure it fits in the circle arena and avoids the deployment area
                CRange<Real> cLengthRange = CRange<Real>(0, 1); 

                //#########################################################################
                // RANDOMLY DISTRIBUTE TASKS ANYWHERE
                CRange<Real> cAngleRange = CRange<Real>(0, 2*ARGOS_PI);
                // Uniformly choose a point based on https://stackoverflow.com/a/50746409
                Real radius = (m_fArenaRadius - fRadius) * Sqrt(m_pcRNG->Uniform(cLengthRange));
                CRadians angle = CRadians(m_pcRNG->Uniform(cAngleRange));
                //#########################################################################

                LOG << "radius: " << radius << ", angle: " << angle << std::endl;
                cCenter = CVector2(radius, angle);

                LOG << "Trying " << cCenter.GetX() << ", " << cCenter.GetY() << std::endl;

                bool bInvalidTaskPos = false;

                /* Check whether the chosen position does not overlap with the deployment area */
                // LOG << "cCenter.Length(): " << cCenter.Length() << ", m_fDeploymentRadius + fRadius: " << (m_fDeploymentRadius + fRadius) << ", m_fArenaRadius - fRadius: " << (m_fArenaRadius - fRadius) << std::endl;
                if(cCenter.Length() < (m_fDeploymentRadius + fRadius)) {
                    bInvalidTaskPos = true;
                }

                /* Check whether the chosen position is not too close to existing tasks (2*comm_range) */
                for(auto& [task_id, task] : m_mapTaskPos) {

                    if(task.first.Length() > 500)
                        continue; // Skip tasks outside of arena

                    if((task.first - cCenter).Length() < EP_RAB_RANGE || (task.first - cCenter).Length() < 2*fRadius) {
                        bInvalidTaskPos = true;
                    }
                }

                if(bInvalidTaskPos) {
                    /* Position is invalid. Try again */
                    LOG << "invalid position" << std::endl;
                    continue;
                } else {
                    /* Position is valid. Place task at the chosen position */
                    LOG << "VALID position" << std::endl;
                    break;
                }
            }

            m_unTotalTasks++;
            m_unTaskDemand += unDemand;
        } else {
            // Place it out of sight
            cCenter = CVector2(1000, 1000);
        }

        // LOG << "Arena radius " << m_fArenaRadius << std::endl;
        // LOG << "Task pos " << cCenter.GetX() << ", " << cCenter.GetY() << std::endl;

        // PlaceTask
        PlaceCircleTask(cCenter, fRadius, fHeight, unDemand, unMinRobotNum, unMaxRobotNum, m_unNextTaskId);

        m_mapTaskPos[m_unNextTaskId] = {cCenter, fRadius};

        /* Update task count */
        m_unNextTaskId++;
    }

    LOG << "[LOG] Added tasks" << std::endl;
}

/****************************************/
/****************************************/

void CExperimentLoopFunctionsNop::AssignTasks() {
       
    /* Sort tasks according to their angle */
    std::vector<std::pair<UInt32, CVector2>> v;
    for(const auto& [id, pair] : m_mapTaskPos) {
        v.push_back({id, pair.first});
    }

    std::sort(v.begin(), v.end(), [](auto &left, auto &right) {
        Real lv = left.second.Angle().GetValue();
        Real rv = right.second.Angle().GetValue();
        if(lv < 0)
            lv += 2*ARGOS_PI;
        if(rv < 0)
            rv += 2*ARGOS_PI;
        return lv < rv;
    });

    /* Create a sequence of leader names to use to assign tasks */
    std::vector<std::string> leaders;
    std::ostringstream cId;
    
    size_t startLeaderId;
    if(m_bNoDemandTasks) {
        startLeaderId = 2;
    } else {
        startLeaderId = 1;
    }

    for(size_t i = startLeaderId; i <= m_unTotalLeaders; i++) {
        cId.str("");
        cId << "L" << i;
        leaders.push_back(cId.str());
    }

    // for(const auto& t : v) {
    //     LOG << t.first << ", " << t.second.Angle() << std::endl;
    // }

    /* Find the task with the smallest angle for L1 from the arena centre */
    CEntity& cEntity = GetSpace().GetEntity(leaders[0]);
    CEPuckLeaderEntity* cEPuckLeader = dynamic_cast<CEPuckLeaderEntity*>(&cEntity);
    CVector2 L1Angle = CVector2(cEPuckLeader->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                                cEPuckLeader->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
    Real firstTask = (v.front().second.Angle() - L1Angle.Angle()).GetAbsoluteValue(); // first task in the vector
    Real lastTask = (v.back().second.Angle() - L1Angle.Angle()).GetAbsoluteValue(); // last task in the vector
    // std::cout << firstTask << " VS " << lastTask << std::endl;
    if(lastTask < firstTask) {
        // Put the last task to first
        auto tmp = v.back();
        v.pop_back();
        v.insert(v.begin(), tmp);
        LOG << "Moved last task in the vector to the front" << std::endl;
    }

    for(const auto& t : v) {
        LOG << t.first << ", " << t.second.Angle() << std::endl;
    }

    UInt32 count = 0;

    /* Assign task to leaders */
    for(const auto& pair : v) {
        LOG << "-------------" << std::endl;
        LOG << "id: " << pair.first << ", vec: " << pair.second << std::endl;

        std::queue<CVector2> waypoints; // Queue to provide to the leader

        CEntity& cEntity = GetSpace().GetEntity(leaders[count]);
        CEPuckLeaderEntity* cEPuckLeader = dynamic_cast<CEPuckLeaderEntity*>(&cEntity);
        CLeader& cController = dynamic_cast<CLeader&>(cEPuckLeader->GetControllableEntity().GetController());

        /* Assign waypoint to exit the deployment area */
        CVector2 exitAreaVec = CVector2(cEPuckLeader->GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                                        cEPuckLeader->GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
        exitAreaVec.Normalize();
        exitAreaVec.Scale(m_fDeploymentRadius, m_fDeploymentRadius);
        waypoints.push(exitAreaVec);

        /* Assign task to leader */
        waypoints.push(pair.second);

        cController.SetWaypoints(waypoints);

        LOG << "Assign " << (int)pair.first << " to " << leaders[count] << std::endl;

        count++;
    }
}

/****************************************/
/****************************************/

void CExperimentLoopFunctionsNop::CheckInitialConnectorInRange() {

    // loop all teams except for team 1
    for(size_t i = 2; i <= m_unTeams; i++) {
        LOG << "Checking team " << i << std::endl;
        std::ostringstream cEPId;
        cEPId.str("");
        cEPId << "L" << i;
        std::string leaderId = cEPId.str();
        CEntity& cEntity = GetSpace().GetEntity(leaderId);
        CEPuckLeaderEntity* cEPuckLeader = dynamic_cast<CEPuckLeaderEntity*>(&cEntity);
        CLeader& cController = dynamic_cast<CLeader&>(cEPuckLeader->GetControllableEntity().GetController());

        bool bRobotInRange = false;

        // loop all robots in the team
        for(size_t j = 1; j <= m_unWorkerPerTeam; j++) {
            LOG << "Checking robot " << (i-1)*m_unWorkerPerTeam + j << std::endl;
            cEPId.str("");
            cEPId << "F" << (i-1)*m_unWorkerPerTeam + j;
            std::string robotId = cEPId.str();
            CEntity& cEntity = GetSpace().GetEntity(robotId);
            CEPuckEntity* cEPuck = dynamic_cast<CEPuckEntity*>(&cEntity);

            // check if the minimum distance of a team member to the arena center is less than the communication range
            LOG << "Distance to center: " << (cEPuck->GetEmbodiedEntity().GetOriginAnchor().Position - CVector3(0, 0, 0)).Length() << std::endl;
            LOG << "EP_RAB_RANGE: " << EP_RAB_RANGE << std::endl;
            if((cEPuck->GetEmbodiedEntity().GetOriginAnchor().Position - CVector3(0, 0, 0)).Length() < EP_RAB_RANGE - 0.05) {
                bRobotInRange = true;
                break;
            }
        }

        if( !bRobotInRange ) {
            LOG << "No robot in range for team " << leaderId << std::endl;
            LOG << "Placing a robot in range..." << std::endl;

            // Get the ID of the first member of the team
            std::ostringstream cEPId;
            cEPId.str("");
            cEPId << "F" << (i-1)*m_unWorkerPerTeam + 1;
            std::string robotId = cEPId.str();
            CEntity& cEntity = GetSpace().GetEntity(robotId);
            CEPuckEntity* cEPuck = dynamic_cast<CEPuckEntity*>(&cEntity);

            Real deployRange = 0;
            CRadians deployAngle = CRadians();

            deployRange = 2 * m_fTeamDeployRadius;
            deployAngle.SetValue(2 * ARGOS_PI / m_unTotalLeaders * (i-1));

            LOG << "radius: " << deployRange << std::endl;
            LOG << "angle: " << deployAngle << std::endl;

            CVector2 cCenter = CVector2(deployRange, deployAngle);

            // randomly select a position until the robot's distance to the arena center is less than the communication range
            UInt32 unTrials = 0;
            bool bDone;
            CRange<Real> cLengthRange = CRange<Real>(0, 1); 
            CRange<Real> cAngleRange = CRange<Real>(0, 2*ARGOS_PI);
            do {
                ++unTrials;

                Real radius = m_fTeamDeployRadius * Sqrt(m_pcRNG->Uniform(cLengthRange));
                CRadians angle = CRadians(m_pcRNG->Uniform(cAngleRange));
                CVector2 randPos = CVector2(radius, angle);      
                CVector3 cEPPos = CVector3(randPos.GetX() + cCenter.GetX(),
                                randPos.GetY() + cCenter.GetY(),
                                0.0f);      
                CQuaternion cEPRot = CQuaternion(m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
                                     CVector3::Z);
                bDone = MoveEntity(cEPuck->GetEmbodiedEntity(), cEPPos, cEPRot);
                LOG << "angle: " << angle << ", radius: " << radius << std::endl;
                if(bDone) {
                    LOG << "Distance to center (trial="<< unTrials << "): " << (cEPuck->GetEmbodiedEntity().GetOriginAnchor().Position - CVector3(0, 0, 0)).Length() << std::endl;
                }
            } while(!bDone && unTrials <= MAX_PLACE_TRIALS && (cEPuck->GetEmbodiedEntity().GetOriginAnchor().Position - CVector3(0, 0, 0)).Length() >= EP_RAB_RANGE - 0.05);
            if(!bDone) {
                THROW_ARGOSEXCEPTION("Can't place " << cEPId.str());
            }
            LOG << "Successfully placed " << cEPId.str() << " in range" << std::endl;
        }
    }
}

/****************************************/
/****************************************/

void CExperimentLoopFunctionsNop::PlaceCluster(const CVector2& c_center,
                                            UInt32 un_leaders,
                                            UInt32 un_robots,
                                            Real f_density,
                                            std::string str_worker_type,
                                            UInt32 un_leader_id_start,
                                            UInt32 un_robot_id_start) {

    try {

        CRange<Real> cLengthRange = CRange<Real>(0, 1); 
        CRange<Real> cAngleRange = CRange<Real>(0, 2*ARGOS_PI);

        /* Place robots */
        UInt32 unTrials;
        CEPuckLeaderEntity* pcEPL;
        CEPuckEntity* pcEP;
        std::ostringstream cEPId;
        CVector3 cEPPos;
        CQuaternion cEPRot;
        std::string leaderID = "";

        UInt32 number_of_trials = 0;

        /* Determine worker type */
        std::string strController = "worker";

        /* For each leader */ // CURRENTLY ONLY ONE LEADER PER TEAM IS SUPPORTED
        for(size_t i = 0; i < un_leaders; ++i) {
            /* Make the id */
            cEPId.str("");
            cEPId << "L" << (i + un_leader_id_start);
            leaderID = cEPId.str();
            /* Create the leader in the origin and add it to ARGoS space */
            pcEPL = new CEPuckLeaderEntity(cEPId.str(),
                                           LE_CONTROLLER,
                                           CVector3(),
                                           CQuaternion(),
                                           EP_RAB_RANGE,
                                           MESSAGE_BYTE_SIZE,
                                           "");
            AddEntity(*pcEPL);
            m_vecEntityID.push_back(cEPId.str());

            /* Assign initial number of followers */
            CLeader& clController = dynamic_cast<CLeader&>(pcEPL->GetControllableEntity().GetController());
            clController.SetFollowerCount(un_robots);

            /* Try to place it in the arena */
            unTrials = 0;
            bool bDone;
            do {
                /* Choose a random position */
                ++unTrials;
                // cEPPos.Set(m_pcRNG->Uniform(cAreaRange) + c_center.GetX(),
                //            m_pcRNG->Uniform(cAreaRange) + c_center.GetY(),
                //            0.0f);
                // CVector2 posInPlane = CVector2(m_pcRNG->Uniform(cLengthRange),
                //                                CRadians(m_pcRNG->Uniform(cAngleRange)));
                // Uniformly choose a point based on https://stackoverflow.com/a/50746409
                Real radius = m_fTeamDeployRadius * Sqrt(m_pcRNG->Uniform(cLengthRange));
                CRadians angle = CRadians(m_pcRNG->Uniform(cAngleRange));
                CVector2 randPos = CVector2(radius, angle);
                // cEPPos.Set(posInPlane.GetX() + c_center.GetX(),
                //            posInPlane.GetY() + c_center.GetY(),
                //            0.0f);        
                cEPPos.Set(randPos.GetX() + c_center.GetX(),
                           randPos.GetY() + c_center.GetY(),
                           0.0f);                                   
                cEPRot.FromAngleAxis(m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
                                     CVector3::Z);
                bDone = MoveEntity(pcEPL->GetEmbodiedEntity(), cEPPos, cEPRot);
                                // LOG << "trials " << unTrials << std::endl;

            } while(!bDone && unTrials <= MAX_PLACE_TRIALS);
            if(!bDone) {
                THROW_ARGOSEXCEPTION("Can't place " << cEPId.str());
            }
        }
        // number_of_trials += unTrials;
        // LOG << "sum trials for leaders: " << number_of_trials << std::endl;

        /* For each robot worker */
        for(size_t i = 0; i < m_unMaxPerTeam; ++i) {
            /* Make the id */
            cEPId.str("");
            cEPId << "F" << (i + un_robot_id_start);
            /* Create the robot in the origin and add it to ARGoS space */
            pcEP = new CEPuckEntity(cEPId.str(),
                                    strController,
                                    CVector3(),
                                    CQuaternion(),
                                    EP_RAB_RANGE,
                                    MESSAGE_BYTE_SIZE,
                                    "");
            AddEntity(*pcEP);
            m_vecEntityID.push_back(cEPId.str());

            CWorker* cfController = dynamic_cast<CWorker*>(&pcEP->GetControllableEntity().GetController());
            cfController->SetTeamID(un_leader_id_start);  // Assign teamID of first team leader
            cfController->SetRABRange(EP_RAB_RANGE);

            /* Try to place it in the arena */
            unTrials = 0;
            bool bDone;
            do {
                /* Choose a random position */
                ++unTrials;
                // cEPPos.Set(m_pcRNG->Uniform(cAreaRange) + c_center.GetX(),
                //            m_pcRNG->Uniform(cAreaRange) + c_center.GetY(),
                //            0.0f);
                // CVector2 posInPlane;
                // if(i + un_robot_id_start == 1) {
                //     posInPlane = CVector2(m_pcRNG->Uniform(cLengthRangeInitConnector),
                //                           CRadians(m_pcRNG->Uniform(cAngleRange)));
                // } else {
                //     posInPlane = CVector2(m_pcRNG->Uniform(cLengthRange),
                //                           CRadians(m_pcRNG->Uniform(cAngleRange)));
                // }
                // cEPPos.Set(posInPlane.GetX() + c_center.GetX(),
                //            posInPlane.GetY() + c_center.GetY(),
                //            0.0f);    
                Real radius = m_fTeamDeployRadius * Sqrt(m_pcRNG->Uniform(cLengthRange));
                CRadians angle = CRadians(m_pcRNG->Uniform(cAngleRange));
                CVector2 randPos = CVector2(radius, angle);      
                cEPPos.Set(randPos.GetX() + c_center.GetX(),
                           randPos.GetY() + c_center.GetY(),
                           0.0f);      
                cEPRot.FromAngleAxis(m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
                                     CVector3::Z);
                bDone = MoveEntity(pcEP->GetEmbodiedEntity(), cEPPos, cEPRot);
                                // LOG << "trials " << unTrials << std::endl;

            } while(!bDone && unTrials <= MAX_PLACE_TRIALS);
            if(!bDone) {
                THROW_ARGOSEXCEPTION("Can't place " << cEPId.str());
            }
        }

        /* Get entity with the name "F1" and cast it as an epuckentity */
        if(m_bNoDemandTasks) {
            if(un_leader_id_start == 2) {
                LOG << "Moving F1, next leader " << un_leader_id_start << std::endl;
                CEPuckEntity& cEntity = dynamic_cast<CEPuckEntity&>(GetSpace().GetEntity(std::string("F1")));
                // move to 0.15,0,0
                cEPPos.Set(0.15, 0, 0);
                // cEPRot.FromAngleAxis(m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE), CVector3::Z);
                MoveEntity(cEntity.GetEmbodiedEntity(), cEPPos, cEPRot);
            }
        } else {
            if(un_leader_id_start == 1) {
                LOG << "Moving F1, next leader " << un_leader_id_start << std::endl;
                CEPuckEntity& cEntity = dynamic_cast<CEPuckEntity&>(GetSpace().GetEntity(std::string("F1")));
                // move to 0.15,0,0
                cEPPos.Set(0, 0, 0);
                // cEPRot.FromAngleAxis(m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE), CVector3::Z);
                MoveEntity(cEntity.GetEmbodiedEntity(), cEPPos, cEPRot);
            }
        }

    } catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("While placing robots in a cluster", ex);
    }
}

/****************************************/
/****************************************/

void CExperimentLoopFunctionsNop::PlaceCustomPosition(const CVector2& c_center,
                                                   std::string str_type,
                                                   UInt32 un_leader_id_start,
                                                   UInt32 un_robot_id_start) {

    try{
        /* Place robot */
        std::ostringstream cEPId;
        CVector3 cEPPos;
        CQuaternion cEPRot;

        if(str_type == "leader") {
            CEPuckLeaderEntity* pcEPL;
            /* Make the id */
            cEPId.str("");
            cEPId << "L" << (un_leader_id_start);
            /* Create the leader in the origin and add it to ARGoS space */
            pcEPL = new CEPuckLeaderEntity(cEPId.str(),
                                           LE_CONTROLLER,
                                           CVector3(),
                                           CQuaternion(),
                                           EP_RAB_RANGE,
                                           MESSAGE_BYTE_SIZE,
                                           "");
            AddEntity(*pcEPL);
            m_vecEntityID.push_back(cEPId.str());

            /* Try to place it in the arena */
            bool bDone;
            /* Place on specified position */
            cEPPos.Set(c_center.GetX(),
                        c_center.GetY(),
                        0.0f);
            cEPRot.FromAngleAxis(m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
                                    CVector3::Z);
            bDone = MoveEntity(pcEPL->GetEmbodiedEntity(), cEPPos, cEPRot);

            if(!bDone) {
                THROW_ARGOSEXCEPTION("Can't place " << cEPId.str());
            }
        }
        else if(str_type == "follower") {
            CEPuckEntity* pcEP;
            /* Make the id */
            cEPId.str("");
            cEPId << "F" << (un_robot_id_start);
            /* Create the robot in the origin and add it to ARGoS space */
            pcEP = new CEPuckEntity(cEPId.str(),
                                    WO_CONTROLLER,
                                    CVector3(),
                                    CQuaternion(),
                                    EP_RAB_RANGE,
                                    MESSAGE_BYTE_SIZE,
                                    "");
            AddEntity(*pcEP);
            m_vecEntityID.push_back(cEPId.str());

            /* Assign initial team id */
            CWorker& cController = dynamic_cast<CWorker&>(pcEP->GetControllableEntity().GetController());
            cController.SetTeamID(un_leader_id_start);  // Assign teamID of team leader

            /* Try to place it in the arena */
            bool bDone;
            /* Place on specified position */
            cEPPos.Set(c_center.GetX(),
                        c_center.GetY(),
                        0.0f);
            cEPRot.FromAngleAxis(m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
                                    CVector3::Z);
            bDone = MoveEntity(pcEP->GetEmbodiedEntity(), cEPPos, cEPRot);
        }
    } catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("While placing robot in a custom position", ex);
    }
}

/****************************************/
/****************************************/

void CExperimentLoopFunctionsNop::PlaceLeader(const CVector2& c_center,
                                            UInt32 un_leader_id_start) {

    try {
        /* Place robots */
        UInt32 unTrials;
        CEPuckLeaderEntity* pcEPL;
        std::ostringstream cEPId;
        CVector3 cEPPos;
        CQuaternion cEPRot;

        /* Make the id */
        cEPId.str("");
        cEPId << "L" << un_leader_id_start;
        /* Create the leader in the origin and add it to ARGoS space */
        pcEPL = new CEPuckLeaderEntity(cEPId.str(),
                                        LE_CONTROLLER,
                                        CVector3(),
                                        CQuaternion(),
                                        EP_RAB_RANGE,
                                        MESSAGE_BYTE_SIZE,
                                        "");
        AddEntity(*pcEPL);
        m_vecEntityID.push_back(cEPId.str());

        /* Assign initial number of followers */
        CLeader& clController = dynamic_cast<CLeader&>(pcEPL->GetControllableEntity().GetController());
        clController.SetFollowerCount(0);

        /* Try to place it in the arena */
        bool bDone;     
        cEPPos.Set(0.0f, 0.0f, 0.0f);                                   
        cEPRot.FromAngleAxis(CRadians(), CVector3::Z);
        bDone = MoveEntity(pcEPL->GetEmbodiedEntity(), cEPPos, cEPRot);

    } catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("While placing a leader", ex);
    }
}

/****************************************/
/****************************************/

void CExperimentLoopFunctionsNop::PlaceLeaderAndInitConnector(const CVector2& c_center,
                                                            Real f_density,
                                                            std::string str_worker_type,
                                                            UInt32 un_leader_id_start,
                                                            UInt32 un_robot_id_start) {

    try {
        /* Place robots */
        UInt32 unTrials;
        CEPuckLeaderEntity* pcEPL;
        std::ostringstream cEPId;
        CVector3 cEPPos;
        CQuaternion cEPRot;

        /* Make the id */
        cEPId.str("");
        cEPId << "L" << un_leader_id_start;
        /* Create the leader in the origin and add it to ARGoS space */
        pcEPL = new CEPuckLeaderEntity(cEPId.str(),
                                        LE_CONTROLLER,
                                        CVector3(),
                                        CQuaternion(),
                                        EP_RAB_RANGE,
                                        MESSAGE_BYTE_SIZE,
                                        "");

        m_vecEntityID.push_back(cEPId.str());

        /* Assign initial number of followers */
        CLeader& clController = dynamic_cast<CLeader&>(pcEPL->GetControllableEntity().GetController());
        clController.SetFollowerCount(0);

        // /* Try to place it in the arena */
        // bool bDone;     
        // cEPPos.Set(0.0f, 0.0f, 0.0f);                                   
        // cEPRot.FromAngleAxis(CRadians(), CVector3::Z);
        // bDone = MoveEntity(pcEPL->GetEmbodiedEntity(), cEPPos, cEPRot);

        AddEntity(*pcEPL);

    } catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("While placing a leader", ex);
    }

    try {
        /* Place init connector */
        /* Place robots */
        UInt32 unTrials;
        CVector3 cEPPos;
        CQuaternion cEPRot;
        /* Place F1 at the center */
        std::string initConnectorId = "F1";
        /* Create the robot in the origin and add it to ARGoS space */
        CEPuckEntity* pcEP = new CEPuckEntity(initConnectorId,
                                            str_worker_type,
                                            CVector3(),
                                            CQuaternion(),
                                            EP_RAB_RANGE,
                                            MESSAGE_BYTE_SIZE,
                                            "");

        AddEntity(*pcEP);
        m_vecEntityID.push_back(initConnectorId);

        /* Assign initial team id */
        CWorker& cfController = dynamic_cast<CWorker&>(pcEP->GetControllableEntity().GetController());
        cfController.SetTeamID(1);  // Assign teamID of first team leader
        
        /* Try to place it in the arena */
        bool bDone;     
        cEPPos.Set(0.15f, 0.0f, 0.0f);                                   
        // cEPRot.FromAngleAxis(CRadians(), CVector3::Z);
        cEPRot.FromAngleAxis(m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE),
                                CVector3::Z);
        bDone = MoveEntity(pcEP->GetEmbodiedEntity(), cEPPos, cEPRot);
    } catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("While placing init connector", ex);
    }
}

/****************************************/
/****************************************/

void CExperimentLoopFunctionsNop::PlaceTask(const CVector2& c_center,
                                         Real f_radius,
                                         UInt32 un_demand,
                                         UInt32 un_min_robot_num,
                                         UInt32 un_max_robot_num,
                                         UInt32 un_task_id_start) {

    try {
        CCircleTaskEntity* pcCTS;
        std::ostringstream cTSId;

        /* Make the id */
        cTSId.str("");
        cTSId << "task_" << un_task_id_start;
        /* Create the task and add it to ARGoS space */
        pcCTS = new CCircleTaskEntity(cTSId.str(),
                                      c_center,
                                      f_radius,
                                      un_demand,
                                      un_min_robot_num,
                                      un_max_robot_num);
        AddEntity(*pcCTS);
        m_vecEntityID.push_back(cTSId.str());

    } catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("While placing a task", ex);
    }
}

/****************************************/
/****************************************/

void CExperimentLoopFunctionsNop::PlaceCircleTask(const CVector2& c_center,
                                                  Real f_radius,
                                                  Real f_height,
                                                  UInt32 un_demand,
                                                  UInt32 un_min_robot_num,
                                                  UInt32 un_max_robot_num,
                                                  UInt32 un_task_id_start) {

    try {
        std::ostringstream cTSId;

        /* Make the id */
        cTSId.str("");
        cTSId << "task_" << un_task_id_start;
        /* Create the task and add it to ARGoS space */
        if(m_bNoDemandTasks) {
            CCircleTaskNoDemandEntity* pcCTS;
            pcCTS = new CCircleTaskNoDemandEntity(cTSId.str(),
                                        c_center,
                                        f_radius,
                                        f_height);
            AddEntity(*pcCTS);

        } else {
            CCircleTaskEntity* pcCTS;
            pcCTS = new CCircleTaskEntity(cTSId.str(),
                                        c_center,
                                        f_radius,
                                        f_height,
                                        un_demand,
                                        un_min_robot_num,
                                        un_max_robot_num);
            AddEntity(*pcCTS);
        }

        m_vecEntityID.push_back(cTSId.str());

    } catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("While placing a task", ex);
    }
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CExperimentLoopFunctionsNop, "experiment_loop_functions_nop")
