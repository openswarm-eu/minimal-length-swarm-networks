#ifndef EXPERIMENT_LOOP_FUNCTIONS_NOP_H
#define EXPERIMENT_LOOP_FUNCTIONS_NOP_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>

#include <unordered_map>
#include <unordered_set>

using namespace argos;

class CExperimentLoopFunctionsNop : public CLoopFunctions {

public:

   CExperimentLoopFunctionsNop();
   virtual ~CExperimentLoopFunctionsNop() {}

   virtual void Init(TConfigurationNode& t_tree);
   virtual void Reset();
   virtual void Destroy();
   virtual CColor GetFloorColor(const CVector2& c_position_on_plane);
   virtual void PreStep();
   virtual void PostStep();
   virtual Real GetArenaRadius() const;
   virtual Real GetDeploymentRadius() const;
   // virtual std::vector<CVector2> GetArenaSize() const;
   virtual bool IsDrawRobotLabel() const;
   virtual bool IsLogging();
   virtual std::string GetCommandFilePath();
   virtual std::unordered_map<std::string, UInt32> GetRobotPerTask();
   virtual UInt32 GetCurrentPoints();
   virtual std::string GetWorkerType() const;
   virtual std::string GetTaskType() const;

private:

   TConfigurationNode config;
   std::vector<std::string> m_vecEntityID;

   std::vector<CVector2> m_vecWaypointPos;
   CFloorEntity* m_pcFloor;
   CRandom::CRNG* m_pcRNG;
   UInt32 m_unTeams;
   UInt32 m_unWorkerPerTeam;
   UInt32 m_unTotalLeaders;
   UInt32 m_unTotalWorkers;
   Real m_fArenaRadius;
   Real m_fDeploymentRadius;
   Real m_fTeamDeployRadius;
   // std::vector<CVector2> m_vecArenaSize;
   bool m_bTaskExists;
   bool m_bTaskComplete;
   UInt32 m_unNextTaskId;
   UInt32 m_unTotalTasks;
   UInt32 m_unTaskDemand;
   UInt32 m_unPointsObtained;

   std::string m_strWorkerType;

   std::vector<std::string> m_vecIDToRemove;
   UInt32 m_unMaxPerTeam;

   // std::vector<std::unordered_map<std::string,UInt32>> m_vecTaskDemand;
   std::map<UInt32, std::pair<CVector2, Real>> m_mapTaskPos; // position & radius
   /* Number of robots working on each task in the current timestep */
   std::unordered_map<std::string,UInt32> m_mapRobotPerTask;
   /* Map of whether a robot is performing a task in the current timestep */
   std::unordered_map<std::string,bool> m_mapRobotTaskStatus;

   CRange<Real> cArenaSideX = CRange<Real>(-1.45f, 1.45f);
   CRange<Real> cArenaSideY = CRange<Real>(-1.45f, 1.45f);

   /* Output file */
   bool m_bLogging;
   std::string m_strOutput;
   std::string m_strRunNumber;
   std::string m_strDirPath;
   std::string m_strBinaryFilePath;
   std::string m_strSummaryFilePath;
   std::string m_strCommandFilePath;
   std::fstream m_cOutput;

   /* Draw configurations */
   bool m_bDrawRobotLabel;

   /* Frame Grabbing */
   bool m_bFrameGrabbing;
   UInt32 m_unCameraIndex;

   /* robots that have collided */
   std::unordered_set<std::string> robotsCollided;
   UInt32 finishDelay;

   /* Init logging */
   void InitLogging();

   /* Init robots */
   void InitRobots();

   /* Init tasks */
   void InitTasks(); // Testing
   void InitTasksCircular(); // Scalability analysis experiment

   /* Assign tasks */
   void AssignTasks();

   /* Check whether there is at least one follower in each team that can detect the initial connector. Move one member otherwise */
   void CheckInitialConnectorInRange();

   /* Distribute a leader-robot team */
   void PlaceCluster(const CVector2& c_center,
                     UInt32 un_leaders,
                     UInt32 un_robots,
                     Real f_density,
                     std::string str_worker_type,
                     UInt32 un_leader_id_start,
                     UInt32 un_robot_id_start);

   void PlaceCustomPosition(const CVector2& c_center,
                            std::string str_type,
                            UInt32 un_leader_id_start,
                            UInt32 un_robot_id_start);

   void PlaceLeader(const CVector2& c_center,
                    UInt32 un_leader_id_start);

   void PlaceLeaderAndInitConnector(const CVector2& c_center,
                                    Real f_density,
                                    std::string str_worker_type,
                                    UInt32 un_leader_id_start,
                                    UInt32 un_robot_id_start);

   /* Place a task */
   void PlaceTask(const CVector2& c_center,
                  Real f_radius,
                  UInt32 un_demand,
                  UInt32 un_min_robot_num,
                  UInt32 un_max_robot_num,
                  UInt32 un_task_id_start);

   /* Place a task */
   void PlaceCircleTask(const CVector2& c_center,
                        Real f_radius,
                        Real f_height,
                        UInt32 un_demand,
                        UInt32 un_min_robot_num,
                        UInt32 un_max_robot_num,
                        UInt32 un_task_id_start);
};

#endif
