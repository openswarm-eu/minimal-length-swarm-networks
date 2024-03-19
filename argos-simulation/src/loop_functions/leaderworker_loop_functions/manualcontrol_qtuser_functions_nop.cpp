#include "manualcontrol_qtuser_functions_nop.h"
#include <QKeyEvent>
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/simulator/entities/rab_equipped_entity.h>
#include <controllers/worker/worker.h>

/****************************************/
/****************************************/

static const Real DIRECTION_VECTOR_FACTOR = 10.;

/****************************************/
/****************************************/

CManualControlQTUserFunctionsNop::CManualControlQTUserFunctionsNop() :
   m_pcController(NULL),
   m_bSignal(false),
   m_strSendCommand("") {
   /* No key is pressed initially */
   m_punPressedKeys[DIRECTION_FORWARD]  = 0;
   m_punPressedKeys[DIRECTION_BACKWARD] = 0;
   m_punPressedKeys[DIRECTION_LEFT]     = 0;
   m_punPressedKeys[DIRECTION_RIGHT]    = 0;
   m_punPressedKeys[TASK_SIGNAL]        = 0;
   m_punPressedKeys[SEND_CONFIRM]       = 0;
   m_punPressedKeys[SEND_CANCEL]        = 0;
   m_punPressedKeys[NUM_0]              = 0;
   m_punPressedKeys[NUM_1]              = 0;
   m_punPressedKeys[NUM_2]              = 0;
   m_punPressedKeys[NUM_3]              = 0;
   m_punPressedKeys[NUM_4]              = 0;
   m_punPressedKeys[NUM_5]              = 0;
   m_punPressedKeys[NUM_6]              = 0;
   m_punPressedKeys[NUM_7]              = 0;
   m_punPressedKeys[NUM_8]              = 0;
   m_punPressedKeys[NUM_9]              = 0;
   m_punPressedKeys[KEY_MINUS]          = 0;

   m_pcExperimentLoopFunctions = static_cast<CExperimentLoopFunctionsNop *>(
        &CSimulator::GetInstance().GetLoopFunctions());

   m_bDrawRobotLabel = m_pcExperimentLoopFunctions->IsDrawRobotLabel();

   /* Register function to draw entity name */
   RegisterUserFunction<CManualControlQTUserFunctionsNop,CEPuckEntity>(&CManualControlQTUserFunctionsNop::Draw);
   RegisterUserFunction<CManualControlQTUserFunctionsNop,CEPuckLeaderEntity>(&CManualControlQTUserFunctionsNop::Draw);
}

/****************************************/
/****************************************/

void CManualControlQTUserFunctionsNop::KeyPressed(QKeyEvent* pc_event) {
   /* Make sure that a controller was set */
   if(!m_pcController) {
      GetQTOpenGLWidget().KeyPressed(pc_event);
      return;
   }
   switch(pc_event->key()) {
      case Qt::Key_I:
         /* Forwards */
         m_punPressedKeys[DIRECTION_FORWARD] = 1;
         SetDirectionFromKeyEvent();
         break;
      case Qt::Key_K:
         /* Backwards */
         m_punPressedKeys[DIRECTION_BACKWARD] = 1;
         SetDirectionFromKeyEvent();
         break;
      case Qt::Key_J:
         /* Left */
         m_punPressedKeys[DIRECTION_LEFT] = 1;
         SetDirectionFromKeyEvent();
         break;
      case Qt::Key_L:
         /* Right */
         m_punPressedKeys[DIRECTION_RIGHT] = 1;
         SetDirectionFromKeyEvent();
         break;
      case Qt::Key_T:
         /* Toggle task signal */
         m_punPressedKeys[TASK_SIGNAL] = 1;
         SetTaskSignalFromKeyEvent();
         break;
      case Qt::Key_Return:
         /* Confirm a send command */
         m_punPressedKeys[SEND_CONFIRM] = 1;
         SetSendCommandFromKeyEvent();
         break;
      case Qt::Key_Backspace:
         /* Cancel sending a send command (must be before confirming the send command) */
         m_punPressedKeys[SEND_CANCEL] = 1;
         SetSendCommandFromKeyEvent();
         break;
      case Qt::Key_0:
         /* Append the number to the send command */
         m_punPressedKeys[NUM_0] = 1;
         m_strSendCommand += "0";
         break;
      case Qt::Key_1:
         m_punPressedKeys[NUM_1] = 1;
         m_strSendCommand += "1";
         break;
      case Qt::Key_2:
         m_punPressedKeys[NUM_2] = 1;
         m_strSendCommand += "2";
         break;
      case Qt::Key_3:
         m_punPressedKeys[NUM_3] = 1;
         m_strSendCommand += "3";
         break;
      case Qt::Key_4:
         m_punPressedKeys[NUM_4] = 1;
         m_strSendCommand += "4";
         break;
      case Qt::Key_5:
         m_punPressedKeys[NUM_5] = 1;
         m_strSendCommand += "5";
         break;
      case Qt::Key_6:
         m_punPressedKeys[NUM_6] = 1;
         m_strSendCommand += "6";
         break;
      case Qt::Key_7:
         m_punPressedKeys[NUM_7] = 1;
         m_strSendCommand += "7";
         break;
      case Qt::Key_8:
         m_punPressedKeys[NUM_8] = 1;
         m_strSendCommand += "8";
         break;
      case Qt::Key_9:
         m_punPressedKeys[NUM_9] = 1;
         m_strSendCommand += "9";
         break;
      case Qt::Key_Minus:
         m_punPressedKeys[KEY_MINUS] = 1;
         m_strSendCommand += "-";
         break;
      default:
         /* Unknown key */
         GetQTOpenGLWidget().KeyPressed(pc_event);
         break;
   }
}

/****************************************/
/****************************************/

void CManualControlQTUserFunctionsNop::KeyReleased(QKeyEvent* pc_event) {
   /* Make sure that a controller was set */
   if(!m_pcController) {
      GetQTOpenGLWidget().KeyReleased(pc_event);
      return;
   }
   switch(pc_event->key()) {
      case Qt::Key_I:
         /* Forwards */
         m_punPressedKeys[DIRECTION_FORWARD] = 0;
         SetDirectionFromKeyEvent();
         break;
      case Qt::Key_K:
         /* Backwards */
         m_punPressedKeys[DIRECTION_BACKWARD] = 0;
         SetDirectionFromKeyEvent();
         break;
      case Qt::Key_J:
         /* Left */
         m_punPressedKeys[DIRECTION_LEFT] = 0;
         SetDirectionFromKeyEvent();
         break;
      case Qt::Key_L:
         /* Right */
         m_punPressedKeys[DIRECTION_RIGHT] = 0;
         SetDirectionFromKeyEvent();
         break;
      case Qt::Key_Return:
         m_punPressedKeys[SEND_CONFIRM] = 0;
         break;
      case Qt::Key_Backspace:
         m_punPressedKeys[SEND_CANCEL] = 0;
         break;
      default:
         /* Unknown key */
         GetQTOpenGLWidget().KeyReleased(pc_event);
         break;
   }
}

/****************************************/
/****************************************/

void CManualControlQTUserFunctionsNop::EntitySelected(CEntity& c_entity) {
   /* Make sure the entity is an e-puck */
   CEPuckLeaderEntity* pcFB = dynamic_cast<CEPuckLeaderEntity*>(&c_entity);
   if(!pcFB) return;
   /* It's an e-puck_leader; extract its controller */
   m_pcController = dynamic_cast<CLeader*>(&pcFB->GetControllableEntity().GetController());
   /* Tell that e-puck that it is selected */
   m_pcController->Select();
   /* Reset key press information */
   m_punPressedKeys[DIRECTION_FORWARD]  = 0;
   m_punPressedKeys[DIRECTION_BACKWARD] = 0;
   m_punPressedKeys[DIRECTION_LEFT]     = 0;
   m_punPressedKeys[DIRECTION_RIGHT]    = 0;
   m_punPressedKeys[TASK_SIGNAL]        = 0;
   m_punPressedKeys[SEND_CONFIRM]       = 0;
   m_punPressedKeys[SEND_CANCEL]        = 0;
   m_punPressedKeys[NUM_0]              = 0;
   m_punPressedKeys[NUM_1]              = 0;
   m_punPressedKeys[NUM_2]              = 0;
   m_punPressedKeys[NUM_3]              = 0;
   m_punPressedKeys[NUM_4]              = 0;
   m_punPressedKeys[NUM_5]              = 0;
   m_punPressedKeys[NUM_6]              = 0;
   m_punPressedKeys[NUM_7]              = 0;
   m_punPressedKeys[NUM_8]              = 0;
   m_punPressedKeys[NUM_9]              = 0;
   m_punPressedKeys[KEY_MINUS]          = 0;
}

/****************************************/
/****************************************/

void CManualControlQTUserFunctionsNop::EntityDeselected(CEntity& c_entity) {
   /* Make sure that a controller was set (should always be true...) */
   if(!m_pcController) return;
   /* Tell the e-puck that it is deselected */
   m_pcController->Deselect();
   /* Forget the controller */
   m_pcController = NULL;
}

/****************************************/
/****************************************/

void CManualControlQTUserFunctionsNop::SetDirectionFromKeyEvent() {
   /* Forward/backward direction factor (local robot X axis) */
   SInt32 FBDirection = 0;
   /* Left/right direction factor (local robot Y axis) */
   SInt32 LRDirection = 0;
   /* Calculate direction factor */
   if(m_punPressedKeys[DIRECTION_FORWARD])  ++FBDirection;
   if(m_punPressedKeys[DIRECTION_BACKWARD]) --FBDirection;
   if(m_punPressedKeys[DIRECTION_LEFT])     ++LRDirection;
   if(m_punPressedKeys[DIRECTION_RIGHT])    --LRDirection;
   /* Calculate direction */
   CVector2 cDir =
      DIRECTION_VECTOR_FACTOR *
      (CVector2(FBDirection, 0.0f) +
       CVector2(0.0f, LRDirection));
   /* Set direction */
   m_pcController->SetControlVector(cDir);
}

/****************************************/
/****************************************/

void CManualControlQTUserFunctionsNop::SetTaskSignalFromKeyEvent() {
   m_bSignal = !m_bSignal;
   m_pcController->SetSignal(m_bSignal);
}

/****************************************/
/****************************************/

void CManualControlQTUserFunctionsNop::SetSendCommandFromKeyEvent() {
   
   // If cancel pressed, reset string

   // If confirm pressed, 
      // Read the team and number of robots from string, separated by a minus sign '-'
      // send team and robot num to team
      // reset string

   if(m_punPressedKeys[SEND_CANCEL]) {
      std::cerr << "CANCEL PRESSED" << std::endl;
   }

   if(m_punPressedKeys[SEND_CONFIRM]) {
      std::cerr << "CONFIRM PRESSED" << std::endl;
      if( !m_strSendCommand.empty() ) {

         // Based on https://www.geeksforgeeks.org/how-to-split-a-string-in-cc-python-and-java/
         // A quick way to split strings separated via any character delimiter.
         std::stringstream ss(m_strSendCommand);
         char del = '-';
         std::string word;
         std::vector<std::string> list;
         while (!ss.eof()) {
            getline(ss, word, del);
            list.push_back(word);
         }
         
         UInt8 teamID;
         UInt8 numRobots;
         bool validTeamID = false;
         bool validNumRobot = false;

         try{
            teamID = stoi(list[0]);
            validTeamID = true;
         } catch(std::invalid_argument &e) {
            // std::cerr << e.what();
            std::cerr << "Team ID has to be 0~255" << std::endl;
         }

         if(list.size() > 1) {
            try{
               numRobots = stoi(list[1]);

               validNumRobot = true;
            } catch(std::invalid_argument &e) {
               // std::cerr << e.what();
               std::cerr << "Number of robots has to be 0~255" << std::endl;
            }
         } else {
            std::cerr << "Missing number of robots to send" << std::endl;
         }

         if(validTeamID && validNumRobot) {
            std::cerr << "INFORM LEADER" << std::endl;
            std::cerr << " to team: " << teamID << ", num: " << numRobots << std::endl;
            /* Inform send command to the leader */
            m_pcController->SetRobotsToSend(numRobots, teamID);
         }
      }
   }

   /* Reset send command */
   m_strSendCommand = "";
}

/****************************************/
/****************************************/

void CManualControlQTUserFunctionsNop::Draw(CEPuckEntity& c_entity) {
   /* The position of the text is expressed wrt the reference point of the e-puck
    * See also the description in
    * $ argos3 -q e-puck
    */

   if(m_bDrawRobotLabel) {
      
      try {
         CWorker& cController = dynamic_cast<CWorker&>(c_entity.GetControllableEntity().GetController());

         std::string text = c_entity.GetId().c_str();
         // std::string text = "   ";

         /* Draw the hop count to each team */
         // if(cController.GetRobotState() == RobotState::CONNECTOR) {
         //    std::map<UInt8,HopMsg> hops = cController.GetHops();

         //    for(const auto& it : hops) {
         //       text += "(T" + std::to_string(it.first);
         //       if( !it.second.ID.empty() ) {
         //          text += "-" + it.second.ID;
         //       } else {
         //          text += "-__";
         //       }
         //       text += "-" + std::to_string(it.second.count);

         //       if(cController.GetRobotState() == RobotState::FOLLOWER) {
         //          text += "-" + std::to_string(it.second.resendCount) + ")";
         //       } else {
         //          text += ")";
         //       }
         //    }
         // }

         CColor cColor = CColor(0, 0, 0, 255); // BLACK

         QFont workerFont("Helvetica [Cronyx]", 10);
         // QFont workerFont("Helvetica [Cronyx]", 15, QFont::Bold);
         DrawText(CVector3(0.0, 0.0, 0.2),   // position
                  text,
                  cColor,
                  workerFont); // text
      } catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("While casting robot as a worker", ex);
      } catch(const std::bad_cast& e) {
         std::cout << e.what() << " in Draw" << '\n';
      }
   }
}

/****************************************/
/****************************************/

void CManualControlQTUserFunctionsNop::Draw(CEPuckLeaderEntity& c_entity) {
   /* The position of the text is expressed wrt the reference point of the e-puck_leader
    * See also the description in
    * $ argos3 -q e-puck_leader
    */
   // QFont leaderFont("Helvetica [Cronyx]", 20, QFont::Bold);
   // DrawText(CVector3(0.0, 0.0, 0.2),   // position
   //          c_entity.GetId().c_str(),
   //          CColor::BLACK,
   //          leaderFont); // text

   if(m_bDrawRobotLabel) {

      /* Show hop count like worker */
      try {
         CLeader& cController = dynamic_cast<CLeader&>(c_entity.GetControllableEntity().GetController());

         std::string text = c_entity.GetId().c_str();

         /* Draw the hop count to each team */
         // std::map<UInt8,HopMsg> hops = cController.GetHops();

         // for(const auto& it : hops) {
         //    text += "(T" + std::to_string(it.first);
         //    if( !it.second.ID.empty() ) {
         //       text += "-" + it.second.ID;
         //    } else {
         //       text += "-__";
         //    }
         //    text += "-" + std::to_string(it.second.count);
         //    text += "-" + std::to_string(it.second.resendCount) + ")";
         // }

         // DrawText(CVector3(0.0, 0.0, 0.2),   // position
         //          text); // text
      } catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("While casting robot as a leader", ex);
      } catch(const std::bad_cast& e) {
         std::cout << e.what() << " in Draw" << '\n';
      }
   }
}

/****************************************/
/****************************************/

void CManualControlQTUserFunctionsNop::DrawInWorld() {

   /*
   * Draw arena boundary
   */
   DrawCircle(CVector3(0, 0, 0.001),
               CQuaternion(),
               m_pcExperimentLoopFunctions->GetArenaRadius(),
               CColor::BLACK,
               false,
               40U);

   /* Draw deployment area */
   // DrawCircle(CVector3(0, 0, 0.001),
   //             CQuaternion(),
   //             m_pcExperimentLoopFunctions->GetDeploymentRadius(),
   //             CColor::BLACK,
   //             false,
   //             40U);

   // /* Draw arena boundary (where task will appear) */
   // std::vector<CVector2> points = m_pcExperimentLoopFunctions->GetArenaSize();
   // DrawPolygon(CVector3(0, 0, 0.001),
   //             CQuaternion(),
   //             points,
   //             CColor::BLACK,
   //             false);

   /* 
   * Draw Circle Tasks 
   */

   /* Get all the tasks */
   CSpace::TMapPerType* cTasks;
   bool b_tasks_exists = false;
   std::string strTaskType = m_pcExperimentLoopFunctions->GetTaskType();

   try {
      if(strTaskType == "circle_task_no_demand")
         cTasks = &CSimulator::GetInstance().GetSpace().GetEntitiesByType("circle_task_no_demand");
      else if(strTaskType == "circle_task")
         cTasks = &CSimulator::GetInstance().GetSpace().GetEntitiesByType("circle_task");
      // cTasks = &CSimulator::GetInstance().GetSpace().GetEntitiesByType("rectangle_task");
      b_tasks_exists = true;
   } catch(CARGoSException& ex) {
      std::cout << "No task found in argos file (DrawInWorld)" << std::endl;
   }

   if( b_tasks_exists ) {
      for(CSpace::TMapPerType::iterator it = cTasks->begin();
       it != cTasks->end();
       ++it) {

         /* Draw circle task */
         CVector2 pos;
         UInt32 demand;
         Real radius;

         /* Get handle to the circle task entity */
         if(strTaskType == "circle_task_no_demand") {
            CCircleTaskNoDemandEntity& cTask = *any_cast<CCircleTaskNoDemandEntity*>(it->second);
            pos = cTask.GetPosition();
            demand = cTask.GetWorkPerformed();
            radius = cTask.GetRadius();
         } else if(strTaskType == "circle_task") {
            CCircleTaskEntity& cTask = *any_cast<CCircleTaskEntity*>(it->second);
            pos = cTask.GetPosition();
            demand = cTask.GetDemand();
            radius = cTask.GetRadius();
         } else {
            LOGERR << "Unknown task type: " << strTaskType << std::endl;
         }
         // CCircleTaskEntity& cTask = *any_cast<CCircleTaskEntity*>(it->second);
         // CRectangleTaskEntity& cTask = *any_cast<CRectangleTaskEntity*>(it->second);



         // std::cout << cTask.GetWidth() << " " << cTask.GetHeight() << std::endl;

         // std::vector<CVector2> points = { CVector2(-0.5*cTask.GetWidthX(),0.5*cTask.GetWidthY()),
         //                                  CVector2(0.5*cTask.GetWidthX(),0.5*cTask.GetWidthY()),
         //                                  CVector2(0.5*cTask.GetWidthX(),-0.5*cTask.GetWidthY()),
         //                                  CVector2(-0.5*cTask.GetWidthX(),-0.5*cTask.GetWidthY())};

         // for(const auto& pt : points) {
         //    std::cout << pt << std::endl;
         // }

         // if(demand > 0) {
         //    DrawCircle(CVector3(pos.GetX(), pos.GetY(), 0.001),
         //             CQuaternion(),
         //             cTask.GetRadius(),
         //             CColor(255U, 128U, 128U, 255U),
         //             false,
         //             40U);
         //    // DrawPolygon(CVector3(pos.GetX(), pos.GetY(), 0.001),
         //    //             CQuaternion(),
         //    //             points,
         //    //             CColor(255U, 128U, 128U, 255U));
         // } else {
         //    DrawCircle(CVector3(pos.GetX(), pos.GetY(), 0.001),
         //             CQuaternion(),
         //             cTask.GetRadius(),
         //             CColor(128U, 255U, 128U, 255U),
         //             false,
         //             40U);
         //    // std::vector<CVector2> points = {CVector2(-0.5,0.5),CVector2(0.5,0.5),CVector2(0.5,-0.5),CVector2(-0.5,-0.5)};
         //    // DrawPolygon(CVector3(pos.GetX(), pos.GetY(), 0.001),
         //    //             CQuaternion(),
         //    //             points,
         //    //             CColor(128U, 255U, 128U, 255U));
         // }
         
         /* Draw task info */
         std::ostringstream cText;
         cText.str("");
         // cText << ceil(cTask.GetDemand() / 10);
         cText << demand;
         QFont taskFont("Helvetica [Cronyx]", 15, QFont::Bold);
         // DrawText(CVector3(pos.GetX(), pos.GetY()+cTask.GetRadius()/2, 0.01),
         //          cText.str(),
         //          CColor::BLACK,
         //          taskFont);
         DrawText(CVector3(pos.GetX(), pos.GetY() - radius, 0.01),
                  cText.str(),
                  CColor::BLACK,
                  taskFont);

         // cText.str("");
         // cText << cTask.GetCurrentRobotNum() << " / " << cTask.GetMinRobotNum();
         // // LOGERR << "draw " << cTask.GetId() << ", " << cTask.GetCurrentRobotNum() << std::endl;
         // QFont numFont("Helvetica [Cronyx]", 20);
         // // DrawText(CVector3(pos.GetX()-0.3, pos.GetY()-0.25, 0.01),
         // DrawText(CVector3(pos.GetX()-0.3, pos.GetY()+0.38, 0.01),
         //          cText.str(),
         //          CColor::BLACK,
         //          numFont);
      }
   }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CManualControlQTUserFunctionsNop, "manualcontrol_qtuser_functions_nop")
