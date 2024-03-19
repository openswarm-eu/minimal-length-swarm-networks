/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example for QT user functions that override key management.
 *
 * These QT user functions allow the user to select a robot and employ
 * the keys I,J,K,L to control its direction.
 *
 * This controller is meant to be used with the XML file:
 *    experiments/manualcontrol.argos
 */

#ifndef MANUALCONTROL_QTUSER_FUNCTIONS_NOP_H
#define MANUALCONTROL_QTUSER_FUNCTIONS_NOP_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <loop_functions/leaderworker_loop_functions/experiment_loop_functions_nop.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/plugins/robots/e-puck_leader/simulator/epuckleader_entity.h>

#include <argos3/plugins/simulator/entities/circle_task_entity.h>
#include <argos3/plugins/simulator/entities/rectangle_task_entity.h>

#include <utility/robot_message.h>

#include <controllers/leader/leader.h>

using namespace argos;

class CManualControlQTUserFunctionsNop : public CQTOpenGLUserFunctions {

public:

   CManualControlQTUserFunctionsNop();

   virtual ~CManualControlQTUserFunctionsNop() {}

   /**
    * Called when a key press event occurs.
    * The focus must be on the QTOpenGL widget.
    * QTOpenGL reserves the following keys for camera movement:
    * - arrows
    * - Q W E A S D
    * If this function does not manage a keypress, it must end by
    * calling CQTOpenGLWidget::KeyPressed().
    */
   virtual void KeyPressed(QKeyEvent* pc_event);

   /**
    * Called when a key release event occurs.
    * The focus must be on the QTOpenGL widget.
    * QTOpenGL reserves the following keys for camera movement:
    * - arrows
    * - Q W E A S D
    * If this function does not manage a key release, it must end by
    * calling CQTOpenGLWidget::KeyReleased().
    */
   virtual void KeyReleased(QKeyEvent* pc_event);

   /**
    * Called every time an entity is selected.
    * @param c_entity The selected entity.
    */
   virtual void EntitySelected(CEntity& c_entity);

   /**
    * Called every time an entity is deselected.
    * @param c_entity The deselected entity.
    */
   virtual void EntityDeselected(CEntity& c_entity);

   /**
    * Draws the entity ID above the e-puck.
    */
   virtual void Draw(CEPuckEntity& c_entity);

   /**
    * Draws the entity ID above the e-puck_leader.
    */
   virtual void Draw(CEPuckLeaderEntity& c_entity);

   /**
    * Draws objects to the simulation.
    */
   virtual void DrawInWorld();

private:

   /**
    * Sets the robot direction from a key event.
    */
   void SetDirectionFromKeyEvent();

   /**
    * Sets the broadcast signal from a key event.
    */ 
   void SetTaskSignalFromKeyEvent();

   /**
    * Handles the key inputs related to sending robots.
    */ 
   void SetSendCommandFromKeyEvent();

private:

   CExperimentLoopFunctionsNop *m_pcExperimentLoopFunctions;

   /**
    * Mapping of keys to functionality.
    */
   enum EKeyMap {
      DIRECTION_FORWARD = 0,
      DIRECTION_BACKWARD,
      DIRECTION_LEFT,
      DIRECTION_RIGHT,
      TASK_SIGNAL,
      SEND_CONFIRM,
      SEND_CANCEL,
      NUM_0,
      NUM_1,
      NUM_2,
      NUM_3,
      NUM_4,
      NUM_5,
      NUM_6,
      NUM_7,
      NUM_8,
      NUM_9,
      KEY_MINUS
   };

   /**
    * Pointer to the controller of the currently selected entity.
    * NULL if no robot is selected.
    */
   CLeader* m_pcController;

   /**
    * Current state of each key.
    */
   UInt8 m_punPressedKeys[18];

   /**
    * Current state of toggle switch for signal.
    */
   bool m_bSignal;

   /**
    * String to keep track of the send command.
    * Format: teamID + '-' + number of robots to send
    * Example: 1-2 (send 2 robots to team 1)
    */
   std::string m_strSendCommand;

   /**
    * True when we want to print the hop count of each robot
    */
   bool m_bDrawRobotLabel;

};

#endif
