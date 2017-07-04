#include <XBotInterface/StateMachine.h>

#include <ros/node_handle.h>

#include <trajectory_utils/trajectory_utils.h>

#include <ADVR_ROS/advr_cartesian_control.h>
#include <ADVR_ROS/advr_segment_control.h>
#include <ADVR_ROS/advr_grasp_control.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

namespace myfsm
{


class CartesianTrajReceived : public XBot::FSM::Message
{

public:
        CartesianTrajReceived ( ADVR_ROS::advr_cartesian_controlRequest cartesian_trj ) 
        {
            this->cartesian_trj = cartesian_trj;
        }

        ADVR_ROS::advr_cartesian_controlRequest cartesian_trj;
};

class SegmentTrajReceived : public XBot::FSM::Message
{

public:
        SegmentTrajReceived ( ADVR_ROS::advr_segment_controlRequest segment_trj )
        {
            this->segment_trj = segment_trj;
        }

        ADVR_ROS::advr_segment_controlRequest segment_trj;
};

class DummyReady : public XBot::FSM::Message
{

public:
        DummyReady () {}
};


struct SharedData {

        XBot::RobotInterface::Ptr _robot;
        std::shared_ptr<ros::NodeHandle> _nh;

};

class MacroState : public  XBot::FSM::State< MacroState , SharedData >
{

public:

        virtual void entry ( const XBot::FSM::Message& msg ) {};
        virtual void react ( const XBot::FSM::Event& e ) {};
        
        virtual void entry ( const DummyReady& e ) {};
        virtual void entry ( const CartesianTrajReceived& e ) {};
        virtual void entry ( const SegmentTrajReceived& e ) {};

};


class Ready : public MacroState
{

public:
        virtual std::string get_name() const {
                return "Ready";
        }
        virtual void run ( double time, double period );
        
        virtual void entry ( const XBot::FSM::Message& msg );
        virtual void entry ( const DummyReady& msg );
        
        virtual void react ( const XBot::FSM::Event& e );
        
        virtual void exit();
        
        bool callback_cartesian_control(ADVR_ROS::advr_cartesian_controlRequest&  req, 
                                        ADVR_ROS::advr_cartesian_controlResponse& res);
        
        bool callback_segment_control(  ADVR_ROS::advr_segment_controlRequest&  req, 
                                        ADVR_ROS::advr_segment_controlResponse& res);
        
private:
    
        ros::ServiceServer _cartesian_control_srv, 
                           _segment_control_srv;   
        ros::Publisher _feedBack;
        std_msgs::Bool msg;

};

class Move : public MacroState
{

        virtual std::string get_name() const {
                return "Move";
        }
        virtual void run ( double time, double period );
        virtual void entry ( const XBot::FSM::Message& msg );
        
        virtual void entry ( const CartesianTrajReceived& m );
        virtual void entry ( const SegmentTrajReceived& m );
        
        virtual void react ( const XBot::FSM::Event& e );
        virtual void exit();

private:

    std::shared_ptr<trajectory_utils::trajectory_generator> _trj_gen;

    KDL::Frame _F;
    KDL::Twist _v;
    KDL::Twist _a;
    
    ros::Publisher _pub;
    geometry_msgs::PoseStamped _pose;    

};

class HandCmd : public MacroState
{

        virtual std::string get_name() const {
                return "HandCmd";
        }

        virtual void run ( double time, double period );

        virtual void entry ( const XBot::FSM::Message& msg );

        virtual void react ( const XBot::FSM::Event& e );

        virtual void exit ();

private:


};


}
