#include <fsm_definition.h>
#include <tf_conversions/tf_kdl.h>


/*BEGIN Ready*/

void myfsm::Ready::react ( const XBot::FSM::Event& e )
{
    std::cout << " myfsm::Ready::react " << std::endl;

}

void myfsm::Ready::entry ( const DummyReady& m )
{

    std::cout << " myfsm::Ready::react " << std::endl;
    msg.data = true;
    _feedBack.publish(msg);

}

void myfsm::Ready::entry ( const XBot::FSM::Message& msg )
{

        std::cout << " myfsm::Ready::entry " << std::endl;
        
        // ROS handle
        int argc = 1;
        const char *arg = "ManipulationPlugin";
        char* argg = const_cast<char*>(arg);
        char** argv = &argg;

        if(!ros::isInitialized()){
            ros::init(argc, argv, "ManipulationPlugin");
        }
        
        ros::NodeHandle* node_handle = new ros::NodeHandle;
        shared_data()._nh =  std::shared_ptr<ros::NodeHandle>(node_handle);
        
        _cartesian_control_srv = shared_data()._nh->advertiseService<ADVR_ROS::advr_cartesian_controlRequest, ADVR_ROS::advr_cartesian_controlResponse>
        ( "/cartesian_control",
          boost::bind ( &Ready::callback_cartesian_control,
                        this,
                        _1, _2 )
        );
        
        _segment_control_srv = shared_data()._nh->advertiseService<ADVR_ROS::advr_segment_controlRequest, ADVR_ROS::advr_segment_controlResponse>
        ( "/segment_control",
          boost::bind ( &Ready::callback_segment_control,
                        this,
                        _1, _2 )
        );
       
	_feedBack = shared_data()._nh->advertise<std_msgs::Bool>("Manipulation_status",1);

}


bool myfsm::Ready::callback_cartesian_control ( ADVR_ROS::advr_cartesian_controlRequest&  req,
                                                ADVR_ROS::advr_cartesian_controlResponse& res )
{
    std::cout << " callback_cartesian_control called!" << std::endl;
    
    // save the requested traj
    CartesianTrajReceived t(req);
    
    // success
    res.success = 1;
    
    // go to Move state
//     transit("Move", t);
    
    // NOTE cartesian_control not yet implemented
    std::cout << "callback_cartesian_control NOT YET IMPLEMENTED" << std::endl;
    
    return true;
}

bool myfsm::Ready::callback_segment_control (   ADVR_ROS::advr_segment_controlRequest&  req,
                                                ADVR_ROS::advr_segment_controlResponse& res )
{
    std::cout << " callback_segment_control called!" << std::endl;
    
    // save the requested traj
    SegmentTrajReceived t(req);
    
    // success
    res.success = 1;
    
    // go to Move state
    transit("Move", t);
    
    return true;
    
}


void myfsm::Ready::run ( double time, double period )
{
    
    msg.data = false;
    _feedBack.publish(msg);
    ros::spinOnce();
}

void myfsm::Ready::exit ()
{



}

/*END Ready*/


/*BEGIN Move*/

void myfsm::Move::react ( const XBot::FSM::Event& e )
{

}

void myfsm::Move::entry ( const CartesianTrajReceived& m ) 
{
    std::cout << "myfsm::Move::entry CartesianTrajReceived" << std::endl;
    m.cartesian_trj;
}


void myfsm::Move::entry ( const SegmentTrajReceived& m ) 
{
    std::cout << "myfsm::Move::entry SegmentTrajReceived" << std::endl;
    
    std::vector<trajectory_utils::segment> segments =  m.segment_trj.segment_trj.segments;

    _trj_gen.reset(new trajectory_utils::trajectory_generator(
                           0.005,
                           m.segment_trj.segment_trj.header.frame_id, 
                           segments.at(0).start.distal_frame)
                           );
    
    for(unsigned int i = 0; i < segments.size(); ++i) {
        trajectory_utils::segment trj = segments[i];

        KDL::Frame start;
        KDL::Frame end;

        tf::poseMsgToKDL(trj.start.frame.pose, start);
        tf::poseMsgToKDL(trj.end.frame.pose, end);
        
        if(trj.type.data == 0){

            _trj_gen->addMinJerkTrj(start, end, trj.T.data);
        }
        else if(trj.type.data == 1){
            // NOTE TBD implement it
//             _trj_gen->addArcTrj(trj.start, trj.end_rot, trj.angle_rot,
//                                 trj.circle_center, trj.plane_normal, trj.T.data);
        }
    }
    
    if( _trj_gen->getDistalFrame() == "LSoftHand" ) {
        _pub = shared_data()._nh->advertise<geometry_msgs::PoseStamped>("w_T_left_ee", 1);
    }
    else if( _trj_gen->getDistalFrame() == "RSoftHand" ) {
        _pub = shared_data()._nh->advertise<geometry_msgs::PoseStamped>("w_T_right_ee", 1);
    }


}

void myfsm::Move::entry ( const XBot::FSM::Message& msg )
{
}


void myfsm::Move::run ( double time, double period )
{

    if (!_trj_gen->isFinished()) {
        // get the next pose
        _F = _trj_gen->Pos();
        _v = _trj_gen->Vel();
        _a = _trj_gen->Acc();
        
        // fill the ROS pose stamped
        tf::poseKDLToMsg(_F, _pose.pose);
        _pose.header.stamp = ros::Time::now();
        
        // publish on ROS topic
        _pub.publish(_pose);
        
        // update trajectory
        _trj_gen->updateTrj();
    }
    else {
    	
        // come back to ready when the trajectory is over
        transit("Ready", DummyReady());
    }

}

void myfsm::Move::exit ()
{



}

/*END Move*/


/*BEGIN HandCmd*/

void myfsm::HandCmd::react ( const XBot::FSM::Event& e )
{



}

void myfsm::HandCmd::entry ( const XBot::FSM::Message& msg )
{



}


void myfsm::HandCmd::run ( double time, double period )
{



}

void myfsm::HandCmd::exit ()
{



}

/*END HandCmd*/



