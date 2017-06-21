#include <fsm_definition.h>

/*BEGIN Ready*/

void myfsm::Ready::react ( const XBot::FSM::Event& e )
{
    std::cout << " myfsm::Ready::react " << std::endl;

}

void myfsm::Ready::entry ( const DummyReady& m )
{

    std::cout << " myfsm::Ready::react " << std::endl;

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
        
        shared_data()._nh = std::make_shared<ros::NodeHandle>();
        
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
    transit("Move", t);
    
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
    m.segment_trj;
}

void myfsm::Move::entry ( const XBot::FSM::Message& msg )
{


}


void myfsm::Move::run ( double time, double period )
{

    // NOTE test!
    sleep(5);
    
    // come back to ready when the trajectory is over
    transit("Ready", DummyReady());

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



