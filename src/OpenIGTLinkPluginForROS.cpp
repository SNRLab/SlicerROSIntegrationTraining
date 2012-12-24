/* 
 * Atsushi Yamada,PhD
 * Research Associate, Surgical Navigation and Robotics Laboratory (www.snr.spl.harvard.edu) 
 * Brigham and Women's Hospital and Harvard Medical School
*/

#include <boost/bind.hpp>
#include "physics/physics.h"
#include "transport/Node.hh"
#include "transport/TransportTypes.hh"
#include "msgs/MessageTypes.hh"
#include "common/Time.hh"
#include "common/Plugin.hh"
#include "common/Events.hh"
#include "common/PID.hh"

// for own thread
#include "ros/callback_queue.h"
#include <boost/thread.hpp>
#include "boost/thread/mutex.hpp"

// for own OpenIGTLink
#include "igtlOSUtil.h"
#include "igtlMessageHeader.h"
#include "igtlTransformMessage.h"
#include "igtlServerSocket.h"
#include "igtlStatusMessage.h"

#include <iostream>
#include <math.h>
#include <cstdlib>
#include <cstring>

namespace gazebo
{
  class MoveModel : public ModelPlugin
  {
    public: void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
    {
      // Get the world name.
      this->world = _parent->GetWorld();

      // Get a pointer to the model
      this->model = _parent;

      this->node = transport::NodePtr(new transport::Node());
      this->node->Init(this->world->GetName());
      this->statsSub = this->node->Subscribe("~/world_stats", &MoveModel::OnStats, this);
      
      // initialize a PID class
      for(int i=0; i<40; i++)
      {
      this->target_position_[i] = 0.0;
      this->pid[i].Init(10, 0, 2, 0, 0, 100, -100);
      this->pid[i].SetCmd(this->target_position_[i]);
      this->pid1[i].Init(10, 0, 2, 0, 0, 100, -100);
      this->pid1[i].SetCmd(this->target_position_[i]);
      }

      this->last_update_time_ = this->model->GetWorld()->GetSimTime();

      // Load parameters for this plugin
      if (this->LoadParams(_sdf))
      {
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateStart(
            boost::bind(&MoveModel::OnUpdate, this));
      }

     // start custom queue for joint trajectory plugin ros topics
     this->callback_queue_thread_ = boost::thread( boost::bind( &MoveModel::OpenIGTLinkReceiverThread,this ) );

    }

    public: bool LoadParams(sdf::ElementPtr _sdf) 
    {
      if (
          this->FindJointByParam(_sdf, this->controlled_joint_[0],
                             "joint1") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[1],
                             "joint2") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[2],
                             "joint3")    
          )
        return true;
      else
        return false;
    }

    public: bool FindJointByParam(sdf::ElementPtr _sdf,
                                  physics::JointPtr &_joint,
                                  std::string _param)
    {
      if (!_sdf->HasElement(_param))
      {
        gzerr << "param [" << _param << "] not found\n";
        return false;
      }
      else
      {
        _joint = this->model->GetJoint(
          _sdf->GetElement(_param)->GetValueString());

        if (!_joint)
        {
          gzerr << "joint by name ["
                << _sdf->GetElement(_param)->GetValueString()
                << "] not found in model\n";
          return false;
        }
      }
      return true;
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // mutex lock
      boost::mutex::scoped_lock lock(this->update_mutex);

      // PID controller
      //this->target_position_[0] = 3.14*10.0/180.0;
      //this->target_position_[1] = 3.14*20.0/180.0;

      common::Time current_time = this->model->GetWorld()->GetSimTime();
      double dt    = current_time.Double()
                   - this->last_update_time_.Double();
      
      for(int i = 0; i < 3; i++)
      {
      this->pid[i].SetCmd(this->target_position_[0]);
      this->pid1[i].SetCmd(this->target_position_[1]);

      this->error[i] = this->controlled_joint_[i]->GetAngle(0).GetAsRadian()
                   - target_position_[i];
      this->pid[i].Update(this->error[i], dt);

      this->error[i] = this->controlled_joint_[i]->GetAngle(0).GetAsRadian()
                   - target_position_[i];
      this->pid1[i].Update(this->error[i], dt);
      this->controlled_joint_[i]->SetForce(0, this->pid[i].GetCmd()+this->pid1[i].GetCmd());
      }
      
      this->last_update_time_ = current_time;

    }

    public: int ReceiveTransform(igtl::Socket * socket, igtl::MessageHeader * header)
    {
      int result = 0;
      //std::cerr << "Receiving TRANSFORM data type." << std::endl;
      // Create a message buffer to receive transform data
      igtl::TransformMessage::Pointer transMsg;
      transMsg = igtl::TransformMessage::New();
      transMsg->SetMessageHeader(header);
      transMsg->AllocatePack();
      
      // Receive transform data from the socket
      socket->Receive(transMsg->GetPackBodyPointer(), transMsg->GetPackBodySize());
      
      // Deserialize the transform data
      // If you want to skip CRC check, call Unpack() without argument.
      int c = transMsg->Unpack(1);
      
      if (c & igtl::MessageHeader::UNPACK_BODY){ // if CRC check is OK
        // Retrive the transform data
        igtl::Matrix4x4 matrix;
        transMsg->GetMatrix(matrix);
        //igtl::PrintMatrix(matrix);

        this->target_position_[0] = matrix[0][3];
        this->target_position_[1] = matrix[1][3];
        this->target_position_[2] = matrix[2][3];
        
        result = 1;
      }
      return result;
    }

    public: void OnStats( const boost::shared_ptr<msgs::WorldStatistics const> &_msg)
    {
      this->simTime  = msgs::Convert( _msg->sim_time() );

    }

    // own thread
    private: void OpenIGTLinkReceiverThread()
    {
      int    port     = 18944;//atoi(argv[1]);
      std::cout << "Input socket port:";
      std::cin >> port;

      igtl::ServerSocket::Pointer serverSocket;
      serverSocket = igtl::ServerSocket::New();
      int r = serverSocket->CreateServer(port);

      if (r < 0){
        std::cerr << "Cannot create a server socket." << std::endl;
        exit(0);
      }
      std::cout << "Create server socket. \nPlease be connected to port:" << port <<std::endl;

      igtl::Socket::Pointer socket;

      while (1){
        //------------------------------------------------------------
        // Waiting for Connection
        socket = serverSocket->WaitForConnection(1000);
        
        if (socket.IsNotNull()){ // if client connected
          // Create a message buffer to receive header
          igtl::MessageHeader::Pointer headerMsg;
          headerMsg = igtl::MessageHeader::New();

          //------------------------------------------------------------
          // Allocate a time stamp 
          igtl::TimeStamp::Pointer ts;
          ts = igtl::TimeStamp::New();

          //------------------------------------------------------------
          // loop
          for (int i = 0; i < 100; i ++){

            // Initialize receive buffer
            headerMsg->InitPack();

            // Receive generic header from the socket
            int r = socket->Receive(headerMsg->GetPackPointer(), headerMsg->GetPackSize());
            if (r == 0){
              socket->CloseSocket();
            }
            if (r != headerMsg->GetPackSize()){
              continue;
            }

            
            // Deserialize the header
            headerMsg->Unpack();

            // Get time stamp
            igtlUint32 sec;
            igtlUint32 nanosec;
            
            headerMsg->GetTimeStamp(ts);
            ts->GetTimeStamp(&sec, &nanosec);

            /*
            std::cerr << "Time stamp: "
                      << sec << "." << std::setw(9) << std::setfill('0') 
                      << nanosec << std::endl;
            */

            // Check data type and receive data body
            if (strcmp(headerMsg->GetDeviceType(), "TRANSFORM") == 0){
              // mutex lock
              boost::mutex::scoped_lock lock(this->update_mutex);
              ReceiveTransform(socket, headerMsg);
            }
          }
        }
      }
      //------------------------------------------------------------
      // Close connection (The example code never reaches to this section ...)
      
      socket->CloseSocket();
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // subscribe to world stats
    private: transport::NodePtr node;
    private: transport::SubscriberPtr statsSub;
    private: common::Time simTime;
    private: physics::WorldPtr world;

    private: physics::JointPtr controlled_joint_[40];

    // for own thread
    private: boost::mutex update_mutex;
    private: ros::CallbackQueue queue_;
    private: boost::thread callback_queue_thread_;

    int communicationFlag;
    int testCount;

    // PID control
    int ccount;
    int controlFlag;

    common::PID pid[40];
    common::PID pid1[40];

    double target_position_[40];
    double error[40];

    physics::JointPtr joint_;
    event::ConnectionPtr update_connection_;
    common::Time last_update_time_;

  // Constructer
  public: MoveModel(){}

  // Destructer
  public: virtual ~MoveModel()
  {
  this->queue_.clear();
  this->queue_.disable();
  this->callback_queue_thread_.join();

  }


  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MoveModel)
}
// push test comment
