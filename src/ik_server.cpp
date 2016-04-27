/********************************************************************************
Copyright (c) 2016, TRACLabs, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, 
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software 
       without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>

class BaxterTracIKServer {
private:
    std::string _side;
    ros::NodeHandle _nh;
    double _timeout;
    std::string _urdf_param;
    TRAC_IK::TRAC_IK *_tracik_solver;
    KDL::Chain _chain;
    KDL::JntArray *_nominal;
    bool _ready;

public:
    BaxterTracIKServer(ros::NodeHandle& nh, std::string side, double timeout, std::string urdf_param) {
        double eps = 1e-5;
        this->_ready = false;
        this->_side = side;
        this->_nh = nh;
        this->_urdf_param = urdf_param;
        this->_timeout = timeout;
        this->_tracik_solver = new TRAC_IK::TRAC_IK("base", side + "_gripper", urdf_param, timeout, eps);

        KDL::JntArray ll, ul; //lower joint limits, upper joint limits

        if(!(this->_tracik_solver->getKDLChain(this->_chain))) {
          ROS_ERROR("There was no valid KDL chain found");
          return;
        }

        if(!(this->_tracik_solver->getKDLLimits(ll,ul))) {
          ROS_ERROR("There were no valid KDL joint limits found");
          return;
        }

        if(!(this->_chain.getNrOfJoints() == ll.data.size())
           || !(this->_chain.getNrOfJoints() == ul.data.size())) {
            ROS_ERROR("Inconsistent joint limits found");
            return;
        }

        // Create Nominal chain configuration midway between all joint limits
        this->_nominal = new KDL::JntArray(this->_chain.getNrOfJoints());

        for (uint j=0; j < this->_nominal->data.size(); j++) {
          this->_nominal->operator()(j) = (ll(j)+ul(j))/2.0;
        }

        this->_ready = true;
    }

    ~BaxterTracIKServer() {
        delete this->_tracik_solver;
        delete this->_nominal;
    }

    void test()
        {
          KDL::JntArray result;
          KDL::Frame end_effector_pose(KDL::Rotation::Quaternion(-0.113, 0.992, -0.026, 0.046),
                                       KDL::Vector(0.611, -0.230, 0.085));

          int rc;
          rc = this->_tracik_solver->CartToJnt(*(this->_nominal), end_effector_pose, result);

          if (rc>=0)
              ROS_INFO("SUCCESS");
          else
              ROS_ERROR("FAILURE");
        }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ik_server");
  ros::NodeHandle nh("~");

  std::string urdf_param;
  double timeout;
  
  nh.param("timeout", timeout, 0.005);
  nh.param("urdf_param", urdf_param, std::string("/robot_description"));

  BaxterTracIKServer ik(nh, "right", timeout, urdf_param);
  ik.test();

  return 0;
}
