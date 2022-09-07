#include <ignition/math/Color.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/common/Plugin.hh>
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <boost/bind.hpp>
#include <sstream>
#include <ostream>

using std::stringstream;

namespace gazebo
{
    class DrawLinesPlugin : public VisualPlugin
    {
        public:
        DrawLinesPlugin():line(NULL){}
        ~DrawLinesPlugin(){
            this->visual->DeleteDynamicLine(this->line);
        }

        void Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf )
        {
            printf("=========================*************************\n");
            this->visual = _parent;
            this->visual_namespace = "visual/";
            if (!_sdf->HasElement("topicName")){
                ROS_INFO("pose draw plugin missing <topicName>, defaults to /default_force_draw");
                this->topic_name = "/default_force_draw";
            } else{
                this->topic_name = _sdf->Get<std::string>("topicName");
            }
            if (!ros::isInitialized()){
                int argc = 0;
                char** argv = NULL;
                ros::init(argc,argv,"gazebo_visual",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
            }

            this->line = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
            this->line->AddPoint(ignition::math::Vector3d(0, 0, 0));
            this->line->AddPoint(ignition::math::Vector3d(100, 100, 100));
            this->line->setMaterial("Gazebo/Red");
            this->line->setVisibilityFlags(GZ_VISIBILITY_GUI);
            this->visual->SetVisible(true);

            this->rosnode = new ros::NodeHandle(this->visual_namespace);
            this->pose_sub = this->rosnode->subscribe(this->topic_name+"/"+"xyz_line", 30, &DrawLinesPlugin::GetLineCallback, this);
            this->update_connection = event::Events::ConnectPreRender(boost::bind(&DrawLinesPlugin::OnUpdate, this));
            ROS_INFO("Load %s Draw Lines plugin.", this->topic_name.c_str());
        }

        void OnUpdate()
        {
            // this->line->SetPoint(0, ignition::math::Vector3d(x, y, 0));
            this->line->SetPoint(1, ignition::math::Vector3d(x, y, z));
        }

        void GetLineCallback(const geometry_msgs::Pose::ConstPtr & msg)
        {
            x = msg->position.x;
            y = msg->position.y;
            z = msg->position.z;
            // fmt.str("");
            // fmt << "x = " << x << " y = " << y << " z = " << z;
            // std::cout << fmt.str() <<std::endl;
            // ROS_INFO("x = %f, y = %f z = %f ", x,y,z);

            // msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
        }

        private:
            ros::NodeHandle* rosnode;
            std::string topic_name;
            rendering::VisualPtr visual;
            rendering::DynamicLines *line;
            std::string visual_namespace;
            ros::Subscriber pose_sub;
            stringstream fmt;
            double x=1, y=1, z=1;
            double roll=0, pitch=0, yaw=0;
            event::ConnectionPtr update_connection;
    };
    GZ_REGISTER_VISUAL_PLUGIN(DrawLinesPlugin)
}
