#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/rendering/Visual.hh>

namespace gazebo
{
    class DrawTrajectoryPlugin : public VisualPlugin
    {
        public:
        DrawTrajectoryPlugin() : VisualPlugin(){}
        ~DrawTrajectoryPlugin()
        {
        }

        void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
        {
            visual = _visual;
            if (_sdf->HasElement("position"))
            {
                position = _sdf->Get<ignition::math::Vector3d>("position");
            }
            else
            {
                position = ignition::math::Vector3d(0, 0, 0);
            }

            std::string material;
            if (_sdf->HasElement("material"))
            {
                material = _sdf->Get<std::string>("material");
            }

            // this->updateConnection = event::Events::ConnectPreRender(std::bind(&DrawTrajectoryPlugin::OnUpdate, this));
            this->updateConnection = event::Events::ConnectRender(std::bind(&DrawTrajectoryPlugin::OnUpdate, this));

            prevPoint = visual->WorldPose().Pos();
            markerMsg.set_ns("visual_draw_trajectory/");
            markerMsg.set_id(0);
            markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
            markerMsg.set_type(ignition::msgs::Marker::LINE_STRIP);
            ignition::msgs::Material *matMsg = markerMsg.mutable_material();
            matMsg->mutable_script()->set_name(material);
            ignition::msgs::Time *timeMsg = markerMsg.mutable_lifetime();
            timeMsg->set_sec(5);
            timeMsg->set_nsec(0);
        }
        
        void OnUpdate()
        {
            ignition::math::Vector3d point = visual->WorldPose().Pos() + position;
            if (point != ignition::math::Vector3d(0, 0, 0) && point != prevPoint)
            {
                ignition::msgs::Set(markerMsg.add_point(), point);
                //ignition::msgs::Set(markerMsg.mutable_scale(), ignition::math::Vector3d(1, 1, 1));
                if (markerMsg.point_size() > step)
                {
                    node.Request("/marker", markerMsg);
                    markerMsg.mutable_point()->DeleteSubrange(0, step);
                }
                prevPoint = point;
            }
        }

        private:
        rendering::VisualPtr visual;
        ignition::math::Vector3d position;
        event::ConnectionPtr updateConnection;
        ignition::transport::Node node;
        ignition::msgs::Marker markerMsg;
        ignition::math::Vector3d prevPoint;
        int step = 2;
    };
    GZ_REGISTER_VISUAL_PLUGIN(DrawTrajectoryPlugin)
}