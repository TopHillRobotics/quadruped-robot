#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/sensors.hh>

namespace gazebo
{
    class DrawContactPlugin : public SensorPlugin
    {
        public:
        DrawContactPlugin() : SensorPlugin(){}
        ~DrawContactPlugin()
        {
        }

        void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
        {
            this->parentSensor = std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);
            if (!this->parentSensor)
            {
                gzerr << "ContactPlugin requires a ContactSensor.\n";
                return;
            }
            this->updateConnection = this->parentSensor->ConnectUpdated(std::bind(&DrawContactPlugin::OnUpdate, this));
            this->parentSensor->SetActive(true);

            std::string material;
            if (_sdf->HasElement("material"))
            {
                material = _sdf->Get<std::string>("material");
            }

            prevPoint = ignition::math::Vector3d(0, 0, 0);
            markerMsg.set_ns("visual_draw_contact/");
            markerMsg.set_id(0);
            markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
            markerMsg.set_type(ignition::msgs::Marker::POINTS);
            ignition::msgs::Material *matMsg = markerMsg.mutable_material();
            matMsg->mutable_script()->set_name(material);
            ignition::msgs::Time *timeMsg = markerMsg.mutable_lifetime();
            timeMsg->set_sec(5);
            timeMsg->set_nsec(0);
        }

        void OnUpdate()
        {
            ignition::math::Vector3d point = ignition::math::Vector3d(0, 0, 0);
            msgs::Contacts contacts = this->parentSensor->Contacts();
            for (int i = 0; i < contacts.contact_size(); i++)
            {
                for (int j = 0; j < contacts.contact(i).position_size(); j++)
                {
                    point[0] += contacts.contact(i).position(j).x();
                    point[1] += contacts.contact(i).position(j).y();
                    point[2] += contacts.contact(i).position(j).z();
                    // std::cout << contacts.contact(i).position_size() << " "
                    // << j << "  Position:"
                    // << contacts.contact(i).position(j).x() << " "
                    // << contacts.contact(i).position(j).y() << " "
                    // << contacts.contact(i).position(j).z() << std::endl;
                }
                if (contacts.contact(i).position_size())
                {
                    point[0] /= contacts.contact(i).position_size();
                    point[1] /= contacts.contact(i).position_size();
                    point[2] /= contacts.contact(i).position_size();
                }
                // std::cout << contacts.contact_size() << " " << i << std::endl;
            }
            if (contacts.contact_size())
            {
                point[0] /= contacts.contact_size();
                point[1] /= contacts.contact_size();
                point[2] /= contacts.contact_size();
            }
            // std::cout << point[0] << " " << point[1] << " " << point[2] << std::endl;

            if (point != ignition::math::Vector3d(0, 0, 0) && point != prevPoint)
            {
                int step = 5;
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
        sensors::ContactSensorPtr parentSensor;
        event::ConnectionPtr updateConnection;
        ignition::transport::Node node;
        ignition::msgs::Marker markerMsg;
        ignition::math::Vector3d prevPoint;
    };
    GZ_REGISTER_SENSOR_PLUGIN(DrawContactPlugin)
}