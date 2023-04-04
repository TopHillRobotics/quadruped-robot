/*
 *  Copyright (C) 2018 João Borrego
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

/*!
    \file utils/DRInterface.cc
    \brief Domain randomization interface

    \author João Borrego : jsbruglie
*/

#include "DRInterface.hh"

// Constants
const char DRInterface::REQUEST_TOPIC[]  = "~/gap/dr";
const char DRInterface::RESPONSE_TOPIC[] = "~/gap/dr/response";
const char DRInterface::VISUAL_TOPIC[]   = "~/visual";
const int  DRInterface::POSITION = 0;
const int  DRInterface::VELOCITY = 1;

//////////////////////////////////////////////////
DRInterface::DRInterface(
    const std::string & req_topic_,
    const std::string & res_topic_) :
        req_topic(req_topic_), res_topic(res_topic_)
{
    node = gazebo::transport::NodePtr(new gazebo::transport::Node());
    node->Init();

    pub = node->Advertise<DRRequest>(req_topic);
    pub->WaitForConnection();
    pub_visual = node->Advertise<gazebo::msgs::Visual>(VISUAL_TOPIC);
    // pub_visual->WaitForConnection();
    sub = node->Subscribe(res_topic, &DRInterface::onResponse, this);

    debugPrintTrace("DRInterface initialized." << std::endl <<
        "   Requests topic: " << req_topic << std::endl <<
        "   Response topic: " << res_topic);
}

//////////////////////////////////////////////////
DRInterface::DRInterface() : DRInterface(REQUEST_TOPIC, RESPONSE_TOPIC)
{
}

//////////////////////////////////////////////////
DRInterface::~DRInterface()
{
    this->sub->Unsubscribe();
}

//////////////////////////////////////////////////
DRRequest DRInterface::createRequest()
{
    DRRequest msg;
    return msg;
}

//////////////////////////////////////////////////
void DRInterface::publish(DRRequest & msg, bool blocking)
{
    if (blocking)
    {
        msg.set_feedback(true);
        std::unique_lock<std::mutex> lock(mutex);
        pub->Publish(msg);
        debugPrintTrace("Waiting for feedback.");
        cond_var.wait(lock);
    }
    else
    {
        pub->Publish(msg);
    }
}

//////////////////////////////////////////////////
void DRInterface::publish(gazebo::msgs::Visual & msg, bool blocking)
{
    if (blocking){ /*TODO - Unused */ }
    pub_visual->Publish(msg);
}

//////////////////////////////////////////////////
void DRInterface::addGravity(DRRequest & msg,
    const ignition::math::Vector3d & gravity)
{
    gazebo::msgs::Physics *physics_msg;
    gazebo::msgs::Vector3d *gravity_msg;

    physics_msg = msg.mutable_physics();
    gravity_msg = physics_msg->mutable_gravity();
    gazebo::msgs::Set(gravity_msg, gravity);
}

//////////////////////////////////////////////////
void DRInterface::addModelScale(DRRequest & msg,
    const std::string & model,
    const ignition::math::Vector3d & scale)
{
    gazebo::msgs::Model *model_msg;
    gazebo::msgs::Vector3d *scale_msg;

    model_msg = msg.add_model();
    model_msg->set_name(model);
    scale_msg = model_msg->mutable_scale();
    gazebo::msgs::Set(scale_msg, scale);
}

//////////////////////////////////////////////////
void DRInterface::addLinkMass(DRRequest & msg,
    const std::string & model,
    const std::string & link,
    double mass)
{
    gazebo::msgs::Model *model_msg;
    gazebo::msgs::Link *link_msg;
    gazebo::msgs::Inertial *inertial_msg;

    model_msg = msg.add_model();
    model_msg->set_name(model);
    link_msg = model_msg->add_link();
    link_msg->set_name(link);
    inertial_msg = link_msg->mutable_inertial();
    inertial_msg->set_mass(mass);
}

//////////////////////////////////////////////////
void DRInterface::addInertiaMatrix(DRRequest & msg,
    const std::string & model,
    const std::string & link,
    double ixx, double iyy, double izz,
    double ixy, double ixz, double iyz)
{
    gazebo::msgs::Model *model_msg;
    gazebo::msgs::Link *link_msg;
    gazebo::msgs::Inertial *inertial_msg;

    model_msg = msg.add_model();
    model_msg->set_name(model);
    link_msg = model_msg->add_link();
    link_msg->set_name(link);
    inertial_msg = link_msg->mutable_inertial();
    inertial_msg->set_ixx(ixx);
    inertial_msg->set_iyy(iyy);
    inertial_msg->set_izz(izz);
    inertial_msg->set_ixy(ixy);
    inertial_msg->set_ixz(ixz);
    inertial_msg->set_iyz(iyz);
}

//////////////////////////////////////////////////
void DRInterface::addSurface(DRRequest & msg,
    const std::string & model,
    const std::string & link,
    const std::string & collision,
    gazebo::msgs::Surface *surface_ptr)
{
    gazebo::msgs::Model *model_msg;
    gazebo::msgs::Link *link_msg;
    gazebo::msgs::Collision *collision_msg;

    model_msg = msg.add_model();
    model_msg->set_name(model);
    link_msg = model_msg->add_link();
    link_msg->set_name(link);
    collision_msg = link_msg->add_collision();
    collision_msg->set_id(0);
    collision_msg->set_name(collision);
    collision_msg->set_allocated_surface(surface_ptr);
}

//////////////////////////////////////////////////
void DRInterface::addJoint(DRRequest & msg,
    const std::string & model,
    const std::string & joint,
    double limit_lower,
    double limit_upper,
    double limit_effort,
    double limit_velocity,
    double damping,
    double friction)
{
    gazebo::msgs::Model *model_msg;
    gazebo::msgs::Joint *joint_msg;
    gazebo::msgs::Axis *axis_msg;

    model_msg = msg.add_model();
    model_msg->set_name(model);
    joint_msg = model_msg->add_joint();
    joint_msg->set_name(joint);
    axis_msg = joint_msg->mutable_axis1();
    axis_msg->set_limit_lower(limit_lower);
    axis_msg->set_limit_upper(limit_upper);
    axis_msg->set_limit_effort(limit_effort);
    axis_msg->set_limit_velocity(limit_velocity);
    axis_msg->set_damping(damping);
    axis_msg->set_friction(friction);

    // Required fields - set to dummy unused values
    ignition::math::Vector3d xyz(0,0,0);
    gazebo::msgs::Vector3d *xyz_msg;
    xyz_msg = axis_msg->mutable_xyz();
    gazebo::msgs::Set(xyz_msg, xyz);
    axis_msg->set_use_parent_model_frame(true);
}

//////////////////////////////////////////////////
void DRInterface::addModelCmd(DRRequest & msg,
    const std::string & model,
    const std::string & joint,
    int type,
    double p_gain,
    double i_gain,
    double d_gain)
{
    ModelCmdMsg *model_cmd;
    gazebo::msgs::JointCmd *joint_cmd;
    gazebo::msgs::PID *pid;

    if (type != POSITION && type != VELOCITY) { return; }

    model_cmd = msg.add_model_cmd();
    model_cmd->set_model_name(model);
    joint_cmd = model_cmd->add_joint_cmd();
    joint_cmd->set_name(joint);

    if (type == POSITION) { pid = joint_cmd->mutable_position(); }
    else                  { pid = joint_cmd->mutable_velocity(); }

    if (p_gain != INFINITY) { pid->set_p_gain(p_gain); }
    if (i_gain != INFINITY) { pid->set_i_gain(i_gain); }
    if (d_gain != INFINITY) { pid->set_d_gain(d_gain); }
}

//////////////////////////////////////////////////
void DRInterface::addColors(gazebo::msgs::Visual & msg,
    const std::string & visual,
    const std::string & parent,
    const ignition::math::Color & ambient,
    const ignition::math::Color & diffuse,
    const ignition::math::Color & emissive,
    const ignition::math::Color & specular)
{
    gazebo::msgs::Material *material_msg;
    gazebo::msgs::Color *ambient_msg,
        *diffuse_msg, *emissive_msg, *specular_msg;

    msg.set_name(visual);
    msg.set_parent_name(parent);
    material_msg = msg.mutable_material();
    ambient_msg = material_msg->mutable_ambient();
    diffuse_msg = material_msg->mutable_diffuse();
    emissive_msg = material_msg->mutable_emissive();
    specular_msg = material_msg->mutable_specular();
    gazebo::msgs::Set(ambient_msg, ambient);
    gazebo::msgs::Set(diffuse_msg, diffuse);
    gazebo::msgs::Set(emissive_msg, emissive);
    gazebo::msgs::Set(specular_msg, specular);
}

/////////////////////////////////////////////////
void DRInterface::onResponse(DRResponsePtr & _msg)
{
    debugPrintTrace("Received response!");
    std::lock_guard<std::mutex> lock(mutex);
    cond_var.notify_one();
}
