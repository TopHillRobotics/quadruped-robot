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
    \file plugins/domain_randomization/DRPlugin.cc
    \brief Domain Randomization Gazebo plugin

    Plugin for handling Domain Randomization requests

    \author João Borrego : jsbruglie
*/

#include "DRPlugin.hh"

namespace gazebo {

// Constants
const char DRPlugin::REQUEST_TOPIC[]  = "~/gap/dr";
const char DRPlugin::RESPONSE_TOPIC[] = "~/gap/dr/response";
const int  DRPlugin::POSITION = 0;
const int  DRPlugin::VELOCITY = 1;

// SDF Parameter tags
const char DRPlugin::PARAM_REQ_TOPIC[] = "request_topic";
const char DRPlugin::PARAM_RES_TOPIC[] = "response_topic"; 

/// \brief Class for private Domain Randomization plugin data.
class DRPluginPrivate
{
    /// Gazebo transport node
    public: transport::NodePtr node;
    /// Gazebo topic subscriber
    public: transport::SubscriberPtr sub;
    /// Gazebo topic publisher
    public: transport::PublisherPtr pub;

    /// Mutex for safe data access
    public: std::mutex mutex;
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(DRPlugin)

/////////////////////////////////////////////////
DRPlugin::DRPlugin() : WorldPlugin(),
    data_ptr(new DRPluginPrivate)
{
    gzmsg << "[DRPlugin] Launched plugin." << std::endl;
}

/////////////////////////////////////////////////
DRPlugin::~DRPlugin()
{
    gzmsg << "[DRPlugin] Unloaded plugin." << std::endl;
}

/////////////////////////////////////////////////
void DRPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
    NULL_CHECK(_world, "World pointer is NULL");
    this->world = _world;
    this->physics_engine = world->Physics();

    // Read topic names from SDF
    loadTopicNames(_sdf);

    // Connect to world update event
    this->update_connection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&DRPlugin::onUpdate, this));
    // Setup transport node
    this->data_ptr->node = transport::NodePtr(new transport::Node());
    this->data_ptr->node->Init();
    // Subcribe to the monitored requests topic
    this->data_ptr->sub = this->data_ptr->node->Subscribe(req_topic,
        &DRPlugin::onRequest, this);
    // Publish to the response topic
    this->data_ptr->pub = this->data_ptr->node->
        Advertise<DRResponse>(res_topic);
    gzmsg << "[DRPlugin] Loaded plugin." << std::endl;
}

/////////////////////////////////////////////////
void DRPlugin::loadTopicNames(sdf::ElementPtr _sdf)
{
    if (_sdf->HasElement(PARAM_REQ_TOPIC))
    {
        req_topic = _sdf->Get<std::string>(PARAM_REQ_TOPIC);
        gzdbg << "Requests topic: " << req_topic << std::endl;
    }
    if (_sdf->HasElement(PARAM_RES_TOPIC))
    {
        res_topic = _sdf->Get<std::string>(PARAM_RES_TOPIC);   
        gzdbg << "Response topic: " << res_topic << std::endl;
    }
}

/////////////////////////////////////////////////
void DRPlugin::onUpdate()
{
    // Provide client with feedback
    // Has 1-Cycle delay; could not get it to work in same cycle
    if (feedback_pending)
    {
        DRResponse response;
        response.set_success(true);
        data_ptr->pub->WaitForConnection();
        data_ptr->pub->Publish(response, true);
        feedback_pending = false;
        gzdbg << common::Time::GetWallTimeAsISOString() <<
            " - Provided feedback to client." << std::endl;
    }

    std::lock_guard<std::mutex> lock(data_ptr->mutex);

    // If no requests are pending
    if (!msg) { return; }
    // Process stored message
    if (msg->has_feedback())
    {
        feedback_pending = true;
    }
    if (msg->has_physics())
    {
        processPhysics(msg->physics());
    }
    for (const auto & model : msg->model())
    {
        processModel(model);
    }
    for (const auto & model_cmd : msg->model_cmd())
    {
        processModelCmd(model_cmd);
    }
    msg.reset();
}

/////////////////////////////////////////////////
void DRPlugin::onRequest(DRRequestPtr & _msg)
{
    std::lock_guard<std::mutex> lock(data_ptr->mutex);

    NULL_CHECK(_msg, "Invalid request");
    // TODO - Create message queue
    // Store message
    if (!msg) { msg = _msg; }
}

// Process requests

/////////////////////////////////////////////////
void DRPlugin::processPhysics(const msgs::Physics & msg)
{
    ignition::math::Vector3d gravity;

    if (msg.has_gravity())
    {
        gravity = msgs::ConvertIgn(msg.gravity());
        physics_engine->SetGravity(gravity);
    }
}

/////////////////////////////////////////////////
void DRPlugin::processModel(const msgs::Model & msg)
{
    physics::ModelPtr model;
    ignition::math::Vector3d scale;

    std::string model_name = msg.name();
    model = world->ModelByName(model_name);
    NULL_CHECK(model, "Model not found:" + model_name);

    for (const auto & joint : msg.joint())
    {
        processJoint(model, joint);
    }
    for (const auto & link : msg.link())
    {
        processLink(model, link);
    }
    if (msg.has_scale())
    {
        scale = msgs::ConvertIgn(msg.scale());
        model->SetScale(scale, true);
        gzdbg << "Scaled " << model_name << " by " << scale << std::endl;
    }
    for (const auto & nested_model : msg.model())
    {
        processModel(nested_model);
    }
}

/////////////////////////////////////////////////
void DRPlugin::processJoint(
    physics::ModelPtr model,
    const msgs::Joint & msg)
{
    std::string joint_name;
    physics::JointPtr joint;
    msgs::Axis axis_msg;
    double value;

    NULL_CHECK(model, "Invalid model");
    joint_name = msg.name();
    joint = model->GetJoint(joint_name);
    NULL_CHECK(joint, "Joint not found: " + joint_name);
    
    // axis2 is not yet used by Gazebo
    if (msg.has_axis1())
    {
        // Since every field is required,
        // filter out unwanted fields by checking for INFINITY
        axis_msg = msg.axis1();
        // Joint lower limit
        value = axis_msg.limit_lower();
        if (value != INFINITY) { joint->SetLowerLimit(0, value); }
        // Joint upper limit
        value = axis_msg.limit_upper();
        if (value != INFINITY) { joint->SetUpperLimit(0, value); }
        // Joint effort limit
        value = axis_msg.limit_effort();
        if (value != INFINITY) { joint->SetEffortLimit(0, value); }
        // Joint velocity limit
        value = axis_msg.limit_velocity();
        if (value != INFINITY) { joint->SetVelocityLimit(0, value); }
        // Joint physical velocity dependent on viscous damping coefficient
        value = axis_msg.damping();
        if (value != INFINITY) { joint->SetDamping(0, value); }
        // Joint static friction
        value = axis_msg.friction();
        if (value != INFINITY) { joint->SetParam("friction", 0, value); }
    }
    // ODE-specific parameters are not evaluated

    gzdbg << "Processed joint " << joint_name << std::endl;
}

/////////////////////////////////////////////////
void DRPlugin::processLink(
    physics::ModelPtr model,
    const msgs::Link & msg)
{
    std::string link_name;
    physics::LinkPtr link;
    msgs::Collision collision_msg;
    physics::CollisionPtr collision;

    link_name = msg.name();
    link = model->GetChildLink(link_name);
    NULL_CHECK(link, "Link not found");

    if (msg.has_inertial())
    {
        processInertial(link, msg.inertial());
    }
    for (const auto & collision_msg : msg.collision())
    {
        gzdbg << "Collision received!" << std::endl;
        if (collision_msg.has_surface())
        {
            collision = link->GetCollision(collision_msg.name());
            NULL_CHECK(collision, "Collision not found");
            processSurface(collision, collision_msg.surface());
        }
    }
}

/////////////////////////////////////////////////
void DRPlugin::processInertial(
    physics::LinkPtr link,
    const msgs::Inertial & msg)
{
    physics::InertialPtr inertial;
    double mass;
    double ixx, ixy, ixz, iyy, iyz, izz;

    NULL_CHECK(link, "Invalid link");
    inertial = link->GetInertial();

    if (msg.has_mass())
    {
        mass = msg.mass();
        inertial->SetMass(mass);
    }
    if (msg.has_ixx() && msg.has_ixy() && 
        msg.has_ixz() && msg.has_iyy() && 
        msg.has_iyz() && msg.has_izz())
    {
        ixx = msg.ixx(); ixy = msg.ixy();
        ixz = msg.ixz(); iyy = msg.iyy();
        iyz = msg.iyz(); izz = msg.izz();
        inertial->SetInertiaMatrix(ixx, iyy, izz, ixy, ixz, iyz);
    }
}

/////////////////////////////////////////////////
void DRPlugin::processSurface(
    physics::CollisionPtr collision,
    const msgs::Surface & msg)
{
    physics::SurfaceParamsPtr surface;
    surface = collision->GetSurface();
    NULL_CHECK(surface, "Invalid surface");
    surface->ProcessMsg(msg);

    gzdbg << "Processed surface " << collision->GetName() << std::endl;
}

/////////////////////////////////////////////////
void DRPlugin::processModelCmd(
    const ModelCmdMsg & msg)
{
    std::string model_name;
    physics::ModelPtr model;

    model_name = msg.model_name();
    model = world->ModelByName(model_name);
    NULL_CHECK(model, "Model not found");

    for (const auto & joint_cmd : msg.joint_cmd())
    {
        processJointCmd(model, joint_cmd);
    }
}

/////////////////////////////////////////////////
void DRPlugin::processJointCmd(
    physics::ModelPtr model,
    const msgs::JointCmd & msg)
{
    // Joint scoped name
    std::string joint_name;
    physics::JointPtr joint;
    physics::JointControllerPtr controller;

    NULL_CHECK(model, "Invalid model");
    controller = model->GetJointController();
    joint_name = msg.name();

    if (msg.has_position())
    {
        processPID(POSITION, controller, joint_name, msg.position());
    }
    if (msg.has_velocity())
    {
        processPID(VELOCITY, controller, joint_name, msg.velocity());
    }
}

/////////////////////////////////////////////////
void DRPlugin::processPID(
    int type,
    physics::JointControllerPtr controller,
    const std::string & joint,
    const msgs::PID & msg)
{
    bool requires_search {false};
    common::PID pid;
    std::map<std::string,common::PID> pids;   

    requires_search = msg.has_p_gain() || msg.has_i_gain() ||
        msg.has_d_gain() || msg.has_i_max() || msg.has_i_min();

    if (type == POSITION)
    {
        if (requires_search)
        {
            pids = controller->GetPositionPIDs();
            pid = pids[joint];
        }
        if (msg.has_target())
        {
            controller->SetPositionTarget(joint, msg.target());
        }
    }
    else if (type == VELOCITY)
    {
        if (requires_search)
        {
            pids = controller->GetVelocityPIDs();
            pid = pids[joint];
        }
        if (msg.has_target())
        {
            controller->SetVelocityTarget(joint, msg.target());
        }
    }

    if (msg.has_p_gain()) {
        pid.SetPGain(msg.p_gain());
    }
    if (msg.has_i_gain()) {
        pid.SetIGain(msg.i_gain());
    }
    if (msg.has_d_gain()) {
        pid.SetDGain(msg.d_gain());
    }
    if (msg.has_i_max()) {
        pid.SetIMax(msg.i_max());
    }
    if (msg.has_i_min()) {
        pid.SetIMin(msg.i_min());
    }

    if (type == POSITION)      { controller->SetPositionPID(joint, pid); }
    else if (type == VELOCITY) { controller->SetVelocityPID(joint, pid); }

    gzdbg << "Processed joint PID " << joint << std::endl;
}

}
