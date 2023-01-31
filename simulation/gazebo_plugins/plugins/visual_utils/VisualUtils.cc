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
    \file visual_utils/VisualUtils.cc
    \brief Visual Utils plugin implementation

    A custom gazebo plugin that provides an interface to programatically
    change the visual properties of an object.

    \author João Borrego : jsbruglie
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

#include "VisualUtils.hh"

namespace gazebo {

/// \brief Class for private visual utils plugin data.
class VisualUtilsPrivate
{
    /// Visual to which the plugin is attached
    public: rendering::VisualPtr visual;
    /// Connects to rendering update event
    public: event::ConnectionPtr updateConnection;
    /// Gazebo transport node
    public: transport::NodePtr node;
    /// Visual utils topic subscriber
    public: transport::SubscriberPtr sub;
    /// A publisher to the reply topic
    public: transport::PublisherPtr pub;

    /// Unique name
    public: std::string name;
    /// Material name patterns
    public: std::vector<std::string> patterns;
    /// Available materials
    public: std::vector<std::string> materials;
    /// Internal counter for used materials
    public: unsigned int used_materials {0};

    /// Default pose
    public: ignition::math::Pose3d default_pose;

    /// Mutex
    public: std::mutex mutex;

    /// Flag to update pose
    public: bool update_pose {false};
    /// Flag to update material
    public: bool update_material {false};
    /// Flag to update scale
    public: bool update_scale {false};

    /// New pose
    public: ignition::math::Pose3d new_pose;
    /// New material
    public: std::string new_material;
    /// New scale
    public: ignition::math::Vector3d new_scale;
};

/// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(VisualUtils)

/////////////////////////////////////////////////
VisualUtils::VisualUtils(): VisualPlugin(), dataPtr(new VisualUtilsPrivate)
{
}

/////////////////////////////////////////////////
VisualUtils::~VisualUtils()
{
    dataPtr->sub.reset();
    dataPtr->node->Fini();
    gzmsg << "[VisualUtils] Unloaded visual tools: " << dataPtr->name << std::endl;
}

/////////////////////////////////////////////////
void VisualUtils::Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
{
    // Check if attached to valid visual
    if (!_visual || !_sdf) {
        gzerr << "[VisualUtils] Invalid visual or SDF element." << std::endl;
        return;
    }
    dataPtr->visual = _visual;

    // Plugin parameters

    // Unique name
    if (_sdf->HasElement("uid")) {
        dataPtr->name = _sdf->Get<std::string>("uid");
    } else {
        dataPtr->name = DEFAULT_NAME;
    }

    // Possible patterns for material names
    if (_sdf->HasElement("patterns")) {
        std::string patterns_arg(_sdf->Get<std::string>("patterns"));
        boost::split(dataPtr->patterns, patterns_arg,
            boost::is_any_of(" "), boost::token_compress_on);
    } else {
        gzerr << "[VisualUtils] No material name patterns provided.";
        return;
    }

    // Visual settings
    dataPtr->visual->SetShaderType("vertex");
    dataPtr->visual->SetCastShadows(true);
    dataPtr->visual->SetLighting(true);

    // Connect to the world update signal
    dataPtr->updateConnection = event::Events::ConnectPreRender(
        std::bind(&VisualUtils::Update, this));
     // Setup transport node
    dataPtr->node = transport::NodePtr(new transport::Node());
    dataPtr->node->Init();
    // Subcribe to the monitored requests topic
    dataPtr->sub = dataPtr->node->Subscribe(REQUEST_TOPIC,
        &VisualUtils::onRequest, this);
    // Setup publisher for the response topic
    dataPtr->pub = dataPtr->node->
        Advertise<gap::msgs::VisualUtilsResponse>(RESPONSE_TOPIC);

    // Default pose
    dataPtr->default_pose = _visual->Pose();
    // Load materials
    loadResources();

    gzmsg << "[VisualUtils] Loaded visual tools: " << dataPtr->name << std::endl;
}

/////////////////////////////////////////////////
void VisualUtils::Update()
{
    std::lock_guard<std::mutex> lock(dataPtr->mutex);

    gap::msgs::VisualUtilsResponse msg;
    bool updated = false;

    // Update scale
    if (dataPtr->update_scale) {
        if (dataPtr->visual->Scale() != dataPtr->new_scale) {
            dataPtr->visual->SetScale(dataPtr->new_scale);
        }
        dataPtr->update_scale = false;
        updated = true;
    }
    // Update pose
    if (dataPtr->update_pose) {
        if (dataPtr->visual->WorldPose() != dataPtr->new_pose) {
            dataPtr->visual->SetWorldPose(dataPtr->new_pose);
        }
        dataPtr->update_pose = false;
        updated = true;
    }
    // Update material
    if (dataPtr->update_material) {
        dataPtr->visual->SetMaterial(dataPtr->new_material,false,false);
        dataPtr->update_material = false;
        updated = true;
    }

    // Notify subscribers to the response topic that visual was updated
    if (updated) {
        msg.set_type(UPDATED);
        msg.set_origin(dataPtr->name);
        dataPtr->pub->Publish(msg);
    }
}

/////////////////////////////////////////////////
void VisualUtils::onRequest(VisualUtilsRequestPtr &_msg)
{
    // Relevant index of commands in message for the current visual
    int index = -1;

    // Validate msg structure
    if (!_msg->has_type()) {
        gzwarn <<" [VisualUtils] Invalid request received" << std::endl;
        return;
    }

    // Check if current visual is targeted
    for (int i = 0; i < _msg->targets_size(); i++) {
        if (dataPtr->name == _msg->targets(i)) {
            index = i; break;
        }
    }

    if (_msg->type() == UPDATE)
    {
        std::lock_guard<std::mutex> lock(dataPtr->mutex);

        if (index == -1) {
            // Ĩf visual is not targeted, set new pose to default pose
            dataPtr->new_pose = dataPtr->default_pose;
            dataPtr->update_pose = true;
        } else {
            if (index < _msg->poses_size()) {
                dataPtr->new_pose = gazebo::msgs::ConvertIgn(_msg->poses(index));
                dataPtr->update_pose = true;
            }
            if (index < _msg->scale_size()) {
                dataPtr->new_scale = gazebo::msgs::ConvertIgn(_msg->scale(index));
                dataPtr->update_scale = true;
            }
            randomMaterialName(dataPtr->new_material);
            dataPtr->update_material = true;
        }
    }
    else if (_msg->type() == DEFAULT_POSE)
    {
        std::lock_guard<std::mutex> lock(dataPtr->mutex);

        if (index != -1) {
            if (index < _msg->poses_size()) {
                dataPtr->default_pose = gazebo::msgs::ConvertIgn(
                    _msg->poses(index));
            }
        }
    }
}

/////////////////////////////////////////////////
void VisualUtils::loadResources()
{
    std::lock_guard<std::mutex> lock(dataPtr->mutex);

    // Clear materials
    dataPtr->materials.clear();
    // Get list of OGRE resources
    Ogre::ResourceManager::ResourceMapIterator resources = 
        Ogre::MaterialManager::getSingleton().getResourceIterator();
    // Add materials that match material patterns
    std::string name;
    for (auto & material : resources)
    {
        name = material.second->getName();
        for (auto & pattern : dataPtr->patterns)
        {
            // Check if pattern matches prefix
            auto res = std::mismatch(
                pattern.begin(), pattern.end(), name.begin());
            if (res.first == pattern.end()) {
                dataPtr->materials.push_back(name);
            }
        }
    }

    // Shuffle materials vector
    unsigned int seed =
        std::chrono::system_clock::now().time_since_epoch().count();
    auto rng = std::mt19937 {seed};
    std::shuffle(std::begin(dataPtr->materials),
        std::end(dataPtr->materials), rng);

    /*
    gzdbg << dataPtr->materials.size()
        << " matching materials found." << std::endl;
    */
}

/////////////////////////////////////////////////
void VisualUtils::randomMaterialName(std::string &name)
{
    if (dataPtr->used_materials == dataPtr->materials.size())
    {
        // All materials have been used once. Reshuffle    
        unsigned int seed =
            std::chrono::system_clock::now().time_since_epoch().count();
        auto rng = std::mt19937 {seed};
        std::shuffle(std::begin(dataPtr->materials),
            std::end(dataPtr->materials), rng);
        dataPtr->used_materials = 0;
    }
    else
    {
        name = dataPtr->materials.at(dataPtr->used_materials++);
    }
} 

}
