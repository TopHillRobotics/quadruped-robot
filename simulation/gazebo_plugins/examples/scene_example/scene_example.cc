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
    \file examples/scene_example/scene_example.cc
    \brief Random scene generation example implementation

    Generates a scene with up to 10 objects in a 4x4 grid.

    \author João Borrego : jsbruglie
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

#include "scene_example.hh"

// Global variables

// 3 x 3 object Grid
// 1 x 1 x 1 cells
ObjectGrid g_grid(3, 3, 3, 3, 1);
// Minimum number of objects
const int g_obj_min {2};
// Maximum number of objects
const int g_obj_max {7};
// Change light position
const bool g_move_light {true};
// Light default position
const ignition::math::Vector3d g_light_pos {1.5, 1.5, 5.5};
// Camera default position
const ignition::math::Vector3d g_camera_pos {1.5, 1.5, 3.2};
// Viewpoint variation
const int g_viewpoint {FIXED_VIEW};

// Variables that lock progress for synchronous scene generation
bool g_moved {false};
std::mutex g_moved_mutex;
bool g_camera_ready {false};
std::mutex g_camera_ready_mutex;
bool g_points_ready {false};
std::mutex g_points_ready_mutex;
bool g_visuals_ready {false};
std::mutex g_visuals_ready_mutex;
// Set of names of existing objects
std::set<std::string> g_names;
// Global camera pose
ignition::math::Pose3d g_camera_pose;
// Regex objects
std::regex g_regex_uid(REGEX_XML_UID);
std::regex g_regex_model(REGEX_XML_MODEL);

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
    // Command-line arguments
    unsigned int scenes {0};
    unsigned int start {0};
    std::string imgs_dir;
    std::string dataset_dir;

    bool success {false};

    // Parse command-line arguments
    parseArgs(argc, argv, scenes, start, imgs_dir, dataset_dir);
    // Create output directories
    success = createDirectory(dataset_dir);
    success &= createDirectory(imgs_dir);
    for (int i = start; i < scenes + start; i+= 100) {
        std::string imgs_subdir(imgs_dir + std::to_string(i / 100) + "00/");
        success &= createDirectory(imgs_subdir);
    }
    if (!success) {
    	std::cerr << "Error creating directories! Exiting..." << std::endl;
    	exit(EXIT_FAILURE);
    }

    // Setup communication

    // Setup Gazebo client
    gazebo::client::setup(argc, argv);
    // Optional verification for Google Protocol Buffers version
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    // Create the communication node
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Publish to the CameraUtils topic
    gazebo::transport::PublisherPtr pub_camera =
        node->Advertise<gap::msgs::CameraUtilsRequest>(CAMERA_UTILS_TOPIC);
    // Subscribe to the CameraUtils reply topic and link callback function
    gazebo::transport::SubscriberPtr sub_camera =
        node->Subscribe(CAMERA_UTILS_RESPONSE_TOPIC, onCameraUtilsResponse);

    // Publish to the VisualUtils topic
    gazebo::transport::PublisherPtr pub_visual =
        node->Advertise<gap::msgs::VisualUtilsRequest>(VISUAL_UTILS_TOPIC);
    // Subscribe to the VisualUtils reply topic and link callback function
    gazebo::transport::SubscriberPtr sub_visual =
        node->Subscribe(VISUAL_UTILS_RESPONSE_TOPIC, onVisualUtilsResponse);

    // Publish to the WorldUtils request topic
    gazebo::transport::PublisherPtr pub_world =
        node->Advertise<gap::msgs::WorldUtilsRequest>(WORLD_UTILS_TOPIC);
    // Subscribe to the WorldUtils reply topic and link callback function
    gazebo::transport::SubscriberPtr sub_world =
        node->Subscribe(WORLD_UTILS_RESPONSE_TOPIC, onWorldUtilsResponse);

    // Wait for WorldUtils plugin to launch
    pub_world->WaitForConnection();

    debugPrintTrace("Connected to World plugin");

    // Setup scene generation

    // Disable physics engine
    setPhysics(pub_world, false);
    debugPrintTrace("Disable physics engine");

    // Spawn required objects
    gap::msgs::WorldUtilsRequest msg_spawn;
    msg_spawn.set_type(SPAWN);
    addModelFromFile(msg_spawn, "models/custom_sun.sdf");
    addModelFromFile(msg_spawn, "models/custom_ground.sdf");
    addModelFromFile(msg_spawn, "models/custom_camera.sdf");
    addDynamicModels(msg_spawn);
    pub_world->Publish(msg_spawn);
    debugPrintTrace("Spawning objects");

    // Configure camera
    pub_camera->WaitForConnection();
    gap::msgs::CameraUtilsRequest msg_options;
    msg_options.set_type(OPTIONS);
    msg_options.set_output_dir(imgs_dir);
    msg_options.set_extension(".png");
    pub_camera->Publish(msg_options);

    // Wait for a subscriber to connect to this publisher
    pub_visual->WaitForConnection();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    debugPrintTrace("Done waiting for spawn");

    // Light poses
    ignition::math::Pose3d light_pose;

    // Main loop
    for (int iteration = start; iteration < scenes + start; iteration++) {

        // Populate grid with random objects
        int num_objects = (getRandomInt(g_obj_min, g_obj_max));
        g_grid.populate(num_objects);
        // Create a set with the names of created objects
        createNameSet();

        debugPrintTrace("Scene (" << iteration << "/"
            << scenes + start - 1 << "): " << num_objects << " objects");

        // Create message with desired 3D points to project in camera plane
        gap::msgs::CameraUtilsRequest msg_points;
        msg_points.set_type(PROJECTION_REQUEST);
        addProjections(msg_points);

        // Calculate new camera and light poses
        g_camera_pose = getRandomCameraPose();

        // Request move light
        if (g_move_light)
        {
            light_pose = getRandomLightPose();
            gap::msgs::WorldUtilsRequest msg_move;
            msg_move.set_type(WORLD_MOVE);
            addMoveObject(msg_move, "custom_sun", true, light_pose);
            pub_world->Publish(msg_move);
        }

        // Update scene
        gap::msgs::VisualUtilsRequest msg_visual;
        msg_visual.set_type(UPDATE);
        updateObjects(msg_visual);
        pub_visual->Publish(msg_visual);

        moveCamera(pub_camera);
        // Wait for camera to move to new position
        while (waitForMove()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        debugPrintTrace("Camera moved");

        // Wait for visuals to update
        while (waitForVisuals()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        debugPrintTrace("Visuals moved");

        // Capture the scene and save it to a file
        captureScene(pub_camera, iteration);
        while (waitForCamera()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        debugPrintTrace("Scene captured");

        // Request point projection
        pub_camera->Publish(msg_points);

        // Wait for projections
        while (waitForProjections()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        debugPrintTrace("Projections received");

        // Save annotations to file
        storeAnnotations(dataset_dir, iteration);
    }

    // Force save camera raw data buffer

    // Clean up
    sub_world.reset();
    sub_camera.reset();
    node->Fini();
    gazebo::client::shutdown();

    debugPrintTrace("All scenes generated sucessfully! Exiting...");
}

//////////////////////////////////////////////////
void addModelFromFile(
    gap::msgs::WorldUtilsRequest & msg,
    const std::string & file)
{
    // Read file to SDF string
    std::ifstream infile {file};
    std::string model_sdf {
        std::istreambuf_iterator<char>(infile), std::istreambuf_iterator<char>()
    };

    // Add object to request message
    gap::msgs::Object *object = msg.add_object();
    object->set_model_type(CUSTOM);
    object->set_sdf(model_sdf);
}

//////////////////////////////////////////////////
void addDynamicModels(gap::msgs::WorldUtilsRequest & msg)
{
    const std::vector<std::string> types = {"sphere", "cylinder","box"};

    for (int i = 0; i < types.size(); i++)
    {
        for (int j = 1; j <= g_obj_max; j++)
        {
            // Read file to SDF string
            std::string file_name = "models/custom_" + types[i] + ".sdf";
            std::ifstream infile {file_name};
            std::string sdf {
                std::istreambuf_iterator<char>(infile), std::istreambuf_iterator<char>()
            };

            std::string name = types[i] + "_" + std::to_string(j);
            // Replace content inside <uid> tags in model SDF (VisualPlugin parameter)
            std::string uid = "<uid>" + name + "</uid>";
            sdf = std::regex_replace(sdf, g_regex_uid, uid);

            // Replace model name inside <model name=""> tag
            std::string model = "<model name=\"" + name + "\">";
            sdf = std::regex_replace(sdf, g_regex_model, model);

            // Add object to request message
            gap::msgs::Object *object = msg.add_object();
            object->set_model_type(CUSTOM);
            object->set_sdf(sdf);
        }
    }
}

//////////////////////////////////////////////////
void updateObjects(gap::msgs::VisualUtilsRequest & msg)
{
    int total = g_grid.objects.size();

    // Object parameters
    std::string name;
    ignition::math::Pose3d pose;
    ignition::math::Vector3d scale;

    for  (int i = 0; i < total; i++) {
        name = g_grid.objects.at(i).name;
        pose = g_grid.objects.at(i).pose;
        scale = g_grid.objects.at(i).scale;

        gazebo::msgs::Pose *msg_pose = msg.add_poses();
        gazebo::msgs::Vector3d *msg_scale = msg.add_scale();

        msg.add_targets(name);
        gazebo::msgs::Set(msg_pose, pose);
        gazebo::msgs::Set(msg_scale, scale);
    }
    msg.add_targets("ground");
}

//////////////////////////////////////////////////
void addMoveObject(
    gap::msgs::WorldUtilsRequest & msg,
    const std::string & name,
    const bool is_light,
    const ignition::math::Pose3d & pose){

    gap::msgs::Object *object = msg.add_object();
    object->set_name(name);
    if (is_light) {
        object->set_model_type(CUSTOM_LIGHT);
    }

    gazebo::msgs::Pose *pose_msg = new gazebo::msgs::Pose();
    gazebo::msgs::Set(pose_msg, pose);
    object->set_allocated_pose(pose_msg);
}

//////////////////////////////////////////////////
ignition::math::Pose3d getRandomCameraPose()
{
    ignition::math::Pose3d new_pose;
    ignition::math::Vector3d position(g_camera_pos);
    ignition::math::Vector3d offset(0,0,0);
    ignition::math::Quaternion<double> original_orientation(0,0,0);
    ignition::math::Quaternion<double> correct_orientation(
        ignition::math::Vector3d(0,1,0), - M_PI / 2.0);

    // Fixed viewpoint
    if (g_viewpoint == FIXED_VIEW || g_viewpoint == MOVING_VIEW_POS)
    {
        ignition::math::Quaternion<double> aux(0, M_PI/4.0, 0);
        original_orientation.Euler(aux.Euler());
    }
    // Random position (Small variation)
    if (g_viewpoint == MOVING_VIEW_POS)
    {
        offset.Set(
            getRandomDouble(-0.1, 0.1),
            getRandomDouble(-0.1, 0.1),
            getRandomDouble(-0.1, 0.1));
    }
    // Random orientation (Big variation)
    if (g_viewpoint == MOVING_VIEW_POS_ROT)
    {
        ignition::math::Quaternion<double> aux(
            getRandomDouble(0, M_PI / 4.0),
            getRandomDouble(0, M_PI / 4.0),
            getRandomDouble(0, M_PI / 4.0));
        original_orientation.Euler(aux.Euler());
    }

    new_pose.Set(position + offset,
        (correct_orientation * original_orientation).Inverse());
    new_pose = new_pose.RotatePositionAboutOrigin(original_orientation);
    return new_pose;
}

//////////////////////////////////////////////////
ignition::math::Pose3d getRandomLightPose()
{
    ignition::math::Quaternion<double> light_orientation(
        getRandomDouble(-M_PI / 5.0, M_PI / 5.0),
        getRandomDouble(-M_PI / 5.0, M_PI / 5.0),
        getRandomDouble(-M_PI / 5.0, M_PI / 5.0));

    ignition::math::Pose3d new_pose;
    ignition::math::Vector3d position(g_light_pos);

    new_pose.Set(position, (light_orientation).Inverse());
    new_pose = new_pose.RotatePositionAboutOrigin(light_orientation);

    return new_pose;
}

//////////////////////////////////////////////////
void createNameSet()
{
    int num_obj = g_grid.objects.size();
    for (int i = 0; i < num_obj; i++)
    {
        std::string name(g_grid.objects.at(i).name);
        g_names.emplace(name);
    }

    std::set<std::string>::iterator it;
    for (it = g_names.begin(); it != g_names.end(); ++it) {
        std::cout << ' ' << *it;
    }
    std::cout << std::endl;
}

//////////////////////////////////////////////////
void addProjections(gap::msgs::CameraUtilsRequest & msg)
{
    int num_obj = g_grid.objects.size();
    for (int i = 0; i < num_obj; i++)
    {
        gap::msgs::PointProjection *proj = msg.add_projections();

        int num_points = g_grid.objects[i].points.size();
        for (int j = 0; j < num_points; j++)
        {
            gazebo::msgs::Vector3d *points_msg = proj->add_point3();
            points_msg->set_x(g_grid.objects[i].points[j](0));
            points_msg->set_y(g_grid.objects[i].points[j](1));
            points_msg->set_z(g_grid.objects[i].points[j](2));
        }
        proj->set_name(g_grid.objects[i].name);
    }
}

//////////////////////////////////////////////////
void moveCamera(gazebo::transport::PublisherPtr pub)
{
    gap::msgs::CameraUtilsRequest msg;
    msg.set_type(MOVE_REQUEST);

    gazebo::msgs::Pose *pose_msg = new gazebo::msgs::Pose();
    gazebo::msgs::Set(pose_msg, g_camera_pose);
    msg.set_allocated_pose(pose_msg);

    pub->Publish(msg, false);
}

//////////////////////////////////////////////////
void captureScene(gazebo::transport::PublisherPtr pub, int iteration)
{
    gap::msgs::CameraUtilsRequest msg;
    msg.set_type(CAPTURE_REQUEST);
    msg.set_file_name(std::to_string(iteration / 100) + "00/" + std::to_string(iteration));
    pub->Publish(msg, false);
}

//////////////////////////////////////////////////
bool waitForMove()
{
    std::lock_guard<std::mutex> lock(g_moved_mutex);
    if (g_moved) {
        g_moved = false;
        return false;
    }
    return true;
}

//////////////////////////////////////////////////
bool waitForVisuals()
{
    std::lock_guard<std::mutex> lock(g_visuals_ready_mutex);
    if (g_visuals_ready) {
        g_visuals_ready = false;
        return false;
    }
    return true;
}

//////////////////////////////////////////////////
bool waitForCamera()
{
    std::lock_guard<std::mutex> lock(g_camera_ready_mutex);
    if (g_camera_ready) {
        g_camera_ready = false;
        return false;
    }
    return true;
}

//////////////////////////////////////////////////
bool waitForProjections()
{
    std::lock_guard<std::mutex> lock(g_points_ready_mutex);
    if (g_points_ready) {
        g_points_ready = false;
        return false;
    }
    return true;
}

//////////////////////////////////////////////////
void onVisualUtilsResponse(VisualUtilsResponsePtr &_msg)
{
    if (_msg->type() == UPDATED)
    {
        g_names.erase(_msg->origin());
        if (g_names.empty()) {
            std::lock_guard<std::mutex> lock(g_visuals_ready_mutex);
            g_visuals_ready = true;
        }
    }
}

//////////////////////////////////////////////////
void onWorldUtilsResponse(WorldUtilsResponsePtr &_msg)
{
    // Unused
}

//////////////////////////////////////////////////
void onCameraUtilsResponse(CameraUtilsResponsePtr &_msg)
{
    if (_msg->type() == MOVE_RESPONSE)
    {
        std::lock_guard<std::mutex> lock(g_moved_mutex);
        g_moved = true;
    }
    else if (_msg->type() == CAPTURE_RESPONSE)
    {
        if (_msg->success()) {
            std::lock_guard<std::mutex> lock(g_camera_ready_mutex);
            g_camera_ready = true;
        }
    }
    else if (_msg->type() == PROJECTION_RESPONSE)
    {
        int objects = _msg->projections_size();
        int x_min, x_max, y_min, y_max;
        int x_tmp, y_tmp;

        // Ensure projections correspond to desired camera pose
        ignition::math::Pose3d camera_pose(gazebo::msgs::ConvertIgn(_msg->pose()));
        if (camera_pose != g_camera_pose) return;

        for (int i = 0; i < objects; i++)
        {
            int points = _msg->projections(i).point2_size();
            x_min = y_min = INT_MAX;
            x_max = y_max = INT_MIN;
            for (int j = 0; j < points; j++)
            {
                // Obtain 2D bounding box
                x_tmp = _msg->projections(i).point2(j).x();
                y_tmp = _msg->projections(i).point2(j).y();
                if (x_min > x_tmp) x_min = x_tmp;
                if (x_max < x_tmp) x_max = x_tmp;
                if (y_min > y_tmp) y_min = y_tmp;
                if (y_max < y_tmp) y_max = y_tmp;
            }
            // Store bounding box
            g_grid.objects[i].bounding_box.push_back(x_min);
            g_grid.objects[i].bounding_box.push_back(y_min);
            g_grid.objects[i].bounding_box.push_back(x_max);
            g_grid.objects[i].bounding_box.push_back(y_max);
        }

        debugPrintTrace("DONE");

        std::lock_guard<std::mutex> lock(g_points_ready_mutex);
        g_points_ready = true;
    }
}

//////////////////////////////////////////////////
void setPhysics(gazebo::transport::PublisherPtr pub, bool enable)
{
    gap::msgs::WorldUtilsRequest msg;
    msg.set_type(PHYSICS);
    msg.set_state(enable);
    pub->Publish(msg);
}

//////////////////////////////////////////////////
void storeAnnotations(
    const std::string & path,
    const int iteration)
{
    std::string ext_img = ".png";
    std::string ext_data = ".xml";
    std::string image_name = std::to_string(iteration / 100) + "00/" +
        std::to_string(iteration) + ext_img;
    std::string data_name = std::to_string(iteration) + ext_data;

    std::ofstream out(path+"/"+data_name);

    // TODO - Obtain directly from camera instead
    // These values are set in loaded SDF model - models/custom_camera.sdf
    int camera_width = 1920;
    int camera_height = 1080;
    int camera_depth = 3;

    out << "<annotation>\n"
        << "  <folder>images</folder>\n"
        << "  <filename>" + image_name + "</filename>\n"
        << "  <source>\n"
        << "    <database>The SHAPE2018 Database</database>\n"
        << "    <annotation>SHAPE SHAPE2018</annotation>\n"
        << "    <image>" << image_name <<"</image>\n"
        << "    <pose>" << g_camera_pose <<"</pose>\n"
        << "  </source>\n"
        << "  <size>\n"
        << "    <width>"  << camera_width  << "</width>\n"
        << "    <height>" << camera_height << "</height>\n"
        << "    <depth>"  << camera_depth  << "</depth>\n"
        << "  </size>\n"
        << "  <segmented>1</segmented>\n";

    for (int i = 0; i < g_grid.objects.size(); i++)
    {
        Object object = g_grid.objects[i];
        out << "  <object>\n"
            << "    <name>" << g_grid.TYPES[object.type] << "</name>\n"
            << "    <pose>" << object.pose << "</pose>\n"
            << "    <truncated>0</truncated>\n"
            << "    <difficult>1</difficult>\n"
            << "    <bndbox>\n"
            << "      <xmin>"<< object.bounding_box[0] <<"</xmin>\n"
            << "      <ymin>"<< object.bounding_box[1] <<"</ymin>\n"
            << "      <xmax>"<< object.bounding_box[2] <<"</xmax>\n"
            << "      <ymax>"<< object.bounding_box[3] <<"</ymax>\n"
            << "    </bndbox>\n"
            << "  </object>\n";
    }

    out << "</annotation>";
    out.close();
}
