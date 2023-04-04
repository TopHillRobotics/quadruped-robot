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
    \file examples/scene_example/ObjectGrid.cc
    \brief ObjectGrid and Object classes implementation

    \author João Borrego : jsbruglie
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

#include "ObjectGrid.hh"

//////////////////////////////////////////////////
Object::Object(
    int & _type,
    const std::string & _name,
    const ignition::math::Pose3d & _pose,
    const ignition::math::Vector3d & _scale,
    const std::vector<double> & _parameters) :
        type(_type), name(_name), pose(_pose), scale(_scale), parameters(_parameters)
{
    // Store 3D points at the object surface
    sampleSurface();
}

//////////////////////////////////////////////////
void Object::sampleSurface()
{
    // Generate surface points from object parameters
    if (type == CYLINDER) {

        double radius = parameters[0];
        double length = parameters[1];

        // Generate points on the circular edge of the cylinder
        for (int i = 0; i < TOTAL_STEPS_C ; i++) {

            float x,y,z;
            x = cos(ANGLE_STEP_C * i) * radius;
            y = sin(ANGLE_STEP_C * i) * radius;
            z = 0.5 * length;

            Eigen::Vector4f point1;
            point1 << x, y, z, 1.0;
            points.push_back(point1);

            z = -0.5 * length;
            Eigen::Vector4f point2;
            point2 << x, y, z, 1.0;
            points.push_back(point2);
        }

    } else if (type == SPHERE) {

        double radius = parameters[0];

        for (int i = 0; i < TOTAL_STEPS_S; i++) {
            for (int j = 0; j < TOTAL_STEPS_S; j++) {

                float x,y,z;
                x = radius * sin(ANGLE_STEP_S * i) * cos(ANGLE_STEP_S * j);
                y = radius * sin(ANGLE_STEP_S * i) * sin(ANGLE_STEP_S * j);
                z = radius * cos(ANGLE_STEP_S * i);

                Eigen::Vector4f point;
                point << x, y, z, 1.0;
                points.push_back(point);
            }
        }

    } else if (type == BOX) {

        double size_x = parameters[0];
        double size_y = parameters[1];
        double size_z = parameters[2];

        float x,y,z;
        for (int i=-1; i < 2 ; i+= 2){
            for (int j=-1; j < 2; j+= 2){
                for (int k=-1; k < 2; k+= 2){

                    x = i * size_x / 2.0;
                    y = j * size_y / 2.0;
                    z = k * size_z / 2.0;
                    Eigen::Vector4f point;
                    point << x, y, z, 1.0;
                    points.push_back(point);
                }
            }
        }
    }

    // Transform Ignition to Eigen
    Eigen::Matrix3f rot;
    rot = Eigen::Quaternionf(
        pose.Rot().W(),pose.Rot().X(),pose.Rot().Y(),pose.Rot().Z());

    Eigen::Matrix4f transf;
    transf.block(0,0,3,3) = rot;
    transf.block(0,3,3,1) = Eigen::Vector3f(pose.Pos().X(),pose.Pos().Y(),pose.Pos().Z());

    for (int i = 0; i < points.size(); i++){
        points[i] = transf * points[i];
    }
}

//////////////////////////////////////////////////
ObjectGrid::ObjectGrid(
    int num_x,
    int num_y,
    float size_x,
    float size_y,
    float size_z):
        num_cells_x(num_x),
        num_cells_y(num_y),
        grid_x(size_x),
        grid_y(size_y),
        cell_z(size_z)
{
    // Create cell array
    int num_cells = num_cells_x * num_cells_y;
    for (int i = 0; i < num_cells; i++) {
        cells.push_back(i);
    }

    // Compute each cell dimensions
    cell_x = grid_x / num_cells_x;
    cell_y = grid_y / num_cells_y;
}

//////////////////////////////////////////////////
void ObjectGrid::populate(int num_objects)
{
    int cell_id, x, y;
    int num_cells = num_cells_x * num_cells_y;

    if (num_objects > num_cells) {
        std::cerr << "Invalid number of objects inserted in grid\n";
        exit(EXIT_FAILURE);
    }

    // Clear existing objects
    objects.clear();
    // Reset object counters
    for (int i = 0; i < 3; i++) counters[i] = 0;

    // Shuffle grid cells, for random placement
    shuffleIntVector(cells);
    for (int i = 0; i < num_objects; i++) {
        // Get random cell coordinates
        cell_id = cells[i];
        x = floor(cell_id / num_cells_x);
        y = floor(cell_id - x * num_cells_x);
        // Create random object at cell (x,y)
        addRandomObject(x,y);
    }
}

//////////////////////////////////////////////////
void ObjectGrid::addRandomObject(int x, int y)
{
    // Type and name
    int type;
    std::string name;
    // Dimensions
    double min;
    double radius,length, box_x, box_y, box_z;
    std::vector<double> parameters;
    // Rotation
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    // Pose
    double p_x, p_y, p_z;
    // Scale vector
    double s_x, s_y, s_z;
    // Whether cylinder is horizontal
    bool horizontal;

    // Object type and name
    type = getRandomInt(0, 2);
    this->counters[type]++;
    name = TYPES[type] + "_" + std::to_string(counters[type]);

    // Orientation
    yaw = M_PI * getRandomDouble(0.0, 0.5);
    if (type == CYLINDER) {
        horizontal = (getRandomDouble(0.0, 1.0) > 0.5);
        if (horizontal) pitch = M_PI * 0.5;
    }
    if (type == SPHERE) {
        roll = M_PI * getRandomDouble(0.0, 1.0);
        pitch = M_PI * getRandomDouble(0.0, 1.0);
    }

    // Auxiliar calculations
    min = std::min(cell_x, cell_y);

    // Dimensions
    if (type == SPHERE || type == CYLINDER)
    {
        // Radius
        radius = getRandomDouble(0.3 * min, 0.45 * min);
        parameters.push_back(radius);

        if (type == CYLINDER) {
            // Length
            length = getRandomDouble(0.3 * min, 0.9 * min);
            parameters.push_back(length);
        }
    }
    else if (type == BOX)
    {
        // Box size in x,y,z
        box_x = getRandomDouble(0.3 * cell_x, 0.8 * cell_x);
        box_y = getRandomDouble(0.3 * cell_y, 0.8 * cell_y);
        box_z = getRandomDouble(0.3 * cell_z, 0.8 * cell_z);
        parameters.push_back(box_x);
        parameters.push_back(box_y);
        parameters.push_back(box_z);
    }

    // Pose
    p_x = (x + 0.5) * (cell_x);
    p_y = (y + 0.5) * (cell_y);
    if (type == SPHERE) {
        p_z = radius;
    } else if (type == CYLINDER) {
        if (horizontal) p_z = radius;
        else            p_z = length * 0.5;
    } else if (type == BOX) {
        p_z = box_z * 0.5;
    }

    // Apply offset to pose
    ignition::math::Pose3d pose(p_x, p_y, p_z, roll, pitch, yaw);

    // Scale vector
    if (type == SPHERE) {
        s_x = s_y = s_z = 2 * radius;
    } else if (type == CYLINDER) {
        s_x = s_y = 2 * radius;
        s_z = length;
    } else if (type == BOX) {
        s_x = box_x;
        s_y = box_y;
        s_z = box_z;
    }
    ignition::math::Vector3d scale(s_x, s_y, s_z);

    this->objects.emplace_back(type, name, pose, scale, parameters);
}
