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
    \file examples/scene_example/ObjectGrid.hh
    \brief ObjectGrid and Object classes

    \author João Borrego : jsbruglie
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

// WorldUtils request
#include "world_utils_request.pb.h"

// Utilities
#include "utils.hh"

// Gazebo
#include <gazebo/gazebo_client.hh>
// Eigen
#include <Eigen/Dense>

/// Spawn sphere object
#define SPHERE    0
/// Spawn cylinder object
#define CYLINDER  1
/// Spawn box object
#define BOX       2

/// \brief Object in 2D grid
class Object
{
    // Public attributes

    /// Object type
    public: int type;
    /// Object name
    public: std::string name;
    /// Object 3D world pose
    public: ignition::math::Pose3d pose;
    /// Object scale vector
    public: ignition::math::Vector3d scale;
    /// Object parameter values
    public: std::vector<double> parameters;
    /// Object surface 3D points
    public: std::vector<Eigen::Vector4f> points;
    /// Object 2D bounding box
    public: std::vector<int> bounding_box;

    // Private attributes

    /// Sample 3D points on object surface
    private: const double ANGLE_STEP_C = 30.0;
    /// Sample 3D points on object surface
    private: const double TOTAL_STEPS_C = 360.0 / ANGLE_STEP_C;
    /// Sample 3D points on object surface
    private: const double ANGLE_STEP_S = 30.0;
    /// Sample 3D points on object surface
    private: const double TOTAL_STEPS_S = 360.0 / ANGLE_STEP_S;

    /// \brief Constructor
    /// \param _type        Object type
    /// \param _name        Object name
    /// \param _pose        Object 3D world pose
    /// \param _scale       Object scale vector
    /// \param _parameters  Object parameter values
    public: Object(
        int & _type,
        const std::string & _name,
        const ignition::math::Pose3d & _pose,
        const ignition::math::Vector3d & _scale,
        const std::vector<double> & _parameters
    );

    /// \brief Sample 3D points on object surface
    public: void sampleSurface();

};

/// \brief Object 2D grid
class ObjectGrid
{
    // Public attributes

    /// Array of grid cells
    public: std::vector<int> cells;
    /// Size of x dimension
    public: float grid_x;
    /// Size of y dimension
    public: float grid_y;
    /// Number of cells in x dimension
    public: int num_cells_x;
    /// Number of cells in y dimension
    public: int num_cells_y;
    /// Size of each cell in x dimension
    public: float cell_x;
    /// Size of each cell in y dimension
    public: float cell_y;
    /// Height of each cell
    public: float cell_z;
    /// List of objects in grid
    public: std::vector<Object> objects;
    /// Array of counters, one per object type
    public: int counters[3] = {0};

    /// Object types string vector
    public: const std::vector<std::string> TYPES = {"sphere", "cylinder","box"};

    /// \brief Constructor
    /// \param num_x    Number of cells in x dimension
    /// \param num_y    Number of cells in y dimension
    /// \param size_x   Size of grid in x dimension
    /// \param size_y   Size of grid in x dimension
    /// \param size_z   Height of grid cells (z dimension)
    public: ObjectGrid(
        int num_x,
        int num_y,
        float size_x,
        float size_y,
        float size_z);

    /// \brief Populates a grid with random objects
    /// \param num_objects Desired number of objects
    public: void populate(int num_objects);

    /// \brief Adds a single random object to grid in a given cell
    /// \param x X coordinate of cell
    /// \param y Y coordinate of cell
    private: void addRandomObject(int x, int y);

};
