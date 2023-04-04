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
    \file utils/gz_debug.hh
    \brief Gazebo debug utilities

    \warning Should only be included in Gazebo plugins!

    \author João Borrego : jsbruglie
*/

#ifndef _GZ_DEBUG_HH_
#define _GZ_DEBUG_HH_

/// Returns if expression evaluates to false
#define NULL_CHECK(_expr, _msg) if (!_expr) \
{                                           \
    gzdbg << _msg << std::endl;             \
    return;                                 \
}

#endif