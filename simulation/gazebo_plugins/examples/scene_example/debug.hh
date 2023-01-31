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
    \file examples/scene_example/debug.hh
    \brief Debug utilities

    \author João Borrego : jsbruglie
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

#include <iostream>
#include <string.h>

/// Verbose flag. Comment to remove debug features
#define VERBOSE

/// Current filename macro
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#ifdef VERBOSE
/// Print debug information to std::cout
#define debugPrint(x) do { std::cout << x; } while (0)
#else
#define debugPrint(x) do {} while (0)
#endif

#ifdef VERBOSE
/// Print debug information to std::cout, including current filename, line and function
#define debugPrintTrace(x) do {\
    std::cout << __FILENAME__ << ":" << __LINE__ << ":" << __func__ << ": " << \
    x << "\n";\
} while (0)
#else
#define debugPrintTrace(x) do {} while (0)
#endif
