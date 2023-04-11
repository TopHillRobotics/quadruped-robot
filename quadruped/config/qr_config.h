// The MIT License

// Copyright (c) 2022
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef QR_CONFIG_H
#define QR_CONFIG_H

#define A1_BODY_MASS 10
#define Go1_BODY_MASS 10
#define DEFAULT_WINDOW_SIZE 20
#define A1_BODY_HIGHT 0.27  // for robot position init
#define NumLeg 4
#define NumMotor 12
#define BaseFreedomDim 6
#define MAX_TIME_SECONDS 1000.0f
#define JOY_CMD_VELX_MAX 0.3
#define JOY_CMD_VELY_MAX 0.3
#define JOY_CMD_YAW_MAX 0.3
#define JOY_CMD_ROLL_MAX 0.1
#define JOY_CMD_PITCH_MAX 0.1

const int NumMotorOfOneLeg = 3;
const float MAXIMUM_STEP = 0.001f;

#endif //QR_CONFIG_H
