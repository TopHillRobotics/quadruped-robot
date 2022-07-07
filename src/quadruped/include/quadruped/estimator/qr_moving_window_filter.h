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

#ifndef QR_MOVING_WINDOW_FILTER_H
#define QR_MOVING_WINDOW_FILTER_H

#include <deque>
#include <cmath>

#define DEFAULT_WINDOW_SIZE 120;

/**
 * @brief The qrMovingWindowFilter class implements a filter based on moving window
 */
class qrMovingWindowFilter{

public:
  /**
   * @brief constructor of qrMovingWindowFilter
   */
  qrMovingWindowFilter();

  /**
   * @brief constructor of qrMovingWindowFilter
   * @param windowSize: size of moving window
   */
  qrMovingWindowFilter(unsigned int windowSize);

  /**
   * @brief destructor of qrMovingWindowFilter
   */
  ~qrMovingWindowFilter();

  // This need to be checked
  /**
   * @brief update the moving window sum using Neumaier's algorithm.
   * @see https://en.wikipedia.org/wiki/Kahan_summation_algorithm#Further_enhancements
   * @param value: new value to be added to the window
   */
  void NeumaierSum(const float &value);

  /**
   * @brief calculate average in moving window
   * @return average of value in window
   */
  float inline Average(){
    return (sum + correction) / windowSize;
  }

  /**
   * @brief calculate average in moving window with new value
   * @param value: new value to be added in the deque
   * @return
   */
  float Average(const float &value);
  
private:

  /**
   * @brief window size pf moving window
   */
  unsigned int windowSize;

  /**
   * @brief summation in moving window
   */
  float sum;

  /**
   * @brief correction term to compensate numerical precision loss during calculation.
   */
  float correction;

  /**
   * @brief queue that saves the data
   */
  std::deque<float> values;
};

#endif // QR_MOVING_WINDOW_FILTER_H
