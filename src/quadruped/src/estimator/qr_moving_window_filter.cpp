#include "estimator/qr_moving_window_filter.h"

qrMovingWindowFilter::qrMovingWindowFilter()
{
  this->windowSize = DEFAULT_WINDOW_SIZE;
  this->sum        = 0.0f;
  this->correction = 0.0f;
}

qrMovingWindowFilter::qrMovingWindowFilter(unsigned int windowSize)
{
  this->windowSize = windowSize;
  sum = 0.0f;
  correction = 0.0f;
}

void qrMovingWindowFilter::NeumaierSum(const float &value)
{
  float newSum = sum + value;
  if (std::abs(sum) >= std::abs(value)) {
      correction += (sum - newSum) + value;

  } else {
      correction += (value - newSum) + sum;
  }
  sum = newSum;
}

float qrMovingWindowFilter::Average(const float &value)
{
  size_t dequeLen = values.size();
  if (dequeLen >= windowSize) {
      NeumaierSum(-values[0]);
      values.pop_front();
  }
  NeumaierSum(value);
  values.push_back(value);
  return Average();
}
