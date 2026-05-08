#ifndef ALGORITHM_FEEDBACK_ADAPTER__METRIC_SAMPLE_HPP_
#define ALGORITHM_FEEDBACK_ADAPTER__METRIC_SAMPLE_HPP_

#include <string>

namespace algorithm_feedback_adapter
{

struct MetricSample
{
  std::string name;
  double value;
  bool valid;
};

}  // namespace algorithm_feedback_adapter

#endif  // ALGORITHM_FEEDBACK_ADAPTER__METRIC_SAMPLE_HPP_
