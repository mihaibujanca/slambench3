#ifndef CONFUSIONMATRIXMETRIC_H
#define CONFUSIONMATRIXMETRIC_H

#include "metrics/Metric.h"
#include "metrics/SemanticMetric.h"
#include "outputs/Output.h"

namespace slambench {
    namespace metrics {

        using slambench::values::LabelledFrameValue;

        class ConfusionMatrixMetric : public Metric {
            private:
                const slambench::outputs::BaseOutput * const tested_segmentation;
                const slambench::outputs::BaseOutput * const ground_truth;

                void printConfusionMatrix(const cv::Mat &confusion,
                                          const std::map<int, std::string> &gt_map,
                                          const std::map<int, std::string> &pred_map);

                cv::Mat getConfusionMatrix(const cv::Mat &pred, const cv::Mat &gt);

                double classAccuracy(const cv::Mat &confusion, int classNo);
                double classRecall(const cv::Mat &confusion, int classNo);
                double findAverageClassAccuracy(const cv::Mat &confusion);
                double findAverageClassRecall(const cv::Mat &confusion);

            public:
                ConfusionMatrixMetric(const slambench::outputs::BaseOutput * const tested_segmentation,
                                      const slambench::outputs::BaseOutput * const ground_truth,
                                      const std::map<int, int> &map);

                ConfusionMatrixMetric(const slambench::outputs::BaseOutput * const tested_segmentation,
                                      const slambench::outputs::BaseOutput * const ground_truth);

                ConfusionMatrixMetric();


                ~ConfusionMatrixMetric() = default;

                const slambench::values::ValueDescription& GetValueDescription() const override;
                const std::string& GetDescription() const override;

                void MeasureStart(Phase* phase) override;
                void MeasureEnd(Phase* phase) override;

                Value *GetValue(Phase* phase) override;

                // Maps GT classes to algorithm classes
                const std::map<int, int> dictionary;
        };
    }
}


#endif
