#ifndef SEMANTICPIXELMETRIC_H
#define SEMANTICPIXELMETRIC_H

#include "metrics/Metric.h"
#include "metrics/SemanticMetric.h"
#include "outputs/Output.h"

namespace slambench {
    namespace metrics {

        using slambench::values::LabelledFrameValue;

        class SemanticPixelMetric : public Metric {
            private:
                const slambench::outputs::BaseOutput * const tested_segmentation;
                const slambench::outputs::BaseOutput * const ground_truth;

                enum MatchStatus { UNKNOWN, FAILED, MATCHED};

                cv::Mat_<MatchStatus> getMatched(const cv::Mat_<ushort> &pred,
                                                 const cv::Mat_<ushort> &gt);

                template <typename PixelType>
                MatchStatus match(const PixelType &classified, const PixelType &translated_ground_truth) {

                    if (translated_ground_truth == 0)
                        return UNKNOWN;

                    if (classified == translated_ground_truth)
                        return MATCHED;

                    return FAILED;
                }

            public:
                SemanticPixelMetric(const slambench::outputs::BaseOutput * const tested_segmentation,
                                    const slambench::outputs::BaseOutput * const ground_truth,
                                    const std::map<int, int> &map);

                SemanticPixelMetric(const slambench::outputs::BaseOutput * const tested_segmentation,
                                    const slambench::outputs::BaseOutput * const ground_truth);

                ~SemanticPixelMetric() = default;

                void viewMatches(const cv::Mat &mat);

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
