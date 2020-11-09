#ifndef SEMANTICMETRIC_H
#define SEMANTICMETRIC_H

#include "MetricValue.h"
#include "values/Value.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <iomanip>

namespace slambench {
        namespace metrics {

                // Helper for semantic metrics
                class SemanticMetric {
                    public:
                        SemanticMetric(const slambench::values::LabelledFrameValue* gt,
                                       const slambench::values::LabelledFrameValue* pred,
                                       const std::map<int, int> &dictionary);

                        virtual ~SemanticMetric() = default;

                        cv::Mat getTranslatedMap() const;

                        cv::Mat getPred() const {
                            return resized_pred_mat;
                        }

                        cv::Mat getGT() const {
                            return gt_mat;
                        }

                    private:
                        const std::map<int, int> dictionary;
                        const std::map<int, std::string> translated_map;

                        const cv::Mat gt_mat;
                        const cv::Mat pred_mat;
                        cv::Mat resized_pred_mat;
                        cv::Mat translated_mat;

                        cv::Mat translate(const cv::Mat &gt, const std::map<int, int> &dictionary);

                        void printConfusionMatrix(const cv::Mat &confusion,
                                                  const std::map<int, std::string> &gt_map,
                                                  const std::map<int, std::string> &pred_map);

                        cv::Mat getConfusionMatrix(const cv::Mat &pred, const cv::Mat &gt);

                };
        }
}

#endif
