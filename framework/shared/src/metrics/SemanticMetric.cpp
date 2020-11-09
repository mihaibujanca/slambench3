#include "metrics/SemanticMetric.h"

using slambench::metrics::SemanticMetric;

SemanticMetric::SemanticMetric(const slambench::values::LabelledFrameValue* gt,
                               const slambench::values::LabelledFrameValue* pred,
                               const std::map<int, int> &dictionary) :

    dictionary(dictionary),
    gt_mat(gt->GetHeight(), gt->GetWidth(), CV_16UC1,
           const_cast<ushort*>(reinterpret_cast<const ushort*>(gt->GetData()))),
    pred_mat(pred->GetHeight(), pred->GetWidth(), CV_16UC1,
             const_cast<ushort*>(reinterpret_cast<const ushort*>(pred->GetData())))
{
    const std::map<int, std::string> &gt_map = gt->GetMap();
    const std::map<int, std::string> &pred_map = pred->GetMap();

    std::map<int, std::string> translated_map = gt_map;
    for (const auto &entry : pred_map) {
            translated_map[entry.first] = entry.second;
    }


    cv::resize(pred_mat, resized_pred_mat, gt_mat.size());

    translated_mat = translate(gt_mat, dictionary);
}

cv::Mat SemanticMetric::getTranslatedMap() const {
    return translated_mat;
}


cv::Mat SemanticMetric::translate(const cv::Mat &gt, const std::map<int, int> &dictionary) {
    cv::Mat translated(gt.size(), CV_16UC1);

    for (int row = 0; row < gt.rows; row++) {
        for (int col = 0; col < gt.cols; col++) {
            auto pixel = gt.at<ushort>(row, col);
            auto search = dictionary.find(pixel);
            if (search != dictionary.end())
                translated.at<ushort>(row, col) = search->second;
            else
                translated.at<ushort>(row, col) = pixel;
        }
    }

    return translated;
}

