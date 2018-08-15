#include "metrics/SemanticPixelMetric.h"


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


using namespace slambench::metrics;

SemanticPixelMetric::SemanticPixelMetric(const slambench::outputs::BaseOutput * const tested_segmentation,
                                         const slambench::outputs::BaseOutput * const ground_truth) :
    SemanticPixelMetric(tested_segmentation, ground_truth, std::map<int, int>())
{ }

SemanticPixelMetric::SemanticPixelMetric(const slambench::outputs::BaseOutput * const tested_segmentation,
                                         const slambench::outputs::BaseOutput * const ground_truth,
                                         const std::map<int, int> &dictionary) :
    Metric("Pixel_Accuracy"), tested_segmentation(tested_segmentation), ground_truth(ground_truth),
    dictionary(dictionary)
{ }


const slambench::values::ValueDescription &SemanticPixelMetric::GetValueDescription() const {
	static const slambench::values::ValueDescription desc = slambench::values::ValueDescription({
		{"Accuracy", slambench::values::VT_DOUBLE}});
	return  desc;
}

const std::string& SemanticPixelMetric::GetDescription() const {
	static std::string desc = "Pixel Segmentation Accuracy";
	return desc;
}

void SemanticPixelMetric::MeasureStart(Phase* /* unused */) { }

void SemanticPixelMetric::MeasureEnd(Phase* /* unused */) { }

cv::Mat_<SemanticPixelMetric::MatchStatus> SemanticPixelMetric::getMatched(const cv::Mat_<ushort> &pred,
                                                                           const cv::Mat_<ushort> &gt) {
    assert(pred.size() == gt.size());

    cv::Mat_<SemanticPixelMetric::MatchStatus> matches(pred.size());

    for (int row = 0; row < gt.rows; row++) {
        for (int col = 0; col < gt.cols; col++) {
            matches(row, col) = match(pred(row, col), gt(row, col));
        }
    }

    return matches;
}

void SemanticPixelMetric::viewMatches(const cv::Mat &matches) {
    cv::Mat viz(matches.size(), CV_8UC1);

    for (int row = 0; row < matches.rows; row++) {
        for (int col = 0; col < matches.cols; col++) {
            const auto value = matches.at<MatchStatus>(row, col);
            auto &target = viz.at<uchar>(row, col);
            if (value == UNKNOWN)
                target = 128;
            else if (value == FAILED)
                target = 0;
            else if (value == MATCHED)
                target = 255;
        }
    }

    cv::namedWindow("win");
    cv::imshow("win", viz);
    cv::waitKey(0);
}


Value *SemanticPixelMetric::GetValue(Phase* /* unused */) {
    const outputs::Output::value_map_t::value_type tested_segmented_frame = tested_segmentation->GetMostRecentValue();

    const outputs::BaseOutput::timestamp_t timestamp = tested_segmented_frame.first;
    
    outputs::Output::value_map_t gt_segmented_frames = ground_truth->GetValues();


    auto gt_entry = gt_segmented_frames.find(timestamp);
    if (gt_entry == gt_segmented_frames.end()) {
        return new slambench::values::TypeForVT<slambench::values::VT_DOUBLE>::type(std::nan(""));
    } else {

        const auto gt_value = reinterpret_cast<const LabelledFrameValue*>(gt_entry->second);
        const auto pred_value = reinterpret_cast<const LabelledFrameValue*>(tested_segmented_frame.second);

        SemanticMetric sm(gt_value, pred_value, dictionary);

        cv::Mat translated = sm.getTranslatedMap();

        cv::Mat_<MatchStatus> matches = getMatched(sm.getPred(), sm.getTranslatedMap());

        int totalPixels = 0;
        int correctPixels = 0;

        for (int row = 0; row < matches.rows; row++)
            for (int col = 0; col < matches.cols; col++) {
                auto px = matches.at<MatchStatus>(row, col);
                if (px == MATCHED)
                    correctPixels++;
                if (px != UNKNOWN)
                    totalPixels++;
            }

        //viewMatches(matches);



        double prob = correctPixels / (double)(totalPixels) * 100;

        return new slambench::values::TypeForVT<slambench::values::VT_DOUBLE>::type(prob);
        return new slambench::values::TypeForVT<slambench::values::VT_DOUBLE>::type(0);
    }

}


