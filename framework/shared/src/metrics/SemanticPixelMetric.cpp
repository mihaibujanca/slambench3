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

SemanticPixelMetric::MatchStatusMatrix SemanticPixelMetric::getMatched(const cv::Mat_<ushort> &pred,
                                                                       const cv::Mat_<ushort> &gt) {
    assert(pred.size() == gt.size());

    std::cout << pred.size() << std::endl;

    SemanticPixelMetric::MatchStatusMatrix matches(pred.rows, std::vector<MatchStatus>(pred.cols));

    for (int row = 0; row < gt.rows; row++) {
        for (int col = 0; col < gt.cols; col++) {
            matches[row][col] = match(pred(row, col), gt(row, col));
        }
    }

    return matches;
}

void SemanticPixelMetric::viewMatches(const SemanticPixelMetric::MatchStatusMatrix &matches) {

    cv::Mat viz(matches.size(), matches.at(0).size(), CV_8UC1);
    for(int i=0; i<viz.rows; ++i)
         for(int j=0; j<viz.cols; ++j)
               viz.at<uchar>(i, j) = static_cast<uchar>(matches.at(i).at(j));

    for (size_t row = 0; row < matches.size(); row++) {
        for (size_t col = 0; col < matches[row].size(); col++) {
            const auto value = matches.at(row).at(col);
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

        cv::namedWindow("pred");
        cv::namedWindow("translated");
        cv::namedWindow("gt");
        cv::imshow("pred", sm.getPred() * 1000);
        cv::imshow("translated", sm.getTranslatedMap() * 1000);
        cv::imshow("gt", sm.getGT() * 1000);

        const auto matches = getMatched(sm.getPred(), sm.getTranslatedMap());

        int totalPixels = 0;
        int correctPixels = 0;

        for (const auto &row : matches)
            for (const auto &px : row) {
                if (px == MATCHED)
                    correctPixels++;
                if (px != UNKNOWN)
                    totalPixels++;
            }

        viewMatches(matches);

        double prob = correctPixels / (double)(totalPixels) * 100;

        return new slambench::values::TypeForVT<slambench::values::VT_DOUBLE>::type(prob);
        return new slambench::values::TypeForVT<slambench::values::VT_DOUBLE>::type(0);
    }

}


