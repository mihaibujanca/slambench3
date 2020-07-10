#include "metrics/ConfusionMatrixMetric.h"
#include <set>

using namespace slambench::metrics;
using namespace slambench::values;

ConfusionMatrixMetric::ConfusionMatrixMetric(const slambench::outputs::BaseOutput * const tested_segmentation,
                                             const slambench::outputs::BaseOutput * const ground_truth) :
    ConfusionMatrixMetric(tested_segmentation, ground_truth, std::map<int, int>())
{ }

ConfusionMatrixMetric::ConfusionMatrixMetric(const slambench::outputs::BaseOutput * const tested_segmentation,
                                             const slambench::outputs::BaseOutput * const ground_truth,
                                             const std::map<int, int> &dictionary) :
    Metric("Confusion_Matrix"), tested_segmentation(tested_segmentation), ground_truth(ground_truth),
    dictionary(dictionary)
{ }


const slambench::values::ValueDescription &ConfusionMatrixMetric::GetValueDescription() const {
	static const slambench::values::ValueDescription desc = slambench::values::ValueDescription({
		{"AverageAccuracy", slambench::values::VT_DOUBLE},
		{"AverageRecall", slambench::values::VT_DOUBLE}});
	return  desc;
}

const std::string& ConfusionMatrixMetric::GetDescription() const {
	static std::string desc = "Class Determination Accuracy";
	return desc;
}

void ConfusionMatrixMetric::MeasureStart(Phase* /* unused */) { }

void ConfusionMatrixMetric::MeasureEnd(Phase* /* unused */) { }

void ConfusionMatrixMetric::printConfusionMatrix(const cv::Mat &confusion,
                                                 const std::map<int, std::string> &gt_map,
                                                 const std::map<int, std::string> &pred_map) {

    std::stringstream str;

    str << std::setw(25) << " ";
    for (int i = 0; i < confusion.cols; i++) {
        str << std::setw(12) << pred_map.at(i) <<  "(" << std::setw(2) << i << ")";
    }
    str << std::endl;

    for (int row = 0; row < confusion.rows; row++) {
        bool print = false;
        int sum = 0;
        for (int col = 0; col < confusion.cols; col++) {
            auto value = confusion.at<ushort>(row, col);
            if (value != 0)
                print = true;
            sum += value;
        }
        if (print) {
            std::stringstream class_name;
            try {
                class_name << gt_map.at(row);
            } catch (std::out_of_range &e) {
                class_name << "unknown";
            }
            class_name << "(" << std::setw(2) << row << ")";
            str << std::setw(25) << class_name.str() << " ";

            for (int col = 0; col < confusion.cols; col++) {
                str << std::setw(15) << confusion.at<ushort>(row, col) << " ";
            }
            str << std::endl;
        }
    }

    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << str.str();
    std::cout << std::endl;
    std::cout << std::endl;
}

int count_unique(const cv::Mat &pred) {
    std::set<int> counter;

    for (int row = 0; row < pred.rows; row++) {
        for (int col = 0; col < pred.cols; col++) {
            const auto val = pred.at<ushort>(row, col);
            if (val)
                counter.insert(val);
        }
    }

    std::cout << counter.size() << " classes" << std::endl;

    return counter.size();
}

cv::Mat ConfusionMatrixMetric::getConfusionMatrix(const cv::Mat &pred, const cv::Mat &gt) {

    double gt_min, gt_max;
    cv::minMaxLoc(gt, &gt_min, &gt_max);

    double pred_min, pred_max;
    cv::minMaxLoc(pred, &pred_min, &pred_max);

    cv::Mat confusion((int)gt_max + 1, (int)pred_max + 1, CV_16UC1, cv::Scalar(0));

    for (int row = 0; row < pred.rows; row++) {
        for (int col = 0; col < pred.cols; col++) {
            const auto gt_pixel = gt.at<ushort>(row, col);
            const auto pred_pixel = pred.at<ushort>(row, col);
            confusion.at<ushort>(gt_pixel, pred_pixel) += 1;
        }
    }

    return confusion;
}

double ConfusionMatrixMetric::classAccuracy(const cv::Mat &confusion, int classNo) {
    int sum = 0;
    for (int i = 1; i < confusion.rows; i++)
        sum += confusion.at<ushort>(classNo, i);
    const double ret = (double)confusion.at<ushort>(classNo, classNo) / sum;
    std::cout << "Accuracy of class " << classNo << " " << ret << std::endl;
    return ret;
}

double ConfusionMatrixMetric::classRecall(const cv::Mat &confusion, int classNo) {
    int sum = 0;
    for (int i = 1; i < confusion.cols; i++)
        sum += confusion.at<ushort>(i, classNo);
    const double ret = (double)confusion.at<ushort>(classNo, classNo) / sum;
    std::cout << "Recall of class " << classNo << " " << ret << std::endl;
    return ret;
}

double ConfusionMatrixMetric::findAverageClassAccuracy(const cv::Mat &confusion) {
    int noOfClasses = 0;
    double sum = 0;
    for (int i = 1; i < confusion.cols; i++) {
        const auto acc = classAccuracy(confusion, i);
        if (!std::isnan(acc)) {
            sum += acc;
            noOfClasses++;
        }
    }
    return sum / noOfClasses * 100;
}

double ConfusionMatrixMetric::findAverageClassRecall(const cv::Mat &confusion) {
    int noOfClasses = 0;
    double sum = 0;
    for (int i = 0; i < confusion.cols; i++) {
        const auto rec = classRecall(confusion, i);
        if (!std::isnan(rec)) {
            sum += rec;
            noOfClasses++;
        }
    }
    return sum / noOfClasses * 100;
}

Value *ConfusionMatrixMetric::GetValue(Phase* /* unused */) {
    const outputs::Output::value_map_t::value_type tested_segmented_frame = tested_segmentation->GetMostRecentValue();

    const outputs::BaseOutput::timestamp_t timestamp = tested_segmented_frame.first;
    
    outputs::Output::value_map_t gt_segmented_frames = ground_truth->GetValues();

    double averageClassAccuracy = std::nan("");
    double averageClassRecall = std::nan("");

    auto gt_entry = gt_segmented_frames.find(timestamp);
    if (gt_entry != gt_segmented_frames.end()) {

        const auto gt_value = reinterpret_cast<const LabelledFrameValue*>(gt_entry->second);
        const auto pred_value = reinterpret_cast<const LabelledFrameValue*>(tested_segmented_frame.second);
        SemanticMetric sm(gt_value, pred_value, dictionary);

        const cv::Mat &pred = sm.getPred();
        const cv::Mat &gt = sm.getGT();
        const cv::Mat &translated = sm.getTranslatedMap();

        const std::map<int, std::string> &gt_map = gt_value->GetMap();
        const std::map<int, std::string> &pred_map = pred_value->GetMap();

        std::map<int, std::string> translated_map = gt_value->GetMap();
        for (const auto &entry : pred_map)
            translated_map[entry.first] = entry.second;

        cv::Mat confusion, confusionTranslated;
        try {
            confusion = getConfusionMatrix(pred, gt);
            confusionTranslated = getConfusionMatrix(pred, translated);
        } catch (const std::out_of_range &e) {
            std::cout << "Exception thrown during confusion generation: " << e.what() << std::endl;
        }

        try {
            printConfusionMatrix(confusion, gt_map, pred_map);
        } catch (const std::out_of_range &e) {
            std::cout << "Exception thrown during confusion display 1: " << e.what() << std::endl;
        }

        try {
            printConfusionMatrix(confusionTranslated, translated_map, pred_map);
        } catch (const std::out_of_range &e) {
            std::cout << "Exception thrown during confusion display 2: " << e.what() << std::endl;
        }


        averageClassAccuracy = findAverageClassAccuracy(confusionTranslated);
        averageClassRecall   = findAverageClassRecall(confusionTranslated);
    }

    const auto accuracyVal = new TypeForVT<VT_DOUBLE>::type(averageClassAccuracy);
    const auto recallVal   = new TypeForVT<VT_DOUBLE>::type(averageClassRecall);

    return new slambench::values::TypeForVT<slambench::values::VT_COLLECTION>::type({
                {"AverageAccuracy", accuracyVal},
                {"AverageRecall", recallVal}});
}


