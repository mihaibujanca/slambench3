#include "metrics/SemanticPointcloudMetric.h"
#include <pcl/registration/icp.h>
#include <pcl/io/ply_io.h>
#include "io/format/PointCloud.h"

using slambench::values::SemanticPointCloudValue;
using namespace slambench::metrics;
typedef  pcl::PointXYZRGB point_t;

SemanticPointCloudMetric::SemanticPointCloudMetric(const slambench::outputs::BaseOutput * const tested,
                                   const slambench::outputs::BaseOutput * const gt) :
    Metric("SemanticPointCloud_Metric"), tested(tested), gt(gt), lastMeasurement{0,0}
{ }

const slambench::values::ValueDescription &SemanticPointCloudMetric::GetValueDescription() const {
	static const slambench::values::ValueDescription desc = slambench::values::ValueDescription({
		{"SemanticPointCloud_Metric", slambench::values::VT_DOUBLE}});
	return desc;
}

const std::string& SemanticPointCloudMetric::GetDescription() const {
	static std::string desc = "Point Cloud Labelling Accuracy";
	return desc;
}

void SemanticPointCloudMetric::MeasureStart(Phase* /* unused */) { }

void SemanticPointCloudMetric::MeasureEnd(Phase* /* unused */) { }

float getScorre(const pcl::PointCloud<point_t>::Ptr &gt,
                const pcl::PointCloud<point_t>::Ptr &test) {

    pcl::search::KdTree<point_t>::Ptr tree (new pcl::search::KdTree<point_t>);
    tree->setInputCloud(gt);

    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);

    double totalSum = 0;

    for(size_t i = 0; i < test->size(); i++) {
        tree->nearestKSearch(test->at(i), 1, pointIdxNKNSearch, pointNKNSquaredDistance);
        totalSum += sqrt(pointNKNSquaredDistance.at(0));
    }
    return totalSum / (double)test->size();
}

void writeplyheaders(std::ofstream &file, size_t noOfPoints) {
    file << "ply" << std::endl;
    file << "format binary_little_endian 1.0" << std::endl;
    file << "element vertex " << noOfPoints << std::endl;
    file << "property float x" << std::endl;
    file << "property float y" << std::endl;
    file << "property float z" << std::endl;
    file << "property uchar red" << std::endl;
    file << "property uchar green" << std::endl;
    file << "property uchar blue" << std::endl;
    file << "property uchar alpha" << std::endl;
    file << "end_header" << std::endl;
}

void writepointcloud(const std::string &filename, const SemanticPointCloudValue *pcv) {
    auto points = pcv->GetPoints();
    std::ofstream file(filename);

    writeplyheaders(file, points.size());

    for (const auto &point : points) {
        struct { float x, y, z; } data { point.X, point.Y, point.Z };
        file.write((char*)&data, sizeof(data));
    }

    for (const auto &point : points) {
        struct { uint8_t r, g, b, a; } data { point.R, point.G, point.B, 255 };
        file.write((char*)&data, sizeof(data));
    }
}

void writepointcloud(const std::string &filename, const pcl::PointCloud<point_t>::Ptr &pcv) {
    std::ofstream file(filename);

    writeplyheaders(file, pcv.get()->size());

    for (const auto &point : *(pcv.get())) {
        struct { float x, y, z; } data { point.x, point.y, point.z };
        file.write((char*)&data, sizeof(data));
    }

    for (const auto &point : *(pcv.get())) {
        struct { uint8_t r, g, b, a; } data { point.r, point.g, point.b, 255};
        file.write((char*)&data, sizeof(data));
    }
}

Value *SemanticPointCloudMetric::GetValue(Phase* /* unused */) {

    if (tested->Empty())
        return new slambench::values::TypeForVT<slambench::values::VT_DOUBLE>::type(std::nan(""));

    const outputs::Output::value_map_t::value_type tested_frame = tested->GetMostRecentValue();
    const outputs::Output::value_map_t::value_type gt_frame = gt->GetMostRecentValue();

    const slambench::TimeStamp currentTimestamp = tested_frame.first;

    const auto diff = currentTimestamp - lastMeasurement;

    if (diff.count() < 120000000)
        return new slambench::values::TypeForVT<slambench::values::VT_DOUBLE>::type(std::nan(""));

    lastMeasurement = tested_frame.first;

    const SemanticPointCloudValue *tested_pointcloud = reinterpret_cast<const SemanticPointCloudValue*>(tested_frame.second);
    const SemanticPointCloudValue *gt_pointcloud = reinterpret_cast<const SemanticPointCloudValue*>(gt_frame.second);

    pcl::PointCloud<point_t>::Ptr tested_cloud = pcl::PointCloud<point_t>::Ptr(new pcl::PointCloud<point_t>);
    writepointcloud("originaltested.ply", tested_pointcloud);

    for (const auto &point : tested_pointcloud->GetPoints()) {
        const Eigen::Matrix4f &transform = tested_pointcloud->GetTransform();
        const Eigen::Vector4f eigenPoint(point.X, point.Y, point.Z, 1);
        const Eigen::Vector4f transformedPoint = transform * eigenPoint;

        point_t newPoint(point.R, point.G, point.B);
        newPoint.x = transformedPoint(0);
        newPoint.y = transformedPoint(1);
        newPoint.z = transformedPoint(2);

        tested_cloud->push_back(newPoint);
    }

    pcl::PointCloud<point_t>::Ptr gt_cloud = pcl::PointCloud<point_t>::Ptr(new pcl::PointCloud<point_t>);
    for (const auto &point : gt_pointcloud->GetPoints()) {
        point_t newPoint(point.R, point.G, point.B);
        newPoint.x = point.X;
        newPoint.y = point.Y;
        newPoint.z = point.Z;

        gt_cloud->push_back(newPoint);
    }

    writepointcloud("gt.ply", gt_pointcloud);
    writepointcloud("tested.ply", tested_cloud);


    float value = getScorre(gt_cloud, tested_cloud);

    return new slambench::values::TypeForVT<slambench::values::VT_DOUBLE>::type(value);

}
