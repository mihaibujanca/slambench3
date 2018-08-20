#include "metrics/PointCloudMetric.h"
#include <pcl/registration/icp.h>
#include <pcl/io/ply_io.h>
#include "io/format/PointCloud.h"

using slambench::values::PointCloudValue;
using namespace slambench::metrics;
typedef  pcl::PointXYZ point_t;

PointCloudMetric::PointCloudMetric(const slambench::outputs::BaseOutput * const tested,
                                   const slambench::outputs::BaseOutput * const gt) :
    Metric("PointCloud_Metric"), tested(tested), gt(gt), lastMeasurement{0,0}
{ }

const slambench::values::ValueDescription &PointCloudMetric::GetValueDescription() const {
	static const slambench::values::ValueDescription desc = slambench::values::ValueDescription({
		{"PointCloud_Metric", slambench::values::VT_DOUBLE}});
	return desc;
}

const std::string& PointCloudMetric::GetDescription() const {
	static std::string desc = "Point Cloud Reconstruction Accuracy";
	return desc;
}

void PointCloudMetric::MeasureStart(Phase* /* unused */) { }

void PointCloudMetric::MeasureEnd(Phase* /* unused */) { }

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

void writepointcloud(const std::string &filename, const PointCloudValue *pcv) {
    auto points = pcv->GetPoints();
    std::ofstream file(filename);
    file << "ply" << std::endl;
    file << "format binary_little_endian 1.0" << std::endl;
    file << "element vertex " << points.size() << std::endl;
    file << "property float x" << std::endl;
    file << "property float y" << std::endl;
    file << "property float z" << std::endl;
    file << "end_header" << std::endl;

    for (const auto &point : points) {
        struct { float x, y, z; } data { point.X, point.Y, point.Z };
        file.write((char*)&data, sizeof(data));
    }
}

void writepointcloud(const std::string &filename, const pcl::PointCloud<point_t>::Ptr &pcv) {
    std::ofstream file(filename);
    file << "ply" << std::endl;
    file << "format binary_little_endian 1.0" << std::endl;
    file << "element vertex " << pcv.get()->size() << std::endl;
    file << "property float x" << std::endl;
    file << "property float y" << std::endl;
    file << "property float z" << std::endl;
    file << "end_header" << std::endl;

    for (const auto &point : *(pcv.get())) {
        struct { float x, y, z; } data { point.x, point.y, point.z };
        file.write((char*)&data, sizeof(data));
    }
}

constexpr int COMPUTE_EVERY_FRAMES = 20;

Value *PointCloudMetric::GetValue(Phase* /* unused */) {


    if (tested->Empty())
        return new slambench::values::TypeForVT<slambench::values::VT_DOUBLE>::type(std::nan(""));

    const outputs::Output::value_map_t::value_type tested_frame = tested->GetMostRecentValue();
    const outputs::Output::value_map_t::value_type gt_frame = gt->GetMostRecentValue();

    const slambench::TimeStamp currentTimestamp = tested_frame.first;

    const auto diff = currentTimestamp - lastMeasurement;

    if (diff.count() < 120000000)
        return new slambench::values::TypeForVT<slambench::values::VT_DOUBLE>::type(std::nan(""));

    lastMeasurement = tested_frame.first;

    const PointCloudValue *tested_pointcloud = reinterpret_cast<const PointCloudValue*>(tested_frame.second);
    const PointCloudValue *gt_pointcloud = reinterpret_cast<const PointCloudValue*>(gt_frame.second);

    pcl::PointCloud<point_t>::Ptr tested_cloud = pcl::PointCloud<point_t>::Ptr(new pcl::PointCloud<point_t>);
    writepointcloud("originaltested.ply", tested_pointcloud);

    for (const auto &point : tested_pointcloud->GetPoints()) {
        const Eigen::Matrix4f &transform = tested_pointcloud->GetTransform();
        const Eigen::Vector4f eigenPoint(point.X, point.Y, point.Z, 1);
        const Eigen::Vector4f transformedPoint = transform * eigenPoint;

        tested_cloud->push_back({transformedPoint(0), transformedPoint(1), transformedPoint(2)});
    }

    pcl::PointCloud<point_t>::Ptr gt_cloud = pcl::PointCloud<point_t>::Ptr(new pcl::PointCloud<point_t>);
    for (const auto &point : gt_pointcloud->GetPoints()) {
        gt_cloud->push_back({point.X, point.Y, point.Z});
    }

    writepointcloud("gt.ply", gt_pointcloud);
    writepointcloud("tested.ply", tested_cloud);


    float value = getScorre(gt_cloud, tested_cloud);

    return new slambench::values::TypeForVT<slambench::values::VT_DOUBLE>::type(value);

}

