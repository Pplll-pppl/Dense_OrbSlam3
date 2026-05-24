#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

struct OrderedPoint
{
    Eigen::Vector3f point = Eigen::Vector3f::Zero();
    float theta = 0.0f;
};

struct EdgeGroup
{
    Eigen::Vector2f representative = Eigen::Vector2f::Zero();
    float total_weight = 0.0f;
};

struct PlaneInfo
{
    Eigen::Vector4f plane_eq = Eigen::Vector4f::Zero();
    PointCloudT::Ptr inlier_cloud = PointCloudT::Ptr(new PointCloudT);
    std::vector<Eigen::Vector3f> points;
    float avg_x = 0.0f;
    float avg_y = 0.0f;
    float avg_z = 0.0f;
    int point_num = 0;
};

struct DimensionResult
{
    std::array<Eigen::Vector3f, 3> axes = {
        Eigen::Vector3f::UnitX(), Eigen::Vector3f::UnitY(), Eigen::Vector3f::UnitZ()};
    std::array<float, 3> axis_size = {0.0f, 0.0f, 0.0f};
    std::array<std::string, 3> source = {"plane", "plane", "plane"};
};

struct AxisCandidates
{
    std::vector<PlaneInfo> group;
    std::vector<PlaneInfo> low_planes;
    std::vector<PlaneInfo> high_planes;
    std::vector<float> candidate_distances;
    float raw_distance = 0.0f;
    float voted_distance = 0.0f;
    float spread = 0.0f;
    bool has_reliable_vote = false;
};

struct PlanePositionCluster
{
    std::vector<PlaneInfo> planes;
    float mean_position = 0.0f;
    int total_points = 0;
};


struct ProjectionExtent
{
    float low = 0.0f;
    float high = 0.0f;
    float size = 0.0f;
    bool valid = false;
};

struct SupportedExtent
{
    float low = 0.0f;
    float high = 0.0f;
    float size = 0.0f;
    int left_bin = -1;
    int right_bin = -1;
    float confidence = 0.0f;
    bool valid = false;
};

struct Frame3D
{
    Eigen::Vector3f axis0 = Eigen::Vector3f::UnitX();
    Eigen::Vector3f axis1 = Eigen::Vector3f::UnitY();
    Eigen::Vector3f axis2 = Eigen::Vector3f::UnitZ();
};

struct LayerCenterSample
{
    float z = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;
    int count = 0;
};

struct AxisCouplingSample
{
    float driver = 0.0f;
    float target = 0.0f;
    int count = 0;
};

struct ShearEstimate
{
    float kx = 0.0f;
    float ky = 0.0f;
    float bx = 0.0f;
    float by = 0.0f;
    float z_ref = 0.0f;
    bool valid_x = false;
    bool valid_y = false;
};

struct Config
{
    std::string path =
        "/home/ricky/WCR_ws/dense_orbslam3/Examples/RGB-D/PointCloudMapping_RGBD.pcd";
    std::string hull_output_path = "fast_demo_hull.pcd";
    std::string light_bar_output_path = "fast_demo_contour_after_light_bar.pcd";
    std::string simplified_output_path = "fast_demo_contour_simplified.pcd";
    float voxel_leaf = 0.008f;
    bool use_sor = true;
    int sor_mean_k = 5;
    double sor_stddev = 2;
    bool use_hull = true;
    float alpha_factor = 60.0f;
    int hull_dimension = 3;
    bool save_hull = true;
    float light_bar_factor = 3.5f;
    float angle_prune_deg = 8.0f;
    float direction_group_deg = 10.0f;
    int angular_bins = 720;
    float plane_dist_threshold = 0.08f;
    int plane_max_iter = 100000;
    int fit_plane_num = 100;
    int min_plane_points = 5;    //20
    int outer_plane_candidates = 3;
    int min_voting_plane_points = 20;
    int min_axis_plane_group = 4;
    int min_extreme_plane_points = 30;
    float min_plane_axis_alignment = 0.85f;
    float plane_position_cluster_gap = 0.12f;
    int min_plane_cluster_points = 40;
    float hull_end_ratio = 0.08f;
    int min_hull_end_points = 20;

    bool save_dense_contour = true;
    std::string dense_contour_output_path = "fast_demo_contour_dense.pcd";
    float contour_resample_spacing_factor = 1.0f;
    float projection_trim_ratio = 0.01f;
    float support_bin_size = 0.02f;
    int support_threshold = 3;
    int continuity_bins = 2;
    float shear_layer_thickness = 0.02f;
    int shear_min_points_per_layer = 20;
};
class StageTimer
{
public:
    explicit StageTimer(const std::string& label)
        : label_(label), start_(std::chrono::steady_clock::now())
    {
    }
    ~StageTimer()
    {
        const auto end = std::chrono::steady_clock::now();
        const double seconds =
            std::chrono::duration_cast<std::chrono::milliseconds>(end - start_).count() / 1000.0;
        std::cout << "[TIMER] " << label_ << ": " << std::fixed << std::setprecision(3)
                  << seconds << " s" << std::endl;
    }

private:
    std::string label_;
    std::chrono::steady_clock::time_point start_;
};

void calculatePlaneBasicStats(PlaneInfo& plane)
{
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;
    for (const auto& p : plane.points)
    {
        sum_x += p.x();
        sum_y += p.y();
        sum_z += p.z();
    }

    const float denom = static_cast<float>(plane.points.size());
    plane.avg_x = sum_x / denom;
    plane.avg_y = sum_y / denom;
    plane.avg_z = sum_z / denom;
    plane.point_num = static_cast<int>(plane.points.size());
}

PointCloudT::Ptr loadCloud(const std::string& path)
{
    StageTimer timer("load cloud");
    PointCloudT::Ptr cloud(new PointCloudT);
    const std::string ext = path.substr(path.find_last_of('.') + 1);

    int status = -1;
    if (ext == "pcd")
    {
        status = pcl::io::loadPCDFile<PointT>(path, *cloud);
    }
    else if (ext == "ply")
    {
        status = pcl::io::loadPLYFile<PointT>(path, *cloud);
    }
    else
    {
        throw std::runtime_error("Unsupported file extension: " + ext);
    }

    if (status != 0)
    {
        throw std::runtime_error("Failed to load point cloud: " + path);
    }

    std::cout << "Loaded points: " << cloud->size() << std::endl;
    return cloud;
}

PointCloudT::Ptr preprocessCloud(const PointCloudT::Ptr& input, const Config& cfg)
{
    StageTimer timer("preprocess");

    PointCloudT::Ptr cloud_down(new PointCloudT);
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(input);
    voxel.setLeafSize(cfg.voxel_leaf, cfg.voxel_leaf, cfg.voxel_leaf);
    voxel.filter(*cloud_down);
    std::cout << "After voxel grid: " << cloud_down->size() << std::endl;

    if (!cfg.use_sor)
    {
        return cloud_down;
    }

    PointCloudT::Ptr cloud_clean(new PointCloudT);
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud_down);
    sor.setMeanK(cfg.sor_mean_k);
    sor.setStddevMulThresh(cfg.sor_stddev);
    sor.filter(*cloud_clean);
    std::cout << "After SOR: " << cloud_clean->size() << std::endl;
    return cloud_clean;
}

float computeAverageSpacing(const PointCloudT::Ptr& cloud, int k = 2, int sample_ratio = 10)
{
    StageTimer timer("compute average spacing");

    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloud);

    std::vector<float> distances;
    distances.reserve(cloud->size() / sample_ratio);
    std::vector<int> indices(k);
    std::vector<float> sq_dist(k);

    for (size_t i = 0; i < cloud->size(); i += sample_ratio)
    {
        const PointT& query = cloud->points[i];
        if (kdtree.nearestKSearch(query, k, indices, sq_dist) == k)
        {
            distances.push_back(std::sqrt(sq_dist[1]));
        }
    }

    if (distances.empty())
    {
        throw std::runtime_error("Failed to estimate average spacing from the input cloud.");
    }

    const float sum = std::accumulate(distances.begin(), distances.end(), 0.0f);
    return sum / static_cast<float>(distances.size());
}

PointCloudT::Ptr extractHull(const PointCloudT::Ptr& input, const Config& cfg)
{
    if (!cfg.use_hull)
    {
        return input;
    }

    const float avg_spacing = computeAverageSpacing(input, 2, 10);
    const float alpha = cfg.alpha_factor * avg_spacing;

    StageTimer timer("concave hull");

    PointCloudT::Ptr hull_cloud(new PointCloudT);
    pcl::ConcaveHull<PointT> hull;
    hull.setInputCloud(input);
    hull.setDimension(cfg.hull_dimension);
    hull.setAlpha(alpha);
    hull.reconstruct(*hull_cloud);

    std::cout << "Average spacing: " << avg_spacing << " m, alpha = " << alpha << std::endl;
    std::cout << "Hull points: " << hull_cloud->size() << std::endl;
    if (hull_cloud->empty())
    {
        throw std::runtime_error("Concave hull extraction returned an empty cloud.");
    }

    return hull_cloud;
}

void saveHullCloud(const PointCloudT::Ptr& hull_cloud, const Config& cfg)
{
    if (!cfg.save_hull)
    {
        return;
    }

    StageTimer timer("save hull");
    const int status = pcl::io::savePCDFileBinary(cfg.hull_output_path, *hull_cloud);
    if (status != 0)
    {
        throw std::runtime_error("Failed to save hull cloud: " + cfg.hull_output_path);
    }

    std::cout << "Saved hull to: " << cfg.hull_output_path << std::endl;
}

std::array<Eigen::Vector3f, 3> estimatePointCloudPCAAxes(const PointCloudT::Ptr& cloud)
{
    if (cloud->size() < 3)
    {
        throw std::runtime_error("Need at least three points for PCA axis estimation.");
    }

    Eigen::Vector4f centroid4 = Eigen::Vector4f::Zero();
    pcl::compute3DCentroid(*cloud, centroid4);
    const Eigen::Vector3f centroid = centroid4.head<3>();

    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for (const auto& p : cloud->points)
    {
        const Eigen::Vector3f delta(p.x - centroid.x(), p.y - centroid.y(), p.z - centroid.z());
        cov += delta * delta.transpose();
    }
    cov /= static_cast<float>(cloud->size());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
    std::array<Eigen::Vector3f, 3> axes = {
        solver.eigenvectors().col(2), solver.eigenvectors().col(1), solver.eigenvectors().col(0)};

    if (axes[0].cross(axes[1]).dot(axes[2]) < 0.0f)
    {
        axes[2] = -axes[2];
    }

    return axes;
}

std::vector<PlaneInfo> fitFixedNumPlanes(const PointCloudT::Ptr& cloud, const Config& cfg)
{
    StageTimer timer("fit planes on preprocessed cloud");

    std::vector<PlaneInfo> planes;
    PointCloudT::Ptr cloud_remaining(new PointCloudT);
    *cloud_remaining = *cloud;

    pcl::SACSegmentation<PointT> seg;
    pcl::ExtractIndices<PointT> extract;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(cfg.plane_dist_threshold);
    seg.setMaxIterations(cfg.plane_max_iter);

    int fit_count = 0;
    while (fit_count < cfg.fit_plane_num &&
           static_cast<int>(cloud_remaining->size()) > cfg.min_plane_points)
    {
        seg.setInputCloud(cloud_remaining);
        seg.segment(*inliers, *coeffs);
        if (static_cast<int>(inliers->indices.size()) < cfg.min_plane_points)
        {
            break;
        }

        PointCloudT::Ptr plane_cloud(new PointCloudT);
        extract.setInputCloud(cloud_remaining);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*plane_cloud);

        PlaneInfo plane;
        plane.plane_eq = Eigen::Vector4f(
            coeffs->values[0], coeffs->values[1], coeffs->values[2], coeffs->values[3]);
        plane.inlier_cloud = plane_cloud;
        plane.points.reserve(plane_cloud->size());
        for (const auto& p : *plane_cloud)
        {
            plane.points.emplace_back(p.x, p.y, p.z);
        }
        calculatePlaneBasicStats(plane);
        planes.push_back(plane);

        extract.setNegative(true);
        extract.filter(*cloud_remaining);
        ++fit_count;
    }

    std::cout << "Fitted planes: " << planes.size() << std::endl;
    return planes;
}

std::array<Eigen::Vector3f, 3> estimatePrincipalAxesFromNormals(
    const std::vector<Eigen::Vector3f>& normals)
{
    if (normals.size() < 3)
    {
        throw std::runtime_error("Need at least three plane normals for stable axis estimation.");
    }

    StageTimer timer("estimate axes from plane normals");

    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (const auto& n : normals)
    {
        mean += n;
    }
    mean /= static_cast<float>(normals.size());

    for (const auto& n : normals)
    {
        const Eigen::Vector3f d = n - mean;
        cov += d * d.transpose();
    }
    cov /= static_cast<float>(normals.size());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
    std::array<Eigen::Vector3f, 3> axes = {
        solver.eigenvectors().col(2), solver.eigenvectors().col(1), solver.eigenvectors().col(0)};

    if (axes[0].cross(axes[1]).dot(axes[2]) < 0.0f)
    {
        axes[2] = -axes[2];
    }

    return axes;
}

std::array<std::vector<PlaneInfo>, 3> groupPlanesByPrincipalAxis(
    const std::vector<PlaneInfo>& all_planes, const std::array<Eigen::Vector3f, 3>& axes)
{
    std::array<std::vector<PlaneInfo>, 3> groups;

    for (const auto& plane : all_planes)
    {
        Eigen::Vector3f n(plane.plane_eq[0], plane.plane_eq[1], plane.plane_eq[2]);
        if (n.norm() < 1e-6f)
        {
            continue;
        }
        n.normalize();

        int best_axis = 0;
        float max_dot = std::abs(n.dot(axes[0]));
        for (int axis = 1; axis < 3; ++axis)
        {
            const float dot = std::abs(n.dot(axes[axis]));
            if (dot > max_dot)
            {
                max_dot = dot;
                best_axis = axis;
            }
        }
        groups[best_axis].push_back(plane);
    }

    return groups;
}

void selectExtremePlanesPerAxis(const std::vector<PlaneInfo>& all_planes,
                                const std::array<Eigen::Vector3f, 3>& axes,
                                PlaneInfo (&plane_min)[3],
                                PlaneInfo (&plane_max)[3])
{
    StageTimer timer("select extreme planes");

    const auto groups = groupPlanesByPrincipalAxis(all_planes, axes);

    auto proj = [&](const PlaneInfo& plane, int axis) {
        return plane.avg_x * axes[axis].x() + plane.avg_y * axes[axis].y() +
               plane.avg_z * axes[axis].z();
    };

    for (int axis = 0; axis < 3; ++axis)
    {
        if (groups[axis].empty())
        {
            plane_min[axis] = PlaneInfo();
            plane_max[axis] = PlaneInfo();
            continue;
        }

        size_t min_idx = 0;
        size_t max_idx = 0;
        float min_val = proj(groups[axis][0], axis);
        float max_val = min_val;

        for (size_t i = 1; i < groups[axis].size(); ++i)
        {
            const float val = proj(groups[axis][i], axis);
            if (val < min_val)
            {
                min_val = val;
                min_idx = i;
            }
            if (val > max_val)
            {
                max_val = val;
                max_idx = i;
            }
        }

        plane_min[axis] = groups[axis][min_idx];
        plane_max[axis] = groups[axis][max_idx];
    }
}

std::array<AxisCandidates, 3> collectAxisCandidates(const std::vector<PlaneInfo>& all_planes,
                                                    const std::array<Eigen::Vector3f, 3>& axes,
                                                    const Config& cfg)
{
    StageTimer timer("collect axis candidates");

    const auto groups = groupPlanesByPrincipalAxis(all_planes, axes);

    auto proj = [&](const PlaneInfo& plane, int axis) {
        return plane.avg_x * axes[axis].x() + plane.avg_y * axes[axis].y() +
               plane.avg_z * axes[axis].z();
    };

    std::array<AxisCandidates, 3> candidates;

    for (int axis = 0; axis < 3; ++axis)
    {
        // Sort planes by signed position along the current axis so we can inspect outer candidates.
        auto sorted_group = groups[axis];
        AxisCandidates axis_info;
        if (sorted_group.empty())
        {
            candidates[axis] = axis_info;
            continue;
        }

        std::sort(sorted_group.begin(), sorted_group.end(), [&](const PlaneInfo& a, const PlaneInfo& b) {
            return proj(a, axis) < proj(b, axis);
        });

        axis_info.group = sorted_group;
        axis_info.raw_distance = proj(sorted_group.back(), axis) - proj(sorted_group.front(), axis);

        for (const auto& plane : sorted_group)
        {
            if (plane.point_num >= cfg.min_voting_plane_points)
            {
                axis_info.low_planes.push_back(plane);
                if (static_cast<int>(axis_info.low_planes.size()) == cfg.outer_plane_candidates)
                {
                    break;
                }
            }
        }

        for (auto it = sorted_group.rbegin(); it != sorted_group.rend(); ++it)
        {
            if (it->point_num >= cfg.min_voting_plane_points)
            {
                axis_info.high_planes.push_back(*it);
                if (static_cast<int>(axis_info.high_planes.size()) == cfg.outer_plane_candidates)
                {
                    break;
                }
            }
        }

        if (axis_info.low_planes.empty())
        {
            axis_info.low_planes.push_back(sorted_group.front());
        }
        if (axis_info.high_planes.empty())
        {
            axis_info.high_planes.push_back(sorted_group.back());
        }

        for (const auto& low_plane : axis_info.low_planes)
        {
            for (const auto& high_plane : axis_info.high_planes)
            {
                const float distance = proj(high_plane, axis) - proj(low_plane, axis);
                if (distance > 0.0f)
                {
                    axis_info.candidate_distances.push_back(distance);
                }
            }
        }

        if (axis_info.candidate_distances.empty())
        {
            axis_info.candidate_distances.push_back(axis_info.raw_distance);
        }

        auto sorted_distances = axis_info.candidate_distances;
        std::sort(sorted_distances.begin(), sorted_distances.end());
        // Keep a diagnostic "vote" only to judge stability; final dimensions still use raw extremes.
        axis_info.voted_distance = sorted_distances[sorted_distances.size() / 2];
        axis_info.spread = sorted_distances.back() - sorted_distances.front();
        axis_info.has_reliable_vote = sorted_distances.size() >= 2;
        candidates[axis] = axis_info;
    }

    return candidates;
}

void printAxisCandidateSummary(const std::array<AxisCandidates, 3>& candidates,
                               const std::array<Eigen::Vector3f, 3>& axes)
{
    for (int axis = 0; axis < 3; ++axis)
    {
        std::cout << "\n===== Axis " << axis << " candidate summary =====" << std::endl;
        std::cout << "Axis dir: " << axes[axis].transpose() << std::endl;
        std::cout << "Group size: " << candidates[axis].group.size() << std::endl;
        std::cout << "Raw extreme distance: " << std::fixed << std::setprecision(3)
                  << candidates[axis].raw_distance << " m" << std::endl;
        if (candidates[axis].has_reliable_vote)
        {
            std::cout << "Diagnostic voted distance: " << std::fixed << std::setprecision(3)
                      << candidates[axis].voted_distance << " m" << std::endl;
        }
        else
        {
            std::cout << "Diagnostic voted distance: unavailable"
                      << " (only " << candidates[axis].candidate_distances.size()
                      << " candidate)" << std::endl;
        }
        std::cout << "Candidate spread: " << std::fixed << std::setprecision(3)
                  << candidates[axis].spread << " m" << std::endl;
        std::cout << "Candidate distances:";
        for (const float d : candidates[axis].candidate_distances)
        {
            std::cout << " " << std::fixed << std::setprecision(3) << d;
        }
        std::cout << std::endl;
    }
}

DimensionResult computeDimensionsByExtremePlanes(const PlaneInfo (&plane_min)[3],
                                                 const PlaneInfo (&plane_max)[3],
                                                 const std::array<Eigen::Vector3f, 3>& axes)
{
    StageTimer timer("extreme-plane dimensions");

    DimensionResult result;
    result.axes = axes;

    // Use the outermost structural planes for the final room size to preserve the envelope size.
    result.axis_size[0] = std::abs(
        plane_max[0].avg_x * axes[0].x() + plane_max[0].avg_y * axes[0].y() +
        plane_max[0].avg_z * axes[0].z() - plane_min[0].avg_x * axes[0].x() -
        plane_min[0].avg_y * axes[0].y() - plane_min[0].avg_z * axes[0].z());

    result.axis_size[1] = std::abs(
        plane_max[1].avg_x * axes[1].x() + plane_max[1].avg_y * axes[1].y() +
        plane_max[1].avg_z * axes[1].z() - plane_min[1].avg_x * axes[1].x() -
        plane_min[1].avg_y * axes[1].y() - plane_min[1].avg_z * axes[1].z());

    result.axis_size[2] = std::abs(
        plane_max[2].avg_x * axes[2].x() + plane_max[2].avg_y * axes[2].y() +
        plane_max[2].avg_z * axes[2].z() - plane_min[2].avg_x * axes[2].x() -
        plane_min[2].avg_y * axes[2].y() - plane_min[2].avg_z * axes[2].z());

    return result;
}

float estimateAxisSizeFromContourEnds(const std::vector<Eigen::Vector3f>& contour,
                                      const Eigen::Vector3f& axis,
                                      float end_ratio,
                                      int min_end_points)
{
    if (contour.empty())
    {
        throw std::runtime_error("Contour-end fallback needs a non-empty contour.");
    }

    std::vector<float> projections;
    projections.reserve(contour.size());

    for (const auto& pt : contour)
    {
        projections.push_back(pt.dot(axis));
    }

    auto minmax = std::minmax_element(projections.begin(), projections.end());
    const float min_proj = *minmax.first;
    const float max_proj = *minmax.second;
    const float range = max_proj - min_proj;

    if (range < 1e-5f)
    {
        throw std::runtime_error("Contour-end fallback found a degenerate axis range.");
    }

    const float end_band = std::max(range * end_ratio, 0.05f);
    std::vector<float> low_end;
    std::vector<float> high_end;
    low_end.reserve(projections.size() / 8);
    high_end.reserve(projections.size() / 8);

    for (const float proj : projections)
    {
        if (proj <= min_proj + end_band)
        {
            low_end.push_back(proj);
        }
        if (proj >= max_proj - end_band)
        {
            high_end.push_back(proj);
        }
    }

    if (static_cast<int>(low_end.size()) < min_end_points ||
        static_cast<int>(high_end.size()) < min_end_points)
    {
        throw std::runtime_error("Contour-end fallback could not find enough end points.");
    }

    std::sort(low_end.begin(), low_end.end());
    std::sort(high_end.begin(), high_end.end());

    const float low_center = low_end[low_end.size() / 2];
    const float high_center = high_end[high_end.size() / 2];
    return high_center - low_center;
}

float planeDistanceAlongAxis(const PlaneInfo& plane_min,
                             const PlaneInfo& plane_max,
                             const Eigen::Vector3f& axis)
{
    const float min_proj =
        plane_min.avg_x * axis.x() + plane_min.avg_y * axis.y() + plane_min.avg_z * axis.z();
    const float max_proj =
        plane_max.avg_x * axis.x() + plane_max.avg_y * axis.y() + plane_max.avg_z * axis.z();
    return std::abs(max_proj - min_proj);
}

float planePositionAlongAxis(const PlaneInfo& plane, const Eigen::Vector3f& axis)
{
    Eigen::Vector3f normal(plane.plane_eq[0], plane.plane_eq[1], plane.plane_eq[2]);
    const float normal_norm = normal.norm();
    if (normal_norm < 1e-6f)
    {
        throw std::runtime_error("Plane normal is degenerate.");
    }

    normal /= normal_norm;
    const float d = plane.plane_eq[3] / normal_norm;
    const float denom = normal.dot(axis);
    if (std::abs(denom) < 1e-4f)
    {
        throw std::runtime_error("Plane is nearly parallel to the measurement axis.");
    }

    return -d / denom;
}

std::array<std::vector<PlanePositionCluster>, 3> collectPlanePositionClusters(
    const std::vector<PlaneInfo>& all_planes,
    const std::array<Eigen::Vector3f, 3>& axes,
    const Config& cfg)
{
    StageTimer timer("cluster plane positions");

    const auto groups = groupPlanesByPrincipalAxis(all_planes, axes);
    std::array<std::vector<PlanePositionCluster>, 3> axis_clusters;

    for (int axis = 0; axis < 3; ++axis)
    {
        struct PlanePositionSample
        {
            PlaneInfo plane;
            float position = 0.0f;
        };

        std::vector<PlanePositionSample> samples;
        samples.reserve(groups[axis].size());

        for (const auto& plane : groups[axis])
        {
            Eigen::Vector3f normal(plane.plane_eq[0], plane.plane_eq[1], plane.plane_eq[2]);
            const float normal_norm = normal.norm();
            if (normal_norm < 1e-6f)
            {
                continue;
            }

            normal /= normal_norm;
            if (std::abs(normal.dot(axes[axis])) < cfg.min_plane_axis_alignment)
            {
                continue;
            }

            PlanePositionSample sample;
            sample.plane = plane;
            sample.position = planePositionAlongAxis(plane, axes[axis]);
            if (!std::isfinite(sample.position))
            {
                continue;
            }
            samples.push_back(sample);
        }

        if (samples.empty())
        {
            continue;
        }

        std::sort(samples.begin(), samples.end(), [](const PlanePositionSample& a, const PlanePositionSample& b) {
            return a.position < b.position;
        });

        PlanePositionCluster current_cluster;
        float weighted_sum = 0.0f;
        float last_position = samples.front().position;

        auto flush_cluster = [&]() {
            if (current_cluster.planes.empty() || current_cluster.total_points <= 0)
            {
                current_cluster = PlanePositionCluster();
                weighted_sum = 0.0f;
                return;
            }

            current_cluster.mean_position = weighted_sum / static_cast<float>(current_cluster.total_points);
            axis_clusters[axis].push_back(current_cluster);
            current_cluster = PlanePositionCluster();
            weighted_sum = 0.0f;
        };

        for (const auto& sample : samples)
        {
            if (!current_cluster.planes.empty() &&
                std::abs(sample.position - last_position) > cfg.plane_position_cluster_gap)
            {
                flush_cluster();
            }

            current_cluster.planes.push_back(sample.plane);
            current_cluster.total_points += sample.plane.point_num;
            weighted_sum += sample.position * static_cast<float>(sample.plane.point_num);
            last_position = sample.position;
        }
        flush_cluster();
    }

    return axis_clusters;
}

void printPlaneClusterSummary(const std::array<std::vector<PlanePositionCluster>, 3>& axis_clusters,
                              const std::array<Eigen::Vector3f, 3>& axes)
{
    for (int axis = 0; axis < 3; ++axis)
    {
        std::cout << "\n===== Axis " << axis << " plane clusters =====" << std::endl;
        std::cout << "Axis dir: " << axes[axis].transpose() << std::endl;
        std::cout << "Cluster count: " << axis_clusters[axis].size() << std::endl;
        for (size_t i = 0; i < axis_clusters[axis].size(); ++i)
        {
            const auto& cluster = axis_clusters[axis][i];
            std::cout << "  Cluster " << i << ": position=" << std::fixed << std::setprecision(3)
                      << cluster.mean_position << " m"
                      << ", planes=" << cluster.planes.size()
                      << ", points=" << cluster.total_points << std::endl;
        }
    }
}

float contourProjectionSpan(const std::vector<Eigen::Vector3f>& contour, const Eigen::Vector3f& axis)
{
    if (contour.empty())
    {
        throw std::runtime_error("Contour projection fallback needs a non-empty contour.");
    }

    float min_proj = std::numeric_limits<float>::max();
    float max_proj = std::numeric_limits<float>::lowest();
    for (const auto& pt : contour)
    {
        const float proj = pt.dot(axis);
        min_proj = std::min(min_proj, proj);
        max_proj = std::max(max_proj, proj);
    }
    return max_proj - min_proj;
}

DimensionResult computeDimensionsHybrid(const std::array<AxisCandidates, 3>& axis_candidates,
                                        const std::array<std::vector<PlanePositionCluster>, 3>&
                                            axis_plane_clusters,
                                        const std::array<Eigen::Vector3f, 3>& contour_axes,
                                        const std::vector<Eigen::Vector3f>& contour_simplified,
                                        const Config& cfg)
{
    StageTimer timer("hybrid dimensions");

    DimensionResult result;
    result.axes = contour_axes;

    auto compute_axis = [&](int axis) {
        const auto& clusters = axis_plane_clusters[axis];
        std::vector<const PlanePositionCluster*> strong_clusters;
        strong_clusters.reserve(clusters.size());
        for (const auto& cluster : clusters)
        {
            if (cluster.total_points >= cfg.min_plane_cluster_points)
            {
                strong_clusters.push_back(&cluster);
            }
        }

        const bool strong_plane_support =
            static_cast<int>(axis_candidates[axis].group.size()) >= cfg.min_axis_plane_group &&
            strong_clusters.size() >= 2;

        if (strong_plane_support)
        {
            const PlanePositionCluster& low_cluster = *strong_clusters.front();
            const PlanePositionCluster& high_cluster = *strong_clusters.back();
            result.source[axis] = "plane-cluster";
            return high_cluster.mean_position - low_cluster.mean_position;
        }

        try
        {
            result.source[axis] = "contour-end";
            return estimateAxisSizeFromContourEnds(
                contour_simplified, contour_axes[axis], cfg.hull_end_ratio, cfg.min_hull_end_points);
        }
        catch (const std::exception&)
        {
            result.source[axis] = "contour-proj";
            return contourProjectionSpan(contour_simplified, contour_axes[axis]);
        }
    };

    auto fallback_to_plane_cluster = [&](int axis) {
        const auto& clusters = axis_plane_clusters[axis];
        if (clusters.size() >= 2)
        {
            result.source[axis] = "plane-cluster";
            return clusters.back().mean_position - clusters.front().mean_position;
        }
        result.source[axis] = "contour-proj";
        return contourProjectionSpan(contour_simplified, contour_axes[axis]);
    };

    for (int axis = 0; axis < 3; ++axis)
    {
        try
        {
            result.axis_size[axis] = compute_axis(axis);
        }
        catch (const std::exception&)
        {
            result.axis_size[axis] = fallback_to_plane_cluster(axis);
        }
    }
    return result;
}

std::vector<OrderedPoint> orderContourPointsByPrincipalPlane(
    const PointCloudT::Ptr& cloud, const std::array<Eigen::Vector3f, 3>& axes)
{
    StageTimer timer("order contour points");

    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (const auto& p : cloud->points)
    {
        centroid += Eigen::Vector3f(p.x, p.y, p.z);
    }
    centroid /= static_cast<float>(cloud->size());

    std::vector<OrderedPoint> ordered;
    ordered.reserve(cloud->size());

    for (const auto& p : cloud->points)
    {
        const Eigen::Vector3f pt(p.x, p.y, p.z);
        const Eigen::Vector3f delta = pt - centroid;
        const float u = delta.dot(axes[0]);
        const float v = delta.dot(axes[1]);

        OrderedPoint op;
        op.point = pt;
        op.theta = std::atan2(v, u);
        ordered.push_back(op);
    }

    std::sort(ordered.begin(), ordered.end(), [](const OrderedPoint& a, const OrderedPoint& b) {
        return a.theta < b.theta;
    });

    return ordered;
}

float pointLineDistance3D(const Eigen::Vector3f& point,
                          const Eigen::Vector3f& line_start,
                          const Eigen::Vector3f& line_end)
{
    const Eigen::Vector3f line = line_end - line_start;
    const float line_norm = line.norm();
    if (line_norm < 1e-6f)
    {
        return (point - line_start).norm();
    }

    return ((point - line_start).cross(line)).norm() / line_norm;
}

std::vector<Eigen::Vector3f> lightBarSimplifyContour(const std::vector<OrderedPoint>& ordered_points,
                                                     float tolerance)
{
    StageTimer timer("light-bar simplify");

    if (ordered_points.size() < 3)
    {
        throw std::runtime_error("Not enough ordered hull points for simplification.");
    }

    std::vector<Eigen::Vector3f> simplified;
    simplified.reserve(ordered_points.size());
    simplified.push_back(ordered_points.front().point);

    size_t anchor_idx = 0;
    size_t candidate_idx = 1;

    while (candidate_idx + 1 < ordered_points.size())
    {
        const Eigen::Vector3f& anchor = ordered_points[anchor_idx].point;
        const Eigen::Vector3f& next = ordered_points[candidate_idx + 1].point;

        bool within_strip = true;
        for (size_t j = anchor_idx + 1; j <= candidate_idx; ++j)
        {
            const float dist = pointLineDistance3D(ordered_points[j].point, anchor, next);
            if (dist > tolerance)
            {
                within_strip = false;
                break;
            }
        }

        if (within_strip)
        {
            ++candidate_idx;
        }
        else
        {
            simplified.push_back(ordered_points[candidate_idx].point);
            anchor_idx = candidate_idx;
            candidate_idx = anchor_idx + 1;
        }
    }

    simplified.push_back(ordered_points.back().point);
    if ((simplified.front() - simplified.back()).norm() > tolerance)
    {
        simplified.push_back(simplified.front());
    }

    std::cout << "After light-bar simplify: " << simplified.size() << " points" << std::endl;
    return simplified;
}

std::vector<Eigen::Vector3f> pruneByAngle(const std::vector<Eigen::Vector3f>& contour,
                                          float angle_threshold_rad)
{
    StageTimer timer("angle prune");

    if (contour.size() < 4)
    {
        return contour;
    }

    std::vector<Eigen::Vector3f> pts = contour;
    bool changed = true;

    while (changed && pts.size() > 4)
    {
        changed = false;
        std::vector<Eigen::Vector3f> next;
        next.reserve(pts.size());
        next.push_back(pts.front());

        for (size_t i = 1; i + 1 < pts.size(); ++i)
        {
            const Eigen::Vector3f v1 = pts[i] - pts[i - 1];
            const Eigen::Vector3f v2 = pts[i + 1] - pts[i];
            const float n1 = v1.norm();
            const float n2 = v2.norm();

            if (n1 < 1e-6f || n2 < 1e-6f)
            {
                changed = true;
                continue;
            }

            const float cos_value = std::clamp(v1.dot(v2) / (n1 * n2), -1.0f, 1.0f);
            const float angle = std::acos(cos_value);

            if (std::abs(static_cast<float>(M_PI) - angle) < angle_threshold_rad)
            {
                changed = true;
                continue;
            }

            next.push_back(pts[i]);
        }

        next.push_back(pts.back());
        pts.swap(next);
    }

    std::cout << "After angle prune: " << pts.size() << " points" << std::endl;
    return pts;
}

PointCloudT::Ptr buildCloudFromPoints(const std::vector<Eigen::Vector3f>& points)
{
    PointCloudT::Ptr cloud(new PointCloudT);
    cloud->reserve(points.size());
    for (const auto& p : points)
    {
        cloud->push_back(PointT(p.x(), p.y(), p.z()));
    }
    return cloud;
}

void savePointCloudFromVector(const std::vector<Eigen::Vector3f>& points,
                              const std::string& output_path,
                              const std::string& label)
{
    PointCloudT::Ptr cloud = buildCloudFromPoints(points);
    StageTimer timer(label);
    const int status = pcl::io::savePCDFileBinary(output_path, *cloud);
    if (status != 0)
    {
        throw std::runtime_error("Failed to save point cloud: " + output_path);
    }

    std::cout << "Saved point cloud to: " << output_path << std::endl;
}

std::vector<EdgeGroup> groupContourDirections(const std::vector<Eigen::Vector3f>& contour,
                                              const std::array<Eigen::Vector3f, 3>& pca_axes,
                                              float group_threshold_rad)
{
    StageTimer timer("group contour directions");

    std::vector<EdgeGroup> groups;

    auto canonicalize = [](float angle) {
        while (angle < 0.0f)
        {
            angle += static_cast<float>(M_PI);
        }
        while (angle >= static_cast<float>(M_PI))
        {
            angle -= static_cast<float>(M_PI);
        }
        return angle;
    };

    for (size_t i = 0; i + 1 < contour.size(); ++i)
    {
        const Eigen::Vector3f edge3 = contour[i + 1] - contour[i];
        const float edge_len = edge3.norm();
        if (edge_len < 1e-4f)
        {
            continue;
        }

        Eigen::Vector2f edge2(edge3.dot(pca_axes[0]), edge3.dot(pca_axes[1]));
        if (edge2.norm() < 1e-4f)
        {
            continue;
        }

        const float angle = canonicalize(std::atan2(edge2.y(), edge2.x()));
        bool assigned = false;

        for (auto& group : groups)
        {
            float group_angle = canonicalize(std::atan2(group.representative.y(), group.representative.x()));
            float diff = std::abs(angle - group_angle);
            diff = std::min(diff, static_cast<float>(M_PI) - diff);
            if (diff < group_threshold_rad)
            {
                Eigen::Vector2f aligned = edge2;
                if (group.representative.dot(aligned) < 0.0f)
                {
                    aligned = -aligned;
                }
                group.representative += aligned;
                group.total_weight += edge_len;
                assigned = true;
                break;
            }
        }

        if (!assigned)
        {
            EdgeGroup group;
            group.representative = edge2;
            group.total_weight = edge_len;
            groups.push_back(group);
        }
    }

    std::sort(groups.begin(), groups.end(), [](const EdgeGroup& a, const EdgeGroup& b) {
        return a.total_weight > b.total_weight;
    });

    std::cout << "Direction groups: " << groups.size() << std::endl;
    return groups;
}

std::array<Eigen::Vector3f, 3> rectifyAxesWithContourDirections(
    const std::array<Eigen::Vector3f, 3>& pca_axes, const std::vector<EdgeGroup>& groups)
{
    StageTimer timer("rectify axes");

    std::array<Eigen::Vector3f, 3> rectified = pca_axes;

    if (!groups.empty() && groups[0].representative.norm() > 1e-6f)
    {
        const Eigen::Vector2f dominant2 = groups[0].representative.normalized();
        rectified[0] = (dominant2.x() * pca_axes[0] + dominant2.y() * pca_axes[1]).normalized();
    }

    rectified[2] = pca_axes[2].normalized();
    rectified[1] = rectified[2].cross(rectified[0]).normalized();
    rectified[0] = rectified[1].cross(rectified[2]).normalized();

    if (rectified[0].cross(rectified[1]).dot(rectified[2]) < 0.0f)
    {
        rectified[1] = -rectified[1];
    }

    return rectified;
}

DimensionResult computeDimensionsByProjection(const PointCloudT::Ptr& cloud,
                                              const std::array<Eigen::Vector3f, 3>& axes)
{
    StageTimer timer("projection dimensions");

    DimensionResult result;
    result.axes = axes;

    std::array<float, 3> min_proj = {
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max()};
    std::array<float, 3> max_proj = {
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::lowest()};

    for (const auto& p : cloud->points)
    {
        const Eigen::Vector3f pt(p.x, p.y, p.z);
        for (int axis = 0; axis < 3; ++axis)
        {
            const float proj = pt.dot(axes[axis]);
            min_proj[axis] = std::min(min_proj[axis], proj);
            max_proj[axis] = std::max(max_proj[axis], proj);
        }
    }

    result.axis_size[0] = max_proj[0] - min_proj[0];
    result.axis_size[1] = max_proj[1] - min_proj[1];
    result.axis_size[2] = max_proj[2] - min_proj[2];
    return result;
}


std::vector<Eigen::Vector3f> resampleClosedContour(const std::vector<Eigen::Vector3f>& contour,
                                                   float spacing)
{
    StageTimer timer("resample contour");

    if (contour.size() < 2)
    {
        throw std::runtime_error("Need at least two contour points for resampling.");
    }

    const float safe_spacing = std::max(spacing, 1e-3f);
    std::vector<Eigen::Vector3f> dense;
    dense.reserve(contour.size() * 4);
    dense.push_back(contour.front());

    for (size_t i = 0; i + 1 < contour.size(); ++i)
    {
        const Eigen::Vector3f a = contour[i];
        const Eigen::Vector3f b = contour[i + 1];
        const Eigen::Vector3f edge = b - a;
        const float len = edge.norm();
        if (len < 1e-6f)
        {
            continue;
        }

        const int segments = std::max(1, static_cast<int>(std::ceil(len / safe_spacing)));
        for (int s = 1; s <= segments; ++s)
        {
            const float t = static_cast<float>(s) / static_cast<float>(segments);
            dense.push_back(a + t * edge);
        }
    }

    if ((dense.front() - dense.back()).norm() > 1e-5f)
    {
        dense.push_back(dense.front());
    }

    std::cout << "Dense contour points: " << dense.size() << std::endl;
    return dense;
}

ProjectionExtent computeProjectionExtentOnAxis(const std::vector<Eigen::Vector3f>& points,
                                               const Eigen::Vector3f& axis)
{
    ProjectionExtent result;
    if (points.empty())
    {
        return result;
    }

    float min_proj = std::numeric_limits<float>::max();
    float max_proj = std::numeric_limits<float>::lowest();

    for (const auto& pt : points)
    {
        const float proj = pt.dot(axis);
        min_proj = std::min(min_proj, proj);
        max_proj = std::max(max_proj, proj);
    }

    result.low = min_proj;
    result.high = max_proj;
    result.size = max_proj - min_proj;
    result.valid = std::isfinite(result.size) && result.size >= 0.0f;
    return result;
}

ProjectionExtent computeTrimmedProjectionExtentOnAxis(const std::vector<Eigen::Vector3f>& points,
                                                      const Eigen::Vector3f& axis,
                                                      float trim_ratio)
{
    ProjectionExtent result;
    if (points.empty())
    {
        return result;
    }

    std::vector<float> projections;
    projections.reserve(points.size());
    for (const auto& pt : points)
    {
        projections.push_back(pt.dot(axis));
    }

    std::sort(projections.begin(), projections.end());

    const int n = static_cast<int>(projections.size());
    const int trim = std::max(0, static_cast<int>(std::floor(static_cast<float>(n) * trim_ratio)));
    const int left = std::min(trim, n - 1);
    const int right = std::max(left, n - 1 - trim);

    result.low = projections[left];
    result.high = projections[right];
    result.size = result.high - result.low;
    result.valid = std::isfinite(result.size) && result.size >= 0.0f;
    return result;
}

SupportedExtent computeSupportedExtentOnAxis(const std::vector<Eigen::Vector3f>& points,
                                             const Eigen::Vector3f& axis,
                                             float bin_size,
                                             int support_threshold,
                                             int continuity_bins)
{
    SupportedExtent result;
    if (points.size() < 2 || bin_size <= 1e-6f)
    {
        return result;
    }

    std::vector<float> projections;
    projections.reserve(points.size());

    float min_proj = std::numeric_limits<float>::max();
    float max_proj = std::numeric_limits<float>::lowest();

    for (const auto& pt : points)
    {
        const float proj = pt.dot(axis);
        projections.push_back(proj);
        min_proj = std::min(min_proj, proj);
        max_proj = std::max(max_proj, proj);
    }

    if (!(max_proj > min_proj))
    {
        return result;
    }

    const int num_bins = std::max(1, static_cast<int>(std::ceil((max_proj - min_proj) / bin_size)) + 1);
    std::vector<int> hist(num_bins, 0);

    for (const float proj : projections)
    {
        int idx = static_cast<int>(std::floor((proj - min_proj) / bin_size));
        idx = std::max(0, std::min(num_bins - 1, idx));
        hist[idx] += 1;
    }

    std::vector<int> smooth(num_bins, 0);
    for (int i = 0; i < num_bins; ++i)
    {
        int sum = hist[i];
        if (i > 0) sum += hist[i - 1];
        if (i + 1 < num_bins) sum += hist[i + 1];
        smooth[i] = sum;
    }

    auto has_support_run = [&](int start, int step) {
        int idx = start;
        for (int c = 0; c < continuity_bins; ++c)
        {
            if (idx < 0 || idx >= num_bins || smooth[idx] < support_threshold)
            {
                return false;
            }
            idx += step;
        }
        return true;
    };

    int left_bin = -1;
    int right_bin = -1;

    for (int i = 0; i < num_bins; ++i)
    {
        if (has_support_run(i, +1))
        {
            left_bin = i;
            break;
        }
    }

    for (int i = num_bins - 1; i >= 0; --i)
    {
        if (has_support_run(i, -1))
        {
            right_bin = i;
            break;
        }
    }

    if (left_bin < 0 || right_bin < left_bin)
    {
        return result;
    }

    result.left_bin = left_bin;
    result.right_bin = right_bin;
    result.low = min_proj + static_cast<float>(left_bin) * bin_size;
    result.high = min_proj + static_cast<float>(right_bin + 1) * bin_size;
    result.size = result.high - result.low;
    result.confidence = 0.5f * static_cast<float>(smooth[left_bin] + smooth[right_bin]);
    result.valid = std::isfinite(result.size) && result.size >= 0.0f;
    return result;
}

std::vector<Eigen::Vector3f> cloudToEigenPoints(const PointCloudT::Ptr& cloud)
{
    std::vector<Eigen::Vector3f> points;
    points.reserve(cloud->size());
    for (const auto& p : cloud->points)
    {
        points.emplace_back(p.x, p.y, p.z);
    }
    return points;
}

std::string formatMeters(float v)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << v << " m";
    return oss.str();
}



PointCloudT::Ptr applySOR(const PointCloudT::Ptr& input,
                         int mean_k,
                         double stddev,
                         const std::string& timer_label)
{
    StageTimer timer(timer_label);
    PointCloudT::Ptr filtered(new PointCloudT);
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(input);
    sor.setMeanK(mean_k);
    sor.setStddevMulThresh(stddev);
    sor.filter(*filtered);
    return filtered;
}

void ensureDirectory(const std::string& dir)
{
    std::filesystem::create_directories(dir);
}

std::string numberedStagePath(const std::string& dir, int index, const std::string& name)
{
    std::ostringstream oss;
    oss << dir << "/" << std::setw(2) << std::setfill('0') << index << "_" << name << ".pcd";
    return oss.str();
}

void saveStageCloud(const PointCloudT::Ptr& cloud,
                    const std::string& dir,
                    int index,
                    const std::string& name)
{
    const std::string path = numberedStagePath(dir, index, name);
    const int status = pcl::io::savePCDFileBinary(path, *cloud);
    if (status != 0)
    {
        throw std::runtime_error("Failed to save point cloud: " + path);
    }
    std::cout << "Saved stage cloud: " << path << std::endl;
}

void saveStageVectorCloud(const std::vector<Eigen::Vector3f>& points,
                          const std::string& dir,
                          int index,
                          const std::string& name)
{
    saveStageCloud(buildCloudFromPoints(points), dir, index, name);
}

std::array<Eigen::Vector3f, 3> estimatePlanarAxesFromUp(const PointCloudT::Ptr& cloud,
                                                        const Eigen::Vector3f& up_axis)
{
    if (cloud->size() < 3)
    {
        throw std::runtime_error("Need at least three points for planar PCA.");
    }

    Eigen::Vector4f centroid4 = Eigen::Vector4f::Zero();
    pcl::compute3DCentroid(*cloud, centroid4);
    const Eigen::Vector3f centroid = centroid4.head<3>();

    Eigen::Vector3f ref = (std::abs(up_axis.x()) < 0.9f) ? Eigen::Vector3f::UnitX()
                                                         : Eigen::Vector3f::UnitZ();
    Eigen::Vector3f b0 = (ref - ref.dot(up_axis) * up_axis).normalized();
    Eigen::Vector3f b1 = up_axis.cross(b0).normalized();

    Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
    for (const auto& p : cloud->points)
    {
        Eigen::Vector3f d(p.x, p.y, p.z);
        d -= centroid;
        d -= d.dot(up_axis) * up_axis;
        const Eigen::Vector2f q(d.dot(b0), d.dot(b1));
        cov += q * q.transpose();
    }
    cov /= static_cast<float>(cloud->size());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(cov);
    const Eigen::Vector2f v_major = solver.eigenvectors().col(1).normalized();
    const Eigen::Vector2f v_minor = solver.eigenvectors().col(0).normalized();

    Eigen::Vector3f axis0 = (v_major.x() * b0 + v_major.y() * b1).normalized();
    Eigen::Vector3f axis1 = (v_minor.x() * b0 + v_minor.y() * b1).normalized();

    std::array<Eigen::Vector3f, 3> axes = {axis0, axis1, up_axis.normalized()};
    if (axes[0].cross(axes[1]).dot(axes[2]) < 0.0f)
    {
        axes[1] = -axes[1];
    }
    return axes;
}

std::array<Eigen::Vector3f, 3> chooseAxesFromUpAndContour(const PointCloudT::Ptr& planar_source_cloud,
                                                          const std::vector<Eigen::Vector3f>& contour,
                                                          float direction_group_rad)
{
    const auto pca_axes = estimatePointCloudPCAAxes(planar_source_cloud);

    int up_idx = 0;
    float best_abs_y = std::abs(pca_axes[0].y());
    for (int i = 1; i < 3; ++i)
    {
        const float ay = std::abs(pca_axes[i].y());
        if (ay > best_abs_y)
        {
            best_abs_y = ay;
            up_idx = i;
        }
    }

    Eigen::Vector3f up_axis = pca_axes[up_idx].normalized();
    if (up_axis.y() < 0.0f)
    {
        up_axis = -up_axis;
    }

    auto planar_axes = estimatePlanarAxesFromUp(planar_source_cloud, up_axis);
    const auto groups = groupContourDirections(contour, planar_axes, direction_group_rad);
    auto final_axes = rectifyAxesWithContourDirections(planar_axes, groups);

    const float span0 = contourProjectionSpan(contour, final_axes[0]);
    const float span1 = contourProjectionSpan(contour, final_axes[1]);
    if (span1 > span0)
    {
        std::swap(final_axes[0], final_axes[1]);
        final_axes[1] = final_axes[2].cross(final_axes[0]).normalized();
        final_axes[0] = final_axes[1].cross(final_axes[2]).normalized();
    }

    if (final_axes[2].y() < 0.0f)
    {
        final_axes[2] = -final_axes[2];
        final_axes[1] = -final_axes[1];
    }

    return final_axes;
}

struct MethodDimensions
{
    std::string name;
    std::array<float, 3> axis_size = {0.0f, 0.0f, 0.0f};
};

static float medianOfVector(std::vector<float> v)
{
    if (v.empty())
    {
        return 0.0f;
    }

    const size_t mid = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + mid, v.end());
    float med = v[mid];
    if (v.size() % 2 == 0)
    {
        const auto max_it = std::max_element(v.begin(), v.begin() + mid);
        med = 0.5f * (med + *max_it);
    }
    return med;
}

static int chooseUpAxisIndexByLargestAbsY(const std::array<Eigen::Vector3f, 3>& axes)
{
    int best_idx = 0;
    float best_abs_y = std::abs(axes[0].y());
    for (int i = 1; i < 3; ++i)
    {
        const float ay = std::abs(axes[i].y());
        if (ay > best_abs_y)
        {
            best_abs_y = ay;
            best_idx = i;
        }
    }
    return best_idx;
}

static Frame3D buildOrthonormalFrameFromUp(const Eigen::Vector3f& up_axis)
{
    Frame3D frame;
    frame.axis2 = up_axis.normalized();

    Eigen::Vector3f ref = std::abs(frame.axis2.dot(Eigen::Vector3f::UnitX())) < 0.9f
                             ? Eigen::Vector3f::UnitX()
                             : Eigen::Vector3f::UnitZ();

    frame.axis0 = (ref - ref.dot(frame.axis2) * frame.axis2).normalized();
    frame.axis1 = frame.axis2.cross(frame.axis0).normalized();
    frame.axis0 = frame.axis1.cross(frame.axis2).normalized();
    return frame;
}

static Eigen::Vector3f projectToFrame(const Eigen::Vector3f& p, const Frame3D& frame)
{
    return Eigen::Vector3f(p.dot(frame.axis0), p.dot(frame.axis1), p.dot(frame.axis2));
}

static Eigen::Vector3f unprojectFromFrame(const Eigen::Vector3f& local, const Frame3D& frame)
{
    return local.x() * frame.axis0 + local.y() * frame.axis1 + local.z() * frame.axis2;
}

static std::vector<LayerCenterSample> collectLayerCenters(
    const PointCloudT::Ptr& cloud,
    const Frame3D& frame,
    float layer_thickness,
    int min_points_per_layer)
{
    std::vector<LayerCenterSample> samples;
    if (!cloud || cloud->empty() || layer_thickness <= 1e-6f)
    {
        return samples;
    }

    std::vector<Eigen::Vector3f> local_pts;
    local_pts.reserve(cloud->size());
    float z_min = std::numeric_limits<float>::max();
    float z_max = -std::numeric_limits<float>::max();

    for (const auto& pt : cloud->points)
    {
        Eigen::Vector3f p(pt.x, pt.y, pt.z);
        const Eigen::Vector3f q = projectToFrame(p, frame);
        local_pts.push_back(q);
        z_min = std::min(z_min, q.z());
        z_max = std::max(z_max, q.z());
    }

    if (!(z_max > z_min))
    {
        return samples;
    }

    const int num_bins = std::max(1, static_cast<int>(std::ceil((z_max - z_min) / layer_thickness)));
    std::vector<std::vector<float>> xs(num_bins), ys(num_bins), zs(num_bins);

    for (const auto& q : local_pts)
    {
        int idx = static_cast<int>((q.z() - z_min) / layer_thickness);
        idx = std::max(0, std::min(idx, num_bins - 1));
        xs[idx].push_back(q.x());
        ys[idx].push_back(q.y());
        zs[idx].push_back(q.z());
    }

    for (int i = 0; i < num_bins; ++i)
    {
        if (static_cast<int>(xs[i].size()) < min_points_per_layer)
        {
            continue;
        }

        LayerCenterSample s;
        s.z = medianOfVector(zs[i]);
        s.cx = medianOfVector(xs[i]);
        s.cy = medianOfVector(ys[i]);
        s.count = static_cast<int>(xs[i].size());
        samples.push_back(s);
    }

    std::sort(samples.begin(), samples.end(), [](const LayerCenterSample& a, const LayerCenterSample& b) {
        return a.z < b.z;
    });
    return samples;
}

static bool fitLineWeighted(const std::vector<LayerCenterSample>& samples, bool fit_x, float& k, float& b)
{
    if (samples.size() < 2)
    {
        return false;
    }

    double Sw = 0.0;
    double Sz = 0.0;
    double Sv = 0.0;
    double Szz = 0.0;
    double Szv = 0.0;

    for (const auto& s : samples)
    {
        const double w = std::max(1, s.count);
        const double z = s.z;
        const double v = fit_x ? s.cx : s.cy;
        Sw += w;
        Sz += w * z;
        Sv += w * v;
        Szz += w * z * z;
        Szv += w * z * v;
    }

    const double denom = Sw * Szz - Sz * Sz;
    if (std::abs(denom) < 1e-12)
    {
        return false;
    }

    k = static_cast<float>((Sw * Szv - Sz * Sv) / denom);
    b = static_cast<float>((Sv - k * Sz) / Sw);
    return true;
}

static ShearEstimate estimateShearFromLayerCenters(const std::vector<LayerCenterSample>& samples)
{
    ShearEstimate est;
    if (samples.empty())
    {
        return est;
    }

    std::vector<float> zvals;
    zvals.reserve(samples.size());
    for (const auto& s : samples)
    {
        zvals.push_back(s.z);
    }
    est.z_ref = medianOfVector(zvals);
    est.valid_x = fitLineWeighted(samples, true, est.kx, est.bx);
    est.valid_y = fitLineWeighted(samples, false, est.ky, est.by);
    return est;
}

static void saveLayerCenterDebugCSV(const std::string& path,
                                    const std::vector<LayerCenterSample>& samples,
                                    const ShearEstimate& est)
{
    std::ofstream ofs(path);
    if (!ofs.is_open())
    {
        throw std::runtime_error("Failed to write layer debug CSV: " + path);
    }

    ofs << "z,cx,cy,count,cx_fit,cy_fit\n";
    for (const auto& s : samples)
    {
        const float cx_fit = est.bx + est.kx * s.z;
        const float cy_fit = est.by + est.ky * s.z;
        ofs << s.z << ',' << s.cx << ',' << s.cy << ',' << s.count << ',' << cx_fit << ','
            << cy_fit << '\n';
    }
}

static PointCloudT::Ptr applyShearCorrection(const PointCloudT::Ptr& cloud,
                                        const Frame3D& frame,
                                        const ShearEstimate& est)
{
    PointCloudT::Ptr corrected(new PointCloudT);
    if (!cloud)
    {
        return corrected;
    }

    corrected->reserve(cloud->size());
    for (const auto& pt : cloud->points)
    {
        const Eigen::Vector3f p(pt.x, pt.y, pt.z);
        const Eigen::Vector3f q = projectToFrame(p, frame);
        const float dz = q.z() - est.z_ref;

        float x_corr = q.x();
        float y_corr = q.y();
        if (est.valid_x)
        {
            x_corr -= est.kx * dz;
        }
        if (est.valid_y)
        {
            y_corr -= est.ky * dz;
        }

        const Eigen::Vector3f corrected_world = unprojectFromFrame(Eigen::Vector3f(x_corr, y_corr, q.z()), frame);
        PointT out;
        out.x = corrected_world.x();
        out.y = corrected_world.y();
        out.z = corrected_world.z();
        corrected->push_back(out);
    }

    corrected->width = static_cast<uint32_t>(corrected->size());
    corrected->height = 1;
    corrected->is_dense = false;
    return corrected;
}

static PointCloudT::Ptr applyShearCorrectionPipeline(const PointCloudT::Ptr& source_cloud,
                                                     const PointCloudT::Ptr& hull_cloud_for_estimation,
                                                     const Eigen::Vector3f& up_axis,
                                                     const std::string& output_dir,
                                                     const Config& cfg)
{
    const Frame3D frame = buildOrthonormalFrameFromUp(up_axis);

    std::cout << "\n===== Shear correction frame =====" << std::endl;
    std::cout << "Frame axis0: " << frame.axis0.transpose() << std::endl;
    std::cout << "Frame axis1: " << frame.axis1.transpose() << std::endl;
    std::cout << "Frame axis2: " << frame.axis2.transpose() << std::endl;

    const auto samples = collectLayerCenters(
        hull_cloud_for_estimation, frame, cfg.shear_layer_thickness, cfg.shear_min_points_per_layer);
    std::cout << "Layer center samples: " << samples.size() << std::endl;

    const ShearEstimate est = estimateShearFromLayerCenters(samples);
    std::cout << "Estimated shear:" << std::endl;
    std::cout << "  kx = " << est.kx << " (axis0 drift per meter of up-axis)" << std::endl;
    std::cout << "  ky = " << est.ky << " (axis1 drift per meter of up-axis)" << std::endl;
    std::cout << "  z_ref = " << est.z_ref << std::endl;
    std::cout << "  valid_x = " << est.valid_x << ", valid_y = " << est.valid_y << std::endl;

    const std::string csv_path = output_dir + "/09_layer_centers_before_shear_correction.csv";
    saveLayerCenterDebugCSV(csv_path, samples, est);
    std::cout << "Saved layer center CSV: " << csv_path << std::endl;

    PointCloudT::Ptr corrected = applyShearCorrection(source_cloud, frame, est);
    saveStageCloud(corrected, output_dir, 10, "preprocessed_cloud_after_shear_correction");
    return corrected;
}



struct AxisProcessingResult
{
    PointCloudT::Ptr hull_raw = PointCloudT::Ptr(new PointCloudT);
    PointCloudT::Ptr hull_sor = PointCloudT::Ptr(new PointCloudT);
    std::vector<Eigen::Vector3f> ordered_points;
    std::vector<Eigen::Vector3f> contour_light;
    PointCloudT::Ptr contour_light_sor = PointCloudT::Ptr(new PointCloudT);
    std::vector<Eigen::Vector3f> contour_simplified;
    std::vector<Eigen::Vector3f> contour_dense;
    std::array<Eigen::Vector3f, 3> pca_axes = {
        Eigen::Vector3f::UnitX(), Eigen::Vector3f::UnitY(), Eigen::Vector3f::UnitZ()};
    std::array<Eigen::Vector3f, 3> final_axes = {
        Eigen::Vector3f::UnitX(), Eigen::Vector3f::UnitY(), Eigen::Vector3f::UnitZ()};
    float avg_spacing = 0.0f;
    float direction_group_rad = 0.0f;
};

static std::vector<AxisCouplingSample> collectAxisCouplingSamples(
    const PointCloudT::Ptr& cloud,
    const Frame3D& frame,
    int driver_idx,
    int target_idx,
    float layer_thickness,
    int min_points_per_layer)
{
    std::vector<AxisCouplingSample> samples;
    if (!cloud || cloud->empty() || layer_thickness <= 1e-6f) return samples;
    if (driver_idx < 0 || driver_idx > 2 || target_idx < 0 || target_idx > 2 || driver_idx == target_idx)
    {
        return samples;
    }

    std::vector<Eigen::Vector3f> local_pts;
    local_pts.reserve(cloud->size());

    float dmin = std::numeric_limits<float>::max();
    float dmax = -std::numeric_limits<float>::max();

    for (const auto& pt : cloud->points)
    {
        const Eigen::Vector3f p(pt.x, pt.y, pt.z);
        const Eigen::Vector3f q = projectToFrame(p, frame);
        local_pts.push_back(q);

        const float u[3] = {q.x(), q.y(), q.z()};
        dmin = std::min(dmin, u[driver_idx]);
        dmax = std::max(dmax, u[driver_idx]);
    }

    if (dmax <= dmin) return samples;

    const int num_bins = std::max(1, static_cast<int>(std::ceil((dmax - dmin) / layer_thickness)));
    std::vector<std::vector<float>> driver_bins(num_bins), target_bins(num_bins);

    for (const auto& q : local_pts)
    {
        const float u[3] = {q.x(), q.y(), q.z()};
        int idx = static_cast<int>((u[driver_idx] - dmin) / layer_thickness);
        idx = std::max(0, std::min(idx, num_bins - 1));
        driver_bins[idx].push_back(u[driver_idx]);
        target_bins[idx].push_back(u[target_idx]);
    }

    for (int i = 0; i < num_bins; ++i)
    {
        if (static_cast<int>(target_bins[i].size()) < min_points_per_layer) continue;
        AxisCouplingSample s;
        s.driver = medianOfVector(driver_bins[i]);
        s.target = medianOfVector(target_bins[i]);
        s.count = static_cast<int>(target_bins[i].size());
        samples.push_back(s);
    }

    std::sort(samples.begin(), samples.end(),
              [](const AxisCouplingSample& a, const AxisCouplingSample& b)
              {
                  return a.driver < b.driver;
              });
    return samples;
}

static bool fitAxisCouplingLine(const std::vector<AxisCouplingSample>& samples, float& k, float& b)
{
    if (samples.size() < 2) return false;

    double Sw = 0.0, Sd = 0.0, St = 0.0, Sdd = 0.0, Sdt = 0.0;
    for (const auto& s : samples)
    {
        const double w = std::max(1, s.count);
        const double d = s.driver;
        const double t = s.target;
        Sw += w;
        Sd += w * d;
        St += w * t;
        Sdd += w * d * d;
        Sdt += w * d * t;
    }

    const double denom = Sw * Sdd - Sd * Sd;
    if (std::abs(denom) < 1e-12) return false;

    k = static_cast<float>((Sw * Sdt - Sd * St) / denom);
    b = static_cast<float>((St - k * Sd) / Sw);
    return true;
}

static void saveAxisCouplingCSV(const std::string& path,
                                const std::vector<AxisCouplingSample>& samples,
                                float k,
                                float b)
{
    std::ofstream ofs(path);
    if (!ofs.is_open())
    {
        throw std::runtime_error("Failed to write axis coupling CSV: " + path);
    }

    ofs << "driver,target,count,target_fit\n";
    for (const auto& s : samples)
    {
        const float fit = b + k * s.driver;
        ofs << s.driver << ',' << s.target << ',' << s.count << ',' << fit << '\n';
    }
}

static PointCloudT::Ptr applySingleAxisCouplingCorrection(const PointCloudT::Ptr& cloud,
                                                          const Frame3D& frame,
                                                          int driver_idx,
                                                          int target_idx,
                                                          float slope_k,
                                                          float driver_ref)
{
    PointCloudT::Ptr corrected(new PointCloudT);
    if (!cloud)
    {
        return corrected;
    }

    corrected->reserve(cloud->size());
    for (const auto& pt : cloud->points)
    {
        const Eigen::Vector3f p(pt.x, pt.y, pt.z);
        const Eigen::Vector3f q = projectToFrame(p, frame);

        float u[3] = {q.x(), q.y(), q.z()};
        u[target_idx] -= slope_k * (u[driver_idx] - driver_ref);

        const Eigen::Vector3f corrected_world =
            unprojectFromFrame(Eigen::Vector3f(u[0], u[1], u[2]), frame);

        PointT out;
        out.x = corrected_world.x();
        out.y = corrected_world.y();
        out.z = corrected_world.z();
        corrected->push_back(out);
    }

    corrected->width = static_cast<uint32_t>(corrected->size());
    corrected->height = 1;
    corrected->is_dense = false;
    return corrected;
}

static PointCloudT::Ptr applyAxisCouplingCorrectionPipeline(const PointCloudT::Ptr& source_cloud,
                                                            const Frame3D& frame,
                                                            int driver_idx,
                                                            int target_idx,
                                                            const std::string& output_dir,
                                                            float layer_thickness,
                                                            int min_points_per_layer,
                                                            int stage_cloud_id,
                                                            const std::string& label,
                                                            bool& valid_out,
                                                            float& slope_out)
{
    valid_out = false;
    slope_out = 0.0f;

    const auto samples = collectAxisCouplingSamples(
        source_cloud, frame, driver_idx, target_idx, layer_thickness, min_points_per_layer);

    std::cout << "\n===== Axis coupling diagnosis: " << label << " =====" << std::endl;
    std::cout << "Driver axis = " << driver_idx << ", target axis = " << target_idx << std::endl;
    std::cout << "Sample count = " << samples.size() << std::endl;

    if (samples.size() < 2)
    {
        std::cout << "Not enough samples for fitting." << std::endl;
        return source_cloud;
    }

    float k = 0.0f;
    float b = 0.0f;
    if (!fitAxisCouplingLine(samples, k, b))
    {
        std::cout << "Line fitting failed." << std::endl;
        return source_cloud;
    }

    std::vector<float> driver_vals;
    driver_vals.reserve(samples.size());
    for (const auto& s : samples)
    {
        driver_vals.push_back(s.driver);
    }
    const float driver_ref = medianOfVector(driver_vals);

    std::cout << "Fitted slope k = " << k << std::endl;
    std::cout << "Fitted intercept b = " << b << std::endl;
    std::cout << "Driver reference = " << driver_ref << std::endl;

    const std::string csv_path = output_dir + "/" + label + ".csv";
    saveAxisCouplingCSV(csv_path, samples, k, b);
    std::cout << "Saved axis coupling CSV: " << csv_path << std::endl;

    PointCloudT::Ptr corrected = applySingleAxisCouplingCorrection(
        source_cloud, frame, driver_idx, target_idx, k, driver_ref);
    saveStageCloud(corrected, output_dir, stage_cloud_id, label + "_corrected_cloud");

    valid_out = true;
    slope_out = k;
    return corrected;
}

static AxisProcessingResult processCloudThroughContourPipeline(const PointCloudT::Ptr& source_cloud,
                                                              const Config& cfg,
                                                              const std::string& out_dir,
                                                              int stage_base,
                                                              const std::string& stage_label)
{
    AxisProcessingResult result;

    result.hull_raw = extractHull(source_cloud, cfg);
    saveStageCloud(result.hull_raw, out_dir, stage_base + 0, stage_label + "_hull_raw");

    result.hull_sor = applySOR(result.hull_raw, cfg.sor_mean_k, cfg.sor_stddev,
                               "sor on " + stage_label + " hull");
    if (result.hull_sor->empty())
    {
        throw std::runtime_error(stage_label + " hull SOR removed all points.");
    }
    saveStageCloud(result.hull_sor, out_dir, stage_base + 1, stage_label + "_hull_sor");

    result.pca_axes = estimatePointCloudPCAAxes(result.hull_sor);
    std::cout << "\n===== PCA axes on " << stage_label << " hull-after-sor =====" << std::endl;
    for (int axis = 0; axis < 3; ++axis)
    {
        std::cout << stage_label << " PCA axis " << axis << ": "
                  << result.pca_axes[axis].transpose() << std::endl;
    }

    const auto ordered = orderContourPointsByPrincipalPlane(result.hull_sor, result.pca_axes);
    result.ordered_points.reserve(ordered.size());
    for (const auto& op : ordered)
    {
        result.ordered_points.push_back(op.point);
    }
    saveStageVectorCloud(result.ordered_points, out_dir, stage_base + 2, stage_label + "_contour_ordered");

    result.avg_spacing = computeAverageSpacing(result.hull_sor, 2, 10);
    const float light_bar_tol = cfg.light_bar_factor * result.avg_spacing;
    const float angle_threshold_rad = cfg.angle_prune_deg * static_cast<float>(M_PI) / 180.0f;
    result.direction_group_rad = cfg.direction_group_deg * static_cast<float>(M_PI) / 180.0f;
    const float contour_resample_spacing =
        std::max(1e-3f, cfg.contour_resample_spacing_factor * result.avg_spacing);

    result.contour_light = lightBarSimplifyContour(ordered, light_bar_tol);
    saveStageVectorCloud(result.contour_light, out_dir, stage_base + 3, stage_label + "_contour_after_light_bar");

    PointCloudT::Ptr contour_light_cloud = buildCloudFromPoints(result.contour_light);
    result.contour_light_sor = applySOR(contour_light_cloud, cfg.sor_mean_k, cfg.sor_stddev,
                                        "sor on " + stage_label + " light-bar contour");
    if (result.contour_light_sor->size() < 4)
    {
        throw std::runtime_error(stage_label + " contour after second SOR is too small.");
    }
    saveStageCloud(result.contour_light_sor, out_dir, stage_base + 4, stage_label + "_contour_after_light_bar_sor");

    const std::vector<Eigen::Vector3f> contour_after_second_sor = cloudToEigenPoints(result.contour_light_sor);
    result.contour_simplified = pruneByAngle(contour_after_second_sor, angle_threshold_rad);
    if (result.contour_simplified.size() < 4)
    {
        throw std::runtime_error(stage_label + " contour after angle prune is too small.");
    }
    saveStageVectorCloud(result.contour_simplified, out_dir, stage_base + 5, stage_label + "_contour_after_angle_prune");

    result.contour_dense = resampleClosedContour(result.contour_simplified, contour_resample_spacing);
    saveStageVectorCloud(result.contour_dense, out_dir, stage_base + 6, stage_label + "_contour_dense_resampled");

    result.final_axes = chooseAxesFromUpAndContour(
        result.hull_sor, result.contour_simplified, result.direction_group_rad);

    std::cout << "\n===== Final fitted axes (" << stage_label << ") =====" << std::endl;
    for (int axis = 0; axis < 3; ++axis)
    {
        std::cout << "Axis " << axis << ": " << result.final_axes[axis].transpose() << std::endl;
    }
    std::cout << "Axis 2 y-component = " << result.final_axes[2].y() << std::endl;

    return result;
}


Config parseArgs(int argc, char** argv)
{
    Config cfg;
    cfg.path = "/home/ricky/WCR_ws/dense_orbslam3/Examples/RGB-D/PointCloudMapping_RGBD.pcd";
    cfg.voxel_leaf = 0.008f;
    cfg.alpha_factor = 30.0f;
    cfg.sor_mean_k = 8;
    cfg.sor_stddev = 1.5;
    cfg.light_bar_factor = 3.0f;
    cfg.angle_prune_deg = 8.0f;
    cfg.direction_group_deg = 10.0f;
    cfg.plane_dist_threshold = 0.02f;
    cfg.fit_plane_num = 30;
    cfg.min_plane_points = 30;
    cfg.min_axis_plane_group = 2;
    cfg.min_plane_cluster_points = 10;
    cfg.min_voting_plane_points = 10;
    cfg.hull_end_ratio = 0.06f;
    cfg.min_hull_end_points = 10;
    cfg.projection_trim_ratio = 0.01f;
    cfg.support_bin_size = 0.02f;
    cfg.support_threshold = 3;
    cfg.continuity_bins = 2;
    if (argc >= 2)
    {
        cfg.path = argv[1];
    }
    return cfg;
}

int main(int argc, char** argv)
{
    try
    {
        const Config cfg = parseArgs(argc, argv);
        const std::string out_dir = "/home/ricky/WCR_ws/dense_orbslam3/Examples/RGB-D/pcd";
        ensureDirectory(out_dir);

        const auto total_start = std::chrono::steady_clock::now();

        PointCloudT::Ptr cloud = loadCloud(cfg.path);
        PointCloudT::Ptr cloud_fast = preprocessCloud(cloud, cfg);
        saveStageCloud(cloud_fast, out_dir, 1, "preprocessed_cloud_voxel_sor");

        PointCloudT::Ptr hull_raw = extractHull(cloud_fast, cfg);
        saveStageCloud(hull_raw, out_dir, 2, "hull_alpha_shape_raw");

        PointCloudT::Ptr hull_sor = applySOR(hull_raw, cfg.sor_mean_k, cfg.sor_stddev, "sor on hull");
        if (hull_sor->empty())
        {
            throw std::runtime_error("Hull SOR removed all points.");
        }
        saveStageCloud(hull_sor, out_dir, 3, "hull_after_sor");

        const auto initial_axes = estimatePointCloudPCAAxes(hull_sor);
        std::cout << "\n===== PCA axes on hull-after-sor =====" << std::endl;
        for (int axis = 0; axis < 3; ++axis)
        {
            std::cout << "Initial PCA axis " << axis << ": " << initial_axes[axis].transpose() << std::endl;
        }

        const int up_idx = chooseUpAxisIndexByLargestAbsY(initial_axes);
        Eigen::Vector3f up_axis = initial_axes[up_idx].normalized();
        if (up_axis.y() < 0.0f)
        {
            up_axis = -up_axis;
        }
        std::cout << "Chosen up axis before shear correction: " << up_axis.transpose() << std::endl;

        PointCloudT::Ptr cloud_shear_corrected = applyShearCorrectionPipeline(
            cloud_fast, hull_sor, up_axis, out_dir, cfg);

        const AxisProcessingResult stage1 = processCloudThroughContourPipeline(
            cloud_shear_corrected, cfg, out_dir, 11, "stage1_after_shear");

        Frame3D frame_stage1;
        frame_stage1.axis0 = stage1.final_axes[0];
        frame_stage1.axis1 = stage1.final_axes[1];
        frame_stage1.axis2 = stage1.final_axes[2];

        bool valid_axis0_to_axis1 = false;
        float slope_axis1_given_axis0 = 0.0f;
        PointCloudT::Ptr cloud_axis_decoupled = applyAxisCouplingCorrectionPipeline(
            cloud_shear_corrected,
            frame_stage1,
            0,
            1,
            out_dir,
            cfg.shear_layer_thickness,
            cfg.shear_min_points_per_layer,
            18,
            "18_axis1_given_axis0",
            valid_axis0_to_axis1,
            slope_axis1_given_axis0);

        if (!valid_axis0_to_axis1 || std::abs(slope_axis1_given_axis0) < 0.05f)
        {
            std::cout << "Axis0->Axis1 coupling is weak; skip this correction." << std::endl;
            cloud_axis_decoupled = cloud_shear_corrected;
        }

        const AxisProcessingResult stage2 = processCloudThroughContourPipeline(
            cloud_axis_decoupled, cfg, out_dir, 19, "stage2_after_axis01_decoupling");

        const std::vector<PlaneInfo> planes = fitFixedNumPlanes(cloud_axis_decoupled, cfg);
        for (size_t i = 0; i < planes.size(); ++i)
        {
            saveStageCloud(planes[i].inlier_cloud,
                           out_dir,
                           30 + static_cast<int>(i),
                           "plane_" + std::to_string(i) + "_inliers");
        }

        const auto axis_candidates = collectAxisCandidates(planes, stage2.final_axes, cfg);
        printAxisCandidateSummary(axis_candidates, stage2.final_axes);
        const auto axis_plane_clusters = collectPlanePositionClusters(planes, stage2.final_axes, cfg);
        printPlaneClusterSummary(axis_plane_clusters, stage2.final_axes);
        const DimensionResult hybrid_dims = computeDimensionsHybrid(
            axis_candidates, axis_plane_clusters, stage2.final_axes, stage2.contour_simplified, cfg);

        MethodDimensions trimmed_result;
        trimmed_result.name = "trimmed projection";
        MethodDimensions support_result;
        support_result.name = "histogram support extent";
        MethodDimensions contour_end_result;
        contour_end_result.name = "contour end median";
        MethodDimensions hybrid_result;
        hybrid_result.name = "plane/contour hybrid";
        hybrid_result.axis_size = hybrid_dims.axis_size;

        for (int axis = 0; axis < 3; ++axis)
        {
            const auto trimmed = computeTrimmedProjectionExtentOnAxis(
                stage2.contour_dense, stage2.final_axes[axis], cfg.projection_trim_ratio);
            const auto support = computeSupportedExtentOnAxis(
                stage2.contour_dense, stage2.final_axes[axis], cfg.support_bin_size, cfg.support_threshold, cfg.continuity_bins);
            trimmed_result.axis_size[axis] = trimmed.size;
            support_result.axis_size[axis] = support.valid ? support.size : 0.0f;
            contour_end_result.axis_size[axis] = estimateAxisSizeFromContourEnds(
                stage2.contour_simplified, stage2.final_axes[axis], cfg.hull_end_ratio, cfg.min_hull_end_points);
        }

        const auto total_end = std::chrono::steady_clock::now();
        const double total_seconds =
            std::chrono::duration_cast<std::chrono::milliseconds>(total_end - total_start).count() / 1000.0;

        std::vector<MethodDimensions> methods = {
            trimmed_result, support_result, contour_end_result, hybrid_result};

        std::cout << "\n===== Method comparison on final fitted axes =====" << std::endl;
        std::cout << std::left
                  << std::setw(28) << "Method"
                  << "| " << std::setw(9) << "Axis0"
                  << "| " << std::setw(9) << "Axis1"
                  << "| " << std::setw(9) << "Axis2"
                  << "| " << std::setw(9) << "Volume" << std::endl;
        std::cout << "----------------------------|-----------|-----------|-----------|-----------" << std::endl;
        for (const auto& m : methods)
        {
            const float volume = m.axis_size[0] * m.axis_size[1] * m.axis_size[2];
            std::cout << std::left
                      << std::setw(28) << m.name
                      << "| " << std::setw(9) << formatMeters(m.axis_size[0])
                      << "| " << std::setw(9) << formatMeters(m.axis_size[1])
                      << "| " << std::setw(9) << formatMeters(m.axis_size[2])
                      << "| " << std::setw(9) << formatMeters(volume) << std::endl;
        }

        std::cout << "\n===== Hybrid source per axis =====" << std::endl;
        for (int axis = 0; axis < 3; ++axis)
        {
            std::cout << "Axis " << axis << " source: " << hybrid_dims.source[axis] << std::endl;
        }

        std::cout << "\nSaved intermediate PCDs to: " << out_dir << std::endl;
        std::cout << "Total: " << std::fixed << std::setprecision(3) << total_seconds << " s" << std::endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}