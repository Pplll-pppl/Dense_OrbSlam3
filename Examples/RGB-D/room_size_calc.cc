#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <pcl/segmentation/sac_segmentation.h> 
#include <pcl/filters/extract_indices.h>         
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/surface/concave_hull.h>           // Alpha shape 轮廓提取
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>      // 用于解线性方程组

// 类型定义
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// 平面结构体（只保留必要字段）
struct PlaneInfo
{
    Eigen::Vector4f plane_eq;      // 平面方程 ax+by+cz+d=0
    PointCloudT::Ptr inlier_cloud;  // 平面内点云（轮廓点）
    std::vector<Eigen::Vector3f> points; // 平面点集(Eigen格式)
    float avg_x, avg_y, avg_z;     // 平面点均值（中心）
    int point_num;                 // 平面点数量
};

// ===================== 全局配置参数 =====================
const std::string PCD_PATH = "/home/ricky/WCR_ws/dense_orbslam3/Examples/RGB-D/PointCloudMapping_RGBD_room.pcd";
// const float DOWNSAMPLE_VOXEL_SIZE = 0.01f;      // 下采样体素大小
const float DOWNSAMPLE_VOXEL_SIZE = 0.02f;      // 下采样体素大小

const float RANSAC_DIST_THRESHOLD = 0.1f;       // 平面拟合距离阈值
const int RANSAC_MAX_ITER = 100000;
const int FIT_PLANE_NUM = 100;                  // 拟合平面数量
const int MIN_PLANE_POINTS = 20;                 // 最小平面点数
const float ALPHA_FACTOR = 60.0;                 // Alpha shape 参数因子

// ---------------------- 1. 计算平面的基本统计信息（均值） ----------------------
void calculatePlaneBasicStats(PlaneInfo& plane)
{
    float sum_x = 0, sum_y = 0, sum_z = 0;
    for (const auto& p : plane.points)
    {
        sum_x += p.x(); sum_y += p.y(); sum_z += p.z();
    }
    plane.avg_x = sum_x / plane.points.size();
    plane.avg_y = sum_y / plane.points.size();
    plane.avg_z = sum_z / plane.points.size();
    plane.point_num = plane.points.size();
}

// ---------------------- 2. 点云读取与预处理 ----------------------
PointCloudT::Ptr loadAndPreprocessCloud(const std::string& path)
{
    PointCloudT::Ptr cloud(new PointCloudT);
    if (path.substr(path.find_last_of(".") + 1) == "pcd")
        pcl::io::loadPCDFile<PointT>(path, *cloud);
    else if (path.substr(path.find_last_of(".") + 1) == "ply")
        pcl::io::loadPLYFile<PointT>(path, *cloud);
    std::cout << "原始点云数量: " << cloud->size() << std::endl;

    PointCloudT::Ptr cloud_down(new PointCloudT);
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(DOWNSAMPLE_VOXEL_SIZE, DOWNSAMPLE_VOXEL_SIZE, DOWNSAMPLE_VOXEL_SIZE);
    voxel.filter(*cloud_down);

    PointCloudT::Ptr cloud_clean(new PointCloudT);
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud_down);
    sor.setMeanK(20);
    sor.setStddevMulThresh(2);
    sor.filter(*cloud_clean);

    PointCloudT::Ptr cloud_clean2(new PointCloudT);
    pcl::RadiusOutlierRemoval<PointT> ror;
    ror.setInputCloud(cloud_clean);
    ror.setRadiusSearch(0.05);          // 搜索半径，根据你的点云尺度调整（很重要！）
    ror.setMinNeighborsInRadius(15);    // 半径内至少要有多少邻居才保留，建议10~30
    ror.setKeepOrganized(false);
    ror.filter(*cloud_clean2);

    std::cout << "RadiusOutlierRemoval 后点数: " << cloud_clean2->size() << std::endl;
    std::cout << "预处理后点云数量: " << cloud_clean->size() << std::endl;
    return cloud_clean2;
}

// 计算平均点间距（用于自适应alpha）
float computeAverageSpacing(PointCloudT::Ptr cloud, int k = 2, int sample_ratio = 10) {
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloud);
    std::vector<float> distances;
    distances.reserve(cloud->size() / sample_ratio);
    std::vector<int> indices(k);
    std::vector<float> sq_dist(k);
    for (size_t i = 0; i < cloud->size(); i += sample_ratio) {
        PointT query = cloud->points[i];
        if (kdtree.nearestKSearch(query, k, indices, sq_dist) == k) {
            distances.push_back(std::sqrt(sq_dist[1])); // 第一个是自身，取第二个的距离
        }
    }
    float sum = std::accumulate(distances.begin(), distances.end(), 0.0f);
    return sum / distances.size();
}

// ---------------------- 3. Alpha shape 提取轮廓 ----------------------
PointCloudT::Ptr extractConcaveHull(PointCloudT::Ptr& cloud, double alpha)
{
    PointCloudT::Ptr hull_points(new PointCloudT);
    pcl::ConcaveHull<PointT> chull;
    chull.setInputCloud(cloud);
    chull.setAlpha(alpha);
    chull.setDimension(3);
    chull.reconstruct(*hull_points);
    std::cout << "轮廓点云数量: " << hull_points->size() << std::endl;
    return hull_points;
}

// ---------------------- 4. 对轮廓点云拟合平面 ----------------------
std::vector<PlaneInfo> fitFixedNumPlanes(PointCloudT::Ptr& cloud)
{
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
    seg.setDistanceThreshold(RANSAC_DIST_THRESHOLD);
    seg.setMaxIterations(RANSAC_MAX_ITER);

    int fit_count = 0;
    while (fit_count < FIT_PLANE_NUM && cloud_remaining->size() > MIN_PLANE_POINTS)
    {
        seg.setInputCloud(cloud_remaining);
        seg.segment(*inliers, *coeffs);
        if (inliers->indices.size() < MIN_PLANE_POINTS) break;

        PointCloudT::Ptr plane_cloud(new PointCloudT);
        extract.setInputCloud(cloud_remaining);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*plane_cloud);

        PlaneInfo plane;
        plane.plane_eq = Eigen::Vector4f(coeffs->values[0], coeffs->values[1], coeffs->values[2], coeffs->values[3]);
        plane.inlier_cloud = plane_cloud;
        plane.point_num = plane_cloud->size();

        for (const auto& p : *plane_cloud)
            plane.points.emplace_back(p.x, p.y, p.z);
        calculatePlaneBasicStats(plane);

        planes.push_back(plane);
        extract.setNegative(true);
        extract.filter(*cloud_remaining);
        fit_count++;
    }
    std::cout << "✅ 拟合平面数量: " << planes.size() << "个 (目标" << FIT_PLANE_NUM << "个)" << std::endl;
    return planes;
}

// ---------------------- 5. 使用 PCA 从法向量中估计三个主轴 ----------------------
std::array<Eigen::Vector3f, 3> estimatePrincipalAxes(const std::vector<Eigen::Vector3f>& normals)
{
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (const auto& n : normals) mean += n;
    mean /= normals.size();
    for (const auto& n : normals) {
        Eigen::Vector3f d = n - mean;
        cov += d * d.transpose();
    }
    cov /= normals.size();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
    Eigen::Vector3f eigenvalues = solver.eigenvalues();          // 升序排列
    Eigen::Matrix3f eigenvectors = solver.eigenvectors();        // 对应升序特征值的特征向量

    std::array<Eigen::Vector3f, 3> axes;
    axes[0] = eigenvectors.col(2);   // 最大特征值对应的特征向量
    axes[1] = eigenvectors.col(1);
    axes[2] = eigenvectors.col(0);

    // 确保三个轴构成右手系
    if (axes[0].cross(axes[1]).dot(axes[2]) < 0)
        axes[2] = -axes[2];

    for (int i = 0; i < 3; ++i)
        std::cout << "主轴" << i << "：" << axes[i].transpose() << std::endl;

    return axes;
}

// ===================== 新增：基于平行度的平面分类与极值平面选择 =====================
/**
 * 将平面按法向量与主轴的平行度分为三组，并在每组中选出投影最小和最大的两个平面
 * @param allPlanes 所有拟合平面
 * @param axes 三个主轴方向（单位向量）
 * @param plane_min 输出：每个轴上投影最小的平面
 * @param plane_max 输出：每个轴上投影最大的平面
 */
void selectExtremePlanesPerAxis(const std::vector<PlaneInfo>& allPlanes,
                                const std::array<Eigen::Vector3f, 3>& axes,
                                PlaneInfo (&plane_min)[3],
                                PlaneInfo (&plane_max)[3])
{
    // 1. 按法向量与主轴的点积绝对值分组
    std::vector<PlaneInfo> groups[3];
    for (const auto& plane : allPlanes) {
        Eigen::Vector3f n(plane.plane_eq[0], plane.plane_eq[1], plane.plane_eq[2]);
        n.normalize();

        int best_axis = 0;
        float max_dot = std::abs(n.dot(axes[0]));
        for (int j = 1; j < 3; ++j) {
            float dot = std::abs(n.dot(axes[j]));
            if (dot > max_dot) {
                max_dot = dot;
                best_axis = j;
            }
        }
        groups[best_axis].push_back(plane);
    }

    for (int axis = 0; axis < 3; ++axis) {
        std::cout << "\n===== 轴 " << axis << " 分组详情 =====" << std::endl;
        std::cout << "共 " << groups[axis].size() << " 个平面" << std::endl;
        // 打印每个平面的中心坐标和点数量
        for (size_t i = 0; i < groups[axis].size(); ++i) {
            const auto& p = groups[axis][i];
            std::cout << "  平面" << i << "：中心(" << p.avg_x << "," << p.avg_y << "," << p.avg_z 
                      << ") 点数量=" << p.point_num << std::endl;
        }
    }

    // 辅助函数：计算平面中心在给定轴上的投影
    auto proj = [&](const PlaneInfo& p, int axis) -> float {
        return p.avg_x * axes[axis].x() + p.avg_y * axes[axis].y() + p.avg_z * axes[axis].z();
    };

    // 2. 在每个组中找出投影最小和最大的平面
    for (int axis = 0; axis < 3; ++axis) {
        if (groups[axis].empty()) {
            std::cerr << "错误：轴 " << axis << " 没有平面，无法计算房间尺寸！" << std::endl;
            throw std::runtime_error("缺少某一方向的平面");
        }

        int min_idx = 0, max_idx = 0;
        float min_val = proj(groups[axis][0], axis);
        float max_val = proj(groups[axis][0], axis);

        for (size_t i = 1; i < groups[axis].size(); ++i) {
            float val = proj(groups[axis][i], axis);
            if (val < min_val) {
                min_val = val;
                min_idx = i;
            }
            if (val > max_val) {
                max_val = val;
                max_idx = i;
            }
        }

        plane_min[axis] = groups[axis][min_idx];
        plane_max[axis] = groups[axis][max_idx];

        std::cout << "轴 " << axis << " 极值平面：最小投影 = " << min_val
                  << " (平面点数=" << plane_min[axis].point_num << ")"
                  << "，最大投影 = " << max_val
                  << " (平面点数=" << plane_max[axis].point_num << ")" << std::endl;
    }
}

// ===================== 新增：通过平面求交计算房间尺寸 =====================
/**
 * 由六个极值平面计算房间沿三个主轴方向的尺寸
 * @param plane_min 每个轴上投影最小的平面
 * @param plane_max 每个轴上投影最大的平面
 * @param axes 三个主轴方向
 * @param length 输出：第一个主轴方向的尺寸
 * @param width  输出：第二个主轴方向的尺寸
 * @param height 输出：第三个主轴方向的尺寸
 * @return true 表示成功
 */
bool computeRoomDimensions(const PlaneInfo (&plane_min)[3],
                           const PlaneInfo (&plane_max)[3],
                           const std::array<Eigen::Vector3f, 3>& axes,
                           float& length, float& width, float& height)
{
    // 求所有 8 个角点（每个轴选 min 或 max 平面）
    std::vector<Eigen::Vector3f> corners;
    corners.reserve(8);

    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            for (int k = 0; k < 2; ++k) {
                const PlaneInfo& pA = (i == 0) ? plane_min[0] : plane_max[0];
                const PlaneInfo& pB = (j == 0) ? plane_min[1] : plane_max[1];
                const PlaneInfo& pC = (k == 0) ? plane_min[2] : plane_max[2];

                Eigen::Vector3f n1(pA.plane_eq[0], pA.plane_eq[1], pA.plane_eq[2]);
                Eigen::Vector3f n2(pB.plane_eq[0], pB.plane_eq[1], pB.plane_eq[2]);
                Eigen::Vector3f n3(pC.plane_eq[0], pC.plane_eq[1], pC.plane_eq[2]);

                float d1 = -pA.plane_eq[3];   // n1·p = -d
                float d2 = -pB.plane_eq[3];
                float d3 = -pC.plane_eq[3];

                Eigen::Matrix3f A;
                A.row(0) = n1;
                A.row(1) = n2;
                A.row(2) = n3;
                Eigen::Vector3f b(d1, d2, d3);

                // 解线性方程组 A * p = b
                Eigen::Vector3f corner = A.colPivHouseholderQr().solve(b);
                corners.push_back(corner);
            }
        }
    }

    if (corners.size() != 8) {
        std::cerr << "警告：只获得 " << corners.size() << " 个角点，尺寸计算可能不准确。" << std::endl;
    }

    // 计算每个主轴方向上的投影极差
    auto spanOnAxis = [&](const Eigen::Vector3f& axis) -> float {
        float min_val = corners[0].dot(axis);
        float max_val = min_val;
        for (size_t idx = 1; idx < corners.size(); ++idx) {
            float val = corners[idx].dot(axis);
            if (val < min_val) min_val = val;
            if (val > max_val) max_val = val;
        }
        return max_val - min_val;
    };

    length = spanOnAxis(axes[0]);
    width  = spanOnAxis(axes[1]);
    height = spanOnAxis(axes[2]);

    return true;
}

// ===================== 主函数 =====================
int main(int argc, char** argv)
{
    // 1. 读取预处理点云
    PointCloudT::Ptr cloud = loadAndPreprocessCloud(PCD_PATH);

    // 2. 计算自适应alpha并提取轮廓
    float avg_spacing = computeAverageSpacing(cloud, 2, 10);
    float alpha = ALPHA_FACTOR * avg_spacing;
    std::cout << "平均点间距: " << avg_spacing << " m, alpha = " << alpha << std::endl;

    PointCloudT::Ptr hull_cloud = extractConcaveHull(cloud, alpha);
    pcl::io::savePCDFileASCII("contour_points.pcd", *hull_cloud);
    std::cout << "轮廓点云已保存到 contour_points.pcd" << std::endl;

    // 3. 对轮廓点云拟合平面
    std::vector<PlaneInfo> allPlanes = fitFixedNumPlanes(hull_cloud);
    if (allPlanes.empty()) {
        std::cerr << "错误：未拟合到任何平面！" << std::endl;
        return -1;
    }

    // 4. 提取所有平面的法向量，用于主轴估计
    std::vector<Eigen::Vector3f> normals;
    for (const auto& p : allPlanes) {
        Eigen::Vector3f n(p.plane_eq[0], p.plane_eq[1], p.plane_eq[2]);
        n.normalize();
        normals.push_back(n);
    }

    // 5. 估计主轴
    auto axes = estimatePrincipalAxes(normals);

    std::cout << "\n估计的主轴方向：" << std::endl;
    for (int i = 0; i < 3; ++i)
        std::cout << "轴 " << i << ": " << axes[i].transpose() << std::endl;

    // 6. 基于平行度分类，选择每个方向上的两个极值平面
    PlaneInfo plane_min[3], plane_max[3];
    try {
        selectExtremePlanesPerAxis(allPlanes, axes, plane_min, plane_max);
    } catch (const std::exception& e) {
        std::cerr << "平面筛选失败：" << e.what() << std::endl;
        return -1;
    }

    // 7. 通过平面求交计算房间尺寸
    float L = 0, W = 0, H = 0;
    if (!computeRoomDimensions(plane_min, plane_max, axes, L, W, H)) {
        std::cerr << "尺寸计算失败！" << std::endl;
        return -1;
    }

    // 8. 输出最终结果
    std::cout << "\n===== ✅ 房间最终精准尺寸（基于平面求交） =====" << std::endl;
    std::cout << "长度 (轴0方向) : " << std::fixed << std::setprecision(3) << L << " m" << std::endl;
    std::cout << "宽度 (轴1方向) : " << std::fixed << std::setprecision(3) << W << " m" << std::endl;
    std::cout << "高度 (轴2方向) : " << std::fixed << std::setprecision(3) << H << " m" << std::endl;
    std::cout << "体积           : " << std::fixed << std::setprecision(3) << L * W * H << " m³" << std::endl;

    return 0;
}


// #include <iostream>
// #include <vector>
// #include <algorithm>
// #include <cmath>
// #include <iomanip>
// #include <pcl/segmentation/sac_segmentation.h> 
// #include <pcl/filters/extract_indices.h>         
// #include <pcl/io/pcd_io.h>
// #include <pcl/io/ply_io.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/filters/statistical_outlier_removal.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <Eigen/Core>
// #include <Eigen/Geometry>
// #include <Eigen/SVD>

// // 类型定义
// typedef pcl::PointXYZ PointT;
// typedef pcl::PointCloud<PointT> PointCloudT;

// // 平面结构体 - 扩展三轴延展度特征
// struct PlaneInfo
// {
//     Eigen::Vector4f plane_eq;      // 平面方程 ax+by+cz+d=0
//     PointCloudT::Ptr inlier_cloud;  // 平面内点云
//     std::vector<Eigen::Vector3f> points; // 平面点集(Eigen格式)
//     float ext_ratio_x;             // X轴延展度 (越大→平面越贴近Y-Z平面)
//     float ext_ratio_y;             // Y轴延展度 (越大→平面越贴近X-Z平面)
//     float ext_ratio_z;             // Z轴延展度 (越大→平面越贴近X-Y平面)
//     float min_x, max_x;            // 平面点X轴极值
//     float min_y, max_y;            // 平面点Y轴极值
//     float min_z, max_z;            // 平面点Z轴极值
//     float avg_x, avg_y, avg_z;     // 平面点均值
//     int point_num;                 // 平面点数量
// };

// // 空间直线结构体（保留）
// struct LineInfo
// {
//     Eigen::Vector3f p0;        // 直线上一点
//     Eigen::Vector3f dir;       // 直线方向向量（单位化）
//     PlaneInfo plane1;          // 生成直线的平面1
//     PlaneInfo plane2;          // 生成直线的平面2
// };

// // ===================== 全局配置参数（按你的需求调整） =====================
// const std::string PCD_PATH = "/home/ricky/WCR_ws/dense_orbslam3/Examples/RGB-D/PointCloudMapping_RGBD.pcd";
// const float DOWNSAMPLE_VOXEL_SIZE = 0.01f; //0.02f
// const float RANSAC_DIST_THRESHOLD = 0.045f; //0.05f
// const int RANSAC_MAX_ITER = 10000;
// const int FIT_PLANE_NUM = 30;        // 拟合30个平面（足够覆盖房间6个主面+杂面）
// const int MIN_PLANE_POINTS = 100;    // 过滤极小平面
// const int CLUSTER_K = 3;             // ✅ 核心：Kmeans分3类（对应X/Y/Z三轴延展）
// const int KEEP_PER_CLUSTER = 10;      // 每类保留点云最多的10个平面（刚好对应每个轴的正反两面）

// // ---------------------- 1. 核心修改：方向余弦法计算三轴延展度 ----------------------
// void calculatePlaneExtFeatures(PlaneInfo& plane)
// {
//     // 第一步：计算平面点在X/Y/Z轴的极值（保留原有逻辑）
//     float x_min = 1e9, x_max = -1e9;
//     float y_min = 1e9, y_max = -1e9;
//     float z_min = 1e9, z_max = -1e9;
//     float sum_x = 0, sum_y = 0, sum_z = 0;

//     for (const auto& p : plane.points)
//     {
//         x_min = std::min(x_min, p.x()); x_max = std::max(x_max, p.x());
//         y_min = std::min(y_min, p.y()); y_max = std::max(y_max, p.y());
//         z_min = std::min(z_min, p.z()); z_max = std::max(z_max, p.z());
//         sum_x += p.x(); sum_y += p.y(); sum_z += p.z();
//     }

//     // 赋值极值和均值（保留原有逻辑）
//     plane.min_x = x_min; plane.max_x = x_max;
//     plane.min_y = y_min; plane.max_y = y_max;
//     plane.min_z = z_min; plane.max_z = z_max;
//     plane.avg_x = sum_x / plane.points.size();
//     plane.avg_y = sum_y / plane.points.size();
//     plane.avg_z = sum_z / plane.points.size();

//     // ===================== 核心修改：方向余弦法计算延展度 =====================
//     // 1. 提取平面法向量并归一化（plane_eq的前3个值是法向量(a,b,c)）
//     Eigen::Vector3f normal(plane.plane_eq[0], plane.plane_eq[1], plane.plane_eq[2]);
//     normal.normalize(); // 归一化为单位法向量

//     // 2. 计算法向量与X/Y/Z轴的夹角余弦（取绝对值，消除方向影响）
//     float cos_x = std::abs(normal.x()); // 法向量与X轴的夹角余弦
//     float cos_y = std::abs(normal.y()); // 法向量与Y轴的夹角余弦
//     float cos_z = std::abs(normal.z()); // 法向量与Z轴的夹角余弦

//     // 3. 计算延展度：(1 - 余弦值) / (余弦值 + eps)
//     // 逻辑：余弦值越小→法向量越垂直于该轴→平面越贴近正交面→延展度越大
//     float eps = 1e-3;
//     // plane.ext_ratio_x = (1 - cos_x + eps) / (cos_x + eps); // 越大→贴近Y-Z平面（前后墙）
//     // plane.ext_ratio_y = (1 - cos_y + eps) / (cos_y + eps); // 越大→贴近X-Z平面（地面/天花板）
//     // plane.ext_ratio_z = (1 - cos_z + eps) / (cos_z + eps); // 越大→贴近X-Y平面（左右墙）
//     plane.ext_ratio_x = cos_x; // 越大→贴近Y-Z平面（前后墙）
//     plane.ext_ratio_y = cos_y; // 越大→贴近X-Z平面（地面/天花板）
//     plane.ext_ratio_z = cos_z; // 越大→贴近X-Y平面（左右墙）
// }
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
// // ---------------------- 2. 核心升级：Kmeans三分聚类（基于三轴延展度） ----------------------
// void kmeans3DCluster(const std::vector<PlaneInfo>& planes, 
//                      std::vector<PlaneInfo>& clusterX,  // X轴延展类（贴近Y-Z平面：前后墙）
//                      std::vector<PlaneInfo>& clusterY,  // Y轴延展类（贴近X-Z平面：地面/天花板）
//                      std::vector<PlaneInfo>& clusterZ)  // Z轴延展类（贴近X-Y平面：左右墙）
// {
//     int n = planes.size();
//     // 特征矩阵：每个平面3个特征（ext_ratio_x, ext_ratio_y, ext_ratio_z）
//     std::vector<Eigen::Vector3f> features(n);
//     for (int i = 0; i < n; i++)
//     {
//         features[i] = Eigen::Vector3f(planes[i].ext_ratio_x, planes[i].ext_ratio_y, planes[i].ext_ratio_z);
//     }

//     // 初始化3个聚类中心（选极值点，加速收敛）
//     Eigen::Vector3f c0 = features[0];  // X轴类中心
//     Eigen::Vector3f c1 = features[1];  // Y轴类中心
//     Eigen::Vector3f c2 = features[2];  // Z轴类中心
//     // 优化初始化：选特征差异最大的3个点
//     float max_dist = 0;
//     for (int i = 0; i < n; i++){

//     }
//     for (int i = 0; i < n; i++)
//         for (int j = i+1; j < n; j++)
//             for (int k = j+1; k < n; k++)
//             {
//                 float d1 = (features[i] - features[j]).norm();
//                 float d2 = (features[i] - features[k]).norm();
//                 float d3 = (features[j] - features[k]).norm();
//                 if (d1 + d2 + d3 > max_dist)
//                 {
//                     max_dist = d1 + d2 + d3;
//                     c0 = features[i];
//                     c1 = features[j];
//                     c2 = features[k];
//                 }
//             }
//     std::cout << "initial centroids: " << c0.transpose() << ", " << c1.transpose() << ", " << c2.transpose() << std::endl;
//     std::vector<int> labels(n, 0);
//     bool converged = false;
//     int iter = 0;

//     std::vector<int> clusterZ_idx, clusterY_idx, clusterX_idx;
//     for (int i=0; i<n; i++) {
//         if (features[i].z() > 0.9) {
//             clusterZ_idx.push_back(i);
//         }
//         else if (features[i].y() > 0.9) {
//             clusterY_idx.push_back(i);
//         }
//         else if (features[i].x() > 0.9) {
//             clusterX_idx.push_back(i);
//         }
//     }
//     for (int idx : clusterX_idx) {
//         std::cout << "numbers of point clods in clusterX: " << planes[idx].points.size() << std::endl;
//     }
//     for (int idx : clusterY_idx) {
//         std::cout << "numbers of point clods in clusterY: " << planes[idx].points.size() << std::endl;
//     }
//     std::cout << "numbers of point clods in clusterZ: " << planes[clusterZ_idx[0]].points.size() << std::endl;
    
//     std::cout << "clusterZ_idx size: " << clusterZ_idx.size() << std::endl;
//     std::cout << "clusterY_idx size: " << clusterY_idx.size() << std::endl;
//     std::cout << "clusterX_idx size: " << clusterX_idx.size() << std::endl;
//     for (int idx : clusterZ_idx) {
//         std::cout << "features of potential clusterZ planes: " << features[idx] << std::endl;
//     }

//     // Kmeans迭代收敛
//     while (!converged && iter < 50)
//     {
//         // 第一步：分配标签（每个平面归到最近的聚类中心）
//         for (int i = 0; i < n; i++)
//         {
//             float d0 = (features[i] - c0).norm();
//             float d1 = (features[i] - c1).norm();
//             float d2 = (features[i] - c2).norm();
//             if (d0 <= d1 && d0 <= d2) labels[i] = 0;
//             else if (d1 <= d0 && d1 <= d2) labels[i] = 1;
//             else labels[i] = 2;
//         }

//         // 第二步：更新聚类中心
//         Eigen::Vector3f new_c0(0,0,0), new_c1(0,0,0), new_c2(0,0,0);
//         int cnt0 = 0, cnt1 = 0, cnt2 = 0;
//         for (int i = 0; i < n; i++)
//         {
//             if (labels[i] == 0) { new_c0 += features[i]; cnt0++; }
//             else if (labels[i] == 1) { new_c1 += features[i]; cnt1++; }
//             else { new_c2 += features[i]; cnt2++; }
//         }
//         new_c0 = cnt0 > 0 ? new_c0 / cnt0 : c0;
//         new_c1 = cnt1 > 0 ? new_c1 / cnt1 : c1;
//         new_c2 = cnt2 > 0 ? new_c2 / cnt2 : c2;

//         // 检查收敛
//         if ((new_c0 - c0).norm() < 1e-4 && 
//             (new_c1 - c1).norm() < 1e-4 && 
//             (new_c2 - c2).norm() < 1e-4)
//         {
//             converged = true;
//         }
//         c0 = new_c0;
//         c1 = new_c1;
//         c2 = new_c2;
//         iter++;
//         std::cout << "iteration " << iter << " done." << std::endl;
//     }
//     std::cout << "final centroids: " << c0.transpose() << ", " << c1.transpose() << ", " << c2.transpose() << std::endl;
//     // 分配聚类结果（按轴归类）
//     for (int i = 0; i < n; i++)
//     {
//         if (labels[i] == 0) clusterX.push_back(planes[i]);
//         else if (labels[i] == 1) clusterY.push_back(planes[i]);
//         else clusterZ.push_back(planes[i]);
//     }

//     // 修正聚类标签（确保每个聚类对应正确的轴）
//     // 核心规则：X轴类的ext_ratio_x最大，Y轴类的ext_ratio_y最大，Z轴类的ext_ratio_z最大
//     float avg_x = 0, avg_y = 0, avg_z = 0;

//     for (const auto& p : clusterX) { 
//         avg_x += p.ext_ratio_x; 
//         avg_y += p.ext_ratio_y; 
//         avg_z += p.ext_ratio_z; 
//     }
//     avg_x /= clusterX.size(); 
//     avg_y /= clusterX.size(); 
//     avg_z /= clusterX.size();
//     std::vector<PlaneInfo> tmp;
//     if (avg_y > avg_x && avg_y > avg_z) { // in this case clusterX has largest avg_y, which should be clusterY
//         tmp = clusterX; 
//         clusterX = clusterY; 
//         clusterY = tmp; 
//     }
//     else if (avg_z > avg_x && avg_z > avg_y) { 
//         tmp = clusterX; 
//         clusterX = clusterZ; 
//         clusterZ = tmp; 
//     }

//     avg_x = avg_y = avg_z = 0;

//     for (const auto& p : clusterY) { 
//         avg_x += p.ext_ratio_x; 
//         avg_y += p.ext_ratio_y; 
//         avg_z += p.ext_ratio_z; 
//     }

//     avg_x /= clusterY.size(); 
//     avg_y /= clusterY.size(); 
//     avg_z /= clusterY.size();
    
//     if (avg_x > avg_y && avg_x > avg_z) { // in this case clusterX has largest avg_y, which should be clusterY
//         tmp = clusterY; 
//         clusterY = clusterX; 
//         clusterX = tmp; 
//     }
//     else if (avg_z > avg_x && avg_z > avg_y) { 
//         tmp = clusterY; 
//         clusterY = clusterZ; 
//         clusterZ = tmp; 
//     }

//     avg_x = avg_y = avg_z = 0;

//     for (const auto& p : clusterZ) { 
//         avg_x += p.ext_ratio_x; 
//         avg_y += p.ext_ratio_y; 
//         avg_z += p.ext_ratio_z; 
//     }

//     avg_x /= clusterY.size(); 
//     avg_y /= clusterY.size(); 
//     avg_z /= clusterY.size();
    
//     if (avg_x > avg_y && avg_x > avg_z) { // in this case clusterX has largest avg_y, which should be clusterY
//         tmp = clusterZ; 
//         clusterZ = clusterX; 
//         clusterX = tmp; 
//     }
//     else if (avg_y > avg_x && avg_y > avg_z) { 
//         tmp = clusterZ; 
//         clusterZ = clusterY; 
//         clusterY = tmp; 
//     }

//     // ========== 新增：聚类验证 ==========
//     // 1. 定义函数：计算单个聚类的三轴延展度均值
//     auto calcClusterAvg = [](const std::vector<PlaneInfo>& cluster) -> Eigen::Vector3f {
//         if (cluster.empty()) return Eigen::Vector3f(0,0,0);
//         float sum_x = 0, sum_y = 0, sum_z = 0;
//         for (const auto& p : cluster) {
//             sum_x += p.ext_ratio_x;
//             sum_y += p.ext_ratio_y;
//             sum_z += p.ext_ratio_z;
//         }
//         int size = cluster.size();
//         return Eigen::Vector3f(sum_x/size, sum_y/size, sum_z/size);
//     };

//     // 2. 计算三个聚类的均值
//     Eigen::Vector3f avgX = calcClusterAvg(clusterX); // clusterX的(ext_x, ext_y, ext_z)均值
//     Eigen::Vector3f avgY = calcClusterAvg(clusterY); // clusterY的均值
//     Eigen::Vector3f avgZ = calcClusterAvg(clusterZ); // clusterZ的均值

//     // 3. 打印可视化对比表格（直观！）
//     std::cout << "\n=====================================" << std::endl;
//     std::cout << "          聚类结果验证表（均值）          " << std::endl;
//     std::cout << "=====================================" << std::endl;
//     std::cout << std::fixed << std::setprecision(3); // 保留3位小数
//     std::cout << "聚类名称       | ext_ratio_x | ext_ratio_y | ext_ratio_z | 主导特征" << std::endl;
//     std::cout << "-------------------------------------" << std::endl;
//     // 打印clusterX
//     std::string domX = (avgX[0] > avgX[1] && avgX[0] > avgX[2]) ? "ext_x ✔️" : 
//                        (avgX[1] > avgX[0] && avgX[1] > avgX[2]) ? "ext_y ❌" : "ext_z ❌";
//     std::cout << "clusterX(前后墙) | " << avgX[0] << "      | " << avgX[1] << "      | " << avgX[2] << "      | " << domX << std::endl;
//     // 打印clusterY
//     std::string domY = (avgY[1] > avgY[0] && avgY[1] > avgY[2]) ? "ext_y ✔️" : 
//                        (avgY[0] > avgY[1] && avgY[0] > avgY[2]) ? "ext_x ❌" : "ext_z ❌";
//     std::cout << "clusterY(地/天花) | " << avgY[0] << "      | " << avgY[1] << "      | " << avgY[2] << "      | " << domY << std::endl;
//     // 打印clusterZ
//     std::string domZ = (avgZ[2] > avgZ[0] && avgZ[2] > avgZ[1]) ? "ext_z ✔️" : 
//                        (avgZ[0] > avgZ[2] && avgZ[0] > avgZ[1]) ? "ext_x ❌" : "ext_y ❌";
//     std::cout << "clusterZ(左右墙) | " << avgZ[0] << "      | " << avgZ[1] << "      | " << avgZ[2] << "      | " << domZ << std::endl;
//     std::cout << "=====================================\n" << std::endl;

//     // 4. 自动校验并给出总结提示
//     bool isXCorrect = (avgX[0] > avgX[1] && avgX[0] > avgX[2]);
//     bool isYCorrect = (avgY[1] > avgY[0] && avgY[1] > avgY[2]);
//     bool isZCorrect = (avgZ[2] > avgZ[0] && avgZ[2] > avgZ[1]);

//     if (isXCorrect && isYCorrect && isZCorrect) {
//         std::cout << "✅ 聚类验证通过！所有聚类的主导特征符合预期。" << std::endl;
//     } else {
//         std::cout << "❌ 聚类验证失败！部分聚类的主导特征不符合预期：" << std::endl;
//         if (!isXCorrect) std::cout << "  - clusterX应该以ext_ratio_x为主导特征（最大）" << std::endl;
//         if (!isYCorrect) std::cout << "  - clusterY应该以ext_ratio_y为主导特征（最大）" << std::endl;
//         if (!isZCorrect) std::cout << "  - clusterZ应该以ext_ratio_z为主导特征（最大）" << std::endl;
//     }

//     std::cout << "✅ Kmeans三分聚类完成：" << std::endl;
//     std::cout << "  - X轴延展类（Y-Z平面）：" << clusterX.size() << "个平面（前后墙）" << std::endl;
//     std::cout << "  - Y轴延展类（X-Z平面）：" << clusterY.size() << "个平面（地面/天花板）" << std::endl;
//     std::cout << "  - Z轴延展类（X-Y平面）：" << clusterZ.size() << "个平面（左右墙）" << std::endl;
// }

// // ---------------------- 3. 点云读取与预处理（保留） ----------------------
// PointCloudT::Ptr loadAndPreprocessCloud(const std::string& path)
// {
//     PointCloudT::Ptr cloud(new PointCloudT);
//     if (path.substr(path.find_last_of(".") + 1) == "pcd")
//     {
//         pcl::io::loadPCDFile<PointT>(path, *cloud);
//     }
//     else if (path.substr(path.find_last_of(".") + 1) == "ply")
//     {
//         pcl::io::loadPLYFile<PointT>(path, *cloud);
//     }
//     std::cout << "原始点云数量: " << cloud->size() << std::endl;

//     PointCloudT::Ptr cloud_down(new PointCloudT);
//     pcl::VoxelGrid<PointT> voxel;
//     voxel.setInputCloud(cloud);
//     voxel.setLeafSize(DOWNSAMPLE_VOXEL_SIZE, DOWNSAMPLE_VOXEL_SIZE, DOWNSAMPLE_VOXEL_SIZE);
//     voxel.filter(*cloud_down);

//     PointCloudT::Ptr cloud_clean(new PointCloudT);
//     pcl::StatisticalOutlierRemoval<PointT> sor;
//     sor.setInputCloud(cloud_down);
//     sor.setMeanK(20); //20
//     sor.setStddevMulThresh(2.5); //2
//     sor.filter(*cloud_clean);

//     std::cout << "预处理后点云数量: " << cloud_clean->size() << std::endl;
//     return cloud_clean;
// }

// // ---------------------- 4. 拟合平面（扩展三轴特征计算） ----------------------
// std::vector<PlaneInfo> fitFixedNumPlanes(PointCloudT::Ptr& cloud)
// {
//     std::vector<PlaneInfo> planes;
//     PointCloudT::Ptr cloud_remaining(new PointCloudT);
//     *cloud_remaining = *cloud;

//     pcl::SACSegmentation<PointT> seg;
//     pcl::ExtractIndices<PointT> extract;
//     pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//     pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);

//     seg.setOptimizeCoefficients(true);
//     seg.setModelType(pcl::SACMODEL_PLANE);
//     seg.setMethodType(pcl::SAC_RANSAC);
//     seg.setDistanceThreshold(RANSAC_DIST_THRESHOLD);
//     seg.setMaxIterations(RANSAC_MAX_ITER);

//     int fit_count = 0;
//     while (fit_count < FIT_PLANE_NUM && cloud_remaining->size() > MIN_PLANE_POINTS)
//     {
//         seg.setInputCloud(cloud_remaining);
//         seg.segment(*inliers, *coeffs);
//         if (inliers->indices.size() < MIN_PLANE_POINTS) break;

//         PointCloudT::Ptr plane_cloud(new PointCloudT);
//         extract.setInputCloud(cloud_remaining);
//         extract.setIndices(inliers);
//         extract.setNegative(false);
//         extract.filter(*plane_cloud);

//         PlaneInfo plane;
//         plane.plane_eq = Eigen::Vector4f(coeffs->values[0], coeffs->values[1], coeffs->values[2], coeffs->values[3]);
//         plane.inlier_cloud = plane_cloud;
//         plane.point_num = plane_cloud->size();

//         // 转换点云为Eigen格式
//         for (const auto& p : *plane_cloud) plane.points.emplace_back(p.x, p.y, p.z);
//         // 计算三轴延展度+极值（核心升级）
//         calculatePlaneExtFeatures(plane);

//         planes.push_back(plane);
//         extract.setNegative(true);
//         extract.filter(*cloud_remaining);
//         fit_count++;
//     }
//     std::cout << "✅ 拟合平面数量: " << planes.size() << "个 (目标" << FIT_PLANE_NUM << "个)" << std::endl;
//     return planes;
// }

// // ---------------------- 5. 核心升级：按轴极值筛选房间6个主面 ----------------------
// void filterRoomPlanes(const std::vector<PlaneInfo>& allPlanes,
//                       PlaneInfo& ground, PlaneInfo& ceiling,  // Y轴类：地面（Y最小）、天花板（Y最大）
//                       PlaneInfo& wallXmin, PlaneInfo& wallXmax, // X轴类：X最小墙、X最大墙
//                       PlaneInfo& wallZmin, PlaneInfo& wallZmax) // Z轴类：Z最小墙、Z最大墙
// {
//     // 第一步：Kmeans三分聚类
//     std::vector<PlaneInfo> clusterX, clusterY, clusterZ;
//     kmeans3DCluster(allPlanes, clusterX, clusterY, clusterZ);

//     // 第二步：筛选每类中點云最多的前2个平面（保证主面）
//     auto sortByPointNum = [](const PlaneInfo& a, const PlaneInfo& b) { return a.point_num > b.point_num; };
//     if (clusterX.size() > KEEP_PER_CLUSTER) std::sort(clusterX.begin(), clusterX.end(), sortByPointNum), clusterX.resize(KEEP_PER_CLUSTER);
//     if (clusterY.size() > KEEP_PER_CLUSTER) std::sort(clusterY.begin(), clusterY.end(), sortByPointNum), clusterY.resize(KEEP_PER_CLUSTER);
//     if (clusterZ.size() > KEEP_PER_CLUSTER) std::sort(clusterZ.begin(), clusterZ.end(), sortByPointNum), clusterZ.resize(KEEP_PER_CLUSTER);

//     // 第三步：Y轴类（地面/天花板）- 按Y极值筛选
//     float minY = 1e9, maxY = -1e9;
//     int groundIdx = 0, ceilingIdx = 0;
//     for (int i = 0; i < clusterY.size(); i++)
//     {
//         if (clusterY[i].avg_y < minY) { 
//             minY = clusterY[i].avg_y; 
//             groundIdx = i; 
//         }
//         if (clusterY[i].avg_y > maxY) { 
//             maxY = clusterY[i].avg_y; 
//             ceilingIdx = i; 
//         }
//     }
//     ground = clusterY[groundIdx];
//     ceiling = clusterY[ceilingIdx];
//     std::cout << "✅ 地面/天花板筛选完成：" << std::endl;
//     std::cout << "  - 地面：Y均值=" << ground.avg_y << " | 点数量=" << ground.point_num << std::endl;
//     std::cout << "  - 天花板：Y均值=" << ceiling.avg_y << " | 点数量=" << ceiling.point_num << std::endl;

//     // 第四步：X轴类（前后墙）- 按X极值筛选
//     float minX = 1e9, maxX = -1e9;
//     int xminIdx = 0, xmaxIdx = 0;
//     for (int i = 0; i < clusterX.size(); i++)
//     {
//         if (clusterX[i].avg_x < minX) { 
//             minX = clusterX[i].avg_x; 
//             xminIdx = i; 
//         }
//         if (clusterX[i].avg_x > maxX) { 
//             maxX = clusterX[i].avg_x; 
//             xmaxIdx = i; 
//         }
//     }
//     wallXmin = clusterX[xminIdx];
//     wallXmax = clusterX[xmaxIdx];
//     std::cout << "✅ X轴墙面筛选完成：" << std::endl;
//     std::cout << "  - X最小墙：X均值=" << wallXmin.avg_x << " | 点数量=" << wallXmin.point_num << std::endl;
//     std::cout << "  - X最大墙：X均值=" << wallXmax.avg_x << " | 点数量=" << wallXmax.point_num << std::endl;

//     // 第五步：Z轴类（左右墙）- 按Z极值筛选
//     float minZ = 1e9, maxZ = -1e9;
//     int zminIdx = 0, zmaxIdx = 0;
//     for (int i = 0; i < clusterZ.size(); i++)
//     {
//         if (clusterZ[i].avg_z < minZ) { minZ = clusterZ[i].avg_z; zminIdx = i; }
//         if (clusterZ[i].avg_z > maxZ) { maxZ = clusterZ[i].avg_z; zmaxIdx = i; }
//     }
//     wallZmin = clusterZ[zminIdx];
//     wallZmax = clusterZ[zmaxIdx];
//     std::cout << "✅ Z轴墙面筛选完成：" << std::endl;
//     std::cout << "  - Z最小墙：Z均值=" << wallZmin.avg_z << " | 点数量=" << wallZmin.point_num << std::endl;
//     std::cout << "  - Z最大墙：Z均值=" << wallZmax.avg_z << " | 点数量=" << wallZmax.point_num << std::endl;
// }

// // ---------------------- 6. 平面求交 + 尺寸计算（保留，优化精度） ----------------------
// bool planeToLine(const PlaneInfo& planeA, const PlaneInfo& planeB, LineInfo& line)
// {
//     Eigen::Vector3f n1(planeA.plane_eq[0], planeA.plane_eq[1], planeA.plane_eq[2]);
//     Eigen::Vector3f n2(planeB.plane_eq[0], planeB.plane_eq[1], planeB.plane_eq[2]);
//     Eigen::Vector3f dir = n1.cross(n2);
//     if (dir.norm() < 1e-6) return false;
//     dir.normalize();

//     Eigen::Matrix<float, 2, 3> A;
//     Eigen::Vector2f B;
//     A << n1.x(), n1.y(), n1.z(), n2.x(), n2.y(), n2.z();
//     B << -planeA.plane_eq[3], -planeB.plane_eq[3];
//     Eigen::Vector3f p0 = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);

//     line.p0 = p0; line.dir = dir; line.plane1 = planeA; line.plane2 = planeB;
//     return true;
// }

// bool lineIntersection(const LineInfo& line1, const LineInfo& line2, Eigen::Vector3f& point)
// {
//     Eigen::Vector3f p1 = line1.p0; Eigen::Vector3f d1 = line1.dir;
//     Eigen::Vector3f p2 = line2.p0; Eigen::Vector3f d2 = line2.dir;

//     Eigen::Matrix<float, 3, 2> A; 
//     A << d1, -d2;
//     Eigen::Vector3f b = p2 - p1;
//     Eigen::Vector2f t = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

//     Eigen::Vector3f p_inter1 = p1 + t[0] * d1;
//     Eigen::Vector3f p_inter2 = p2 + t[1] * d2;
//     if ((p_inter1 - p_inter2).norm() < 0.05f)
//     {
//         point = (p_inter1 + p_inter2) / 2.0f;
//         return true;
//     }
//     return false;
// }

// void calculateRoomSize(const PlaneInfo& ground, const PlaneInfo& ceiling,
//                       const PlaneInfo& wallXmin, const PlaneInfo& wallXmax,
//                       const PlaneInfo& wallZmin, const PlaneInfo& wallZmax,
//                       float& length, float& width, float& height)
// {
//     // 高度：Y轴极值差（最精准）
//     height = fabs(ceiling.avg_y - ground.avg_y);
//     // 长度：X轴墙面均值差
//     length = fabs(wallXmax.avg_x - wallXmin.avg_x);
//     // 宽度：Z轴墙面均值差
//     width = fabs(wallZmax.avg_z - wallZmin.avg_z);

//     // 可选：通过棱边交点验证（双重保障）
//     // std::vector<LineInfo> roomLines;
//     // LineInfo l;
//     // // 地面与各墙面的交线
//     // if(planeToLine(ground, wallXmin, l)) roomLines.push_back(l);
//     // if(planeToLine(ground, wallXmax, l)) roomLines.push_back(l);
//     // if(planeToLine(ground, wallZmin, l)) roomLines.push_back(l);
//     // if(planeToLine(ground, wallZmax, l)) roomLines.push_back(l);
//     // // 墙面之间的交线
//     // if(planeToLine(wallXmin, wallZmin, l)) roomLines.push_back(l);
//     // if(planeToLine(wallXmin, wallZmax, l)) roomLines.push_back(l);
//     // if(planeToLine(wallXmax, wallZmin, l)) roomLines.push_back(l);
//     // if(planeToLine(wallXmax, wallZmax, l)) roomLines.push_back(l);

//     // // 计算交点验证尺寸
//     // std::vector<Eigen::Vector3f> corners;
//     // for (int i = 0; i < roomLines.size(); i++)
//     //     for (int j = i + 1; j < roomLines.size(); j++)
//     //     {
//     //         Eigen::Vector3f point;
//     //         if (lineIntersection(roomLines[i], roomLines[j], point)) {
//     //             corners.push_back(point);
//     //         }
//     //     }

//     // // 用交点修正尺寸（取更精准的值）
//     // if (corners.size() >= 4)
//     // {
//     //     float x_min = 1e9, x_max = -1e9;
//     //     float z_min = 1e9, z_max = -1e9;
//     //     for (const auto& p : corners)
//     //     {
//     //         x_min = std::min(x_min, p.x()); x_max = std::max(x_max, p.x());
//     //         z_min = std::min(z_min, p.z()); z_max = std::max(z_max, p.z());
//     //     }
//     //     float len_check = x_max - x_min;
//     //     float wid_check = z_max - z_min;
//     //     // 取更稳定的结果（差值小的为优）
//     //     if (fabs(len_check - length) < 0.1) length = len_check;
//     //     if (fabs(wid_check - width) < 0.1) width = wid_check;
//     // }

//     // std::cout << "\n✅ 尺寸验证完成：" << std::endl;
//     // std::cout << "  - 高度（Y轴）：" << height << " m" << std::endl;
//     // std::cout << "  - 长度（X轴）：" << length << " m" << std::endl;
//     // std::cout << "  - 宽度（Z轴）：" << width << " m" << std::endl;
// }

// // ---------------------- 主函数（按新逻辑串联） ----------------------
// int main(int argc, char** argv)
// {
//     // 1. 读取预处理点云
//     PointCloudT::Ptr cloud = loadAndPreprocessCloud(PCD_PATH);
//     // 2. 拟合平面
//     std::vector<PlaneInfo> allPlanes = fitFixedNumPlanes(cloud);
//     // 3. 核心：筛选房间6个主面
//     PlaneInfo ground, ceiling, wallXmin, wallXmax, wallZmin, wallZmax;
//     filterRoomPlanes(allPlanes, ground, ceiling, wallXmin, wallXmax, wallZmin, wallZmax);
//     // 4. 计算房间尺寸
//     float L=0, W=0, H=0;
//     calculateRoomSize(ground, ceiling, wallXmin, wallXmax, wallZmin, wallZmax, L, W, H);
//     // 输出最终结果
//     std::cout << "\n===== ✅ 房间最终精准尺寸（三轴聚类算法） =====" << std::endl;
//     std::cout << "长度 (X轴) : " << std::fixed << std::setprecision(3) << L << " m" << std::endl;
//     std::cout << "宽度 (Z轴) : " << std::fixed << std::setprecision(3) << W << " m" << std::endl;
//     std::cout << "高度 (Y轴) : " << std::fixed << std::setprecision(3) << H << " m" << std::endl;
//     std::cout << "体积       : " << std::fixed << std::setprecision(3) << L*W*H << " m³" << std::endl;

//     return 0;
// }
