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
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

// 类型定义
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// 平面结构体 - 扩展三轴延展度特征
struct PlaneInfo
{
    Eigen::Vector4f plane_eq;      // 平面方程 ax+by+cz+d=0
    PointCloudT::Ptr inlier_cloud;  // 平面内点云
    std::vector<Eigen::Vector3f> points; // 平面点集(Eigen格式)
    float ext_ratio_x;             // X轴延展度 (越大→平面越贴近Y-Z平面)
    float ext_ratio_y;             // Y轴延展度 (越大→平面越贴近X-Z平面)
    float ext_ratio_z;             // Z轴延展度 (越大→平面越贴近X-Y平面)
    float min_x, max_x;            // 平面点X轴极值
    float min_y, max_y;            // 平面点Y轴极值
    float min_z, max_z;            // 平面点Z轴极值
    float avg_x, avg_y, avg_z;     // 平面点均值
    int point_num;                 // 平面点数量
};

// 空间直线结构体（保留）
struct LineInfo
{
    Eigen::Vector3f p0;        // 直线上一点
    Eigen::Vector3f dir;       // 直线方向向量（单位化）
    PlaneInfo plane1;          // 生成直线的平面1
    PlaneInfo plane2;          // 生成直线的平面2
};

// ===================== 全局配置参数（按你的需求调整） =====================
const std::string PCD_PATH = "/home/ricky/WCR_ws/dense_orbslam3/Examples/RGB-D/PointCloudMapping_RGBD.pcd";
const float DOWNSAMPLE_VOXEL_SIZE = 0.01f; //0.02f
const float RANSAC_DIST_THRESHOLD = 0.045f; //0.05f
const int RANSAC_MAX_ITER = 10000;
const int FIT_PLANE_NUM = 30;        // 拟合30个平面（足够覆盖房间6个主面+杂面）
const int MIN_PLANE_POINTS = 100;    // 过滤极小平面
const int CLUSTER_K = 3;             // ✅ 核心：Kmeans分3类（对应X/Y/Z三轴延展）
const int KEEP_PER_CLUSTER = 10;      // 每类保留点云最多的10个平面（刚好对应每个轴的正反两面）

// ---------------------- 1. 核心修改：方向余弦法计算三轴延展度 ----------------------
void calculatePlaneExtFeatures(PlaneInfo& plane)
{
    // 第一步：计算平面点在X/Y/Z轴的极值（保留原有逻辑）
    float x_min = 1e9, x_max = -1e9;
    float y_min = 1e9, y_max = -1e9;
    float z_min = 1e9, z_max = -1e9;
    float sum_x = 0, sum_y = 0, sum_z = 0;

    for (const auto& p : plane.points)
    {
        x_min = std::min(x_min, p.x()); x_max = std::max(x_max, p.x());
        y_min = std::min(y_min, p.y()); y_max = std::max(y_max, p.y());
        z_min = std::min(z_min, p.z()); z_max = std::max(z_max, p.z());
        sum_x += p.x(); sum_y += p.y(); sum_z += p.z();
    }

    // 赋值极值和均值（保留原有逻辑）
    plane.min_x = x_min; plane.max_x = x_max;
    plane.min_y = y_min; plane.max_y = y_max;
    plane.min_z = z_min; plane.max_z = z_max;
    plane.avg_x = sum_x / plane.points.size();
    plane.avg_y = sum_y / plane.points.size();
    plane.avg_z = sum_z / plane.points.size();

    // ===================== 核心修改：方向余弦法计算延展度 =====================
    // 1. 提取平面法向量并归一化（plane_eq的前3个值是法向量(a,b,c)）
    Eigen::Vector3f normal(plane.plane_eq[0], plane.plane_eq[1], plane.plane_eq[2]);
    normal.normalize(); // 归一化为单位法向量

    // 2. 计算法向量与X/Y/Z轴的夹角余弦（取绝对值，消除方向影响）
    float cos_x = std::abs(normal.x()); // 法向量与X轴的夹角余弦
    float cos_y = std::abs(normal.y()); // 法向量与Y轴的夹角余弦
    float cos_z = std::abs(normal.z()); // 法向量与Z轴的夹角余弦

    // 3. 计算延展度：(1 - 余弦值) / (余弦值 + eps)
    // 逻辑：余弦值越小→法向量越垂直于该轴→平面越贴近正交面→延展度越大
    float eps = 1e-3;
    // plane.ext_ratio_x = (1 - cos_x + eps) / (cos_x + eps); // 越大→贴近Y-Z平面（前后墙）
    // plane.ext_ratio_y = (1 - cos_y + eps) / (cos_y + eps); // 越大→贴近X-Z平面（地面/天花板）
    // plane.ext_ratio_z = (1 - cos_z + eps) / (cos_z + eps); // 越大→贴近X-Y平面（左右墙）
    plane.ext_ratio_x = cos_x; // 越大→贴近Y-Z平面（前后墙）
    plane.ext_ratio_y = cos_y; // 越大→贴近X-Z平面（地面/天花板）
    plane.ext_ratio_z = cos_z; // 越大→贴近X-Y平面（左右墙）
}

// ---------------------- 2. 核心升级：Kmeans三分聚类（基于三轴延展度） ----------------------
void kmeans3DCluster(const std::vector<PlaneInfo>& planes, 
                     std::vector<PlaneInfo>& clusterX,  // X轴延展类（贴近Y-Z平面：前后墙）
                     std::vector<PlaneInfo>& clusterY,  // Y轴延展类（贴近X-Z平面：地面/天花板）
                     std::vector<PlaneInfo>& clusterZ)  // Z轴延展类（贴近X-Y平面：左右墙）
{
    int n = planes.size();
    // 特征矩阵：每个平面3个特征（ext_ratio_x, ext_ratio_y, ext_ratio_z）
    std::vector<Eigen::Vector3f> features(n);
    for (int i = 0; i < n; i++)
    {
        features[i] = Eigen::Vector3f(planes[i].ext_ratio_x, planes[i].ext_ratio_y, planes[i].ext_ratio_z);
    }

    // 初始化3个聚类中心（选极值点，加速收敛）
    Eigen::Vector3f c0 = features[0];  // X轴类中心
    Eigen::Vector3f c1 = features[1];  // Y轴类中心
    Eigen::Vector3f c2 = features[2];  // Z轴类中心
    // 优化初始化：选特征差异最大的3个点
    float max_dist = 0;
    for (int i = 0; i < n; i++){

    }
    for (int i = 0; i < n; i++)
        for (int j = i+1; j < n; j++)
            for (int k = j+1; k < n; k++)
            {
                float d1 = (features[i] - features[j]).norm();
                float d2 = (features[i] - features[k]).norm();
                float d3 = (features[j] - features[k]).norm();
                if (d1 + d2 + d3 > max_dist)
                {
                    max_dist = d1 + d2 + d3;
                    c0 = features[i];
                    c1 = features[j];
                    c2 = features[k];
                }
            }
    std::cout << "initial centroids: " << c0.transpose() << ", " << c1.transpose() << ", " << c2.transpose() << std::endl;
    std::vector<int> labels(n, 0);
    bool converged = false;
    int iter = 0;

    std::vector<int> clusterZ_idx, clusterY_idx, clusterX_idx;
    for (int i=0; i<n; i++) {
        if (features[i].z() > 0.9) {
            clusterZ_idx.push_back(i);
        }
        else if (features[i].y() > 0.9) {
            clusterY_idx.push_back(i);
        }
        else if (features[i].x() > 0.9) {
            clusterX_idx.push_back(i);
        }
    }
    for (int idx : clusterX_idx) {
        std::cout << "numbers of point clods in clusterX: " << planes[idx].points.size() << std::endl;
    }
    for (int idx : clusterY_idx) {
        std::cout << "numbers of point clods in clusterY: " << planes[idx].points.size() << std::endl;
    }
    std::cout << "numbers of point clods in clusterZ: " << planes[clusterZ_idx[0]].points.size() << std::endl;
    
    std::cout << "clusterZ_idx size: " << clusterZ_idx.size() << std::endl;
    std::cout << "clusterY_idx size: " << clusterY_idx.size() << std::endl;
    std::cout << "clusterX_idx size: " << clusterX_idx.size() << std::endl;
    for (int idx : clusterZ_idx) {
        std::cout << "features of potential clusterZ planes: " << features[idx] << std::endl;
    }

    // Kmeans迭代收敛
    while (!converged && iter < 50)
    {
        // 第一步：分配标签（每个平面归到最近的聚类中心）
        for (int i = 0; i < n; i++)
        {
            float d0 = (features[i] - c0).norm();
            float d1 = (features[i] - c1).norm();
            float d2 = (features[i] - c2).norm();
            if (d0 <= d1 && d0 <= d2) labels[i] = 0;
            else if (d1 <= d0 && d1 <= d2) labels[i] = 1;
            else labels[i] = 2;
        }

        // 第二步：更新聚类中心
        Eigen::Vector3f new_c0(0,0,0), new_c1(0,0,0), new_c2(0,0,0);
        int cnt0 = 0, cnt1 = 0, cnt2 = 0;
        for (int i = 0; i < n; i++)
        {
            if (labels[i] == 0) { new_c0 += features[i]; cnt0++; }
            else if (labels[i] == 1) { new_c1 += features[i]; cnt1++; }
            else { new_c2 += features[i]; cnt2++; }
        }
        new_c0 = cnt0 > 0 ? new_c0 / cnt0 : c0;
        new_c1 = cnt1 > 0 ? new_c1 / cnt1 : c1;
        new_c2 = cnt2 > 0 ? new_c2 / cnt2 : c2;

        // 检查收敛
        if ((new_c0 - c0).norm() < 1e-4 && 
            (new_c1 - c1).norm() < 1e-4 && 
            (new_c2 - c2).norm() < 1e-4)
        {
            converged = true;
        }
        c0 = new_c0;
        c1 = new_c1;
        c2 = new_c2;
        iter++;
        std::cout << "iteration " << iter << " done." << std::endl;
    }
    std::cout << "final centroids: " << c0.transpose() << ", " << c1.transpose() << ", " << c2.transpose() << std::endl;
    // 分配聚类结果（按轴归类）
    for (int i = 0; i < n; i++)
    {
        if (labels[i] == 0) clusterX.push_back(planes[i]);
        else if (labels[i] == 1) clusterY.push_back(planes[i]);
        else clusterZ.push_back(planes[i]);
    }

    // 修正聚类标签（确保每个聚类对应正确的轴）
    // 核心规则：X轴类的ext_ratio_x最大，Y轴类的ext_ratio_y最大，Z轴类的ext_ratio_z最大
    float avg_x = 0, avg_y = 0, avg_z = 0;

    for (const auto& p : clusterX) { 
        avg_x += p.ext_ratio_x; 
        avg_y += p.ext_ratio_y; 
        avg_z += p.ext_ratio_z; 
    }
    avg_x /= clusterX.size(); 
    avg_y /= clusterX.size(); 
    avg_z /= clusterX.size();
    std::vector<PlaneInfo> tmp;
    if (avg_y > avg_x && avg_y > avg_z) { // in this case clusterX has largest avg_y, which should be clusterY
        tmp = clusterX; 
        clusterX = clusterY; 
        clusterY = tmp; 
    }
    else if (avg_z > avg_x && avg_z > avg_y) { 
        tmp = clusterX; 
        clusterX = clusterZ; 
        clusterZ = tmp; 
    }

    avg_x = avg_y = avg_z = 0;

    for (const auto& p : clusterY) { 
        avg_x += p.ext_ratio_x; 
        avg_y += p.ext_ratio_y; 
        avg_z += p.ext_ratio_z; 
    }

    avg_x /= clusterY.size(); 
    avg_y /= clusterY.size(); 
    avg_z /= clusterY.size();
    
    if (avg_x > avg_y && avg_x > avg_z) { // in this case clusterX has largest avg_y, which should be clusterY
        tmp = clusterY; 
        clusterY = clusterX; 
        clusterX = tmp; 
    }
    else if (avg_z > avg_x && avg_z > avg_y) { 
        tmp = clusterY; 
        clusterY = clusterZ; 
        clusterZ = tmp; 
    }

    avg_x = avg_y = avg_z = 0;

    for (const auto& p : clusterZ) { 
        avg_x += p.ext_ratio_x; 
        avg_y += p.ext_ratio_y; 
        avg_z += p.ext_ratio_z; 
    }

    avg_x /= clusterY.size(); 
    avg_y /= clusterY.size(); 
    avg_z /= clusterY.size();
    
    if (avg_x > avg_y && avg_x > avg_z) { // in this case clusterX has largest avg_y, which should be clusterY
        tmp = clusterZ; 
        clusterZ = clusterX; 
        clusterX = tmp; 
    }
    else if (avg_y > avg_x && avg_y > avg_z) { 
        tmp = clusterZ; 
        clusterZ = clusterY; 
        clusterY = tmp; 
    }

    // ========== 新增：聚类验证 ==========
    // 1. 定义函数：计算单个聚类的三轴延展度均值
    auto calcClusterAvg = [](const std::vector<PlaneInfo>& cluster) -> Eigen::Vector3f {
        if (cluster.empty()) return Eigen::Vector3f(0,0,0);
        float sum_x = 0, sum_y = 0, sum_z = 0;
        for (const auto& p : cluster) {
            sum_x += p.ext_ratio_x;
            sum_y += p.ext_ratio_y;
            sum_z += p.ext_ratio_z;
        }
        int size = cluster.size();
        return Eigen::Vector3f(sum_x/size, sum_y/size, sum_z/size);
    };

    // 2. 计算三个聚类的均值
    Eigen::Vector3f avgX = calcClusterAvg(clusterX); // clusterX的(ext_x, ext_y, ext_z)均值
    Eigen::Vector3f avgY = calcClusterAvg(clusterY); // clusterY的均值
    Eigen::Vector3f avgZ = calcClusterAvg(clusterZ); // clusterZ的均值

    // 3. 打印可视化对比表格（直观！）
    std::cout << "\n=====================================" << std::endl;
    std::cout << "          聚类结果验证表（均值）          " << std::endl;
    std::cout << "=====================================" << std::endl;
    std::cout << std::fixed << std::setprecision(3); // 保留3位小数
    std::cout << "聚类名称       | ext_ratio_x | ext_ratio_y | ext_ratio_z | 主导特征" << std::endl;
    std::cout << "-------------------------------------" << std::endl;
    // 打印clusterX
    std::string domX = (avgX[0] > avgX[1] && avgX[0] > avgX[2]) ? "ext_x ✔️" : 
                       (avgX[1] > avgX[0] && avgX[1] > avgX[2]) ? "ext_y ❌" : "ext_z ❌";
    std::cout << "clusterX(前后墙) | " << avgX[0] << "      | " << avgX[1] << "      | " << avgX[2] << "      | " << domX << std::endl;
    // 打印clusterY
    std::string domY = (avgY[1] > avgY[0] && avgY[1] > avgY[2]) ? "ext_y ✔️" : 
                       (avgY[0] > avgY[1] && avgY[0] > avgY[2]) ? "ext_x ❌" : "ext_z ❌";
    std::cout << "clusterY(地/天花) | " << avgY[0] << "      | " << avgY[1] << "      | " << avgY[2] << "      | " << domY << std::endl;
    // 打印clusterZ
    std::string domZ = (avgZ[2] > avgZ[0] && avgZ[2] > avgZ[1]) ? "ext_z ✔️" : 
                       (avgZ[0] > avgZ[2] && avgZ[0] > avgZ[1]) ? "ext_x ❌" : "ext_y ❌";
    std::cout << "clusterZ(左右墙) | " << avgZ[0] << "      | " << avgZ[1] << "      | " << avgZ[2] << "      | " << domZ << std::endl;
    std::cout << "=====================================\n" << std::endl;

    // 4. 自动校验并给出总结提示
    bool isXCorrect = (avgX[0] > avgX[1] && avgX[0] > avgX[2]);
    bool isYCorrect = (avgY[1] > avgY[0] && avgY[1] > avgY[2]);
    bool isZCorrect = (avgZ[2] > avgZ[0] && avgZ[2] > avgZ[1]);

    if (isXCorrect && isYCorrect && isZCorrect) {
        std::cout << "✅ 聚类验证通过！所有聚类的主导特征符合预期。" << std::endl;
    } else {
        std::cout << "❌ 聚类验证失败！部分聚类的主导特征不符合预期：" << std::endl;
        if (!isXCorrect) std::cout << "  - clusterX应该以ext_ratio_x为主导特征（最大）" << std::endl;
        if (!isYCorrect) std::cout << "  - clusterY应该以ext_ratio_y为主导特征（最大）" << std::endl;
        if (!isZCorrect) std::cout << "  - clusterZ应该以ext_ratio_z为主导特征（最大）" << std::endl;
    }

    std::cout << "✅ Kmeans三分聚类完成：" << std::endl;
    std::cout << "  - X轴延展类（Y-Z平面）：" << clusterX.size() << "个平面（前后墙）" << std::endl;
    std::cout << "  - Y轴延展类（X-Z平面）：" << clusterY.size() << "个平面（地面/天花板）" << std::endl;
    std::cout << "  - Z轴延展类（X-Y平面）：" << clusterZ.size() << "个平面（左右墙）" << std::endl;
}

// ---------------------- 3. 点云读取与预处理（保留） ----------------------
PointCloudT::Ptr loadAndPreprocessCloud(const std::string& path)
{
    PointCloudT::Ptr cloud(new PointCloudT);
    if (path.substr(path.find_last_of(".") + 1) == "pcd")
    {
        pcl::io::loadPCDFile<PointT>(path, *cloud);
    }
    else if (path.substr(path.find_last_of(".") + 1) == "ply")
    {
        pcl::io::loadPLYFile<PointT>(path, *cloud);
    }
    std::cout << "原始点云数量: " << cloud->size() << std::endl;

    PointCloudT::Ptr cloud_down(new PointCloudT);
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(DOWNSAMPLE_VOXEL_SIZE, DOWNSAMPLE_VOXEL_SIZE, DOWNSAMPLE_VOXEL_SIZE);
    voxel.filter(*cloud_down);

    PointCloudT::Ptr cloud_clean(new PointCloudT);
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud_down);
    sor.setMeanK(20); //20
    sor.setStddevMulThresh(2.5); //2
    sor.filter(*cloud_clean);

    std::cout << "预处理后点云数量: " << cloud_clean->size() << std::endl;
    return cloud_clean;
}

// ---------------------- 4. 拟合平面（扩展三轴特征计算） ----------------------
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

        // 转换点云为Eigen格式
        for (const auto& p : *plane_cloud) plane.points.emplace_back(p.x, p.y, p.z);
        // 计算三轴延展度+极值（核心升级）
        calculatePlaneExtFeatures(plane);

        planes.push_back(plane);
        extract.setNegative(true);
        extract.filter(*cloud_remaining);
        fit_count++;
    }
    std::cout << "✅ 拟合平面数量: " << planes.size() << "个 (目标" << FIT_PLANE_NUM << "个)" << std::endl;
    return planes;
}

// ---------------------- 5. 核心升级：按轴极值筛选房间6个主面 ----------------------
void filterRoomPlanes(const std::vector<PlaneInfo>& allPlanes,
                      PlaneInfo& ground, PlaneInfo& ceiling,  // Y轴类：地面（Y最小）、天花板（Y最大）
                      PlaneInfo& wallXmin, PlaneInfo& wallXmax, // X轴类：X最小墙、X最大墙
                      PlaneInfo& wallZmin, PlaneInfo& wallZmax) // Z轴类：Z最小墙、Z最大墙
{
    // 第一步：Kmeans三分聚类
    std::vector<PlaneInfo> clusterX, clusterY, clusterZ;
    kmeans3DCluster(allPlanes, clusterX, clusterY, clusterZ);

    // 第二步：筛选每类中點云最多的前2个平面（保证主面）
    auto sortByPointNum = [](const PlaneInfo& a, const PlaneInfo& b) { return a.point_num > b.point_num; };
    if (clusterX.size() > KEEP_PER_CLUSTER) std::sort(clusterX.begin(), clusterX.end(), sortByPointNum), clusterX.resize(KEEP_PER_CLUSTER);
    if (clusterY.size() > KEEP_PER_CLUSTER) std::sort(clusterY.begin(), clusterY.end(), sortByPointNum), clusterY.resize(KEEP_PER_CLUSTER);
    if (clusterZ.size() > KEEP_PER_CLUSTER) std::sort(clusterZ.begin(), clusterZ.end(), sortByPointNum), clusterZ.resize(KEEP_PER_CLUSTER);

    // 第三步：Y轴类（地面/天花板）- 按Y极值筛选
    float minY = 1e9, maxY = -1e9;
    int groundIdx = 0, ceilingIdx = 0;
    for (int i = 0; i < clusterY.size(); i++)
    {
        if (clusterY[i].avg_y < minY) { 
            minY = clusterY[i].avg_y; 
            groundIdx = i; 
        }
        if (clusterY[i].avg_y > maxY) { 
            maxY = clusterY[i].avg_y; 
            ceilingIdx = i; 
        }
    }
    ground = clusterY[groundIdx];
    ceiling = clusterY[ceilingIdx];
    std::cout << "✅ 地面/天花板筛选完成：" << std::endl;
    std::cout << "  - 地面：Y均值=" << ground.avg_y << " | 点数量=" << ground.point_num << std::endl;
    std::cout << "  - 天花板：Y均值=" << ceiling.avg_y << " | 点数量=" << ceiling.point_num << std::endl;

    // 第四步：X轴类（前后墙）- 按X极值筛选
    float minX = 1e9, maxX = -1e9;
    int xminIdx = 0, xmaxIdx = 0;
    for (int i = 0; i < clusterX.size(); i++)
    {
        if (clusterX[i].avg_x < minX) { 
            minX = clusterX[i].avg_x; 
            xminIdx = i; 
        }
        if (clusterX[i].avg_x > maxX) { 
            maxX = clusterX[i].avg_x; 
            xmaxIdx = i; 
        }
    }
    wallXmin = clusterX[xminIdx];
    wallXmax = clusterX[xmaxIdx];
    std::cout << "✅ X轴墙面筛选完成：" << std::endl;
    std::cout << "  - X最小墙：X均值=" << wallXmin.avg_x << " | 点数量=" << wallXmin.point_num << std::endl;
    std::cout << "  - X最大墙：X均值=" << wallXmax.avg_x << " | 点数量=" << wallXmax.point_num << std::endl;

    // 第五步：Z轴类（左右墙）- 按Z极值筛选
    float minZ = 1e9, maxZ = -1e9;
    int zminIdx = 0, zmaxIdx = 0;
    for (int i = 0; i < clusterZ.size(); i++)
    {
        if (clusterZ[i].avg_z < minZ) { minZ = clusterZ[i].avg_z; zminIdx = i; }
        if (clusterZ[i].avg_z > maxZ) { maxZ = clusterZ[i].avg_z; zmaxIdx = i; }
    }
    wallZmin = clusterZ[zminIdx];
    wallZmax = clusterZ[zmaxIdx];
    std::cout << "✅ Z轴墙面筛选完成：" << std::endl;
    std::cout << "  - Z最小墙：Z均值=" << wallZmin.avg_z << " | 点数量=" << wallZmin.point_num << std::endl;
    std::cout << "  - Z最大墙：Z均值=" << wallZmax.avg_z << " | 点数量=" << wallZmax.point_num << std::endl;
}

// ---------------------- 6. 平面求交 + 尺寸计算（保留，优化精度） ----------------------
bool planeToLine(const PlaneInfo& planeA, const PlaneInfo& planeB, LineInfo& line)
{
    Eigen::Vector3f n1(planeA.plane_eq[0], planeA.plane_eq[1], planeA.plane_eq[2]);
    Eigen::Vector3f n2(planeB.plane_eq[0], planeB.plane_eq[1], planeB.plane_eq[2]);
    Eigen::Vector3f dir = n1.cross(n2);
    if (dir.norm() < 1e-6) return false;
    dir.normalize();

    Eigen::Matrix<float, 2, 3> A;
    Eigen::Vector2f B;
    A << n1.x(), n1.y(), n1.z(), n2.x(), n2.y(), n2.z();
    B << -planeA.plane_eq[3], -planeB.plane_eq[3];
    Eigen::Vector3f p0 = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);

    line.p0 = p0; line.dir = dir; line.plane1 = planeA; line.plane2 = planeB;
    return true;
}

bool lineIntersection(const LineInfo& line1, const LineInfo& line2, Eigen::Vector3f& point)
{
    Eigen::Vector3f p1 = line1.p0; Eigen::Vector3f d1 = line1.dir;
    Eigen::Vector3f p2 = line2.p0; Eigen::Vector3f d2 = line2.dir;

    Eigen::Matrix<float, 3, 2> A; 
    A << d1, -d2;
    Eigen::Vector3f b = p2 - p1;
    Eigen::Vector2f t = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

    Eigen::Vector3f p_inter1 = p1 + t[0] * d1;
    Eigen::Vector3f p_inter2 = p2 + t[1] * d2;
    if ((p_inter1 - p_inter2).norm() < 0.05f)
    {
        point = (p_inter1 + p_inter2) / 2.0f;
        return true;
    }
    return false;
}

void calculateRoomSize(const PlaneInfo& ground, const PlaneInfo& ceiling,
                      const PlaneInfo& wallXmin, const PlaneInfo& wallXmax,
                      const PlaneInfo& wallZmin, const PlaneInfo& wallZmax,
                      float& length, float& width, float& height)
{
    // 高度：Y轴极值差（最精准）
    height = fabs(ceiling.avg_y - ground.avg_y);
    // 长度：X轴墙面均值差
    length = fabs(wallXmax.avg_x - wallXmin.avg_x);
    // 宽度：Z轴墙面均值差
    width = fabs(wallZmax.avg_z - wallZmin.avg_z);

    // 可选：通过棱边交点验证（双重保障）
    // std::vector<LineInfo> roomLines;
    // LineInfo l;
    // // 地面与各墙面的交线
    // if(planeToLine(ground, wallXmin, l)) roomLines.push_back(l);
    // if(planeToLine(ground, wallXmax, l)) roomLines.push_back(l);
    // if(planeToLine(ground, wallZmin, l)) roomLines.push_back(l);
    // if(planeToLine(ground, wallZmax, l)) roomLines.push_back(l);
    // // 墙面之间的交线
    // if(planeToLine(wallXmin, wallZmin, l)) roomLines.push_back(l);
    // if(planeToLine(wallXmin, wallZmax, l)) roomLines.push_back(l);
    // if(planeToLine(wallXmax, wallZmin, l)) roomLines.push_back(l);
    // if(planeToLine(wallXmax, wallZmax, l)) roomLines.push_back(l);

    // // 计算交点验证尺寸
    // std::vector<Eigen::Vector3f> corners;
    // for (int i = 0; i < roomLines.size(); i++)
    //     for (int j = i + 1; j < roomLines.size(); j++)
    //     {
    //         Eigen::Vector3f point;
    //         if (lineIntersection(roomLines[i], roomLines[j], point)) {
    //             corners.push_back(point);
    //         }
    //     }

    // // 用交点修正尺寸（取更精准的值）
    // if (corners.size() >= 4)
    // {
    //     float x_min = 1e9, x_max = -1e9;
    //     float z_min = 1e9, z_max = -1e9;
    //     for (const auto& p : corners)
    //     {
    //         x_min = std::min(x_min, p.x()); x_max = std::max(x_max, p.x());
    //         z_min = std::min(z_min, p.z()); z_max = std::max(z_max, p.z());
    //     }
    //     float len_check = x_max - x_min;
    //     float wid_check = z_max - z_min;
    //     // 取更稳定的结果（差值小的为优）
    //     if (fabs(len_check - length) < 0.1) length = len_check;
    //     if (fabs(wid_check - width) < 0.1) width = wid_check;
    // }

    // std::cout << "\n✅ 尺寸验证完成：" << std::endl;
    // std::cout << "  - 高度（Y轴）：" << height << " m" << std::endl;
    // std::cout << "  - 长度（X轴）：" << length << " m" << std::endl;
    // std::cout << "  - 宽度（Z轴）：" << width << " m" << std::endl;
}

// ---------------------- 主函数（按新逻辑串联） ----------------------
int main(int argc, char** argv)
{
    // 1. 读取预处理点云
    PointCloudT::Ptr cloud = loadAndPreprocessCloud(PCD_PATH);
    // 2. 拟合平面
    std::vector<PlaneInfo> allPlanes = fitFixedNumPlanes(cloud);
    // 3. 核心：筛选房间6个主面
    PlaneInfo ground, ceiling, wallXmin, wallXmax, wallZmin, wallZmax;
    filterRoomPlanes(allPlanes, ground, ceiling, wallXmin, wallXmax, wallZmin, wallZmax);
    // 4. 计算房间尺寸
    float L=0, W=0, H=0;
    calculateRoomSize(ground, ceiling, wallXmin, wallXmax, wallZmin, wallZmax, L, W, H);
    // 输出最终结果
    std::cout << "\n===== ✅ 房间最终精准尺寸（三轴聚类算法） =====" << std::endl;
    std::cout << "长度 (X轴) : " << std::fixed << std::setprecision(3) << L << " m" << std::endl;
    std::cout << "宽度 (Z轴) : " << std::fixed << std::setprecision(3) << W << " m" << std::endl;
    std::cout << "高度 (Y轴) : " << std::fixed << std::setprecision(3) << H << " m" << std::endl;
    std::cout << "体积       : " << std::fixed << std::setprecision(3) << L*W*H << " m³" << std::endl;

    return 0;
}
