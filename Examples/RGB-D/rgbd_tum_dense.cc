#include <iostream>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <limits>
#include <sstream>
#include <vector>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <dirent.h>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

// ORB-SLAM3核心头文件
#include <System.h>
// 稠密建图模块头文件（专用RGB-D版本）
#include "PointCloudMappingRGBD.h"

using namespace std;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct RGBDFrameRecord {
    double timestamp = 0.0;
    string rgb_path;
    string depth_path;
};

bool FindRGBDFrameForTimestamp(const vector<RGBDFrameRecord>& frames,
                               double timestamp,
                               RGBDFrameRecord& out_record)
{
    if (frames.empty())
        return false;

    auto it = lower_bound(frames.begin(), frames.end(), timestamp,
                          [](const RGBDFrameRecord& frame, double t) {
                              return frame.timestamp < t;
                          });

    auto best = frames.end();
    double best_diff = numeric_limits<double>::max();

    if (it != frames.end())
    {
        best = it;
        best_diff = fabs(it->timestamp - timestamp);
    }

    if (it != frames.begin())
    {
        auto prev = it - 1;
        const double diff = fabs(prev->timestamp - timestamp);
        if (diff < best_diff)
        {
            best = prev;
            best_diff = diff;
        }
    }

    if (best == frames.end() || best_diff > 1e-4)
        return false;

    out_record = *best;
    return true;
}

// ====================== 自定义数据集文件排序（适配rgb/depth文件夹模式） ======================
class StringSort {
public:
    bool operator()(const string &a, const string &b) {
        return stoi(a.substr(0, a.find_last_of('.'))) < stoi(b.substr(0, b.find_last_of('.')));
    }
};

vector<string> listdir(const string& path) {
    vector<string> files;
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir(path.c_str())) != nullptr) {
        while ((ent = readdir(dir)) != nullptr) {
            if (ent->d_type == DT_REG) {
                files.emplace_back(ent->d_name);
            }
        }
        closedir(dir);
    } else {
        cerr << "Error: 无法打开文件夹 " << path << endl;
        return {};
    }
    sort(files.begin(), files.end(), StringSort());
    return files;
}

// ====================== TUM数据集关联文件读取 ======================
void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps) {
    ifstream fAssociation(strAssociationFilename.c_str());
    if (!fAssociation.is_open()) {
        cerr << "Error: 无法打开关联文件 " << strAssociationFilename << endl;
        return;
    }
    while (!fAssociation.eof()) {
        string s;
        getline(fAssociation, s);
        if (!s.empty()) {
            stringstream ss(s);
            double t;
            string sRGB, sD;
            ss >> t >> sRGB >> t >> sD;
            vTimestamps.push_back(t);
            vstrImageFilenamesRGB.push_back(sRGB);
            vstrImageFilenamesD.push_back(sD);
        }
    }
}

// ====================== 主函数 ======================
int main(int argc, char **argv) {
    string voc_path, param_path, root_dir, assoc_file;
    po::options_description desc("ORB-SLAM3 RGB-D稠密建图程序");
    desc.add_options()
        ("help,h", "打印帮助信息")
        ("voc,v", po::value<string>(&voc_path)->default_value("/home/robot/lib/ORB_Dense/Vocabulary/ORBvoc.txt"), "ORB词典路径")
        ("param,p", po::value<string>(&param_path)->default_value("/home/robot/lib/ORB_Dense/MyExample/rgbdslam.yaml"), "配置文件路径")
        ("root_dir,r", po::value<string>(&root_dir), "自定义数据集根目录（包含rgb/depth子文件夹）")
        ("tum,t", po::value<string>(), "TUM数据集模式：格式为 '序列路径:关联文件路径'");

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);
    } catch (const po::error& e) {
        cerr << "参数解析错误: " << e.what() << endl;
        cerr << desc << endl;
        return 1;
    }

    if (vm.count("help") || (!vm.count("root_dir") && !vm.count("tum"))) {
        cerr << desc << endl;
        cerr << "使用示例1：./rgbd_dense --root_dir /dataset/my_rgbd" << endl;
        cerr << "使用示例2：./rgbd_dense --tum /dataset/fr1/desk:/dataset/fr1/association.txt" << endl;
        return 1;
    }

    // 加载数据集
    vector<string> vstrImageFilenamesRGB, vstrImageFilenamesD;
    vector<double> vTimestamps;
    bool is_tum_mode = vm.count("tum");
    int nImages = 0;

    if (is_tum_mode) {
        string tum_str = vm["tum"].as<string>();
        size_t sep = tum_str.find(':');
        string seq_path = tum_str.substr(0, sep);
        assoc_file = tum_str.substr(sep + 1);
        LoadImages(assoc_file, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);
        nImages = vstrImageFilenamesRGB.size();
        for (int i = 0; i < nImages; i++) {
            vstrImageFilenamesRGB[i] = seq_path + "/" + vstrImageFilenamesRGB[i];
            vstrImageFilenamesD[i] = seq_path + "/" + vstrImageFilenamesD[i];
        }
    } else {
        fs::path root_path(root_dir);
        fs::path rgb_path = root_path / "rgb";
        fs::path depth_path = root_path / "depth";
        if (!fs::exists(rgb_path) || !fs::exists(depth_path)) {
            cerr << "Error: rgb/depth文件夹不存在！" << endl;
            return 1;
        }
        vstrImageFilenamesRGB = listdir(rgb_path.string());
        vstrImageFilenamesD = listdir(depth_path.string());
        nImages = vstrImageFilenamesRGB.size();
        if (nImages <= 0 || vstrImageFilenamesRGB.size() != vstrImageFilenamesD.size()) {
            cerr << "Error: RGB/深度图像数量不匹配！" << endl;
            return 1;
        }
        for (int i = 0; i < nImages; i++) {
            vstrImageFilenamesRGB[i] = (rgb_path / vstrImageFilenamesRGB[i]).string();
            vstrImageFilenamesD[i] = (depth_path / vstrImageFilenamesD[i]).string();
            vTimestamps.push_back(i * 0.03);
        }
    }

    cout << "成功加载 " << nImages << " 帧图像" << endl;

    vector<RGBDFrameRecord> rgbd_frames;
    rgbd_frames.reserve(nImages);
    for (int i = 0; i < nImages; ++i) {
        RGBDFrameRecord record;
        record.timestamp = vTimestamps[i];
        record.rgb_path = vstrImageFilenamesRGB[i];
        record.depth_path = vstrImageFilenamesD[i];
        rgbd_frames.push_back(record);
    }
    sort(rgbd_frames.begin(), rgbd_frames.end(),
         [](const RGBDFrameRecord& a, const RGBDFrameRecord& b) {
             return a.timestamp < b.timestamp;
         });

    // 初始化 SLAM 和建图
    ORB_SLAM3::System SLAM(voc_path, param_path, ORB_SLAM3::System::RGBD, true);
    ORB_SLAM3::PointCloudMappingRGBD* pPointCloudMapping = new ORB_SLAM3::PointCloudMappingRGBD(0.01, 50, 2.0, 5000.0);  // TUM unit=5000

    // 逐帧处理
    cv::Mat imRGB, imD;
    auto start_time = chrono::steady_clock::now();
    const int MAX_RUNTIME_SECONDS = 40;
    for (int ni = 0; ni < nImages; ni++) {
        imRGB = cv::imread(vstrImageFilenamesRGB[ni], cv::IMREAD_UNCHANGED);
        imD = cv::imread(vstrImageFilenamesD[ni], cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if (imRGB.empty() || imD.empty()) continue;

        // Check if maximum runtime reached
        auto current_time = chrono::steady_clock::now();
        double elapsed_seconds = chrono::duration_cast<chrono::duration<double>>(current_time - start_time).count();
        if (elapsed_seconds > MAX_RUNTIME_SECONDS) {
            cout << "Runtime limit reached (" << MAX_RUNTIME_SECONDS << " seconds). Shutting down..." << endl;
            break;
        }

        auto t1 = chrono::steady_clock::now();

        SLAM.TrackRGBD(imRGB, imD, tframe);
        
        auto t2 = chrono::steady_clock::now();
        double ttrack = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();

        double T = 0.03;
        if (ttrack < T) usleep((T - ttrack) * 1e6);

        if (ni % 50 == 0) cout << "进度：" << ni << "/" << nImages << " | 耗时：" << ttrack * 1000 << "ms" << endl;
    }

    // 收尾
    SLAM.Shutdown();

    vector<ORB_SLAM3::KeyFrame*> vpKFs = SLAM.GetAtlas()->GetAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), ORB_SLAM3::KeyFrame::lId);

    size_t enqueued_dense_kfs = 0;
    size_t skipped_bad_kfs = 0;
    size_t skipped_missing_images = 0;

    cout << "[Dense] SLAM 已关闭，开始按 Atlas 遍历关键帧并入队稠密融合，共 "
         << vpKFs.size() << " 个关键帧" << endl;

    for (ORB_SLAM3::KeyFrame* pKF : vpKFs) {
        if (!pKF || pKF->isBad()) {
            ++skipped_bad_kfs;
            continue;
        }

        RGBDFrameRecord record;
        if (!FindRGBDFrameForTimestamp(rgbd_frames, pKF->mTimeStamp, record)) {
            ++skipped_missing_images;
            cerr << "[Dense] 找不到关键帧 " << pKF->mnId
                 << " 对应的 RGB-D 图像，timestamp=" << fixed << setprecision(6)
                 << pKF->mTimeStamp << endl;
            continue;
        }

        imRGB = cv::imread(record.rgb_path, cv::IMREAD_UNCHANGED);
        imD = cv::imread(record.depth_path, cv::IMREAD_UNCHANGED);
        if (imRGB.empty() || imD.empty()) {
            ++skipped_missing_images;
            cerr << "[Dense] 读取关键帧 " << pKF->mnId << " 图像失败: "
                 << record.rgb_path << " / " << record.depth_path << endl;
            continue;
        }

        pPointCloudMapping->insertKeyFrame(pKF, imRGB, imD);
        ++enqueued_dense_kfs;

        if (enqueued_dense_kfs % 10 == 0) {
            cout << "[Dense] 已入队关键帧 " << enqueued_dense_kfs
                 << "/" << vpKFs.size() << endl;
        }
    }

    cout << "[Dense] 入队完成: " << enqueued_dense_kfs
         << " 个关键帧，跳过 bad=" << skipped_bad_kfs
         << "，缺图=" << skipped_missing_images
         << "。等待稠密线程融合完队列..." << endl;

    pPointCloudMapping->shutdown();
    delete pPointCloudMapping;
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    cout << "完成！点云保存为 PointCloudMapping_RGBD.pcd" << endl;
    return 0;
}
