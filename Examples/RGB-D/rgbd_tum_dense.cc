#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
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

    // 初始化 SLAM 和建图
    ORB_SLAM3::System SLAM(voc_path, param_path, ORB_SLAM3::System::RGBD, false);
    ORB_SLAM3::PointCloudMappingRGBD* pPointCloudMapping = new ORB_SLAM3::PointCloudMappingRGBD(0.05, 50, 1.0, 5000.0);  // TUM unit=5000

    // 逐帧处理
    cv::Mat imRGB, imD;
    for (int ni = 0; ni < nImages; ni++) {
        imRGB = cv::imread(vstrImageFilenamesRGB[ni], cv::IMREAD_UNCHANGED);
        imD = cv::imread(vstrImageFilenamesD[ni], cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if (imRGB.empty() || imD.empty()) continue;

        auto t1 = chrono::steady_clock::now();

        SLAM.TrackRGBD(imRGB, imD, tframe);
        
        auto t2 = chrono::steady_clock::now();
        double ttrack = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();

        ORB_SLAM3::KeyFrame* pKF = SLAM.GetTracker()->GetLastKeyFrame();

        if (pKF) {
            pPointCloudMapping->insertKeyFrame(pKF, imRGB, imD);
        }

        double T = 0.03;
        if (ttrack < T) usleep((T - ttrack) * 1e6);

        if (ni % 50 == 0) cout << "进度：" << ni << "/" << nImages << " | 耗时：" << ttrack * 1000 << "ms" << endl;
    }

    // 收尾
    SLAM.Shutdown();
    pPointCloudMapping->shutdown();
    delete pPointCloudMapping;
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    cout << "完成！点云保存为 PointCloudMapping_RGBD.pcd" << endl;
    return 0;
}