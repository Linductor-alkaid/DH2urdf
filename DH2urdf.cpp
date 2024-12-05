#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <string>
#include <sstream>

using namespace std;

// 关节和连杆参数结构体
struct JointLinkParam {
    double theta;  // 关节角度
    double d;      // 连杆偏移
    double a;      // 连杆长度
    double alpha;  // 连杆扭转角度
    
    double link_radius;   // 连杆圆柱底面半径

    // 构造函数，给定合理的默认值
    JointLinkParam(double th = 0.0, double dd = 0.0, double aa = 0.5, double alpha_ = 0.0, 
                   double j_radius = 0.5, double j_length = 0.1, double l_radius = 0.05, double l_length = 0.5)
        : theta(th), d(dd), a(aa), alpha(alpha_), link_radius(l_radius) {}
};

// 计算旋转轴（根据alpha值）
void calculateAxis(double alpha, double &x, double &y, double &z) {
    // 如果alpha为0，旋转轴是沿X轴
    if (alpha == 0.0) {
        x = 1.0; y = 0.0; z = 0.0;
    } else {
        // 否则，按照DH参数的旋转矩阵计算旋转轴
        x = cos(alpha); // 旋转轴方向
        y = sin(alpha); // 旋转轴方向
        z = 0.0;         // 默认为0，因为只有绕X或Y轴旋转
    }
}

// 从文件读取DH参数
bool readDHParamsFromFile(const string &filename, vector<JointLinkParam> &params) {
    ifstream inputFile(filename + ".txt");
    
    if (!inputFile.is_open()) {
        cerr << "Failed to open the file: " << filename << endl;
        return false;
    }

    string line;
    while (getline(inputFile, line)) {
        // 跳过空行
        if (line.empty()) continue;

        // 解析DH参数，假设格式为: theta, d, a, alpha
        stringstream ss(line);
        double theta, d, a, alpha;
        char comma;  // 用于跳过逗号

        ss >> theta >> comma >> d >> comma >> a >> comma >> alpha;
        
        if (ss.fail()) {
            cerr << "Failed to parse line: " << line << endl;
            continue;
        }

        // 将解析的参数存入结构体
        params.push_back(JointLinkParam(theta, d, a, alpha));
    }

    inputFile.close();
    return true;
}

// 生成URDF文件
void generateURDF(const vector<JointLinkParam> &params, const string &filename) {
    ofstream urdfFile(filename);

    if (!urdfFile.is_open()) {
        cerr << "Failed to open file for writing URDF." << endl;
        return;
    }

    urdfFile << "<?xml version=\"1.0\"?>" << endl;
    urdfFile << "<robot name=\"robot\">" << endl;

    // 创建URDF链接和关节
    for (size_t i = 0; i < params.size(); ++i) {
        const JointLinkParam &param = params[i];

        // 关节和链接名称
        string jointName = "joint_" + to_string(i);
        string linkName = "link_" + to_string(i);

        // 计算关节的旋转轴
        double axisX, axisY, axisZ;
        calculateAxis(param.alpha, axisX, axisY, axisZ);

        // 输出URDF链接（外观和碰撞体为圆柱）
        urdfFile << "  <link name=\"" << linkName << "\">" << endl;

        // 连杆的外观：圆柱，底面圆半径是 link_radius，高度是 link_length
        urdfFile << "    <inertial>" << endl;
        urdfFile << "      <origin xyz=\""<< param.d << " " << 0 << " " << param.a/2 << "\" rpy=\"0 0 0\"/>" << endl;
        urdfFile << "    </inertial>" << endl;
        urdfFile << "    <visual>" << endl;
        urdfFile << "      <origin xyz=\""<< param.d << " " << 0 << " " << param.a/2 << "\" rpy=\"0 0 0\"/>" << endl;
        urdfFile << "      <geometry>" << endl;
        urdfFile << "        <cylinder radius=\"" << param.link_radius << "\" length=\"" << param.a << "\"/>" << endl;
        urdfFile << "      </geometry>" << endl;
        urdfFile << "      <material name=\"default_material\">" << endl;
        urdfFile << "        <color rgba=\"0.0 0.0 1.0 1.0\"/>" << endl;
        urdfFile << "      </material>" << endl;
        urdfFile << "    </visual>" << endl;

        // 连杆的碰撞体：圆柱，底面圆半径是 link_radius，高度是 link_length
        urdfFile << "    <collision>" << endl;
        urdfFile << "      <origin xyz=\""<< param.d << " " << 0 << " " << param.a/2 << "\" rpy=\"0 0 0\"/>" << endl;
        urdfFile << "      <geometry>" << endl;
        urdfFile << "        <cylinder radius=\"" << param.link_radius << "\" length=\"" << param.a << "\"/>" << endl;
        urdfFile << "      </geometry>" << endl;
        urdfFile << "    </collision>" << endl;

        urdfFile << "  </link>" << endl;

        // 如果不是最后一个关节，创建关节
        if (i < params.size() - 1) {
            urdfFile << "  <joint name=\"" << jointName << "\" type=\"revolute\">" << endl;
            urdfFile << "    <parent link=\"" << linkName << "\"/>" << endl;
            urdfFile << "    <child link=\"link_" << i + 1 << "\"/>" << endl;
            urdfFile << "    <origin rpy=\"" << 0 << " " << 0 << " " << 0 << "\" xyz=\"" << param.d << " " << 0 << " " << param.a << "\"/>" << endl;
            urdfFile << "    <axis xyz=\"" << axisX << " " << axisY << " " << axisZ << "\"/>" << endl;
            urdfFile << "    <limit lower=\"-1.57\" upper=\"1.57\" effort=\"100.0\" velocity=\"1.0\"/>" << endl;
            urdfFile << "  </joint>" << endl;
        }
    }

    urdfFile << "</robot>" << endl;
    urdfFile.close();
    cout << "URDF file generated: " << filename << endl;
}

int main() {
    // 输入文件路径
    string inputFile;
    cout << "Enter the input file name (without .txt extension): ";
    cin >> inputFile;

    vector<JointLinkParam> params;

    // 从文件中读取DH参数
    if (!readDHParamsFromFile(inputFile, params)) {
        return 1; // 如果读取失败，则退出
    }

    // // 生成URDF文件
    // string filename;
    // cout << "Enter the output URDF file name (without extension): ";
    // cin >> filename;

    generateURDF(params, inputFile + ".urdf");

    return 0;
}
