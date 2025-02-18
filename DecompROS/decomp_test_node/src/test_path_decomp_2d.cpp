#include <decomp_test_node/txt_reader.hpp>
#include <decomp_rviz_plugins/data_ros_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <decomp_util/ellipsoid_decomp.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <decomp_ros_msgs/msg/ellipsoid_array.hpp>
#include <decomp_ros_msgs/msg/polyhedron_array.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// 使用标准库中的时间字面量
using namespace std::chrono_literals;

// 定义一个名为 TestPathDecomp2D 的类，继承自 rclcpp::Node
class TestPathDecomp2D : public rclcpp::Node
{
public:
    // 构造函数，初始化节点名称
    TestPathDecomp2D() : Node("decomp_test_node")
    {
        // 声明参数，并设置默认值
        this->declare_parameter("num_lat", num_lat_);
        this->declare_parameter("num_long", num_long_);
        this->declare_parameter("frame_id", frame_id_);

        // 获取参数值
        num_lat_ = this->get_parameter("num_lat").as_int();
        num_long_ = this->get_parameter("num_long").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();

        // 调用 testPathDecomp2D 函数
        testPathDecomp2D();

        // 创建一个发布者，发布 PointCloud2 消息
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);
        // 创建一个定时器，每 1000ms 调用一次 update 函数
        timer_ = this->create_wall_timer(1000ms, std::bind(&TestPathDecomp2D::update, this));

        // 创建一个发布者，发布 Path 消息
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", 1);
        // 创建一个发布者，发布 EllipsoidArray 消息
        es_pub_ = this->create_publisher<decomp_ros_msgs::msg::EllipsoidArray>("ellipsoid_array", 1);
        // 创建一个发布者，发布 PolyhedronArray 消息
        poly_pub_ = this->create_publisher<decomp_ros_msgs::msg::PolyhedronArray>("polyhedron_array", 1);
    }

    // 析构函数
    ~TestPathDecomp2D()
    {
    }

private:
    // 测试路径分解的函数
    void testPathDecomp2D()
    {
        // 定义圆面的半径
        const float radius = 5.0;
        // 遍历角度
        for (int i = 0; i < num_lat_; ++i) {
            // 计算当前角度的比例
            const float fr_lat = static_cast<float>(i) / static_cast<float>(num_lat_);
            // 计算当前角度的角度
            const float theta = fr_lat * M_PI * 2.0;

            // 创建一个点
            pcl::PointXYZRGB pt;
            // 设置点的颜色
            pt = pcl::PointXYZRGB(50 + fr_lat * 205, 255 - fr_lat * 100, 255);
            // 计算点的坐标
            pt.x = radius * cos(theta);
            pt.y = radius * sin(theta);
            pt.z = 0.0;

            // 计算点的颜色
            const uint8_t& pixel_r = 255 * fr_lat;
            const uint8_t& pixel_g = 255 * (1.0 - fr_lat);
            const uint8_t& pixel_b = 255;
            // 定义点的颜色
            uint32_t rgb = (static_cast<uint32_t>(pixel_r) << 16
                | static_cast<uint32_t>(pixel_g) << 8
                | static_cast<uint32_t>(pixel_b));
            pt.rgb = *reinterpret_cast<float*>(&rgb);

            // 将点添加到点云中
            cloud_.points.push_back(pt);
        }

        // 创建一个 PointCloud2 消息
        pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
        // 将点云转换为 ROS 消息
        pcl::toROSMsg(cloud_, *pc2_msg_);

        // 创建一个新的点云
        pcl::PointCloud<pcl::PointXYZRGB> cloud2;
        // 将 ROS 消息转换为点云
        pcl::fromROSMsg(*pc2_msg_, cloud2);
        // 输出点云的大小
        std::cout << cloud2.points.size() << "\n";

        // 设置消息的 frame_id
        pc2_msg_->header.frame_id = frame_id_;
    }

    // 定时器回调函数
    void update()
    {
        // 如果 pc2_msg_ 为空，直接返回
        if (!pc2_msg_) {
            return;
        }
        // 设置消息的时间戳
        pc2_msg_->header.stamp = this->now();
        // 发布消息
        pub_->publish(*pc2_msg_);
        // 将点云转换为向量
        vec_Vec3f obs = DecompROS::cloud_to_vec(pc2_msg_);
        // 定义二维障碍物向量
        vec_Vec2f obs2d;
        for (const auto &it : obs) {
            obs2d.push_back(it.topRows<2>());
        }

        // 定义路径
        vec_Vec2f path;

        // 设置路径
        path = {{1, 1}, {2, 4}, {4, 4}};
        // 将路径转换为 ROS 消息
        nav_msgs::msg::Path path_msg = DecompROS::vec_to_path(path);
        // 设置消息的 frame_id
        path_msg.header.frame_id = "map";
        // 发布路径消息
        path_pub_->publish(path_msg);

        // 使用椭球分解
        EllipsoidDecomp2D decomp_util;
        // 设置障碍物
        decomp_util.set_obs(obs2d);
        // 设置局部边界框
        decomp_util.set_local_bbox(Vec2f(1, 2));
        // 进行膨胀操作
        decomp_util.dilate(path); // 设置最大迭代次数为 10，固定路径

        // 发布可视化消息
        decomp_ros_msgs::msg::EllipsoidArray es_msg = DecompROS::ellipsoid_array_to_ros(decomp_util.get_ellipsoids());
        es_msg.header.frame_id = "map";
        es_pub_->publish(es_msg);

        decomp_ros_msgs::msg::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(decomp_util.get_polyhedrons());
        poly_msg.header.frame_id = "map";
        poly_msg.lifetime = rclcpp::Duration::from_seconds(3.0);
        poly_pub_->publish(poly_msg);

        // 转换为不等式约束 Ax < b
        auto polys = decomp_util.get_polyhedrons();
        for (size_t i = 0; i < path.size() - 1; i++) {
            // 计算路径中的点
            const auto pt_inside = (path[i] + path[i + 1]) / 2;
            // 创建线性约束
            LinearConstraint2D cs(pt_inside, polys[i].hyperplanes());
            // 输出当前索引
            printf("i: %zu\n", i);
            // 输出 A 矩阵
            std::cout << "A: " << cs.A() << std::endl;
            // 输出 b 向量
            std::cout << "b: " << cs.b() << std::endl;
            // 输出路径中的点
            std::cout << "point: " << path[i].transpose();
            // 判断点是否在约束内
            if (cs.inside(path[i]))
                std::cout << " is inside!" << std::endl;
            else
                std::cout << " is outside!" << std::endl;

            // 输出路径中的下一个点
            std::cout << "point: " << path[i + 1].transpose();
            // 判断点是否在约束内
            if (cs.inside(path[i + 1]))
                std::cout << " is inside!" << std::endl;
            else
                std::cout << " is outside!" << std::endl;
        }
    }

    // 纬度数量
    int num_lat_ = 100;
    // 经度数量
    int num_long_ = 100;
    // 坐标系 ID
    std::string frame_id_ = "map";

    // 点云
    pcl::PointCloud<pcl::PointXYZRGB> cloud_;
    // PointCloud2 消息
    sensor_msgs::msg::PointCloud2::SharedPtr pc2_msg_;
    // PointCloud2 消息的发布者
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;
    // Path 消息的发布者
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    // EllipsoidArray 消息的发布者
    rclcpp::Publisher<decomp_ros_msgs::msg::EllipsoidArray>::SharedPtr es_pub_;
    // PolyhedronArray 消息的发布者
    rclcpp::Publisher<decomp_ros_msgs::msg::PolyhedronArray>::SharedPtr poly_pub_;
};

// 主函数
int main(int argc, char* argv[])
{
    // 初始化 ROS 2
    rclcpp::init(argc, argv);
    // 设置标准输出为无缓冲模式
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    // 创建并运行节点
    rclcpp::spin(std::make_shared<TestPathDecomp2D>());
    // 关闭 ROS 2
    rclcpp::shutdown();
    return 0;
}