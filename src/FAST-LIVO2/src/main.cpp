#include "LIVMapper.h" // 包含 LIVMapper 类的声明

int main(int argc, char **argv)
{

	rclcpp::init(argc, argv); // 初始化 ROS 2 客户端库，解析命令行参数（如 --ros-args 等）

	rclcpp::NodeOptions options;								   // 创建节点选项对象，用于配置即将创建的 ROS 2 节点
	options.allow_undeclared_parameters(true);					   // 允许节点使用未声明的参数（ROS 2 中参数通常需要先声明）
	options.automatically_declare_parameters_from_overrides(true); // 自动从参数覆盖（命令行、YAML 文件等）中声明参数

	rclcpp::Node::SharedPtr nh; // 声明一个节点共享指针，先置空；稍后由 LIVMapper 构造函数内部创建节点并赋值

	// 创建 LIVMapper 对象
	// 参数1：nh —— 节点共享指针（按引用传入，构造函数内部会创建节点并让 nh 指向它）
	// 参数2："laserMapping" —— 节点名称
	// 参数3：options —— 节点选项
	LIVMapper mapper(nh, "laserMapping", options);

	// 使用已经创建好的节点 nh，初始化订阅者和发布者
	// 内部会创建订阅激光雷达、IMU、图像等话题的订阅者，以及发布里程计、点云等话题的发布者
	mapper.initializeSubscribersAndPublishers(nh);

	mapper.run(nh);		// 进入主运行循环：处理回调、执行 SLAM 算法、发布结果等
	rclcpp::shutdown(); // 关闭 ROS 2，清理资源

	return 0;
}