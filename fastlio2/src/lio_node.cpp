#include <mutex>
#include <vector>
#include <queue>
#include <memory>
#include <iostream>
#include <chrono>
#include <cmath>
#include <stdexcept>
// #include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include "utils.h"
#include "timing_trace.h"
#include "map_builder/commons.h"
#include "map_builder/map_builder.h"

#include <pcl_conversions/pcl_conversions.h>
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;
struct NodeConfig
{
    std::string imu_topic = "/livox/imu";
    std::string lidar_topic = "/livox/lidar";
    std::string body_frame = "body";
    std::string world_frame = "lidar";
    bool print_time_cost = false;
    bool publish_body_cloud = true;  // Functional input for PGO / ICP localization.
    bool publish_world_cloud = false;
    bool publish_path = false;
    double world_cloud_rate_hz = 5.0;
    int world_cloud_max_points = 2000;
    double path_rate_hz = 1.0;
    int path_max_poses = 1000;
    std::string timing_trace_path;
};
struct StateData
{
    bool lidar_pushed = false;
    std::mutex imu_mutex;
    std::mutex lidar_mutex;
    double last_lidar_time = -1.0;
    double last_imu_time = -1.0;
    std::deque<IMUData> imu_buffer;
    std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> lidar_buffer;
    nav_msgs::msg::Path path;
};

class LIONode : public rclcpp::Node
{
public:
    LIONode() : Node("lio_node")
    {
        RCLCPP_INFO(this->get_logger(), "LIO Node Started");
        loadParameters();
        m_trace = std::make_unique<TimingTrace>(m_node_config.timing_trace_path);

        m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(m_node_config.imu_topic, 10, std::bind(&LIONode::imuCB, this, std::placeholders::_1));
        m_lidar_sub = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(m_node_config.lidar_topic, 10, std::bind(&LIONode::lidarCB, this, std::placeholders::_1));

        // Reliable QoS remains compatible with PGO/localizer message_filters.
        // Bound optional-output queues so a slow visualizer cannot retain 10000 scans.
        m_body_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("body_cloud", 2);
        m_world_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("world_cloud", 2);
        m_path_pub = this->create_publisher<nav_msgs::msg::Path>("lio_path", 1);
        m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("lio_odom", 10);
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        m_state_data.path.poses.clear();
        m_state_data.path.header.frame_id = m_node_config.world_frame;

        m_kf = std::make_shared<IESKF>();
        m_builder = std::make_shared<MapBuilder>(m_builder_config, m_kf);
        m_timer = this->create_wall_timer(20ms, std::bind(&LIONode::timerCB, this));
    }
    ~LIONode() override {
        if (m_trace && !m_trace->flush())
            RCLCPP_ERROR(get_logger(), "Failed to flush timing trace");
    }

    void trace(const char *kind, uint64_t id, double stamp, size_t pending = 0,
               size_t imu_count = 0, double core_ms = 0, double output_ms = 0) {
        if (!m_trace->enabled()) return;
        TimingTrace::Event event{kind, id, stamp};
        event.steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        event.ros_ns = now().nanoseconds();
        event.pending = pending; event.imu_count = imu_count;
        event.core_ms = core_ms; event.output_ms = output_ms;
        if (core_ms > 0 && m_builder->status() == BuilderStatus::MAPPING) {
            event.iterations = m_kf->lastIterations();
            event.points = m_builder->lidar_processor()->downsampledPoints();
            event.map_points = m_builder->lidar_processor()->mapPoints();
        }
        m_trace->record(event);
    }

    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);

        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

        m_node_config.imu_topic = config["imu_topic"].as<std::string>();
        m_node_config.lidar_topic = config["lidar_topic"].as<std::string>();
        m_node_config.body_frame = config["body_frame"].as<std::string>();
        m_node_config.world_frame = config["world_frame"].as<std::string>();
        m_node_config.print_time_cost = config["print_time_cost"].as<bool>();
        rcl_interfaces::msg::ParameterDescriptor output_descriptor;
        output_descriptor.read_only = true;
        m_node_config.timing_trace_path = declare_parameter<std::string>(
            "timing_trace_path", config["timing_trace_path"].as<std::string>(""), output_descriptor);
        m_node_config.publish_body_cloud = declare_parameter<bool>(
            "publish_body_cloud", config["publish_body_cloud"].as<bool>(true), output_descriptor);
        m_node_config.publish_world_cloud = declare_parameter<bool>(
            "publish_world_cloud", config["publish_world_cloud"].as<bool>(false), output_descriptor);
        m_node_config.publish_path = declare_parameter<bool>(
            "publish_path", config["publish_path"].as<bool>(false), output_descriptor);
        m_node_config.world_cloud_rate_hz = declare_parameter<double>(
            "world_cloud_rate_hz", config["world_cloud_rate_hz"].as<double>(5.0), output_descriptor);
        m_node_config.world_cloud_max_points = declare_parameter<int>(
            "world_cloud_max_points", config["world_cloud_max_points"].as<int>(2000), output_descriptor);
        m_node_config.path_rate_hz = declare_parameter<double>(
            "path_rate_hz", config["path_rate_hz"].as<double>(1.0), output_descriptor);
        m_node_config.path_max_poses = declare_parameter<int>(
            "path_max_poses", config["path_max_poses"].as<int>(1000), output_descriptor);
        if (!std::isfinite(m_node_config.world_cloud_rate_hz) || m_node_config.world_cloud_rate_hz <= 0.0 ||
            m_node_config.world_cloud_max_points < 100 || !std::isfinite(m_node_config.path_rate_hz) ||
            m_node_config.path_rate_hz <= 0.0 || m_node_config.path_max_poses < 1)
            throw std::invalid_argument("Invalid optional-output rate or point/path bound");
        RCLCPP_INFO(get_logger(), "Optional outputs: body=%s, world=%s (%.1f Hz, at most %d points), path=%s",
                    m_node_config.publish_body_cloud ? "on" : "off",
                    m_node_config.publish_world_cloud ? "on" : "off",
                    m_node_config.world_cloud_rate_hz, m_node_config.world_cloud_max_points,
                    m_node_config.publish_path ? "on" : "off");

        m_builder_config.lidar_filter_num = config["lidar_filter_num"].as<int>();
        m_builder_config.lidar_min_range = config["lidar_min_range"].as<double>();
        m_builder_config.lidar_max_range = config["lidar_max_range"].as<double>();
        m_builder_config.scan_resolution = config["scan_resolution"].as<double>();
        m_builder_config.map_resolution = config["map_resolution"].as<double>();
        m_builder_config.cube_len = config["cube_len"].as<double>();
        m_builder_config.det_range = config["det_range"].as<double>();
        m_builder_config.move_thresh = config["move_thresh"].as<double>();
        m_builder_config.na = config["na"].as<double>();
        m_builder_config.ng = config["ng"].as<double>();
        m_builder_config.nba = config["nba"].as<double>();
        m_builder_config.nbg = config["nbg"].as<double>();

        m_builder_config.imu_init_num = config["imu_init_num"].as<int>();
        m_builder_config.near_search_num = config["near_search_num"].as<int>();
        m_builder_config.ieskf_max_iter = config["ieskf_max_iter"].as<int>();
        m_builder_config.gravity_align = config["gravity_align"].as<bool>();
        m_builder_config.esti_il = config["esti_il"].as<bool>();
        std::vector<double> t_il_vec = config["t_il"].as<std::vector<double>>();
        std::vector<double> r_il_vec = config["r_il"].as<std::vector<double>>();
        m_builder_config.t_il << t_il_vec[0], t_il_vec[1], t_il_vec[2];
        m_builder_config.r_il << r_il_vec[0], r_il_vec[1], r_il_vec[2], r_il_vec[3], r_il_vec[4], r_il_vec[5], r_il_vec[6], r_il_vec[7], r_il_vec[8];
        m_builder_config.lidar_cov_inv = config["lidar_cov_inv"].as<double>();
    }

    void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        trace("imu_received", 0, Utils::getSec(msg->header));
        std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);
        double timestamp = Utils::getSec(msg->header);
        if (timestamp < m_state_data.last_imu_time)
        {
            RCLCPP_WARN(this->get_logger(), "IMU Message is out of order");
            std::deque<IMUData>().swap(m_state_data.imu_buffer);
        }
        m_state_data.imu_buffer.emplace_back(V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z) * 10.0,
                                             V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                                             timestamp);
        m_state_data.last_imu_time = timestamp;
    }
    void lidarCB(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        trace("lidar_received", 0, Utils::getSec(msg->header));
        CloudType::Ptr cloud = Utils::livox2PCL(msg, m_builder_config.lidar_filter_num, m_builder_config.lidar_min_range, m_builder_config.lidar_max_range);
        std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
        double timestamp = Utils::getSec(msg->header);
        if (timestamp < m_state_data.last_lidar_time)
        {
            RCLCPP_WARN(this->get_logger(), "Lidar Message is out of order");
            std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>>().swap(m_state_data.lidar_buffer);
        }
        m_state_data.lidar_buffer.emplace_back(timestamp, cloud);
        m_state_data.last_lidar_time = timestamp;
    }

    bool syncPackage()
    {
        if (m_state_data.imu_buffer.empty() || m_state_data.lidar_buffer.empty())
            return false;
        if (!m_state_data.lidar_pushed)
        {
            m_package.cloud = m_state_data.lidar_buffer.front().second;
            std::sort(m_package.cloud->points.begin(), m_package.cloud->points.end(), [](PointType &p1, PointType &p2)
                      { return p1.curvature < p2.curvature; });
            m_package.cloud_start_time = m_state_data.lidar_buffer.front().first;
            m_package.cloud_end_time = m_package.cloud_start_time + m_package.cloud->points.back().curvature / 1000.0;
            m_state_data.lidar_pushed = true;
        }
        if (m_state_data.last_imu_time < m_package.cloud_end_time)
            return false;

        Vec<IMUData>().swap(m_package.imus);
        while (!m_state_data.imu_buffer.empty() && m_state_data.imu_buffer.front().time < m_package.cloud_end_time)
        {
            m_package.imus.emplace_back(m_state_data.imu_buffer.front());
            m_state_data.imu_buffer.pop_front();
        }
        m_state_data.lidar_buffer.pop_front();
        m_state_data.lidar_pushed = false;
        return true;
    }

    void publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, CloudType::Ptr cloud, std::string frame_id, const double &time)
    {
        if (pub->get_subscription_count() <= 0)
            return;
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = frame_id;
        cloud_msg.header.stamp = Utils::getTime(time);
        pub->publish(cloud_msg);
    }

    void publishOdometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub, std::string frame_id, std::string child_frame, const double &time)
    {
        if (odom_pub->get_subscription_count() <= 0)
            return;
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = frame_id;
        odom.header.stamp = Utils::getTime(time);
        odom.child_frame_id = child_frame;
        odom.pose.pose.position.x = m_kf->x().t_wi.x();
        odom.pose.pose.position.y = m_kf->x().t_wi.y();
        odom.pose.pose.position.z = m_kf->x().t_wi.z();
        Eigen::Quaterniond q(m_kf->x().r_wi);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        V3D vel = m_kf->x().r_wi.transpose() * m_kf->x().v;
        odom.twist.twist.linear.x = vel.x();
        odom.twist.twist.linear.y = vel.y();
        odom.twist.twist.linear.z = vel.z();
        odom_pub->publish(odom);
    }

    void publishPath(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub, std::string frame_id, const double &time)
    {
        if (path_pub->get_subscription_count() <= 0)
            return;
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frame_id;
        pose.header.stamp = Utils::getTime(time);
        pose.pose.position.x = m_kf->x().t_wi.x();
        pose.pose.position.y = m_kf->x().t_wi.y();
        pose.pose.position.z = m_kf->x().t_wi.z();
        Eigen::Quaterniond q(m_kf->x().r_wi);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        if (m_state_data.path.poses.size() >= static_cast<size_t>(m_node_config.path_max_poses))
            m_state_data.path.poses.erase(m_state_data.path.poses.begin());
        m_state_data.path.poses.push_back(pose);
        m_state_data.path.header.stamp = pose.header.stamp;
        path_pub->publish(m_state_data.path);
    }

    void broadCastTF(std::shared_ptr<tf2_ros::TransformBroadcaster> broad_caster, std::string frame_id, std::string child_frame, const double &time)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = frame_id;
        transformStamped.child_frame_id = child_frame;
        transformStamped.header.stamp = Utils::getTime(time);
        Eigen::Quaterniond q(m_kf->x().r_wi);
        V3D t = m_kf->x().t_wi;
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        broad_caster->sendTransform(transformStamped);
    }

    void timerCB()
    {
        if (!syncPackage())
            return;
        const auto scan_id = ++m_scan_id;
        trace("ready", scan_id, m_package.cloud_end_time, m_state_data.lidar_buffer.size(), m_package.imus.size());
        auto t1 = std::chrono::steady_clock::now();
        m_builder->process(m_package);
        auto t2 = std::chrono::steady_clock::now();
        const double core_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
        trace("core", scan_id, m_package.cloud_end_time, 0, m_package.imus.size(), core_ms);

        if (m_node_config.print_time_cost)
        {
            auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
            RCLCPP_WARN(this->get_logger(), "Time cost: %.2f ms", time_used);
        }

        if (m_builder->status() != BuilderStatus::MAPPING)
            return;

        // Deliver the estimator state before any optional point-cloud work.
        publishOdometry(m_odom_pub, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);
        broadCastTF(m_tf_broadcaster, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);

        // Check both the switch and demand BEFORE allocation / coordinate transformation.
        if (m_node_config.publish_body_cloud && m_body_cloud_pub->get_subscription_count() > 0)
        {
            CloudType::Ptr body_cloud = m_builder->lidar_processor()->transformCloud(
                m_package.cloud, m_kf->x().r_il, m_kf->x().t_il);
            publishCloud(m_body_cloud_pub, body_cloud, m_node_config.body_frame, m_package.cloud_end_time);
        }
        const auto now = std::chrono::steady_clock::now();
        if (m_node_config.publish_world_cloud && m_world_cloud_pub->get_subscription_count() > 0 &&
            std::chrono::duration<double>(now - m_last_world_cloud).count() >= 1.0 / m_node_config.world_cloud_rate_hz)
        {
            // Uniform selection is the same bounded sampling used by the map-quality gate.
            // Select before transforming, and never alter the estimator's input/map clouds.
            CloudType::Ptr selected = m_package.cloud;
            const size_t maximum = static_cast<size_t>(m_node_config.world_cloud_max_points);
            if (selected->size() > maximum)
            {
                const size_t stride = 1 + (selected->size() - 1) / maximum;
                selected.reset(new CloudType);
                selected->reserve(maximum);
                for (size_t i = 0; i < m_package.cloud->size(); i += stride)
                    selected->push_back(m_package.cloud->points[i]);
            }
            CloudType::Ptr world_cloud = m_builder->lidar_processor()->transformCloud(
                selected, m_builder->lidar_processor()->r_wl(), m_builder->lidar_processor()->t_wl());
            publishCloud(m_world_cloud_pub, world_cloud, m_node_config.world_frame, m_package.cloud_end_time);
            m_last_world_cloud = now;
        }
        if (m_node_config.publish_path && m_path_pub->get_subscription_count() > 0 &&
            std::chrono::duration<double>(now - m_last_path).count() >= 1.0 / m_node_config.path_rate_hz)
        {
            publishPath(m_path_pub, m_node_config.world_frame, m_package.cloud_end_time);
            m_last_path = now;
        }
        trace("output", scan_id, m_package.cloud_end_time, 0, m_package.imus.size(), core_ms,
              std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t2).count());
    }

private:
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr m_lidar_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_body_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_world_cloud_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;

    rclcpp::TimerBase::SharedPtr m_timer;
    std::unique_ptr<TimingTrace> m_trace;
    uint64_t m_scan_id = 0;
    std::chrono::steady_clock::time_point m_last_world_cloud{}, m_last_path{};
    StateData m_state_data;
    SyncPackage m_package;
    NodeConfig m_node_config;
    Config m_builder_config;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<MapBuilder> m_builder;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LIONode>());
    rclcpp::shutdown();
    return 0;
}
