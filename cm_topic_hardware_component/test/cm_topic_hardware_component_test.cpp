// Copyright 2025 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>

#if __has_include("hardware_interface/hardware_interface/version.h")
#include "hardware_interface/hardware_interface/version.h"
#else
#include "hardware_interface/version.h"
#endif
#include "hardware_interface/resource_manager.hpp"
#include "pal_statistics_msgs/msg/statistics_names.hpp"
#include "pal_statistics_msgs/msg/statistics_values.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

namespace
{
const auto TIME = rclcpp::Time(0);
const auto PERIOD = rclcpp::Duration::from_seconds(0.1);  // 0.1 seconds for easier math
}  // namespace

// Forward declaration
class TestableResourceManager;

class TestTopicBasedSystem : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    node_ = std::make_shared<rclcpp::Node>(::testing::UnitTest::GetInstance()->current_test_info()->name());

    INITIALIZE_ROS2_CONTROL_INTROSPECTION_REGISTRY(node_, hardware_interface::DEFAULT_INTROSPECTION_TOPIC,
                                                   hardware_interface::DEFAULT_REGISTRY_KEY);
  }

  void TearDown() override
  {
    node_.reset();
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Executor::SharedPtr executor_;
  std::unique_ptr<TestableResourceManager> rm_;
  rclcpp::Publisher<pal_statistics_msgs::msg::StatisticsNames>::SharedPtr names_publisher_;
  rclcpp::Publisher<pal_statistics_msgs::msg::StatisticsValues>::SharedPtr values_publisher_;

  /// Publish pal_statistics_msgs names + values
  /**
   *  names - names of interfaces
   *  points - vector of values
   */
  void publish(const std::vector<std::string>& if_names, const std::vector<double>& values, unsigned int version = 1u)
  {
    int wait_count = 0;
    const auto topic = names_publisher_->get_topic_name();
    while (node_->count_subscribers(topic) == 0)
    {
      if (wait_count >= 5)
      {
        auto error_msg = std::string("publishing to ") + topic + " but no node subscribes to it";
        throw std::runtime_error(error_msg);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      ++wait_count;
    }

    pal_statistics_msgs::msg::StatisticsNames names_msg;
    names_msg.names_version = version;
    names_msg.names = if_names;
    names_publisher_->publish(names_msg);

    pal_statistics_msgs::msg::StatisticsValues values_msg;
    values_msg.names_version = version;
    values_msg.values = values;
    values_publisher_->publish(values_msg);
  }

  void init_publishers(const std::string& hardware_component_name)
  {
    names_publisher_ = node_->create_publisher<pal_statistics_msgs::msg::StatisticsNames>(
        hardware_component_name + std::string("/names"), 10);
    values_publisher_ = node_->create_publisher<pal_statistics_msgs::msg::StatisticsValues>(
        hardware_component_name + std::string("/values"), 10);
  }

  /**
   * @brief wait_for_msg block until a new JointState is received.
   * Requires that the executor is not spinned elsewhere between the
   *  message publication and the call to this function
   */
  void wait_for_msg(const std::chrono::milliseconds& timeout = std::chrono::milliseconds{ 10 })
  {
    auto until = node_->get_clock()->now() + timeout;
    while (node_->get_clock()->now() < until)
    {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
  }

  void init_rm(const std::string& urdf)
  {
    // The API of the ResourceManager has changed in hardware_interface 4.13.0
#if HARDWARE_INTERFACE_VERSION_GTE(4, 13, 0)
    hardware_interface::ResourceManagerParams params;
    params.robot_description = urdf;
    params.clock = node_->get_clock();
    params.logger = node_->get_logger();
    params.executor = executor_;
    rm_ = std::make_unique<TestableResourceManager>(params, true);
#else
    rm_ = std::make_unique<TestableResourceManager>(urdf, true, false);
#endif
  }
};

class TestableResourceManager : public hardware_interface::ResourceManager
{
public:
  friend TestTopicBasedSystem;

  // The API of the ResourceManager has changed in hardware_interface 4.13.0
#if HARDWARE_INTERFACE_VERSION_GTE(4, 13, 0)
  explicit TestableResourceManager(const hardware_interface::ResourceManagerParams& params, bool load)
    : hardware_interface::ResourceManager(params, load)
  {
  }
#else
  explicit TestableResourceManager(const std::string& urdf, bool validate_interfaces = true, bool activate_all = false)
    : hardware_interface::ResourceManager(urdf, validate_interfaces, activate_all)
  {
  }
#endif

  explicit TestableResourceManager(rclcpp::Node::SharedPtr node)
    : hardware_interface::ResourceManager(node->get_node_clock_interface(), node->get_node_logging_interface())
  {
  }

  explicit TestableResourceManager(rclcpp::Node::SharedPtr node, const std::string& urdf, bool activate_all = false,
                                   unsigned int cm_update_rate = 100)
    : hardware_interface::ResourceManager(urdf, node->get_node_clock_interface(), node->get_node_logging_interface(),
                                          activate_all, cm_update_rate)
  {
  }
};

void set_components_state(TestableResourceManager& rm, const std::vector<std::string>& components,
                          const uint8_t state_id, const std::string& state_name)
{
  for (const auto& component : components)
  {
    rclcpp_lifecycle::State state(state_id, state_name);
    rm.set_component_state(component, state);
  }
}

void configure_components(TestableResourceManager& rm,
                          const std::vector<std::string>& components = { "GenericSystem2dof" })
{
  set_components_state(rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                       hardware_interface::lifecycle_state_names::INACTIVE);
};

void activate_components(TestableResourceManager& rm,
                         const std::vector<std::string>& components = { "GenericSystem2dof" })
{
  set_components_state(rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                       hardware_interface::lifecycle_state_names::ACTIVE);
};

void deactivate_components(TestableResourceManager& rm,
                           const std::vector<std::string>& components = { "GenericSystem2dof" })
{
  set_components_state(rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                       hardware_interface::lifecycle_state_names::INACTIVE);
};

TEST_F(TestTopicBasedSystem, topic_based_system_2dof)
{
  const std::string hardware_system_2dof_topic_based =
      R"(
   <ros2_control name="hardware_component_name" type="system">
    <hardware>
      <plugin>cm_topic_hardware_component/CMTopicSystem</plugin>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">0.2</param>
      </state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="joint2">
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">0.3</param>
      </state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="joint3">
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">0.1</param>
      </state_interface>
      <state_interface name="velocity"/>
    </joint>
    <gpio name="flange_analog_IOs">
      <command_interface name="analog_output1" data_type="double"/>
      <state_interface name="analog_output1"/>
      <state_interface name="analog_input1"/>
      <state_interface name="analog_input2"/>
    </gpio>
    <sensor name="force_sensor">
      <state_interface name="force.x">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="force.y">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="force.z">
        <param name="initial_value">0.0</param>
      </state_interface>
    </sensor>
  </ros2_control>
)";
  auto urdf =
      ros2_control_test_assets::urdf_head + hardware_system_2dof_topic_based + ros2_control_test_assets::urdf_tail;

  init_rm(urdf);
  init_publishers("hardware_component_name");

  // Activate components to get all interfaces available
  activate_components(*rm_, { "hardware_component_name" });

  // Check interfaces
  EXPECT_EQ(1u, rm_->system_components_size());
  ASSERT_EQ(12u, rm_->state_interface_keys().size());
  EXPECT_TRUE(rm_->state_interface_exists("joint1/position"));
  EXPECT_TRUE(rm_->state_interface_exists("joint1/velocity"));
  EXPECT_TRUE(rm_->state_interface_exists("joint2/position"));
  EXPECT_TRUE(rm_->state_interface_exists("joint2/velocity"));
  EXPECT_TRUE(rm_->state_interface_exists("joint3/position"));
  EXPECT_TRUE(rm_->state_interface_exists("joint3/velocity"));
  EXPECT_TRUE(rm_->state_interface_exists("joint3/velocity"));
  EXPECT_TRUE(rm_->state_interface_exists("flange_analog_IOs/analog_output1"));
  EXPECT_TRUE(rm_->state_interface_exists("flange_analog_IOs/analog_input1"));
  EXPECT_TRUE(rm_->state_interface_exists("flange_analog_IOs/analog_input2"));
  EXPECT_TRUE(rm_->state_interface_exists("force_sensor/force.x"));
  EXPECT_TRUE(rm_->state_interface_exists("force_sensor/force.y"));
  EXPECT_TRUE(rm_->state_interface_exists("force_sensor/force.z"));

  ASSERT_EQ(4u, rm_->command_interface_keys().size());
  EXPECT_TRUE(rm_->command_interface_exists("joint1/position"));
  EXPECT_TRUE(rm_->command_interface_exists("joint2/position"));
  EXPECT_TRUE(rm_->command_interface_exists("joint3/position"));
  EXPECT_TRUE(rm_->command_interface_exists("flange_analog_IOs/analog_output1"));

  // Check initial values
  hardware_interface::LoanedStateInterface j1_p_s = rm_->claim_state_interface("joint1/position");
  hardware_interface::LoanedStateInterface j1_v_s = rm_->claim_state_interface("joint1/velocity");
  hardware_interface::LoanedStateInterface j2_p_s = rm_->claim_state_interface("joint2/position");
  hardware_interface::LoanedStateInterface j2_v_s = rm_->claim_state_interface("joint2/velocity");
  hardware_interface::LoanedStateInterface j3_p_s = rm_->claim_state_interface("joint3/position");
  hardware_interface::LoanedStateInterface j3_v_s = rm_->claim_state_interface("joint3/velocity");
  hardware_interface::LoanedStateInterface g_ao_s = rm_->claim_state_interface("flange_analog_IOs/analog_output1");
  hardware_interface::LoanedStateInterface g_ai1_s = rm_->claim_state_interface("flange_analog_IOs/analog_input1");
  hardware_interface::LoanedStateInterface g_ai2_s = rm_->claim_state_interface("flange_analog_IOs/analog_input2");
  hardware_interface::LoanedStateInterface f_x_s = rm_->claim_state_interface("force_sensor/force.x");
  hardware_interface::LoanedStateInterface f_y_s = rm_->claim_state_interface("force_sensor/force.y");
  hardware_interface::LoanedStateInterface f_z_s = rm_->claim_state_interface("force_sensor/force.z");
  hardware_interface::LoanedCommandInterface j1_p_c = rm_->claim_command_interface("joint1/position");
  hardware_interface::LoanedCommandInterface j2_p_c = rm_->claim_command_interface("joint2/position");
  hardware_interface::LoanedCommandInterface j3_p_c = rm_->claim_command_interface("joint3/position");
  hardware_interface::LoanedCommandInterface g_a_c = rm_->claim_command_interface("flange_analog_IOs/analog_output1");

  EXPECT_EQ(j1_p_s.get_optional().value(), 0.2);
  EXPECT_EQ(j2_p_s.get_optional().value(), 0.3);
  EXPECT_EQ(j3_p_s.get_optional().value(), 0.1);
  EXPECT_TRUE(std::isnan(g_ao_s.get_optional().value()));
  EXPECT_TRUE(std::isnan(g_ai1_s.get_optional().value()));
  EXPECT_TRUE(std::isnan(g_ai2_s.get_optional().value()));
  EXPECT_EQ(f_x_s.get_optional().value(), 0.0);
  EXPECT_EQ(f_y_s.get_optional().value(), 0.0);
  EXPECT_EQ(f_z_s.get_optional().value(), 0.0);
  EXPECT_TRUE(std::isnan(j1_v_s.get_optional().value()));
  EXPECT_TRUE(std::isnan(j2_v_s.get_optional().value()));
  EXPECT_TRUE(std::isnan(j3_v_s.get_optional().value()));
  EXPECT_TRUE(std::isnan(j1_p_c.get_optional().value()));
  EXPECT_TRUE(std::isnan(j2_p_c.get_optional().value()));
  EXPECT_TRUE(std::isnan(j3_p_c.get_optional().value()));
  EXPECT_TRUE(std::isnan(g_a_c.get_optional().value()));

  // set some new values in commands
  ASSERT_TRUE(j1_p_c.set_value(0.11));
  ASSERT_TRUE(j2_p_c.set_value(0.12));
  ASSERT_TRUE(j3_p_c.set_value(0.13));
  ASSERT_TRUE(g_a_c.set_value(0.14));

  hardware_interface::return_type ret;
  ASSERT_NO_THROW(ret = rm_->read(TIME, PERIOD).result);
  ASSERT_EQ(ret, hardware_interface::return_type::OK);
  ASSERT_NO_THROW(ret = rm_->write(TIME, PERIOD).result);
  ASSERT_EQ(ret, hardware_interface::return_type::OK);

  // command is not propagated to state until topic is received
  EXPECT_EQ(j1_p_s.get_optional().value(), 0.2);
  EXPECT_EQ(j2_p_s.get_optional().value(), 0.3);
  EXPECT_EQ(j3_p_s.get_optional().value(), 0.1);
  EXPECT_TRUE(std::isnan(j1_v_s.get_optional().value()));
  EXPECT_TRUE(std::isnan(j2_v_s.get_optional().value()));
  EXPECT_TRUE(std::isnan(j3_v_s.get_optional().value()));

  EXPECT_EQ(j1_p_c.get_optional().value(), 0.11);
  EXPECT_EQ(j2_p_c.get_optional().value(), 0.12);
  EXPECT_EQ(j3_p_c.get_optional().value(), 0.13);
  EXPECT_EQ(g_a_c.get_optional().value(), 0.14);

  // unknown values should be ignored silently
  std::cout << "Publishing first set of values..." << std::endl;
  publish({ "state_interface.joint1/position", "state_interface.joint2/position", "state_interface.joint3/position",
            "state_interface.joint1/velocity", "state_interface.joint2/velocity", "state_interface.joint3/velocity",
            "state_interface.unknown/value", "other.unknown/value" },
          { 0.21, 0.22, 0.23, 0.31, 0.32, 0.33, 42.0, std::numeric_limits<double>::quiet_NaN() }, 1u);

  std::cout << "Published first set of values." << std::endl;
  wait_for_msg(std::chrono::milliseconds{ 100 });
  std::cout << "Finished waiting for message." << std::endl;

  ASSERT_NO_THROW(ret = rm_->read(TIME, PERIOD).result);
  ASSERT_EQ(ret, hardware_interface::return_type::OK);
  ASSERT_NO_THROW(ret = rm_->write(TIME, PERIOD).result);
  ASSERT_EQ(ret, hardware_interface::return_type::OK);

  // new states should have been updated from topic
  EXPECT_EQ(j1_p_s.get_optional().value(), 0.21);
  EXPECT_EQ(j2_p_s.get_optional().value(), 0.22);
  EXPECT_EQ(j3_p_s.get_optional().value(), 0.23);
  EXPECT_EQ(j1_v_s.get_optional().value(), 0.31);
  EXPECT_EQ(j2_v_s.get_optional().value(), 0.32);
  EXPECT_EQ(j3_v_s.get_optional().value(), 0.33);
  // other states remain unchanged
  EXPECT_TRUE(std::isnan(g_ao_s.get_optional().value()));
  EXPECT_TRUE(std::isnan(g_ai1_s.get_optional().value()));
  EXPECT_TRUE(std::isnan(g_ai2_s.get_optional().value()));
  EXPECT_EQ(f_x_s.get_optional().value(), 0.0);
  EXPECT_EQ(f_y_s.get_optional().value(), 0.0);
  EXPECT_EQ(f_z_s.get_optional().value(), 0.0);

  // commands should remain unchanged
  EXPECT_EQ(j1_p_c.get_optional().value(), 0.11);
  EXPECT_EQ(j2_p_c.get_optional().value(), 0.12);
  EXPECT_EQ(j3_p_c.get_optional().value(), 0.13);
  EXPECT_EQ(g_a_c.get_optional().value(), 0.14);

  // size of published values less then names
  publish({ "state_interface.joint1/position", "state_interface.joint2/position", "state_interface.joint3/position",
            "state_interface.joint1/velocity", "state_interface.joint2/velocity", "state_interface.joint3/velocity",
            "state_interface.flange_analog_IOs/analog_output1", "state_interface.force_sensor/force.x" },
          { 1.21, 1.22, 1.23, 1.31, 1.32, 1.33, 1.4 }, 2u);

  wait_for_msg(std::chrono::milliseconds{ 100 });

  ASSERT_NO_THROW(ret = rm_->read(TIME, PERIOD).result);
  ASSERT_EQ(ret, hardware_interface::return_type::OK);
  ASSERT_NO_THROW(ret = rm_->write(TIME, PERIOD).result);
  ASSERT_EQ(ret, hardware_interface::return_type::OK);

  // new states should have been updated from topic
  EXPECT_EQ(j1_p_s.get_optional().value(), 1.21);
  EXPECT_EQ(j2_p_s.get_optional().value(), 1.22);
  EXPECT_EQ(j3_p_s.get_optional().value(), 1.23);
  EXPECT_EQ(j1_v_s.get_optional().value(), 1.31);
  EXPECT_EQ(j2_v_s.get_optional().value(), 1.32);
  EXPECT_EQ(j3_v_s.get_optional().value(), 1.33);
  EXPECT_EQ(g_ao_s.get_optional().value(), 1.4);
  // wrong size of values in topic, should be ignored silently
  EXPECT_EQ(f_x_s.get_optional().value(), 0.0);
  // other states remain unchanged
  EXPECT_TRUE(std::isnan(g_ai1_s.get_optional().value()));
  EXPECT_TRUE(std::isnan(g_ai2_s.get_optional().value()));
  EXPECT_EQ(f_y_s.get_optional().value(), 0.0);
  EXPECT_EQ(f_z_s.get_optional().value(), 0.0);

  // fix size of published values
  publish({ "state_interface.joint1/position", "state_interface.joint2/position", "state_interface.joint3/position",
            "state_interface.joint1/velocity", "state_interface.joint2/velocity", "state_interface.joint3/velocity",
            "state_interface.flange_analog_IOs/analog_output1", "state_interface.force_sensor/force.x" },
          { 1.21, 1.22, 1.23, 1.31, 1.32, 1.33, 1.4, 1.5 }, 3u);

  wait_for_msg(std::chrono::milliseconds{ 100 });

  ASSERT_NO_THROW(ret = rm_->read(TIME, PERIOD).result);
  ASSERT_EQ(ret, hardware_interface::return_type::OK);
  ASSERT_NO_THROW(ret = rm_->write(TIME, PERIOD).result);
  ASSERT_EQ(ret, hardware_interface::return_type::OK);

  // new states should have been updated from topic
  EXPECT_EQ(f_x_s.get_optional().value(), 1.5);
  // other states remain unchanged
  EXPECT_EQ(j1_p_s.get_optional().value(), 1.21);
  EXPECT_EQ(j2_p_s.get_optional().value(), 1.22);
  EXPECT_EQ(j3_p_s.get_optional().value(), 1.23);
  EXPECT_EQ(j1_v_s.get_optional().value(), 1.31);
  EXPECT_EQ(j2_v_s.get_optional().value(), 1.32);
  EXPECT_EQ(j3_v_s.get_optional().value(), 1.33);
  EXPECT_EQ(g_ao_s.get_optional().value(), 1.4);
  EXPECT_TRUE(std::isnan(g_ai1_s.get_optional().value()));
  EXPECT_TRUE(std::isnan(g_ai2_s.get_optional().value()));
  EXPECT_EQ(f_y_s.get_optional().value(), 0.0);
  EXPECT_EQ(f_z_s.get_optional().value(), 0.0);
}
