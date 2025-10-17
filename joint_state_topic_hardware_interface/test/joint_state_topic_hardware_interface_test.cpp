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
#include <string>
#include <unordered_map>
#include <vector>

#include <gmock/gmock.h>
#if __has_include(<hardware_interface/hardware_interface/version.h>)
#include <hardware_interface/hardware_interface/version.h>
#else
#include <hardware_interface/version.h>
#endif
#include <hardware_interface/resource_manager.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <ros2_control_test_assets/descriptions.hpp>

#include "sensor_msgs/msg/joint_state.hpp"

namespace
{
const auto TIME = rclcpp::Time(0);
const auto PERIOD = rclcpp::Duration::from_seconds(0.1);  // 0.1 seconds for easier math
const auto COMPARE_DELTA = 0.0001;
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

    js_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("/topic_based_custom_joint_states",
                                                                          rclcpp::SystemDefaultsQoS());

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
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_publisher_;

  /// Publish joint_state_message
  /**
   *  names - names of joints
   *  points - vector of trajectory-velocities. One point per controlled joint
   */
  void publish(const std::vector<std::string>& joint_names = {}, const std::vector<double>& points_positions = {},
               const std::vector<double>& points_velocities = {}, const std::vector<double>& points_effort = {})
  {
    int wait_count = 0;
    const auto topic = js_publisher_->get_topic_name();
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

    sensor_msgs::msg::JointState state_msg;
    state_msg.name = joint_names;

    state_msg.position = points_positions;
    state_msg.velocity = points_velocities;
    state_msg.effort = points_effort;

    js_publisher_->publish(state_msg);
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

  FRIEND_TEST(TestTopicBasedSystem, load_topic_based_system_2dof);
  FRIEND_TEST(TestTopicBasedSystem, load_topic_based_system_2dof_missing_position);
  FRIEND_TEST(TestTopicBasedSystem, load_topic_based_system_with_mimic_joint);
  FRIEND_TEST(TestTopicBasedSystem, TestTopicBasedSystem_load_topic_based_system_with_mimic_joint_missing_position);

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

auto configure_components = [](TestableResourceManager& rm,
                               const std::vector<std::string>& components = { "GenericSystem2dof" }) {
  set_components_state(rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                       hardware_interface::lifecycle_state_names::INACTIVE);
};

auto activate_components = [](TestableResourceManager& rm,
                              const std::vector<std::string>& components = { "GenericSystem2dof" }) {
  set_components_state(rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                       hardware_interface::lifecycle_state_names::ACTIVE);
};

auto deactivate_components = [](TestableResourceManager& rm,
                                const std::vector<std::string>& components = { "GenericSystem2dof" }) {
  set_components_state(rm, components, lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                       hardware_interface::lifecycle_state_names::INACTIVE);
};

TEST_F(TestTopicBasedSystem, load_topic_based_system_2dof)
{
  const std::string hardware_system_2dof_topic_based =
      R"(
  <ros2_control name="JointStateTopicBasedSystem2dof" type="system">
    <hardware>
      <plugin>joint_state_topic_hardware_interface/JointStateTopicSystem</plugin>
      <param name="joint_commands_topic">/topic_based_joint_commands</param>
      <param name="joint_states_topic">/topic_based_custom_joint_states</param>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position">
        <param name="initial_value">1.57</param>
      </state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="joint2">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position">
        <param name="initial_value">-1.57</param>
      </state_interface>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
)";
  auto urdf =
      ros2_control_test_assets::urdf_head + hardware_system_2dof_topic_based + ros2_control_test_assets::urdf_tail;

  init_rm(urdf);

  // Activate components to get all interfaces available
  activate_components(*rm_, { "JointStateTopicBasedSystem2dof" });

  // Check interfaces
  EXPECT_EQ(1u, rm_->system_components_size());
  ASSERT_EQ(4u, rm_->state_interface_keys().size());
  EXPECT_TRUE(rm_->state_interface_exists("joint1/position"));
  EXPECT_TRUE(rm_->state_interface_exists("joint1/velocity"));
  EXPECT_TRUE(rm_->state_interface_exists("joint2/position"));
  EXPECT_TRUE(rm_->state_interface_exists("joint2/velocity"));

  ASSERT_EQ(4u, rm_->command_interface_keys().size());
  EXPECT_TRUE(rm_->state_interface_exists("joint1/position"));
  EXPECT_TRUE(rm_->state_interface_exists("joint1/velocity"));
  EXPECT_TRUE(rm_->state_interface_exists("joint2/position"));
  EXPECT_TRUE(rm_->state_interface_exists("joint2/velocity"));

  // Check initial values
  hardware_interface::LoanedStateInterface j1_p_s = rm_->claim_state_interface("joint1/position");
  hardware_interface::LoanedStateInterface j1_v_s = rm_->claim_state_interface("joint1/velocity");
  hardware_interface::LoanedStateInterface j2_p_s = rm_->claim_state_interface("joint2/position");
  hardware_interface::LoanedStateInterface j2_v_s = rm_->claim_state_interface("joint2/velocity");
  hardware_interface::LoanedCommandInterface j1_p_c = rm_->claim_command_interface("joint1/position");
  hardware_interface::LoanedCommandInterface j1_v_c = rm_->claim_command_interface("joint1/velocity");
  hardware_interface::LoanedCommandInterface j2_p_c = rm_->claim_command_interface("joint2/position");
  hardware_interface::LoanedCommandInterface j2_v_c = rm_->claim_command_interface("joint2/velocity");

  EXPECT_EQ(j1_p_s.get_optional().value(), 1.57);
  EXPECT_TRUE(std::isnan(j1_v_s.get_optional().value()));
  EXPECT_EQ(j2_p_s.get_optional().value(), -1.57);
  EXPECT_TRUE(std::isnan(j2_v_s.get_optional().value()));
  EXPECT_TRUE(std::isnan(j1_p_c.get_optional().value()));
  EXPECT_TRUE(std::isnan(j1_v_c.get_optional().value()));
  EXPECT_TRUE(std::isnan(j2_p_c.get_optional().value()));
  EXPECT_TRUE(std::isnan(j2_v_c.get_optional().value()));

  // set some new values in commands
  ASSERT_TRUE(j1_p_c.set_value(0.11));
  ASSERT_TRUE(j1_v_c.set_value(0.12));
  ASSERT_TRUE(j2_p_c.set_value(0.13));
  ASSERT_TRUE(j2_v_c.set_value(0.14));

  hardware_interface::return_type ret;
  ASSERT_NO_THROW(ret = rm_->read(TIME, PERIOD).result);
  ASSERT_EQ(ret, hardware_interface::return_type::OK);
  ASSERT_NO_THROW(ret = rm_->write(TIME, PERIOD).result);
  ASSERT_EQ(ret, hardware_interface::return_type::OK);

  // command is not propagated to state until topic from robot is received
  // same holds for velocity
  EXPECT_EQ(j1_p_s.get_optional().value(), 1.57);
  EXPECT_TRUE(std::isnan(j1_v_s.get_optional().value()));
  EXPECT_EQ(j2_p_s.get_optional().value(), -1.57);
  EXPECT_TRUE(std::isnan(j2_v_s.get_optional().value()));

  EXPECT_EQ(j1_p_c.get_optional().value(), 0.11);
  EXPECT_EQ(j1_v_c.get_optional().value(), 0.12);
  EXPECT_EQ(j2_p_c.get_optional().value(), 0.13);
  EXPECT_EQ(j2_v_c.get_optional().value(), 0.14);

  publish({ "joint1", "joint2" }, { 0.21, 0.23 }, { 0.22, 0.24 });

  wait_for_msg();

  ASSERT_NO_THROW(ret = rm_->read(TIME, PERIOD).result);
  ASSERT_EQ(ret, hardware_interface::return_type::OK);
  ASSERT_NO_THROW(ret = rm_->write(TIME, PERIOD).result);
  ASSERT_EQ(ret, hardware_interface::return_type::OK);

  // new states should have been updated from topic
  EXPECT_EQ(j1_p_s.get_optional().value(), 0.21);
  EXPECT_EQ(j1_v_s.get_optional().value(), 0.22);
  EXPECT_EQ(j2_p_s.get_optional().value(), 0.23);
  EXPECT_EQ(j2_v_s.get_optional().value(), 0.24);

  // commands should remain unchanged
  EXPECT_EQ(j1_p_c.get_optional().value(), 0.11);
  EXPECT_EQ(j1_v_c.get_optional().value(), 0.12);
  EXPECT_EQ(j2_p_c.get_optional().value(), 0.13);
  EXPECT_EQ(j2_v_c.get_optional().value(), 0.14);
}

TEST_F(TestTopicBasedSystem, load_topic_based_system_2dof_velocity_only)
{
  const std::string hardware_system_2dof_topic_based =
      R"(
  <ros2_control name="JointStateTopicBasedSystem2dof" type="system">
    <hardware>
      <plugin>joint_state_topic_hardware_interface/JointStateTopicSystem</plugin>
      <param name="joint_commands_topic">/topic_based_joint_commands</param>
      <param name="joint_states_topic">/topic_based_custom_joint_states</param>
    </hardware>
    <joint name="joint1">
      <command_interface name="velocity"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="joint2">
      <command_interface name="velocity"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
)";
  auto urdf =
      ros2_control_test_assets::urdf_head + hardware_system_2dof_topic_based + ros2_control_test_assets::urdf_tail;

  init_rm(urdf);

  // Activate components to get all interfaces available
  activate_components(*rm_, { "JointStateTopicBasedSystem2dof" });

  // Check interfaces
  EXPECT_EQ(1u, rm_->system_components_size());
  ASSERT_EQ(2u, rm_->state_interface_keys().size());
  EXPECT_TRUE(rm_->state_interface_exists("joint1/velocity"));
  EXPECT_TRUE(rm_->state_interface_exists("joint2/velocity"));

  ASSERT_EQ(2u, rm_->command_interface_keys().size());
  EXPECT_TRUE(rm_->state_interface_exists("joint1/velocity"));
  EXPECT_TRUE(rm_->state_interface_exists("joint2/velocity"));

  // Check initial values
  hardware_interface::LoanedStateInterface j1_v_s = rm_->claim_state_interface("joint1/velocity");
  hardware_interface::LoanedStateInterface j2_v_s = rm_->claim_state_interface("joint2/velocity");
  hardware_interface::LoanedCommandInterface j1_v_c = rm_->claim_command_interface("joint1/velocity");
  hardware_interface::LoanedCommandInterface j2_v_c = rm_->claim_command_interface("joint2/velocity");

  EXPECT_TRUE(std::isnan(j1_v_s.get_optional().value()));
  EXPECT_TRUE(std::isnan(j2_v_s.get_optional().value()));
  EXPECT_TRUE(std::isnan(j1_v_c.get_optional().value()));
  EXPECT_TRUE(std::isnan(j2_v_c.get_optional().value()));

  // set some new values in commands
  ASSERT_TRUE(j1_v_c.set_value(0.12));
  ASSERT_TRUE(j2_v_c.set_value(0.14));

  hardware_interface::return_type ret;
  ASSERT_NO_THROW(ret = rm_->read(TIME, PERIOD).result);
  ASSERT_EQ(ret, hardware_interface::return_type::OK);
  ASSERT_NO_THROW(ret = rm_->write(TIME, PERIOD).result);
  ASSERT_EQ(ret, hardware_interface::return_type::OK);

  // command is not propagated to state until topic from robot is received
  // same holds for velocity
  EXPECT_TRUE(std::isnan(j1_v_s.get_optional().value()));
  EXPECT_TRUE(std::isnan(j2_v_s.get_optional().value()));

  EXPECT_EQ(j1_v_c.get_optional().value(), 0.12);
  EXPECT_EQ(j2_v_c.get_optional().value(), 0.14);

  publish({ "joint1", "joint2" }, { 0.21, 0.23 }, { 0.22, 0.24 });

  wait_for_msg();

  // Reading should fail as position interface is missing
  ASSERT_NO_THROW(ret = rm_->read(TIME, PERIOD).result);
  ASSERT_EQ(ret, hardware_interface::return_type::ERROR);
}

TEST_F(TestTopicBasedSystem, load_topic_based_system_with_mimic_joint)
{
  const std::string hardware_system_2dof_with_mimic_joint =
      R"(
  <ros2_control name="JointStateTopicBasedSystem2dofMimic" type="system">
    <hardware>
      <plugin>joint_state_topic_hardware_interface/JointStateTopicSystem</plugin>
      <param name="joint_commands_topic">/topic_based_joint_commands</param>
      <param name="joint_states_topic">/topic_based_custom_joint_states</param>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">1.57</param>
      </state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="joint2" mimic="true">
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
)";
  auto urdf = ros2_control_test_assets::urdf_head_mimic + hardware_system_2dof_with_mimic_joint +
              ros2_control_test_assets::urdf_tail;

  init_rm(urdf);

  // Activate components to get all interfaces available
  activate_components(*rm_, { "JointStateTopicBasedSystem2dofMimic" });

  // Check interfaces
  EXPECT_EQ(1u, rm_->system_components_size());
  ASSERT_EQ(4u, rm_->state_interface_keys().size());
  EXPECT_TRUE(rm_->state_interface_exists("joint1/position"));
  EXPECT_TRUE(rm_->state_interface_exists("joint1/velocity"));
  EXPECT_TRUE(rm_->state_interface_exists("joint2/position"));
  EXPECT_TRUE(rm_->state_interface_exists("joint2/velocity"));

  ASSERT_EQ(1u, rm_->command_interface_keys().size());
  EXPECT_TRUE(rm_->state_interface_exists("joint1/position"));

  // Check initial values
  hardware_interface::LoanedStateInterface j1_p_s = rm_->claim_state_interface("joint1/position");
  hardware_interface::LoanedStateInterface j1_v_s = rm_->claim_state_interface("joint1/velocity");
  hardware_interface::LoanedStateInterface j2_p_s = rm_->claim_state_interface("joint2/position");
  hardware_interface::LoanedStateInterface j2_v_s = rm_->claim_state_interface("joint2/velocity");
  hardware_interface::LoanedCommandInterface j1_p_c = rm_->claim_command_interface("joint1/position");

  EXPECT_EQ(j1_p_s.get_optional().value(), 1.57);
  EXPECT_TRUE(std::isnan(j1_v_s.get_optional().value()));
  EXPECT_TRUE(std::isnan(j2_p_s.get_optional().value()));
  EXPECT_TRUE(std::isnan(j2_v_s.get_optional().value()));
  EXPECT_TRUE(std::isnan(j1_p_c.get_optional().value()));

  // set some new values in commands
  ASSERT_TRUE(j1_p_c.set_value(0.11));

  hardware_interface::return_type ret;
  ASSERT_NO_THROW(ret = rm_->read(TIME, PERIOD).result);
  ASSERT_EQ(ret, hardware_interface::return_type::OK);
  ASSERT_NO_THROW(ret = rm_->write(TIME, PERIOD).result);
  ASSERT_EQ(ret, hardware_interface::return_type::OK);

  // command is not propagated to state until topic from robot is received
  // same holds for velocity
  // but mimic joint should update its state based on the first joint state
  EXPECT_EQ(j1_p_s.get_optional().value(), 1.57);
  EXPECT_TRUE(std::isnan(j1_v_s.get_optional().value()));
  EXPECT_EQ(j2_p_s.get_optional().value(), -2 * 1.57);
  EXPECT_TRUE(std::isnan(j2_v_s.get_optional().value()));
  EXPECT_EQ(j1_p_c.get_optional().value(), 0.11);

  publish({ "joint1", "joint2" }, { 0.21, 0.23 }, { 0.22, 0.24 });

  wait_for_msg();

  ASSERT_NO_THROW(ret = rm_->read(TIME, PERIOD).result);
  ASSERT_EQ(ret, hardware_interface::return_type::OK);
  ASSERT_NO_THROW(ret = rm_->write(TIME, PERIOD).result);
  ASSERT_EQ(ret, hardware_interface::return_type::OK);

  // new states should have been updated from topic
  EXPECT_EQ(j1_p_s.get_optional().value(), 0.21);
  EXPECT_EQ(j1_v_s.get_optional().value(), 0.22);
  EXPECT_EQ(j2_p_s.get_optional().value(), -2 * 0.21);  // received value ignored due to mimic
  EXPECT_EQ(j2_v_s.get_optional().value(), -2 * 0.22);  // received value ignored due to mimic

  // commands should remain unchanged
  EXPECT_EQ(j1_p_c.get_optional().value(), 0.11);
}

TEST_F(TestTopicBasedSystem, load_topic_based_system_with_mimic_joint_missing_position)
{
  const std::string hardware_system_2dof_with_mimic_joint =
      R"(
  <ros2_control name="JointStateTopicBasedSystem2dofMimic" type="system">
    <hardware>
      <plugin>joint_state_topic_hardware_interface/JointStateTopicSystem</plugin>
      <param name="joint_commands_topic">/topic_based_joint_commands</param>
      <param name="joint_states_topic">/topic_based_custom_joint_states</param>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">1.57</param>
      </state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="joint2" mimic="true">
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
)";
  auto urdf = ros2_control_test_assets::urdf_head_mimic + hardware_system_2dof_with_mimic_joint +
              ros2_control_test_assets::urdf_tail;

  init_rm(urdf);

  // Activate components to get all interfaces available
  activate_components(*rm_, { "JointStateTopicBasedSystem2dofMimic" });

  // Check interfaces
  EXPECT_EQ(1u, rm_->system_components_size());
  ASSERT_EQ(3u, rm_->state_interface_keys().size());
  EXPECT_TRUE(rm_->state_interface_exists("joint1/position"));
  EXPECT_TRUE(rm_->state_interface_exists("joint1/velocity"));
  EXPECT_FALSE(rm_->state_interface_exists("joint2/position"));
  EXPECT_TRUE(rm_->state_interface_exists("joint2/velocity"));

  ASSERT_EQ(1u, rm_->command_interface_keys().size());
  EXPECT_TRUE(rm_->state_interface_exists("joint1/position"));

  // Check initial values
  hardware_interface::LoanedStateInterface j1_p_s = rm_->claim_state_interface("joint1/position");
  hardware_interface::LoanedStateInterface j1_v_s = rm_->claim_state_interface("joint1/velocity");
  hardware_interface::LoanedStateInterface j2_v_s = rm_->claim_state_interface("joint2/velocity");
  hardware_interface::LoanedCommandInterface j1_p_c = rm_->claim_command_interface("joint1/position");

  EXPECT_EQ(j1_p_s.get_optional().value(), 1.57);
  EXPECT_TRUE(std::isnan(j1_v_s.get_optional().value()));
  EXPECT_TRUE(std::isnan(j2_v_s.get_optional().value()));
  EXPECT_TRUE(std::isnan(j1_p_c.get_optional().value()));

  // set some new values in commands
  ASSERT_TRUE(j1_p_c.set_value(0.11));

  // should not throw even if mimic joint is missing position interface
  hardware_interface::return_type ret;
  ASSERT_NO_THROW(ret = rm_->read(TIME, PERIOD).result);
  ASSERT_EQ(ret, hardware_interface::return_type::OK);
  ASSERT_NO_THROW(ret = rm_->write(TIME, PERIOD).result);
  ASSERT_EQ(ret, hardware_interface::return_type::OK);

  // command is not propagated to state until topic from robot is received
  // same holds for velocity
  EXPECT_EQ(j1_p_s.get_optional().value(), 1.57);
  EXPECT_TRUE(std::isnan(j1_v_s.get_optional().value()));
  EXPECT_TRUE(std::isnan(j2_v_s.get_optional().value()));
  EXPECT_EQ(j1_p_c.get_optional().value(), 0.11);

  publish({ "joint1", "joint2" }, { 0.21, 0.23 }, { 0.22, 0.24 });

  wait_for_msg();

  ASSERT_NO_THROW(ret = rm_->read(TIME, PERIOD).result);
  ASSERT_EQ(ret, hardware_interface::return_type::OK);
  ASSERT_NO_THROW(ret = rm_->write(TIME, PERIOD).result);
  ASSERT_EQ(ret, hardware_interface::return_type::OK);

  // new states should have been updated from topic
  EXPECT_EQ(j1_p_s.get_optional().value(), 0.21);
  EXPECT_EQ(j1_v_s.get_optional().value(), 0.22);
  EXPECT_EQ(j2_v_s.get_optional().value(), -2 * 0.22);  // received value ignored due to mimic

  // commands should remain unchanged
  EXPECT_EQ(j1_p_c.get_optional().value(), 0.11);
}
