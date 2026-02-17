#include "mbot_driver/mbot_driver.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "mocks/mock_client.hpp"
#include "mocks/mock_connection.hpp"
#include "mocks/mock_mbot.hpp"
#include "mocks/mock_notification.hpp"
#include "mocks/mock_server.hpp"

void pose_equal(const rix::msg::geometry::Pose2D &a, const rix::msg::geometry::Pose2D &b) {
    EXPECT_EQ(a.x, b.x);
    EXPECT_EQ(a.y, b.y);
    EXPECT_EQ(a.theta, b.theta);
}

TEST(MBotDriverTest, ExitsOnNotification) {
    auto endpoint = Endpoint("mbot_address", 0);
    auto server = std::make_unique<testing::NiceMock<MockServer>>(endpoint);
    auto server_map = std::make_shared<std::map<rix::ipc::Endpoint, MockServer *>>();
    server_map->insert({endpoint, server.get()});
    auto client = std::make_unique<testing::NiceMock<MockClient>>(512, "mbot_address", server_map);

    auto mbot = std::make_unique<testing::NiceMock<MockMBot>>();
    EXPECT_CALL(*mbot, spin).Times(1);

    auto *mbot_ptr = mbot.get();  // Need to get raw pointer for inspection

    auto notif = std::make_unique<testing::NiceMock<MockNotification>>();
    auto mbot_driver =
        std::make_unique<testing::NiceMock<MBotDriver>>(std::move(server), std::move(client), std::move(mbot));

    notif->raise();
    mbot_driver->spin(std::move(notif));

    ASSERT_EQ(mbot_ptr->poses.size(), 0);
    ASSERT_EQ(mbot_ptr->twists.size(), 0);
}

TEST(MBotDriverTest, ConnectsToServer) {
    auto mbot_endpoint = Endpoint("mbot_address", 8200);
    auto mbot_server = std::make_unique<testing::NiceMock<MockServer>>(mbot_endpoint);
    auto server_map = std::make_shared<std::map<rix::ipc::Endpoint, MockServer *>>();
    server_map->insert({mbot_endpoint, mbot_server.get()});
    auto gui_client = std::make_unique<testing::NiceMock<MockClient>>(512, "mbot_address", server_map);

    auto mbot = std::make_unique<testing::NiceMock<MockMBot>>();
    EXPECT_CALL(*mbot, spin).Times(1);

    auto *mbot_ptr = mbot.get();  // Need to get raw pointer for inspection

    auto notif = std::make_unique<testing::NiceMock<MockNotification>>();
    auto notif_ptr = notif.get();
    auto mbot_driver =
        std::make_unique<testing::NiceMock<MBotDriver>>(std::move(mbot_server), std::move(gui_client), std::move(mbot));

    std::thread thr([&]() { mbot_driver->spin(std::move(notif)); });

    // "Start" the GUI
    auto gui_endpoint = Endpoint("gui_address", 8300);
    auto gui_server = std::make_unique<testing::NiceMock<MockServer>>(gui_endpoint);
    server_map->insert({gui_endpoint, gui_server.get()});

    // Connect to the MBot
    auto mbot_client = std::make_unique<testing::NiceMock<MockClient>>(512, "gui_address", server_map);
    EXPECT_TRUE(mbot_client->connect(mbot_endpoint));

    // Accept connection from MBot
    std::weak_ptr<interfaces::Connection> gui_conn;
    EXPECT_TRUE(gui_server->accept(gui_conn));

    // Raise the signal
    notif_ptr->raise();

    // Join the spin thread
    thr.join();
}

TEST(MBotDriverTest, SendsOdometryDataAfterConnection) {
    rix::msg::geometry::Pose2DStamped pose1;
    pose1.header.frame_id = "my_mbot_one";
    pose1.pose.x = 1.0f;
    pose1.pose.y = 2.0f;
    pose1.pose.theta = 3.0f;
    rix::msg::geometry::Pose2DStamped pose2;
    pose1.header.frame_id = "mbot_two";
    pose1.pose.x = 4.0f;
    pose1.pose.y = 5.0f;
    pose1.pose.theta = 6.0f;

    auto mbot_endpoint = Endpoint("mbot_address", 8200);
    auto mbot_server = std::make_unique<testing::NiceMock<MockServer>>(mbot_endpoint);
    auto server_map = std::make_shared<std::map<rix::ipc::Endpoint, MockServer *>>();
    server_map->insert({mbot_endpoint, mbot_server.get()});
    auto gui_client = std::make_unique<testing::NiceMock<MockClient>>(512, "mbot_address", server_map);

    auto mbot = std::make_unique<testing::NiceMock<MockMBot>>();
    EXPECT_CALL(*mbot, spin).Times(1);

    auto *mbot_ptr = mbot.get();  // Need to get raw pointer for inspection

    auto notif = std::make_unique<testing::NiceMock<MockNotification>>();
    auto notif_ptr = notif.get();
    auto mbot_driver =
        std::make_unique<testing::NiceMock<MBotDriver>>(std::move(mbot_server), std::move(gui_client), std::move(mbot));

    std::thread thr([&]() { mbot_driver->spin(std::move(notif)); });

    // "Start" the GUI
    auto gui_endpoint = Endpoint("gui_address", 8300);
    auto gui_server = std::make_unique<testing::NiceMock<MockServer>>(gui_endpoint);
    server_map->insert({gui_endpoint, gui_server.get()});

    // Connect to the MBot
    auto mbot_client = std::make_unique<testing::NiceMock<MockClient>>(512, "gui_address", server_map);
    EXPECT_TRUE(mbot_client->connect(mbot_endpoint));

    // Accept connection from MBot
    std::weak_ptr<interfaces::Connection> gui_conn;
    EXPECT_TRUE(gui_server->accept(gui_conn));

    mbot_ptr->on_pose(pose1);

    EXPECT_TRUE(mbot_client->is_readable());
    rix::msg::standard::UInt32 size_msg;
    std::vector<uint8_t> buffer(size_msg.size());
    mbot_client->read(buffer.data(), size_msg.size());
    size_t offset = 0;
    size_msg.deserialize(buffer.data(), size_msg.size(), offset);
    ASSERT_EQ(size_msg.data, pose1.size());
    
    buffer.resize(size_msg.data);
    mbot_client->read(buffer.data(), size_msg.data);
    rix::msg::geometry::Pose2DStamped pose1_deserialized;
    offset = 0;
    pose1_deserialized.deserialize(buffer.data(), size_msg.data, offset);
    pose_equal(pose1_deserialized.pose, pose1.pose);

    mbot_ptr->on_pose(pose2);
    EXPECT_TRUE(mbot_client->is_readable());

    buffer.resize(size_msg.size());
    mbot_client->read(buffer.data(), size_msg.size());
    offset = 0;
    size_msg.deserialize(buffer.data(), size_msg.size(), offset);
    ASSERT_EQ(size_msg.data, pose2.size());
    
    buffer.resize(size_msg.data);
    mbot_client->read(buffer.data(), size_msg.data);
    rix::msg::geometry::Pose2DStamped pose2_deserialized;
    offset = 0;
    pose2_deserialized.deserialize(buffer.data(), size_msg.data, offset);
    pose_equal(pose2_deserialized.pose, pose2.pose);

    // Raise the signal
    notif_ptr->raise();

    // Join the spin thread
    thr.join();
}

TEST(MBotDriverTest, SetsGoalPoseFromClient) {
    rix::msg::geometry::Pose2DStamped pose1;
    pose1.header.frame_id = "my_mbot_one";
    pose1.pose.x = 1.0f;
    pose1.pose.y = 2.0f;
    pose1.pose.theta = 3.0f;
    rix::msg::geometry::Pose2DStamped pose2;
    pose2.header.frame_id = "mbot_two";
    pose2.pose.x = 4.0f;
    pose2.pose.y = 5.0f;
    pose2.pose.theta = 6.0f;

    auto mbot_endpoint = Endpoint("mbot_address", 8200);
    auto mbot_server = std::make_unique<testing::NiceMock<MockServer>>(mbot_endpoint);
    auto server_map = std::make_shared<std::map<rix::ipc::Endpoint, MockServer *>>();
    server_map->insert({mbot_endpoint, mbot_server.get()});
    auto gui_client = std::make_unique<testing::NiceMock<MockClient>>(512, "mbot_address", server_map);

    auto mbot = std::make_unique<testing::NiceMock<MockMBot>>();
    EXPECT_CALL(*mbot, spin).Times(1);

    auto *mbot_ptr = mbot.get();  // Need to get raw pointer for inspection

    auto notif = std::make_unique<testing::NiceMock<MockNotification>>();
    auto notif_ptr = notif.get();
    auto mbot_driver =
        std::make_unique<testing::NiceMock<MBotDriver>>(std::move(mbot_server), std::move(gui_client), std::move(mbot));

    std::thread thr([&]() { mbot_driver->spin(std::move(notif)); });

    // "Start" the GUI
    auto gui_endpoint = Endpoint("gui_address", 8300);
    auto gui_server = std::make_unique<testing::NiceMock<MockServer>>(gui_endpoint);
    server_map->insert({gui_endpoint, gui_server.get()});

    // Connect to the MBot
    auto mbot_client = std::make_unique<testing::NiceMock<MockClient>>(512, "gui_address", server_map);
    EXPECT_TRUE(mbot_client->connect(mbot_endpoint));

    // Accept connection from MBot
    std::weak_ptr<interfaces::Connection> gui_conn;
    EXPECT_TRUE(gui_server->accept(gui_conn));

    // Send pose commands from gui_conn
    auto ptr = gui_conn.lock();
    ASSERT_NE(ptr, nullptr);
    
    rix::msg::standard::UInt32 size_msg;
    size_msg.data = pose1.size();
    std::vector<uint8_t> buffer(size_msg.size() * 2 + pose1.size() + pose2.size());
    size_t offset = 0;
    size_msg.serialize(buffer.data(), offset);
    pose1.serialize(buffer.data(), offset);
    size_msg.data = pose2.size();
    size_msg.serialize(buffer.data(), offset);
    pose2.serialize(buffer.data(), offset);

    ptr->write(buffer.data(), buffer.size());

    EXPECT_TRUE(mbot_ptr->wait_for_poses(2, rix::util::Duration(5.0)));

    ASSERT_EQ(mbot_ptr->poses.size(), 2);
    pose_equal(mbot_ptr->poses[0].pose, pose1.pose);
    pose_equal(mbot_ptr->poses[1].pose, pose2.pose);

    // Raise the signal
    notif_ptr->raise();

    // Join the spin thread
    thr.join();
}
