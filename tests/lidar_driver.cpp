#include "lidar_driver/lidar_driver.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "mocks/mock_client.hpp"
#include "mocks/mock_connection.hpp"
#include "mocks/mock_lidar.hpp"
#include "mocks/mock_notification.hpp"
#include "mocks/mock_server.hpp"

void laser_scan_equal(const rix::msg::sensor::LaserScan &a, const rix::msg::sensor::LaserScan &b) {
    EXPECT_EQ(a.angle_min, b.angle_min);
    EXPECT_EQ(a.angle_max, b.angle_max);
    EXPECT_EQ(a.angle_increment, b.angle_increment);
    EXPECT_EQ(a.range_min, b.range_min);
    EXPECT_EQ(a.range_max, b.range_max);

    EXPECT_EQ(a.scan_time, b.scan_time);
    EXPECT_EQ(a.time_increment, b.time_increment);

    ASSERT_EQ(a.ranges.size(), b.ranges.size());
    for (size_t i = 0; i < a.ranges.size(); i++) {
        EXPECT_EQ(a.ranges[i], b.ranges[i]);
    }

    ASSERT_EQ(a.intensities.size(), b.intensities.size());
    for (size_t i = 0; i < a.intensities.size(); i++) {
        EXPECT_EQ(a.intensities[i], b.intensities[i]);
    }
}

TEST(LidarDriverTest, ExitsOnNotification) {
    auto endpoint = Endpoint("mbot_address", 0);
    auto server = std::make_unique<testing::NiceMock<MockServer>>(endpoint);
    auto server_map = std::make_shared<std::map<rix::ipc::Endpoint, MockServer *>>();
    server_map->insert({endpoint, server.get()});

    auto lidar = std::make_unique<testing::NiceMock<MockLidar>>();
    EXPECT_CALL(*lidar, spin).Times(1);

    auto notif = std::make_unique<testing::NiceMock<MockNotification>>();
    auto lidar_driver = std::make_unique<testing::NiceMock<LidarDriver>>(std::move(server), std::move(lidar));

    notif->raise();
    lidar_driver->spin(std::move(notif));
}

TEST(LidarDriverTest, ConnectsToServer) {
    auto mbot_endpoint = Endpoint("mbot_address", 8100);
    auto server = std::make_unique<testing::NiceMock<MockServer>>(mbot_endpoint);
    auto server_map = std::make_shared<std::map<rix::ipc::Endpoint, MockServer *>>();
    server_map->insert({mbot_endpoint, server.get()});

    auto lidar = std::make_unique<testing::NiceMock<MockLidar>>();
    EXPECT_CALL(*lidar, spin).Times(1);

    auto *lidar_ptr = lidar.get();  // Need to get raw pointer for inspection

    auto notif = std::make_unique<testing::NiceMock<MockNotification>>();
    auto notif_ptr = notif.get();

    auto lidar_driver = std::make_unique<testing::NiceMock<LidarDriver>>(std::move(server), std::move(lidar));

    std::thread thr([&]() { lidar_driver->spin(std::move(notif)); });

    // Connect to the MBot
    auto lidar_client = std::make_unique<testing::NiceMock<MockClient>>(512, "gui_address", server_map);
    EXPECT_TRUE(lidar_client->connect(mbot_endpoint));

    // Raise the signal
    notif_ptr->raise();

    // Join the spin thread
    thr.join();
}

TEST(LidarDriverTest, SendsLaserScanDataAfterConnection) {
    auto mbot_endpoint = Endpoint("mbot_address", 8100);
    auto server = std::make_unique<testing::NiceMock<MockServer>>(mbot_endpoint);
    auto server_map = std::make_shared<std::map<rix::ipc::Endpoint, MockServer *>>();
    server_map->insert({mbot_endpoint, server.get()});

    auto lidar = std::make_unique<testing::NiceMock<MockLidar>>();
    EXPECT_CALL(*lidar, spin).Times(1);

    auto *lidar_ptr = lidar.get();  // Need to get raw pointer for inspection

    auto notif = std::make_unique<testing::NiceMock<MockNotification>>();
    auto notif_ptr = notif.get();

    auto lidar_driver = std::make_unique<testing::NiceMock<LidarDriver>>(std::move(server), std::move(lidar));

    std::thread thr([&]() { lidar_driver->spin(std::move(notif)); });

    // Connect to the MBot
    auto lidar_client = std::make_unique<testing::NiceMock<MockClient>>(512, "gui_address", server_map);
    EXPECT_TRUE(lidar_client->connect(mbot_endpoint));

    rix::msg::sensor::LaserScan laser_scan1;
    laser_scan1.angle_min = 0.0;
    laser_scan1.angle_max = M_PI;
    laser_scan1.angle_increment = M_PI / 3;
    laser_scan1.range_min = 1.0;
    laser_scan1.range_max = 2.0;
    laser_scan1.ranges.resize(3);
    laser_scan1.ranges[0] = 1.0;
    laser_scan1.ranges[1] = 1.5;
    laser_scan1.ranges[2] = 2.0;
    laser_scan1.intensities.resize(3);
    laser_scan1.intensities[0] = 0.5;
    laser_scan1.intensities[1] = 0.25;
    laser_scan1.intensities[2] = 0.125;
    laser_scan1.scan_time = 2.0;
    laser_scan1.time_increment = 0.125;

    lidar_ptr->on_scan(laser_scan1);

    rix::msg::sensor::LaserScan laser_scan2;
    laser_scan2.angle_min = 0.0;
    laser_scan2.angle_max = M_PI;
    laser_scan2.angle_increment = M_PI / 3;
    laser_scan2.range_min = 2.0;
    laser_scan2.range_max = 4.0;
    laser_scan2.ranges.resize(3);
    laser_scan2.ranges[0] = 2.0;
    laser_scan2.ranges[1] = 3.0;
    laser_scan2.ranges[2] = 4.0;
    laser_scan2.intensities.resize(3);
    laser_scan2.intensities[0] = 1.0;
    laser_scan2.intensities[1] = 0.5;
    laser_scan2.intensities[2] = 0.25;
    laser_scan2.scan_time = 3.0;
    laser_scan2.time_increment = 0.125;

    lidar_ptr->on_scan(laser_scan2);

    EXPECT_TRUE(lidar_client->is_readable());
    rix::msg::standard::UInt32 size_msg;
    std::vector<uint8_t> buffer(size_msg.size());
    lidar_client->read(buffer.data(), size_msg.size());
    size_t offset = 0;
    size_msg.deserialize(buffer.data(), size_msg.size(), offset);
    ASSERT_EQ(size_msg.data, laser_scan1.size());
    
    buffer.resize(size_msg.data);
    lidar_client->read(buffer.data(), size_msg.data);
    rix::msg::sensor::LaserScan laser_scan1_deserialized;
    offset = 0;
    laser_scan1_deserialized.deserialize(buffer.data(), size_msg.data, offset);
    laser_scan_equal(laser_scan1_deserialized, laser_scan1);

    lidar_ptr->on_scan(laser_scan2);
    EXPECT_TRUE(lidar_client->is_readable());

    buffer.resize(size_msg.size());
    lidar_client->read(buffer.data(), size_msg.size());
    offset = 0;
    size_msg.deserialize(buffer.data(), size_msg.size(), offset);
    ASSERT_EQ(size_msg.data, laser_scan2.size());
    
    buffer.resize(size_msg.data);
    lidar_client->read(buffer.data(), size_msg.data);
    rix::msg::sensor::LaserScan laser_scan2_deserialized;
    offset = 0;
    laser_scan2_deserialized.deserialize(buffer.data(), size_msg.data, offset);
    laser_scan_equal(laser_scan2_deserialized, laser_scan2);

    // Raise the signal
    notif_ptr->raise();

    // Join the spin thread
    thr.join();
}