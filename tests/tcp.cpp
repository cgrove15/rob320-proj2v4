
#include <gtest/gtest.h>

#include <fstream>
#include <thread>

#include "rix/ipc/client_tcp.hpp"
#include "rix/ipc/interfaces/client.hpp"
#include "rix/ipc/interfaces/server.hpp"
#include "rix/ipc/server_tcp.hpp"

using namespace rix::ipc;

/*
            Client          Server
Test 1      NonBlocking     Blocking
Test 2      NonBlocking     NonBlocking
Test 3      Blocking        NonBlocking
Test 4      Blocking        Blocking
*/

// TEST 1
static inline void test_1(std::unique_ptr<interfaces::Client> client,
                          std::unique_ptr<interfaces::Server> server) {
    client->set_nonblocking(true);
    server->set_nonblocking(false);

    rix::ipc::Endpoint ep = server->local_endpoint();

    // Connect to server
    bool status = client->connect(ep);
    EXPECT_FALSE(status);

    // Wait for connection to reach the server
    std::weak_ptr<interfaces::Connection> conn;
    status = server->wait_for_accept(rix::util::Duration(1.0));
    EXPECT_TRUE(status);

    // Accept the connection (Assigns the pointer to the new Connection)
    status = server->accept(conn);
    EXPECT_TRUE(status);

    // Assert that the connection pointer is valid
    auto conn_shared = conn.lock();
    ASSERT_NE(conn_shared.get(), nullptr);

    // Wait for the accept to reach the client (this should be immediately available)
    status = client->wait_for_connect(rix::util::Duration(0.0));
    EXPECT_TRUE(status);

    // Write some data from the connection to the client
    const char *out_msg = "Hello";
    size_t bytes = conn_shared->write((const uint8_t*)out_msg, 5);
    EXPECT_EQ(bytes, 5);

    // Wait for the client to become readable (receives the message from the connection)
    uint8_t buffer[6] = {0};
    status = client->wait_for_readable(rix::util::Duration(1.0));
    EXPECT_TRUE(status);

    // Read the data
    bytes = client->read(buffer, 5);
    ASSERT_EQ(bytes, 5);
    EXPECT_EQ(std::string(reinterpret_cast<char *>(buffer), bytes), "Hello");

    // Write data to connection from client
    const char *response_msg = "Ready";
    bytes = client->write((const uint8_t*)response_msg, 5);
    EXPECT_EQ(bytes, 5);

    // Wait for data on connection side
    status = conn_shared->wait_for_readable(rix::util::Duration(1.0));
    EXPECT_TRUE(status);

    // Read data on connection side
    // Validate data
    bytes = conn_shared->read(buffer, 5);
    ASSERT_EQ(bytes, 5);
    EXPECT_EQ(std::string(reinterpret_cast<char *>(buffer), bytes), "Ready");

    // Clean up
    server->close(conn_shared);
}

static inline void test_2(std::unique_ptr<interfaces::Client> client,
                          std::unique_ptr<interfaces::Server> server) {
    client->set_nonblocking(true);
    server->set_nonblocking(true);

    rix::ipc::Endpoint ep = server->local_endpoint();

    // Connect to server
    bool status = client->connect(ep);
    EXPECT_FALSE(status);

    // Wait for connection to reach the server
    std::weak_ptr<interfaces::Connection> conn;
    status = server->wait_for_accept(rix::util::Duration(1.0));
    EXPECT_TRUE(status);

    // Accept the connection (Assigns the pointer to the new Connection)
    status = server->accept(conn);
    EXPECT_TRUE(status);

    // Assert that the connection pointer is valid
    auto conn_shared = conn.lock();
    ASSERT_NE(conn_shared.get(), nullptr);

    // Wait for the accept to reach the client (this should be immediately available)
    status = client->wait_for_connect(rix::util::Duration(0.0));
    EXPECT_TRUE(status);

    // Write some data from the connection to the client
    const char *out_msg = "Hello";
    size_t bytes = conn_shared->write((const uint8_t*)out_msg, 5);
    EXPECT_EQ(bytes, 5);

    // Wait for the client to become readable (receives the message from the connection)
    uint8_t buffer[6] = {0};
    status = client->wait_for_readable(rix::util::Duration(1.0));
    EXPECT_TRUE(status);

    // Read the data
    bytes = client->read(buffer, 5);
    ASSERT_EQ(bytes, 5);
    EXPECT_EQ(std::string(reinterpret_cast<char *>(buffer), bytes), "Hello");

    // Write data to connection from client
    const char *response_msg = "Ready";
    bytes = client->write((const uint8_t*)response_msg, 5);
    EXPECT_EQ(bytes, 5);

    // Wait for data on connection side=
    status = conn_shared->wait_for_readable(rix::util::Duration(1.0));
    EXPECT_TRUE(status);

    // Read data on connection side
    // Validate data
    bytes = conn_shared->read(buffer, 5);
    ASSERT_EQ(bytes, 5);
    EXPECT_EQ(std::string(reinterpret_cast<char *>(buffer), bytes), "Ready");

    // Clean up
    server->close(conn_shared);
}

static inline void test_3(std::unique_ptr<interfaces::Client> client,
                          std::unique_ptr<interfaces::Server> server) {
    client->set_nonblocking(false);
    server->set_nonblocking(true);

    rix::ipc::Endpoint ep = server->local_endpoint();

    // Launch connect in a separate thread due to blocking behavior
    std::thread connect_thread([&]() {
        bool status = client->connect(ep);
        EXPECT_TRUE(status);
    });

    // Wait for connection to reach the server
    bool status = server->wait_for_accept(rix::util::Duration::safe_forever());
    EXPECT_TRUE(status);

    // Accept the connection (Assigns the pointer to the new Connection)
    std::weak_ptr<interfaces::Connection> conn;
    status = server->accept(conn);
    EXPECT_TRUE(status);

    connect_thread.join();

    // Assert that the connection pointer is valid
    auto conn_shared = conn.lock();
    ASSERT_NE(conn_shared.get(), nullptr);

    // Wait for the accept to reach the client (this should be immediately available)
    status = client->wait_for_connect(rix::util::Duration(0.0));
    EXPECT_TRUE(status);

    // Write some data from the connection to the client
    const char *out_msg = "Hello";
    size_t bytes = conn_shared->write((const uint8_t*)out_msg, 5);
    EXPECT_EQ(bytes, 5);

    // Wait for the client to become readable (receives the message from the connection)
    uint8_t buffer[6] = {0};
    status = client->wait_for_readable(rix::util::Duration(1.0));
    EXPECT_TRUE(status);

    // Read the data
    bytes = client->read(buffer, 5);
    ASSERT_EQ(bytes, 5);
    EXPECT_EQ(std::string(reinterpret_cast<char *>(buffer), bytes), "Hello");

    // Write data to connection from client
    const char *response_msg = "Ready";
    bytes = client->write((const uint8_t*)response_msg, 5);
    EXPECT_EQ(bytes, 5);

    // Wait for data on connection side
    status = conn_shared->wait_for_readable(rix::util::Duration(1.0));
    EXPECT_TRUE(status);

    // Read data on connection side
    // Validate data
    bytes = conn_shared->read(buffer, 5);
    ASSERT_EQ(bytes, 5);
    EXPECT_EQ(std::string(reinterpret_cast<char *>(buffer), bytes), "Ready");

    // Clean up
    server->close(conn_shared);
}

static inline void test_4(std::unique_ptr<interfaces::Client> client,
                          std::unique_ptr<interfaces::Server> server) {
    client->set_nonblocking(false);
    server->set_nonblocking(false);

    rix::ipc::Endpoint ep = server->local_endpoint();
    std::weak_ptr<interfaces::Connection> conn;

    // Launch connect in a separate thread due to blocking behavior
    std::thread connect_thread([&]() {
        bool status = client->connect(ep);
        EXPECT_TRUE(status);
    });

    bool status = server->accept(conn);
    EXPECT_TRUE(status);

    auto conn_shared = conn.lock();
    if (conn_shared.get() == nullptr) {
        connect_thread.join();
    }
    ASSERT_NE(conn_shared.get(), nullptr);

    // Join the thread after accept is done to complete the connection
    connect_thread.join();

    status = client->wait_for_connect(rix::util::Duration(0.0));
    EXPECT_TRUE(status);

    // Write from server -> client
    const char *out_msg = "Hello";
    size_t bytes = conn_shared->write((const uint8_t*)out_msg, 5);
    EXPECT_EQ(bytes, 5);

    uint8_t buffer[6] = {0};
    status = client->wait_for_readable(rix::util::Duration(1.0));
    EXPECT_TRUE(status);

    bytes = client->read(buffer, 5);
    ASSERT_EQ(bytes, 5);
    EXPECT_EQ(std::string(reinterpret_cast<char *>(buffer), bytes), "Hello");

    // Write data to connection from client
    const char *response_msg = "Ready";
    bytes = client->write((const uint8_t*)response_msg, 5);
    EXPECT_EQ(bytes, 5);

    // Wait for data on connection side
    status = conn_shared->wait_for_readable(rix::util::Duration(1.0));
    EXPECT_TRUE(status);

    // Read data on connection side
    // Validate data
    bytes = conn_shared->read(buffer, 5);
    ASSERT_EQ(bytes, 5);
    EXPECT_EQ(std::string(reinterpret_cast<char *>(buffer), bytes), "Ready");

    server->close(conn_shared);
}

TEST(TCP_Interfaces, test1) {
    auto client = std::make_unique<ClientTCP>();
    auto server = std::make_unique<ServerTCP>(Endpoint("0.0.0.0", 0));
    test_1(std::move(client), std::move(server));
}

TEST(TCP_Interfaces, test2) {
    auto client = std::make_unique<ClientTCP>();
    auto server = std::make_unique<ServerTCP>(Endpoint("0.0.0.0", 0));
    test_2(std::move(client), std::move(server));
}
TEST(TCP_Interfaces, test3) {
    auto client = std::make_unique<ClientTCP>();
    auto server = std::make_unique<ServerTCP>(Endpoint("0.0.0.0", 0));
    test_3(std::move(client), std::move(server));
}

TEST(TCP_Interfaces, test4) {
    auto client = std::make_unique<ClientTCP>();
    auto server = std::make_unique<ServerTCP>(Endpoint("0.0.0.0", 0));
    test_4(std::move(client), std::move(server));
}
