#include <gtest/gtest.h>
#include <thread>
#include <chrono>

#include "rix/ipc/socket.hpp"
#include "rix/ipc/endpoint.hpp"

using namespace rix::ipc;

class SocketTest : public ::testing::Test {
   protected:
    static constexpr const char* localhost = "127.0.0.1";

    void SetUp() override {}

    void TearDown() override {}
};

TEST_F(SocketTest, DefaultConstructor) {
    Socket sock;
    EXPECT_FALSE(sock.ok());
}

TEST_F(SocketTest, ParameterizedConstructorCreatesSocket) {
    Socket sock(AF_INET, SOCK_STREAM);
    EXPECT_TRUE(sock.ok());
}

TEST_F(SocketTest, BindValidAddress) {
    Socket sock(AF_INET, SOCK_STREAM);
    Endpoint ep(localhost, 0);  // Bind to ephemeral port
    EXPECT_TRUE(sock.bind(ep));
    EXPECT_TRUE(sock.is_bound());
}

TEST_F(SocketTest, ListenAfterBind) {
    Socket sock(AF_INET, SOCK_STREAM);
    Endpoint ep(localhost, 0);
    ASSERT_TRUE(sock.bind(ep));
    EXPECT_TRUE(sock.listen(5));
    EXPECT_TRUE(sock.is_listening());
}

TEST_F(SocketTest, GetsocknameReturnsBoundAddress) {
    Socket sock(AF_INET, SOCK_STREAM);
    Endpoint ep(localhost, 0);
    ASSERT_TRUE(sock.bind(ep));
    Endpoint bound;
    EXPECT_TRUE(sock.getsockname(bound));
    EXPECT_TRUE(bound.port > 0);
}

TEST_F(SocketTest, ConnectToListeningSocket) {
    Socket server(AF_INET, SOCK_STREAM);
    Endpoint ep(localhost, 0);
    ASSERT_TRUE(server.bind(ep));
    ASSERT_TRUE(server.listen(1));

    Endpoint server_ep;
    ASSERT_TRUE(server.getsockname(server_ep));

    std::thread client_thread([server_ep]() {
        Socket client(AF_INET, SOCK_STREAM);
        EXPECT_TRUE(client.connect(server_ep));
    });

    Socket new_sock;
    EXPECT_GT(server.accept(new_sock), 0);
    EXPECT_TRUE(new_sock.ok());

    client_thread.join();
}

TEST_F(SocketTest, AcceptFailsOnUnboundSocket) {
    Socket sock(AF_INET, SOCK_STREAM);
    Socket new_sock;
    EXPECT_FALSE(sock.accept(new_sock));
    EXPECT_FALSE(new_sock.ok());
}

TEST_F(SocketTest, GetPeerNameReturnsCorrectAddress) {
    Socket server(AF_INET, SOCK_STREAM);
    Endpoint ep(localhost, 0);
    ASSERT_TRUE(server.bind(ep));
    ASSERT_TRUE(server.listen(1));

    Endpoint server_ep;
    ASSERT_TRUE(server.getsockname(server_ep));

    std::thread client_thread([server_ep]() {
        Socket client(AF_INET, SOCK_STREAM);
        ASSERT_TRUE(client.connect(server_ep));
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // ensure accept completes
    });

    Socket accepted;
    ASSERT_TRUE(server.accept(accepted));

    Endpoint peer;
    EXPECT_TRUE(accepted.getpeername(peer));
    EXPECT_TRUE(peer.port > 0);

    client_thread.join();
}

TEST_F(SocketTest, SetAndGetSockOpt) {
    Socket sock(AF_INET, SOCK_STREAM);
    int optval = 1;
    EXPECT_TRUE(sock.setsockopt(SOL_SOCKET, SO_REUSEADDR, optval));

    int actual = 0;
    EXPECT_TRUE(sock.getsockopt(SOL_SOCKET, SO_REUSEADDR, actual));
    EXPECT_EQ(actual, SO_REUSEADDR);
}

TEST_F(SocketTest, ConnectFailsWithInvalidEndpoint) {
    Socket sock(AF_INET, SOCK_STREAM);
    Endpoint invalid_ep("256.256.256.256", 9999);  // Invalid IP
    EXPECT_FALSE(sock.connect(invalid_ep));
}
