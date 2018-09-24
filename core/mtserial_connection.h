#pragma once

#include <mutex>
#include <atomic>
#include "connection.h"

namespace dronecode_sdk {

class MTSerialConnection : public Connection {
public:
    explicit MTSerialConnection(Connection::receiver_callback_t receiver_callback,
                              const std::string &path,
                              int baudrate);
    ConnectionResult start();
    ConnectionResult stop();
    ~MTSerialConnection();

    bool send_message(const mavlink_message_t &message);

    // Non-copyable
    MTSerialConnection(const MTSerialConnection &) = delete;
    const MTSerialConnection &operator=(const MTSerialConnection &) = delete;

private:
    ConnectionResult setup_port();
    void start_recv_thread();
    void receive();

    std::string _serial_node;
    int _baudrate;

    std::mutex _mutex = {};
    int _schedfd = -1;
    std::thread *_recv_thread = nullptr;
    std::atomic_bool _should_exit{false};
};

} // namespace dronecode_sdk
