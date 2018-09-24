#include "mtserial_connection.h"
#include "global_include.h"
#include "log.h"

#include <unistd.h>
#include <fcntl.h>
#include <asm/termbits.h>
#include <sys/ioctl.h>
#include <zsrmvapi.h>

#define GET_ERROR() strerror(errno)

namespace dronecode_sdk {

MTSerialConnection::MTSerialConnection(Connection::receiver_callback_t receiver_callback,
                                   const std::string &path,
                                   int baudrate) :
    Connection(receiver_callback),
    _serial_node(path),
    _baudrate(baudrate)
{}

MTSerialConnection::~MTSerialConnection()
{
    // If no one explicitly called stop before, we should at least do it.
    stop();
}

ConnectionResult MTSerialConnection::start()
{
    if (!start_mavlink_receiver()) {
        return ConnectionResult::CONNECTIONS_EXHAUSTED;
    }

    ConnectionResult ret = setup_port();
    if (ret != ConnectionResult::SUCCESS) {
        return ret;
    }

    start_recv_thread();

    return ConnectionResult::SUCCESS;
}

ConnectionResult MTSerialConnection::setup_port() {
	if ((_schedfd = zsv_open_scheduler()) < 0) {
		LogErr() << "zsv_open_scheduler() failed";
		return ConnectionResult::CONNECTION_ERROR;
	}

	if (zsv_mtserial_init(_schedfd, _baudrate) < 0) {
		LogErr() << "zsv_mtserial_init() failed";
		return ConnectionResult::CONNECTION_ERROR;
	}

	return ConnectionResult::SUCCESS;
}

void MTSerialConnection::start_recv_thread()
{
    _recv_thread = new std::thread(&MTSerialConnection::receive, this);
}

ConnectionResult MTSerialConnection::stop()
{
    _should_exit = true;

    if (_recv_thread) {
        _recv_thread->join();
        delete _recv_thread;
        _recv_thread = nullptr;
    }

    // We need to stop this after stopping the receive thread, otherwise
    // it can happen that we interfere with the parsing of a message.
    stop_mavlink_receiver();

    return ConnectionResult::SUCCESS;
}

bool MTSerialConnection::send_message(const mavlink_message_t &message)
{
    if (_serial_node.empty()) {
        LogErr() << "Dev Path unknown";
        return false;
    }

    if (_baudrate == 0) {
        LogErr() << "Baudrate unknown";
        return false;
    }

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t buffer_len = mavlink_msg_to_send_buffer(buffer, &message);

    int send_len = zsv_mtserial_send_finish(_schedfd, 0, buffer, buffer_len);

    if (send_len != buffer_len) {
        LogErr() << "write failure";
        return false;
    }

    return true;
}

void MTSerialConnection::receive()
{
    // Enough for MTU 1500 bytes.
    char buffer[2048];

    while (!_should_exit) {
        int recv_len = zsv_mtserial_recv(_schedfd, 0, buffer, sizeof(buffer));
        if (recv_len < -1) {
            LogErr() << "read failure";
        }
        if (recv_len > static_cast<int>(sizeof(buffer)) || recv_len == 0) {
            continue;
        }
        _mavlink_receiver->set_new_datagram(buffer, recv_len);
        // Parse all mavlink messages in one data packet. Once exhausted, we'll exit while.
        while (_mavlink_receiver->parse_message()) {
            receive_message(_mavlink_receiver->get_last_message());
        }
    }
}
} // namespace dronecode_sdk
