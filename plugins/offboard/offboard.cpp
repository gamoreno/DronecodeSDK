#include "plugins/offboard/offboard.h"
#include "offboard_impl.h"

namespace dronecode_sdk {

Offboard::Offboard(System &system) : PluginBase(), _impl{new OffboardImpl(system)} {}

Offboard::~Offboard() {}

Offboard::Result Offboard::start()
{
    return _impl->start();
}

Offboard::Result Offboard::stop()
{
    return _impl->stop();
}

void Offboard::start_async(result_callback_t callback)
{
    _impl->start_async(callback);
}

void Offboard::stop_async(result_callback_t callback)
{
    _impl->stop_async(callback);
}

bool Offboard::is_active() const
{
    return _impl->is_active();
}

void Offboard::set_velocity_ned(Offboard::VelocityNEDYaw velocity_ned_yaw)
{
    return _impl->set_velocity_ned(velocity_ned_yaw);
}

void Offboard::set_velocity_body(Offboard::VelocityBodyYawspeed velocity_body_yawspeed)
{
    return _impl->set_velocity_body(velocity_body_yawspeed);
}

const char *Offboard::result_str(Result result)
{
    switch (result) {
        case Result::SUCCESS:
            return "Success";
        case Result::NO_SYSTEM:
            return "No system";
        case Result::CONNECTION_ERROR:
            return "Connection error";
        case Result::BUSY:
            return "Busy";
        case Result::COMMAND_DENIED:
            return "Command denied";
        case Result::TIMEOUT:
            return "Timeout";
        case Result::UNKNOWN:
        default:
            return "Unknown";
    }
}

void Offboard::stop_sending_setpoints()
{
    _impl->stop_sending_setpoints();
}

} // namespace dronecode_sdk

void dronecode_sdk::Offboard::send_velocity_ned(const VelocityNEDYaw &velocity_ned_yaw)
{
    _impl->send_velocity_ned(velocity_ned_yaw);
}

bool dronecode_sdk::Offboard::is_using_send_finish() const
{
    return _impl->is_using_send_finish();
}

void dronecode_sdk::Offboard::set_using_send_finish(int zsrm_reservation_id)
{
    _impl->set_using_send_finish(zsrm_reservation_id);
}
