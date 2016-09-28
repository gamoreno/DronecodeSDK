#include "dronelink_impl.h"
#include "global_include.h"
#include <mutex>

namespace dronelink {

DroneLinkImpl::DroneLinkImpl() :
    _connections_mutex(),
    _connections(),
    _devices_mutex(),
    _devices(),
    _device_impls(),
    _on_discover_callback(nullptr),
    _on_timeout_callback(nullptr)
{}

DroneLinkImpl::~DroneLinkImpl()
{
    {
        std::lock_guard<std::mutex> lock(_connections_mutex);

        for (unsigned i = 0; i < _connections.size(); ++i) {
            delete _connections.at(i);
            _connections.at(i) = nullptr;
        }
        _connections.clear();
    }

    {
        std::lock_guard<std::mutex> lock(_devices_mutex);

        for (auto it = _devices.begin(); it != _devices.end(); ++it) {
            delete it->second;
        }

        for (auto it = _device_impls.begin(); it != _device_impls.end(); ++it) {
            delete it->second;
        }
        _devices.clear();
        _device_impls.clear();
    }
}

void DroneLinkImpl::receive_message(const mavlink_message_t &message)
{
    create_device_if_not_existing(message.sysid);

    _device_impls.at(message.sysid)->process_mavlink_message(message);
}

bool DroneLinkImpl::send_message(const mavlink_message_t &message)
{
    std::lock_guard<std::mutex> lock(_connections_mutex);

    for (auto it = _connections.begin(); it != _connections.end(); ++it) {
        if (!(**it).send_message(message)) {
            Debug() << "send fail";
            return false;
        }
    }

    return true;
}

void DroneLinkImpl::add_connection(Connection *new_connection)
{
    std::lock_guard<std::mutex> lock(_connections_mutex);
    _connections.push_back(new_connection);
}

const std::vector<uint64_t> &DroneLinkImpl::get_device_uuids() const
{
    // This needs to survive the scope.
    static std::vector<uint64_t> uuids;

    for (auto it = _device_impls.begin(); it != _device_impls.end(); ++it) {
        uint64_t uuid = it->second->get_target_uuid();
        if (uuid != 0) {
            uuids.push_back(uuid);
        }
    }

    return uuids;
}

Device &DroneLinkImpl::get_device(uint64_t uuid)
{
    uint8_t system_id = 0;
    {
        std::lock_guard<std::mutex> lock(_devices_mutex);
        // TODO: make a cache map for this.
        for (auto it = _device_impls.begin(); it != _device_impls.end(); ++it) {
            if (it->second->get_target_uuid() == uuid) {
                return *(_devices.at(it->first));
            }
        }
    }

    // TODO: this is an error condition that we ought to handle properly.
    if (system_id == 0) {
        Debug() << "device with UUID: " << uuid << " not found";
    }

    create_device_if_not_existing(system_id);

    return *_devices[system_id];
}

void DroneLinkImpl::create_device_if_not_existing(uint8_t system_id)
{
    std::lock_guard<std::mutex> lock(_devices_mutex);

    // existing already.
    if (_devices.find(system_id) != _devices.end()) {
        return;
    }

    // Create both lists in parallel
    DeviceImpl *new_device_impl = new DeviceImpl(this);
    _device_impls.insert(std::pair<uint8_t, DeviceImpl *>(system_id, new_device_impl));

    Device *new_device = new Device(new_device_impl);
    _devices.insert(std::pair<uint8_t, Device *>(system_id, new_device));
}

void DroneLinkImpl::notify_on_discover(uint64_t uuid)
{
    if (_on_discover_callback != nullptr) {
        _on_discover_callback(uuid);
    }
}

void DroneLinkImpl::notify_on_timeout(uint64_t uuid)
{
    if (_on_timeout_callback != nullptr) {
        _on_timeout_callback(uuid);
    }
}

void DroneLinkImpl::register_on_discover(DroneLink::event_callback_t callback)
{
    _on_discover_callback = callback;
}

void DroneLinkImpl::register_on_timeout(DroneLink::event_callback_t callback)
{
    _on_timeout_callback = callback;
}

} // namespace dronelink
