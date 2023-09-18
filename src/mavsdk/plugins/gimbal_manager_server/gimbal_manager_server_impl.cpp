#include "gimbal_manager_server_impl.h"
#include "callback_list.tpp"

#define MAVLINK_NULL_ARG 0

namespace mavsdk {

template class CallbackList<GimbalManagerServer::RoiLocation>;

GimbalManagerServerImpl::GimbalManagerServerImpl(
    std::shared_ptr<ServerComponent> server_component) :
    ServerPluginImplBase(server_component)
{
    _server_component_impl->register_plugin(this);
    CallbackList<GimbalManagerServer::RoiLocation> _test;
}

GimbalManagerServerImpl::~GimbalManagerServerImpl()
{
    _server_component_impl->unregister_plugin(this);
}

void GimbalManagerServerImpl::init()
{
    LogDebug() << "GimbalManagerServerImpl::init "
               << (int)_server_component_impl->get_own_component_id();
    _device_information.information.gimbal_device_id = 0; // No gimbal is active
    _manager_status.status.gimbal_device_id = 0; // No gimbal under control

    _server_component_impl->register_mavlink_command_handler(
        MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
        [this](const MavlinkCommandReceiver::CommandLong& command) {
            return process_gimbal_manager_configure_request(command);
        },
        this);

    _server_component_impl->register_mavlink_command_handler(
        MAV_CMD_REQUEST_MESSAGE,
        [this](const MavlinkCommandReceiver::CommandLong& command) {
            return process_request_message_request(command);
        },
        this);

    _server_component_impl->register_mavlink_command_handler(
        MAV_CMD_DO_SET_ROI_LOCATION,
        [this](const MavlinkCommandReceiver::CommandLong& command) {
            LogDebug() << "MAV_CMD_DO_SET_ROI_LOCATION CmdLong id:" << command.params.param1
                       << "\n";

            if (!is_gimbal_device_discovered(command.params.param1))
                return _server_component_impl->make_command_ack_message(
                    command, MAV_RESULT::MAV_RESULT_DENIED);

            process_set_roi_location_request(
                (uint32_t)command.params.param1,
                (uint32_t)command.params.param5,
                (uint32_t)command.params.param6,
                command.params.param7);
            return _server_component_impl->make_command_ack_message(
                command, MAV_RESULT::MAV_RESULT_ACCEPTED);
        },
        this);

    _server_component_impl->register_mavlink_command_handler(
        MAV_CMD_DO_SET_ROI_LOCATION,
        [this](const MavlinkCommandReceiver::CommandInt& command) {
            LogDebug() << "MAV_CMD_DO_SET_ROI_LOCATION CmdInt id:" << command.params.param1 << "\n";

            if (!is_gimbal_device_discovered(command.params.param1))
                return _server_component_impl->make_command_ack_message(
                    command, MAV_RESULT::MAV_RESULT_DENIED);

            process_set_roi_location_request(
                (uint32_t)command.params.param1,
                command.params.x,
                command.params.y,
                command.params.z);
            return _server_component_impl->make_command_ack_message(
                command, MAV_RESULT::MAV_RESULT_ACCEPTED);
        },
        this);

    _server_component_impl->register_mavlink_message_handler(
        MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS,
        [this](const mavlink_message_t& message) {
            process_gimbal_device_attitude_status(message);
        },
        this);

    _server_component_impl->mavlink_request_message_handler().register_handler(
        MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION,
        [this](uint8_t, uint8_t, const MavlinkRequestMessageHandler::Params&) {
            std::lock_guard<std::mutex> lock(_device_information.mutex);
            // On request only send information if we already received our information and we
            // are not already sending one with a small delay
            if (_device_information.information.gimbal_device_id) {
                _server_component_impl->add_call_every(
                    [this]() { send_gimbal_manager_information(); },
                    _device_information.INFORMATION_DELAY_INTERVAL_S,
                    &_device_information.information_call_every_cookie);
            }
            return MAV_RESULT_ACCEPTED;
        },
        this);

    _server_component_impl->register_mavlink_message_handler(
        MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE,
        [this](const mavlink_message_t& message) {
            process_gimbal_manager_set_attitude(message);
            return MAV_RESULT_ACCEPTED;
        },
        this);

    _server_component_impl->register_mavlink_message_handler(
        MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW,
        [this](const mavlink_message_t& message) {
            process_gimbal_manager_set_pitch_yaw(message);
            return MAV_RESULT_ACCEPTED;
        },
        this);

    /**
     * Always broadcast the manager status
     */
    _server_component_impl->add_call_every(
        [this]() { broadcast_gimbal_manager_status(); },
        _manager_status.STATUS_SEND_INTERVAL_S,
        &_call_every_cookie);

    /**
     * Gimbal device discovery
     */
    _server_component_impl->add_call_every(
        [this]() { discovery_gimbal_device(); },
        _device_information.DISCOVERY_INTERVAL_S,
        &_device_information.discovery_call_every_cookie);

    _server_component_impl->register_mavlink_message_handler(
        MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION,
        [this](const mavlink_message_t& message) { process_gimbal_device_information(message); },
        this);
}

void GimbalManagerServerImpl::deinit()
{
    _server_component_impl->remove_call_every(_call_every_cookie);
}

void GimbalManagerServerImpl::send_gimbal_manager_information()
{
    {
        /* We send manager information only ont time for request */
        std::lock_guard<std::mutex> lock(_device_information.mutex);
        _server_component_impl->remove_call_every(
            _device_information.information_call_every_cookie);
        _device_information.information_call_every_cookie = nullptr;
    }

    LogDebug() << "send_gimbal_manager_information";
    mavlink_message_t msg;
    mavlink_msg_gimbal_manager_information_pack(
        _server_component_impl->get_own_system_id(),
        _server_component_impl->get_own_component_id(),
        &msg,
        _server_component_impl->get_time().elapsed_ms(),
        _device_information.information.cap_flags,
        _device_information.information.gimbal_device_id,
        _device_information.information.roll_min,
        _device_information.information.roll_max,
        _device_information.information.pitch_min,
        _device_information.information.pitch_max,
        _device_information.information.yaw_min,
        _device_information.information.yaw_max);

    _server_component_impl->send_message(msg);
}

GimbalManagerServer::Result
GimbalManagerServerImpl::set_information(GimbalManagerServer::Information information)
{
    /** Gimbal device discovery by API */
    _device_information.information = information;
    LogDebug() << "GimbalManagerServerImpl::set_information " << (int)information.gimbal_device_id;
    return GimbalManagerServer::Result::Success;
}

GimbalManagerServer::GimbalManagerStatus GimbalManagerServerImpl::get_gimbal_manager_status() const
{
    return _manager_status.status;
}

GimbalManagerServer::SetAttitudeHandle GimbalManagerServerImpl::subscribe_set_attitude(
    const GimbalManagerServer::SetAttitudeCallback callback)
{
    std::lock_guard<std::mutex> lock(_set_attitude.mutex);
    return _set_attitude.callbacks.subscribe(callback);
}

void GimbalManagerServerImpl::unsubscribe_set_attitude(
    GimbalManagerServer::SetAttitudeHandle handle)
{
    std::lock_guard<std::mutex> lock(_set_attitude.mutex);
    _set_attitude.callbacks.unsubscribe(handle);
}

GimbalManagerServer::SetRoiLocationHandle GimbalManagerServerImpl::subscribe_set_roi_location(
    const GimbalManagerServer::SetRoiLocationCallback callback)
{
    std::lock_guard<std::mutex> lock(_roi_location.mutex);
    return _roi_location.callbacks.subscribe(callback);
}

void GimbalManagerServerImpl::unsubscribe_set_roi_location(
    GimbalManagerServer::SetRoiLocationHandle handle)
{
    std::lock_guard<std::mutex> lock(_roi_location.mutex);
    _roi_location.callbacks.unsubscribe(handle);
}

void GimbalManagerServerImpl::broadcast_gimbal_manager_status()
{
    // We don't send status packets until first gimbal is discovered
    if (!is_gimbal_device_discovered())
        return;

    mavlink_message_t msg;
    mavlink_msg_gimbal_manager_status_pack(
        _server_component_impl->get_own_system_id(),
        _server_component_impl->get_own_component_id(),
        &msg,
        _server_component_impl->get_time().elapsed_ms(),
        _manager_status.status.flags,
        _manager_status.status.gimbal_device_id,
        MAVLINK_NULL_ARG,
        MAVLINK_NULL_ARG,
        MAVLINK_NULL_ARG,
        MAVLINK_NULL_ARG);

    _server_component_impl->send_message(msg);
}

void GimbalManagerServerImpl::discovery_gimbal_device()
{
    if (is_gimbal_device_discovered())
        return;

    LogDebug() << "GimbalManagerServerImpl::discovery_gimbal_device send request message";

    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        _server_component_impl->get_own_system_id(),
        _server_component_impl->get_own_component_id(),
        &msg,
        0, // To All systems???
        MAV_COMP_ID_ALL,
        MAV_CMD_REQUEST_MESSAGE,
        0, // Confimation: no confirmation
        MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION,
        MAVLINK_NULL_ARG,
        MAVLINK_NULL_ARG,
        MAVLINK_NULL_ARG,
        MAVLINK_NULL_ARG,
        MAVLINK_NULL_ARG,
        2); // Target address for requested message: bradcast
    _server_component_impl->send_message(msg);
}

void GimbalManagerServerImpl::process_gimbal_device_information(const mavlink_message_t& message)
{
    // We only handle one device for now the first that respond to discovery
    if (_device_information.information.gimbal_device_id)
        return;

    mavlink_gimbal_device_information_t gimbal_device_information;
    mavlink_msg_gimbal_device_information_decode(&message, &gimbal_device_information);

    _device_information.information.gimbal_device_id = message.compid;
    _device_information.information.cap_flags = gimbal_device_information.cap_flags;
    _device_information.information.roll_min = gimbal_device_information.roll_min;
    _device_information.information.roll_max = gimbal_device_information.roll_max;
    _device_information.information.pitch_min = gimbal_device_information.pitch_min;
    _device_information.information.pitch_max = gimbal_device_information.pitch_max;
    _device_information.information.yaw_min = gimbal_device_information.yaw_min;
    _device_information.information.yaw_max = gimbal_device_information.yaw_max;

    // We start to handle this device
    _manager_status.status.gimbal_device_id = _device_information.information.gimbal_device_id;

    _device_discovered.callbacks.queue(_device_information.information, [this](const auto& func) {
        _server_component_impl->call_user_callback(func);
    });

    LogDebug() << "process_gimbal_device_information "
               << "discovered gimbal component " << (int)message.compid;
}

void GimbalManagerServerImpl::process_gimbal_device_attitude_status(
    const mavlink_message_t& message)
{
    mavlink_gimbal_device_attitude_status_t device_attitude_status;
    mavlink_msg_gimbal_device_attitude_status_decode(&message, &device_attitude_status);

    GimbalManagerServer::DeviceAttitudeStatus attitude_status;
    attitude_status.time_boot_ms = device_attitude_status.time_boot_ms;
    attitude_status.q.w = device_attitude_status.q[0];
    attitude_status.q.x = device_attitude_status.q[1];
    attitude_status.q.y = device_attitude_status.q[2];
    attitude_status.q.z = device_attitude_status.q[3];
    attitude_status.angular_velocity_x = device_attitude_status.angular_velocity_x;
    attitude_status.angular_velocity_y = device_attitude_status.angular_velocity_y;
    attitude_status.angular_velocity_z = device_attitude_status.angular_velocity_z;
    attitude_status.failure_flags = device_attitude_status.failure_flags;
    _device_attitude_status.callbacks.queue(attitude_status, [this](const auto& func) {
        _server_component_impl->call_user_callback(func);
    });
}

/**
 *  Device Discovered API implementation
 */

GimbalManagerServer::DeviceDiscoveredHandle GimbalManagerServerImpl::subscribe_device_discovered(
    const GimbalManagerServer::DeviceDiscoveredCallback& callback)
{
    std::lock_guard<std::mutex> lock(_device_discovered.mutex);
    return _device_discovered.callbacks.subscribe(callback);
}

void GimbalManagerServerImpl::unsubscribe_device_discovered(
    GimbalManagerServer::DeviceDiscoveredHandle handle)
{
    std::lock_guard<std::mutex> lock(_device_discovered.mutex);
    _device_discovered.callbacks.unsubscribe(handle);
}

GimbalManagerServer::Information GimbalManagerServerImpl::device_discovered() const
{
    return _device_information.information;
}

/**
 *  Gimbal Manager Configuration API implementation
 */

GimbalManagerServer::GimbalManagerConfiguredHandle GimbalManagerServerImpl::subscribe_gimbal_manager_configured(const GimbalManagerServer::GimbalManagerConfiguredCallback& callback) {
    std::lock_guard<std::mutex> lock(_manager_configuration.mutex);
    return _manager_configuration.callbacks.subscribe(callback);
}

void GimbalManagerServerImpl::unsubscribe_gimbal_manager_configured(GimbalManagerServer::GimbalManagerConfiguredHandle handle){
    std::lock_guard<std::mutex> lock(_manager_configuration.mutex);
    _manager_configuration.callbacks.unsubscribe(handle);
}

GimbalManagerServer::GimbalManagerConfiguration GimbalManagerServerImpl::gimbal_manager_configured() const {
    return _manager_configuration.configuration;
}


/**
 *  Device Attitude Status API implementation
 */

GimbalManagerServer::DeviceAttitudeStatusHandle
GimbalManagerServerImpl::subscribe_device_attitude_status(
    const GimbalManagerServer::DeviceAttitudeStatusCallback& callback)
{
    std::lock_guard<std::mutex> lock(_device_attitude_status.mutex);
    return _device_attitude_status.callbacks.subscribe(callback);
}

void GimbalManagerServerImpl::unsubscribe_device_attitude_status(
    GimbalManagerServer::DeviceAttitudeStatusHandle handle)
{
    _device_attitude_status.callbacks.unsubscribe(handle);
}

GimbalManagerServer::DeviceAttitudeStatus GimbalManagerServerImpl::device_attitude_status() const
{
    // FIXME: Complete implementation
    GimbalManagerServer::DeviceAttitudeStatus _status{};
    return _status;
}

void GimbalManagerServerImpl::process_gimbal_manager_set_attitude(const mavlink_message_t& message)
{
    if (!is_gimbal_device_discovered()) {
        LogErr() << "set_attitude received wothout a valid gimbal";
        return;
    }

    mavlink_gimbal_manager_set_attitude_t gimbal_manager_set_attitude;
    mavlink_msg_gimbal_manager_set_attitude_decode(&message, &gimbal_manager_set_attitude);

    mavlink_message_t msg;
    mavlink_msg_gimbal_device_set_attitude_pack(
        _server_component_impl->get_own_system_id(),
        _server_component_impl->get_own_component_id(),
        &msg,
        _server_component_impl->get_own_system_id(),
        static_cast<uint8_t>(_device_information.information.gimbal_device_id),
        gimbal_manager_set_attitude.flags,
        gimbal_manager_set_attitude.q,
        gimbal_manager_set_attitude.angular_velocity_x,
        gimbal_manager_set_attitude.angular_velocity_y,
        gimbal_manager_set_attitude.angular_velocity_z);
    _server_component_impl->send_message(msg);
}

void GimbalManagerServerImpl::process_gimbal_manager_set_pitch_yaw(const mavlink_message_t& message)
{
    mavlink_gimbal_manager_set_pitchyaw_t set_pitchyaw;
    mavlink_msg_gimbal_manager_set_pitchyaw_decode(&message, &set_pitchyaw);

    if (!is_gimbal_device_discovered(set_pitchyaw.gimbal_device_id)) {
        LogWarn() << "process_gimbal_manager_set_pitch_yaw called with wrong gimbal curr:"
                  << _manager_status.status.gimbal_device_id
                  << " req:" << set_pitchyaw.gimbal_device_id << "\n";
        return;
    }

    float q[4];
    mavlink_euler_to_quaternion(0, set_pitchyaw.pitch, set_pitchyaw.yaw, q);

    mavlink_message_t msg;
    mavlink_msg_gimbal_device_set_attitude_pack(
        _server_component_impl->get_own_system_id(),
        _server_component_impl->get_own_component_id(),
        &msg,
        _server_component_impl->get_own_system_id(),
        static_cast<uint8_t>(_device_information.information.gimbal_device_id),
        set_pitchyaw.flags,
        q,
        0,
        set_pitchyaw.pitch_rate,
        set_pitchyaw.yaw_rate);
    _server_component_impl->send_message(msg);
}

std::optional<mavlink_command_ack_t> GimbalManagerServerImpl::process_gimbal_manager_configure_request(
    const MavlinkCommandReceiver::CommandLong& command)
{
    LogDebug() << "GimbalManagerServerImpl::process_gimbal_manager_configure_request p1 "
               << command.params.param1 << " p2 " << command.params.param2 << " p3 "
               << command.params.param3 << " p4 " << command.params.param4 << " p7 "
               << command.params.param7;

    if(!is_gimbal_device_discovered(command.params.param7)) {
        LogWarn() << "Do configure called with wrong gimbal ID:" << command.params.param7;
        return _server_component_impl->make_command_ack_message(
            command, MAV_RESULT::MAV_RESULT_FAILED);
    }

    _manager_configuration.configuration.sysid_primary_control = command.params.param1;
    _manager_configuration.configuration.compid_primary_control = command.params.param2;
    _manager_configuration.configuration.sysid_secondary_control = command.params.param3;
    _manager_configuration.configuration.compid_secondary_control = command.params.param4;
    _manager_configuration.configuration.gimbal_device_id = command.params.param7;

    return _server_component_impl->make_command_ack_message(
        command, MAV_RESULT::MAV_RESULT_ACCEPTED);
}

std::optional<mavlink_command_ack_t> GimbalManagerServerImpl::process_request_message_request(
    const MavlinkCommandReceiver::CommandLong& command)
{
    LogDebug() << "GimbalManagerServerImpl::process_request_message_request p1 "
               << command.params.param1 << " p2 " << command.params.param2 << " p3 "
               << command.params.param3 << " p4 " << command.params.param4 << " p7 "
               << command.params.param7;
    return _server_component_impl->make_command_ack_message(
        command, MAV_RESULT::MAV_RESULT_ACCEPTED);
}

void GimbalManagerServerImpl::process_set_roi_location_request(
    uint32_t gimb_id, int32_t latitude, int32_t longitude, float altitude)
{
    GimbalManagerServer::RoiLocation roi_location;
    roi_location.gimbal_id = gimb_id;
    roi_location.latitude_deg = latitude / 1e7;
    roi_location.longitude_deg = longitude / 1e7;
    roi_location.absolute_altitude_m = (float)altitude;

    _roi_location.callbacks.queue(roi_location, [this](const auto& func) {
        _server_component_impl->call_user_callback(func);
    });
}

} // namespace mavsdk
