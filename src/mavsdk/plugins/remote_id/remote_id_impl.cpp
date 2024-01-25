#include "remote_id_impl.h"

namespace mavsdk {

RemoteIdImpl::RemoteIdImpl(System& system) : PluginImplBase(system)
{
    _system_impl->register_plugin(this);
}

RemoteIdImpl::RemoteIdImpl(std::shared_ptr<System> system) : PluginImplBase(std::move(system))
{
    _system_impl->register_plugin(this);
}

RemoteIdImpl::~RemoteIdImpl()
{
    _system_impl->unregister_plugin(this);
}

void RemoteIdImpl::init()
{
    _id_or_mac.reserve(20);
    _id_or_mac.assign(20, 0x00);
    // Init BasicId
    _basic_id.id_type = RemoteId::BasicId::IdType::None;
    _basic_id.ua_type = RemoteId::BasicId::UasType::None;
    _basic_id.uas_id.reserve(20);
    _basic_id.uas_id.assign(20, 0x00);
    // Init OperatorId
    _operator_id.operator_id_type = RemoteId::OperatorId::OperatorIdType::Caa;
    _operator_id.operator_id.assign(20, 0x00);
    // Init Location
    _location.status = RemoteId::Location::Status::Undeclared;
    _location.direction_deg = 361;
    _location.speed_horizontal_m_s = 255;
    _location.speed_vertical_m_s = 63;
    _location.latitude_deg = 0;
    _location.longitude_deg = 0;
    _location.altitude_barometric_m = -1000;
    _location.altitude_geodetic_m = -1000;
    _location.height_reference = RemoteId::Location::HeightRef::OverTakeoff;
    _location.height_m = -1000;
    _location.time_utc_us = 0;
    // LocationAccuracy
    _location_accuracy.horizontal_accuracy = RemoteId::LocationAccuracy::HorAcc::Unknown;
    _location_accuracy.vertical_accuracy = RemoteId::LocationAccuracy::VerAcc::Unknown;
    _location_accuracy.barometer_accuracy = RemoteId::LocationAccuracy::VerAcc::Unknown;
    _location_accuracy.speed_accuracy = RemoteId::LocationAccuracy::SpeedAcc::Unknown;
    _location_accuracy.timestamp_accuracy = RemoteId::LocationAccuracy::TimeAcc::Unknown;
    // Init System
    _system.operator_location_type = RemoteId::SystemId::OperatorLocationType::Takeoff;
    _system.classification_type = RemoteId::SystemId::ClassificationType::Undeclared;
    _system.operator_latitude_deg = 0;
    _system.operator_longitude_deg = 0;
    _system.area_count = 1;
    _system.area_radius_m = 0;
    _system.area_ceiling_m = -1000;
    _system.area_floor_m = -1000;
    _system.category_eu = RemoteId::SystemId::CategoryEu::Undeclared;
    _system.class_eu = RemoteId::SystemId::ClassEu::Undeclared;
    _system.operator_altitude_geo_m = -1000;
    _system.time_utc_us = 0;
    // Init SelfId
    _self_id.description_type = RemoteId::SelfId::DescType::Text;
    _self_id.description = std::string("");

     _system_impl->register_mavlink_message_handler(
                 MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS,
                 [this](const mavlink_message_t& message) { process_arm_status(message); },
                 this);

    _system_impl->add_call_every(
        [this]() { send_drone_location(); }, 1.0, &_send_drone_location_call_every_cookie);
    _system_impl->add_call_every(
        [this]() { send_drone_basic_id(); }, 1.0, &_send_drone_basic_id_call_every_cookie);
    _system_impl->add_call_every(
        [this]() { send_operator_id(); }, 1.0, &_send_operator_id_call_every_cookie);
    _system_impl->add_call_every(
        [this]() { send_drone_id_system(); }, 1.0, &_send_drone_id_system_call_every_cookie);
    _system_impl->add_call_every(
        [this]() { send_drone_self_id(); }, 1.0, &_send_self_id_call_every_cookie);
}

void RemoteIdImpl::deinit()
{
    _system_impl->remove_call_every(_send_drone_location_call_every_cookie);
    _system_impl->remove_call_every(_send_drone_basic_id_call_every_cookie);
    _system_impl->remove_call_every(_send_drone_id_system_call_every_cookie);
}

void RemoteIdImpl::enable()
{
    // FIXME: Remote endpoint not an autopilot but a remoteId
}

void RemoteIdImpl::disable()
{
    // FIXME: Remote endpoint not an autopilot but a remoteId
}

RemoteId::Result RemoteIdImpl::set_basic_id(RemoteId::BasicId basic_id)
{
    _basic_id = basic_id;
    return RemoteId::Result::Success;
}

RemoteId::Result RemoteIdImpl::set_operator_id(RemoteId::OperatorId operator_id)
{
    _operator_id = operator_id;
    return RemoteId::Result::Success;
}

RemoteId::Result RemoteIdImpl::set_self_id(RemoteId::SelfId self_id)
{
    _self_id = self_id;
    return RemoteId::Result::Success;
}

RemoteId::Result RemoteIdImpl::set_location(RemoteId::Location location)
{
    _location = location;
    return RemoteId::Result::Success;
}

RemoteId::Result RemoteIdImpl::set_location_accuracy(RemoteId::LocationAccuracy location_accuracy)
{
    _location_accuracy = location_accuracy;
    return RemoteId::Result::Success;
}

RemoteId::Result RemoteIdImpl::set_system(RemoteId::SystemId system)
{
    _system = system;
    if (!_system.time_utc_us)
        _system.time_utc_us = get_seconds_after_2019();
    return RemoteId::Result::Success;
}

RemoteId::ArmStatusHandle
RemoteIdImpl::subscribe_arm_status(const RemoteId::ArmStatusCallback& callback)
{
    std::lock_guard<std::mutex> lock(_armStatus.mutex);
    auto handle = _armStatus.subscription_callbacks.subscribe(callback);
    return handle;
}

void RemoteIdImpl::unsubscribe_arm_status(RemoteId::ArmStatusHandle handle)
{
    std::lock_guard<std::mutex> lock(_armStatus.mutex);
    _armStatus.subscription_callbacks.unsubscribe(handle);
}

RemoteId::ArmStatus RemoteIdImpl::arm_status() const
{
    return _armStatus.data;
}

void RemoteIdImpl::send_drone_location()
{
    mavlink_message_t msg{};
    mavlink_msg_open_drone_id_location_pack(
        _system_impl->get_own_system_id(),
        _system_impl->get_own_component_id(),
        &msg,
        0, // System ID (0 for broadcast).
        0, // Component ID (0 for broadcast).
        (const uint8_t*)_id_or_mac.c_str(), // Only used for drone ID data received from other UAs.
        (uint8_t)_location
            .status, // Indicates whether the unmanned aircraft is on the ground or in the air.
        (uint16_t)(_location.direction_deg * 100.0), // Direction over ground
        (uint16_t)(_location.speed_horizontal_m_s * 100), // Ground speed. Positive only.
        (int16_t)(_location.speed_vertical_m_s * 100), // The vertical speed. Up is positive.
        (int32_t)(_location.latitude_deg * 1e7), // Current latitude of the unmanned aircraft.
        (int32_t)(_location.longitude_deg * 1e7), // Current longitude of the unmanned aircraft.
        (float)
            _location.altitude_barometric_m, // The altitude calculated from the barometric pressue.
        (float)_location.altitude_geodetic_m, // The geodetic altitude as defined by WGS84.
        (uint8_t)_location.height_reference, // Indicates the reference point for the height field.
        (float)_location.height_m, // The current height of the unmanned aircraft above the take-off
                                   // location or the ground
        (uint8_t)_location_accuracy.horizontal_accuracy, // The accuracy of the horizontal position.
        (uint8_t)_location_accuracy.vertical_accuracy, // The accuracy of the vertical position.
        (uint8_t)_location_accuracy.barometer_accuracy, // The accuracy of the barometric altitude.
        (uint8_t)
            _location_accuracy.speed_accuracy, // The accuracy of the horizontal and vertical speed.
        (float)get_seconds_in_full_hour_for_utc_time(
                    _location.time_utc_us == 0 ?
                        get_utc_time_in_milliseconds()/1000.0 :
                        _location.time_utc_us/1000000.0
                    ), // Seconds after the full hour with reference to UTC time.
        (uint8_t)_location_accuracy.timestamp_accuracy // The accuracy of the timestamps.
    );

    _system_impl->send_message(msg);
    LogErr() << "send_drone_basic_id " << _location;
}

void RemoteIdImpl::send_drone_basic_id()
{
    mavlink_message_t msg{};
    mavlink_msg_open_drone_id_basic_id_pack(
        _system_impl->get_own_system_id(),
        _system_impl->get_own_component_id(),
        &msg,
        0, // System ID (0 for broadcast).
        0, // Component ID (0 for broadcast).
        (const uint8_t*)_id_or_mac.c_str(), // Only used for drone ID data received from other UAs.
        (uint8_t)_basic_id.id_type, // Indicates the format for the uas_id field of this message.
        (uint8_t)_basic_id.ua_type, // Indicates the type of UA (Unmanned Aircraft).
        (const uint8_t*)(_basic_id.uas_id.c_str()));

    _system_impl->send_message(msg);
    // LogErr() << "send_drone_basic_id " << _basic_id;
}

void RemoteIdImpl::send_operator_id()
{
    mavlink_message_t msg{};
    mavlink_msg_open_drone_id_operator_id_pack(
        _system_impl->get_own_system_id(),
        _system_impl->get_own_component_id(),
        &msg,
        0, // System ID (0 for broadcast).
        0, // Component ID (0 for broadcast).
        (const uint8_t*)_id_or_mac.c_str(), // Only used for drone ID data received from other UAs.
        (uint8_t)_operator_id
            .operator_id_type, // Indicates the format for the uas_id field of this message.
        (const char*)(_operator_id.operator_id.c_str()));

    _system_impl->send_message(msg);
    // LogErr() << "send_drone_basic_id " << _basic_id;
}

void RemoteIdImpl::send_drone_id_system()   // FIXME: Handle time auto increments
{
    mavlink_message_t msg{};

    if (_system.time_utc_us) {
        mavlink_msg_open_drone_id_system_pack(
            _system_impl->get_own_system_id(),
            _system_impl->get_own_component_id(),
            &msg,
            0, // System ID (0 for broadcast).
            0, // Component ID (0 for broadcast).
            (const uint8_t*)
                _id_or_mac.c_str(), // Only used for drone ID data received from other UAs.
            (uint8_t)_system.operator_location_type, // Specifies the operator location type.
            (uint8_t)_system.classification_type, // Specifies the classification type of the UA.
            (int32_t)(_system.operator_latitude_deg * 1e7), // Latitude of the operator.
            (int32_t)(_system.operator_longitude_deg * 1e7), // Longitude of the operator.
            (uint16_t)_system.area_count, // Number of aircraft in the area, group or formation
            (uint16_t)
                _system.area_radius_m, // Radius of the cylindrical area of the group or formation
            (float)_system.area_ceiling_m, // Area Operations Ceiling relative to WGS84
            (float)_system.area_floor_m, // Area Operations Floor relative to WGS84.
            (uint8_t)_system.category_eu, // Specifies the category of the UA.
            (uint8_t)_system.class_eu, // Specifies the class of the UA.
            (float)_system.operator_altitude_geo_m,
            (uint32_t)_system.time_utc_us);
    } else {
        _system.time_utc_us = get_seconds_after_2019();
        mavlink_msg_open_drone_id_system_update_pack(
            _system_impl->get_own_system_id(),
            _system_impl->get_own_component_id(),
            &msg,
            0, // System ID (0 for broadcast).
            0, // Component ID (0 for broadcast).
            (int32_t)(_system.operator_latitude_deg * 1e7), // Latitude of the operator.
            (int32_t)(_system.operator_longitude_deg * 1e7), // Longitude of the operator.
            (float)_system.operator_altitude_geo_m,
            (uint32_t)_system.time_utc_us);
    }

    _system.time_utc_us = 0;
    _system_impl->send_message(msg);
    LogErr() << "send_drone_basic_id " << _system;
}

void RemoteIdImpl::send_drone_self_id()
{
    mavlink_message_t msg{};
    mavlink_msg_open_drone_id_self_id_pack(
        _system_impl->get_own_system_id(),
        _system_impl->get_own_component_id(),
        &msg,
        0, // System ID (0 for broadcast).
        0, // Component ID (0 for broadcast).
        (const uint8_t*)_id_or_mac.c_str(), // Only used for drone ID data received from other UAs.
        (uint8_t)
            _self_id.description_type, // Indicates the format for the uas_id field of this message.
        (const char*)(_self_id.description.c_str()));

    _system_impl->send_message(msg);
    LogErr() << "send_drone_self_id " << _self_id;
}

void RemoteIdImpl::process_arm_status(const mavlink_message_t &message)
{
    mavlink_open_drone_id_arm_status_t arm_status;
    mavlink_msg_open_drone_id_arm_status_decode(&message, &arm_status);
    {
        std::lock_guard<std::mutex> lock(_armStatus.mutex);
        _armStatus.data.status = (RemoteId::ArmStatus::Status)arm_status.status;
        _armStatus.data.error = (const char *)arm_status.error;
    }
}

uint32_t RemoteIdImpl::get_seconds_after_2019()
{
    auto secs = std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::system_clock::now().time_since_epoch())
                    .count() -
                1609459200;
    return (uint32_t)secs;
}

uint64_t RemoteIdImpl::get_utc_time_in_milliseconds()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch())
            .count();
}

uint16_t RemoteIdImpl::get_seconds_in_full_hour_for_utc_time(uint64_t utc_time)
{
    return (uint16_t)(utc_time % (60*60*1000));
}

} // namespace mavsdk
