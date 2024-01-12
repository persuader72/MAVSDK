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
    _basic_id.id_type = MAV_ODID_ID_TYPE_NONE;
    _basic_id.ua_type = MAV_ODID_UA_TYPE_NONE;
    _basic_id.uas_id.reserve(20);
    _basic_id.uas_id.assign(20, 0x00);
    // Init OperatorId
    _operator_id.operator_id_type = MAV_ODID_OPERATOR_ID_TYPE_CAA;
    _operator_id.operator_id.assign(20, 0x00);
    // Init Location
    _location.status = MAV_ODID_STATUS_UNDECLARED;
    _location.direction_deg = 361;
    _location.speed_horizontal_cm_s = 25500;
    _location.speed_vertical_cm_s = 6300;
    _location.latitude_deg = 0;
    _location.longitude_deg = 0;
    _location.altitude_barometric_m = -1000;
    _location.altitude_geodetic_m = -1000;
    _location.height_reference = MAV_ODID_HEIGHT_REF_OVER_TAKEOFF;
    _location.height_m = -1000;
    _location.timestamp_s = 0xFFFF;
    _location.timestamp_accuracy = MAV_ODID_TIME_ACC_UNKNOWN;
    // Init System
    _system.operator_location_type = MAV_ODID_OPERATOR_LOCATION_TYPE_TAKEOFF;
    _system.classification_type = MAV_ODID_CLASSIFICATION_TYPE_EU;
    _system.operator_latitude_deg = 0;
    _system.operator_longitude_deg = 0;
    _system.area_count = 1;
    _system.area_radius_m = 0;
    _system.area_ceiling_m = -1000;
    _system.area_floor_m = -1000;
    _system.category_eu = MAV_ODID_CATEGORY_EU_UNDECLARED;
    _system.class_eu = MAV_ODID_CLASS_EU_UNDECLARED;
    _system.operator_altitude_geo_m = -1000;
    _system.timestamp_s = 0;
    // Init SelfId
    _self_id.description_type = MAV_ODID_DESC_TYPE_TEXT;
    _self_id.description = std::string("");

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

RemoteId::Result RemoteIdImpl::set_system(RemoteId::SystemId system)
{
    _system = system;
    if (!_system.timestamp_s)
        _system.timestamp_s = get_seconds_after_2019();
    return RemoteId::Result::Success;
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
        (uint16_t)_location.speed_horizontal_cm_s, // Ground speed. Positive only.
        (int16_t)_location.speed_vertical_cm_s, // The vertical speed. Up is positive.
        (int32_t)(_location.latitude_deg * 1e7), // Current latitude of the unmanned aircraft.
        (int32_t)(_location.longitude_deg * 1e7), // Current longitude of the unmanned aircraft.
        (float)
            _location.altitude_barometric_m, // The altitude calculated from the barometric pressue.
        (float)_location.altitude_geodetic_m, // The geodetic altitude as defined by WGS84.
        (uint8_t)_location.height_reference, // Indicates the reference point for the height field.
        (float)_location.height_m, // The current height of the unmanned aircraft above the take-off
                                   // location or the ground
        MAV_ODID_HOR_ACC_UNKNOWN, // The accuracy of the horizontal position.
        MAV_ODID_VER_ACC_UNKNOWN, // The accuracy of the vertical position.
        MAV_ODID_VER_ACC_UNKNOWN, // The accuracy of the barometric altitude.
        MAV_ODID_SPEED_ACC_UNKNOWN, // The accuracy of the horizontal and vertical speed.
        (float)_location.timestamp_s, // Seconds after the full hour with reference to UTC time.
        (uint8_t)_location.timestamp_accuracy // The accuracy of the timestamps.
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

void RemoteIdImpl::send_drone_id_system()
{
    mavlink_message_t msg{};

    if (_system.timestamp_s) {
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
            (uint32_t)_system.timestamp_s);
    } else {
        _system.timestamp_s = get_seconds_after_2019();
        mavlink_msg_open_drone_id_system_update_pack(
            _system_impl->get_own_system_id(),
            _system_impl->get_own_component_id(),
            &msg,
            0, // System ID (0 for broadcast).
            0, // Component ID (0 for broadcast).
            (int32_t)(_system.operator_latitude_deg * 1e7), // Latitude of the operator.
            (int32_t)(_system.operator_longitude_deg * 1e7), // Longitude of the operator.
            (float)_system.operator_altitude_geo_m,
            (uint32_t)_system.timestamp_s);
    }

    _system.timestamp_s = 0;
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

uint32_t RemoteIdImpl::get_seconds_after_2019()
{
    auto secs = std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::system_clock::now().time_since_epoch())
                    .count() -
                1609459200;
    return (uint32_t)secs;
}

} // namespace mavsdk
