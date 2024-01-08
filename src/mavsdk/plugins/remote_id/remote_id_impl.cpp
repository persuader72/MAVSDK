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

void RemoteIdImpl::init() {
    _id_or_mac.reserve(20);
    _id_or_mac.assign(20, 0x00);
    // Init BasicId
    _basic_id.id_type = MAV_ODID_ID_TYPE_NONE;
    _basic_id.ua_type = MAV_ODID_UA_TYPE_NONE;
    _basic_id.uas_id.reserve(20);
    _basic_id.uas_id.assign(20, 0x00);
    // Init Location
    _location.status = MAV_ODID_STATUS_UNDECLARED;
    _location.direction = 36100;
    _location.speed_horizontal = 25500;
    _location.speed_vertical = 6300;
    _location.latitude = 0;
    _location.longitude = 0;
    _location.altitude_barometric = -1000;
    _location.altitude_geodetic = -1000;
    _location.height_reference = MAV_ODID_HEIGHT_REF_OVER_TAKEOFF;
    _location.height = -1000;

    _system_impl->add_call_every(
        [this]() { send_drone_location(); }, 1.0, &_send_drone_location_call_every_cookie);
    _system_impl->add_call_every(
        [this]() { send_drone_basic_id(); }, 1.0, &_send_drone_basic_id_call_every_cookie);
    _system_impl->add_call_every(
        [this]() { send_drone_id_system(); }, 1.0, &_send_drone_id_system_call_every_cookie);
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

RemoteId::Result RemoteIdImpl::set_location(RemoteId::Location location)
{
    _location = location;
    return RemoteId::Result::Success;
}

void RemoteIdImpl::send_drone_location()
{
    mavlink_message_t msg{};
    mavlink_msg_open_drone_id_location_pack(
        _system_impl->get_own_system_id(),
        _system_impl->get_own_component_id(),
        &msg,
        0,                                   // System ID (0 for broadcast).
        0,                                   // Component ID (0 for broadcast).
        (const uint8_t *)_id_or_mac.c_str(), // Only used for drone ID data received from other UAs.
        (uint8_t)_location.status,           // Indicates whether the unmanned aircraft is on the ground or in the air.
        (uint16_t)_location.direction,       // Direction over ground
        (uint16_t)_location.speed_horizontal,// Ground speed. Positive only.
        (int16_t)_location.speed_vertical,   // The vertical speed. Up is positive.
        (int32_t)_location.latitude,         // Current latitude of the unmanned aircraft.
        (int32_t)_location.longitude,        // Current longitude of the unmanned aircraft.
        (float)_location.altitude_barometric,// The altitude calculated from the barometric pressue.
        (float)_location.altitude_geodetic,  // The geodetic altitude as defined by WGS84.
        (uint8_t)_location.height_reference, // Indicates the reference point for the height field.
        (float)_location.height,             // The current height of the unmanned aircraft above the take-off location or the ground
        MAV_ODID_HOR_ACC_UNKNOWN,            // The accuracy of the horizontal position.
        MAV_ODID_VER_ACC_UNKNOWN,            // The accuracy of the vertical position.
        MAV_ODID_VER_ACC_UNKNOWN,            // The accuracy of the barometric altitude.
        MAV_ODID_SPEED_ACC_UNKNOWN,          // The accuracy of the horizontal and vertical speed.
        0xFFFF,                              // Seconds after the full hour with reference to UTC time.
        MAV_ODID_TIME_ACC_UNKNOWN            // The accuracy of the timestamps.
        );

    _system_impl->send_message(msg);
}

void RemoteIdImpl::send_drone_basic_id()
{
    mavlink_message_t msg{};
    mavlink_msg_open_drone_id_basic_id_pack(
        _system_impl->get_own_system_id(),
        _system_impl->get_own_component_id(),
        &msg,
        0,                                   // System ID (0 for broadcast).
        0,                                   // Component ID (0 for broadcast).
        (const uint8_t *)_id_or_mac.c_str(), // Only used for drone ID data received from other UAs.
        (uint8_t)_basic_id.id_type,          // Indicates the format for the uas_id field of this message.
        (uint8_t)_basic_id.ua_type,          // Indicates the type of UA (Unmanned Aircraft).
        (const uint8_t *)(_basic_id.uas_id.c_str())
        );

    _system_impl->send_message(msg);
}

void RemoteIdImpl::send_drone_id_system()
{
    LogErr() << "Cannot list photos: camera status has not been received yet!";
    mavlink_message_t msg{};
    mavlink_msg_open_drone_id_system_pack(
        _system_impl->get_own_system_id(),
        _system_impl->get_own_component_id(),
        &msg,
        0, // System ID (0 for broadcast).
        0, // Component ID (0 for broadcast).
        (const uint8_t *)_id_or_mac.c_str(), // Only used for drone ID data received from other UAs.
        MAV_ODID_OPERATOR_LOCATION_TYPE_TAKEOFF, // Specifies the operator location type.
        MAV_ODID_CLASSIFICATION_TYPE_EU,  // Specifies the classification type of the UA.
        0, // Latitude of the operator.
        0, // Longitude of the operator.
        1, // Number of aircraft in the area, group or formation
        0, // Radius of the cylindrical area of the group or formation
        -1000, // Area Operations Ceiling relative to WGS84
        -1000, // Area Operations Floor relative to WGS84.
        MAV_ODID_CATEGORY_EU_OPEN, // When classification_type is MAV_ODID_CLASSIFICATION_TYPE_EU, specifies the category of the UA.
        MAV_ODID_CLASS_EU_CLASS_3, // When classification_type is MAV_ODID_CLASSIFICATION_TYPE_EU, specifies the class of the UA.
        -1000,
        0);

    _system_impl->send_message(msg);
}




} // namespace mavsdk
