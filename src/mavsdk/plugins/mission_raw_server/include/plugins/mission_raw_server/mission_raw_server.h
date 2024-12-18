// WARNING: THIS FILE IS AUTOGENERATED! As such, it should not be edited.
// Edits need to be made to the proto files
// (see
// https://github.com/mavlink/MAVSDK-Proto/blob/main/protos/mission_raw_server/mission_raw_server.proto)

#pragma once

#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "server_plugin_base.h"

#include "handle.h"

namespace mavsdk {

class ServerComponent;
class MissionRawServerImpl;

/**
 * @brief Acts as a vehicle and receives incoming missions from GCS (in raw MAVLINK format).
 * Provides current mission item state, so the server can progress through missions.
 */
class MissionRawServer : public ServerPluginBase {
public:
    /**
     * @brief Constructor. Creates the plugin for a ServerComponent instance.
     *
     * The plugin is typically created as shown below:
     *
     *     ```cpp
     *     auto mission_raw_server = MissionRawServer(server_component);
     *     ```
     *
     * @param server_component The ServerComponent instance associated with this server plugin.
     */
    explicit MissionRawServer(std::shared_ptr<ServerComponent> server_component);

    /**
     * @brief Destructor (internal use only).
     */
    ~MissionRawServer() override;

    /**
     * @brief Mission item exactly identical to MAVLink MISSION_ITEM_INT.
     */
    struct MissionItem {
        uint32_t seq{}; /**< @brief Sequence (uint16_t) */
        uint32_t frame{}; /**< @brief The coordinate system of the waypoint (actually uint8_t) */
        uint32_t command{}; /**< @brief The scheduled action for the waypoint (actually uint16_t) */
        uint32_t current{}; /**< @brief false:0, true:1 (actually uint8_t) */
        uint32_t autocontinue{}; /**< @brief Autocontinue to next waypoint (actually uint8_t) */
        float param1{}; /**< @brief PARAM1, see MAV_CMD enum */
        float param2{}; /**< @brief PARAM2, see MAV_CMD enum */
        float param3{}; /**< @brief PARAM3, see MAV_CMD enum */
        float param4{}; /**< @brief PARAM4, see MAV_CMD enum */
        int32_t x{}; /**< @brief PARAM5 / local: x position in meters * 1e4, global: latitude in
                        degrees * 10^7 */
        int32_t y{}; /**< @brief PARAM6 / y position: local: x position in meters * 1e4, global:
                        longitude in degrees *10^7 */
        float z{}; /**< @brief PARAM7 / local: Z coordinate, global: altitude (relative or absolute,
                      depending on frame) */
        uint32_t mission_type{}; /**< @brief Mission type (actually uint8_t) */
    };

    /**
     * @brief Equal operator to compare two `MissionRawServer::MissionItem` objects.
     *
     * @return `true` if items are equal.
     */
    friend bool
    operator==(const MissionRawServer::MissionItem& lhs, const MissionRawServer::MissionItem& rhs);

    /**
     * @brief Stream operator to print information about a `MissionRawServer::MissionItem`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream&
    operator<<(std::ostream& str, MissionRawServer::MissionItem const& mission_item);

    /**
     * @brief Mission plan type
     */
    struct MissionPlan {
        std::vector<MissionItem> mission_items{}; /**< @brief The mission items */
    };

    /**
     * @brief Equal operator to compare two `MissionRawServer::MissionPlan` objects.
     *
     * @return `true` if items are equal.
     */
    friend bool
    operator==(const MissionRawServer::MissionPlan& lhs, const MissionRawServer::MissionPlan& rhs);

    /**
     * @brief Stream operator to print information about a `MissionRawServer::MissionPlan`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream&
    operator<<(std::ostream& str, MissionRawServer::MissionPlan const& mission_plan);

    /**
     * @brief Mission progress type.
     */
    struct MissionProgress {
        int32_t current{}; /**< @brief Current mission item index (0-based), if equal to total, the
                              mission is finished */
        int32_t total{}; /**< @brief Total number of mission items */
    };

    /**
     * @brief Equal operator to compare two `MissionRawServer::MissionProgress` objects.
     *
     * @return `true` if items are equal.
     */
    friend bool operator==(
        const MissionRawServer::MissionProgress& lhs, const MissionRawServer::MissionProgress& rhs);

    /**
     * @brief Stream operator to print information about a `MissionRawServer::MissionProgress`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream&
    operator<<(std::ostream& str, MissionRawServer::MissionProgress const& mission_progress);

    /**
     * @brief Possible results returned for action requests.
     */
    enum class Result {
        Unknown, /**< @brief Unknown result. */
        Success, /**< @brief Request succeeded. */
        Error, /**< @brief Error. */
        TooManyMissionItems, /**< @brief Too many mission items in the mission. */
        Busy, /**< @brief Vehicle is busy. */
        Timeout, /**< @brief Request timed out. */
        InvalidArgument, /**< @brief Invalid argument. */
        Unsupported, /**< @brief Mission downloaded from the system is not supported. */
        NoMissionAvailable, /**< @brief No mission available on the system. */
        UnsupportedMissionCmd, /**< @brief Unsupported mission command. */
        TransferCancelled, /**< @brief Mission transfer (upload or download) has been cancelled. */
        NoSystem, /**< @brief No system connected. */
        Next, /**< @brief Intermediate message showing progress or instructions on the next steps.
               */
    };

    /**
     * @brief Stream operator to print information about a `MissionRawServer::Result`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream& operator<<(std::ostream& str, MissionRawServer::Result const& result);

    /**
     * @brief Callback type for asynchronous MissionRawServer calls.
     */
    using ResultCallback = std::function<void(Result)>;

    /**
     * @brief Callback type for subscribe_incoming_mission.
     */
    using IncomingMissionCallback = std::function<void(Result, MissionPlan)>;

    /**
     * @brief Handle type for subscribe_incoming_mission.
     */
    using IncomingMissionHandle = Handle<Result, MissionPlan>;

    /**
     * @brief Subscribe to when a new mission is uploaded (asynchronous).
     */
    IncomingMissionHandle subscribe_incoming_mission(const IncomingMissionCallback& callback);

    /**
     * @brief Unsubscribe from subscribe_incoming_mission
     */
    void unsubscribe_incoming_mission(IncomingMissionHandle handle);

    /**
     * @brief Callback type for subscribe_current_item_changed.
     */
    using CurrentItemChangedCallback = std::function<void(MissionItem)>;

    /**
     * @brief Handle type for subscribe_current_item_changed.
     */
    using CurrentItemChangedHandle = Handle<MissionItem>;

    /**
     * @brief Subscribe to when a new current item is set
     */
    CurrentItemChangedHandle
    subscribe_current_item_changed(const CurrentItemChangedCallback& callback);

    /**
     * @brief Unsubscribe from subscribe_current_item_changed
     */
    void unsubscribe_current_item_changed(CurrentItemChangedHandle handle);

    /**
     * @brief Set Current item as completed
     *
     * This function is blocking.
     *
     * @return Result of request.
     */
    void set_current_item_complete() const;

    /**
     * @brief Callback type for subscribe_clear_all.
     */
    using ClearAllCallback = std::function<void(uint32_t)>;

    /**
     * @brief Handle type for subscribe_clear_all.
     */
    using ClearAllHandle = Handle<uint32_t>;

    /**
     * @brief Subscribe when a MISSION_CLEAR_ALL is received
     */
    ClearAllHandle subscribe_clear_all(const ClearAllCallback& callback);

    /**
     * @brief Unsubscribe from subscribe_clear_all
     */
    void unsubscribe_clear_all(ClearAllHandle handle);

    /**
     * @brief Copy constructor.
     */
    MissionRawServer(const MissionRawServer& other);

    /**
     * @brief Equality operator (object is not copyable).
     */
    const MissionRawServer& operator=(const MissionRawServer&) = delete;

private:
    /** @private Underlying implementation, set at instantiation */
    std::unique_ptr<MissionRawServerImpl> _impl;
};

} // namespace mavsdk