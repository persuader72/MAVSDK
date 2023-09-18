#pragma once

#include "plugins/gimbal_manager_server/gimbal_manager_server.h"
#include "server_plugin_impl_base.h"
#include "callback_list.h"

namespace mavsdk {

class GimbalManagerServerImpl : public ServerPluginImplBase {
public:
    explicit GimbalManagerServerImpl(std::shared_ptr<ServerComponent> server_component);

    ~GimbalManagerServerImpl() override;

    void init() override;
    void deinit() override;

    GimbalManagerServer::Result set_information(GimbalManagerServer::Information information);

    GimbalManagerServer::DeviceDiscoveredHandle
    subscribe_device_discovered(const GimbalManagerServer::DeviceDiscoveredCallback& callback);
    void unsubscribe_device_discovered(GimbalManagerServer::DeviceDiscoveredHandle handle);
    GimbalManagerServer::Information device_discovered() const;

    GimbalManagerServer::GimbalManagerConfiguredHandle subscribe_gimbal_manager_configured(const GimbalManagerServer::GimbalManagerConfiguredCallback& callback);
    void unsubscribe_gimbal_manager_configured(GimbalManagerServer::GimbalManagerConfiguredHandle handle);
    GimbalManagerServer::GimbalManagerConfiguration gimbal_manager_configured() const;

    GimbalManagerServer::DeviceAttitudeStatusHandle subscribe_device_attitude_status(
        const GimbalManagerServer::DeviceAttitudeStatusCallback& callback);
    void unsubscribe_device_attitude_status(GimbalManagerServer::DeviceAttitudeStatusHandle handle);
    GimbalManagerServer::DeviceAttitudeStatus device_attitude_status() const;

    GimbalManagerServer::GimbalManagerStatus get_gimbal_manager_status() const;

    GimbalManagerServer::SetAttitudeHandle
    subscribe_set_attitude(const GimbalManagerServer::SetAttitudeCallback callback);
    void unsubscribe_set_attitude(GimbalManagerServer::SetAttitudeHandle handle);

    GimbalManagerServer::SetRoiLocationHandle
    subscribe_set_roi_location(const GimbalManagerServer::SetRoiLocationCallback callback);
    void unsubscribe_set_roi_location(GimbalManagerServer::SetRoiLocationHandle handle);

private:
    struct {
        std::mutex mutex{};
        CallbackList<GimbalManagerServer::Information> callbacks{};
    } _device_discovered{};

    struct {
        std::mutex mutex{};
        GimbalManagerServer::GimbalManagerConfiguration configuration{};
        CallbackList<GimbalManagerServer::GimbalManagerConfiguration> callbacks{};
    } _manager_configuration{};

    struct {
        std::mutex mutex{};
        CallbackList<GimbalManagerServer::DeviceAttitudeStatus> callbacks{};
    } _device_attitude_status{};

    struct {
        std::mutex mutex{};
        GimbalManagerServer::Information information{};
        void* information_call_every_cookie = nullptr;
        void* discovery_call_every_cookie = nullptr;
        const float DISCOVERY_INTERVAL_S = 1.0f;
        const float INFORMATION_DELAY_INTERVAL_S = 0.1f;
        CallbackList<GimbalManagerServer::RoiLocation> callbacks{};
    } _device_information{};

    struct {
        GimbalManagerServer::GimbalManagerStatus status{};
        void* call_every_cookie = nullptr;
        const float STATUS_SEND_INTERVAL_S = 0.2f;
    } _manager_status{};

    struct {
        std::mutex mutex{};
        void* call_every_cookie = nullptr;
        CallbackList<GimbalManagerServer::SetAttitude> callbacks{};
    } _set_attitude{};

    struct {
        std::mutex mutex{};
        CallbackList<GimbalManagerServer::RoiLocation> callbacks{};
    } _roi_location{};

    bool is_gimbal_device_discovered(uint32_t id = 0)
    {
        return _manager_status.status.gimbal_device_id != 0 &&
               (id == 0 || _manager_status.status.gimbal_device_id == id);
    }
    void send_gimbal_manager_information();
    void broadcast_gimbal_manager_status();
    void discovery_gimbal_device();
    void process_gimbal_device_information(const mavlink_message_t& message);
    void process_gimbal_device_attitude_status(const mavlink_message_t& message);
    void process_gimbal_manager_set_attitude(const mavlink_message_t& message);
    void process_gimbal_manager_set_pitch_yaw(const mavlink_message_t& message);
    std::optional<mavlink_command_ack_t>
    process_gimbal_manager_configure_request(const MavlinkCommandReceiver::CommandLong& command);
    std::optional<mavlink_command_ack_t>
    process_request_message_request(const MavlinkCommandReceiver::CommandLong& command);
    void process_set_roi_location_request(
        uint32_t gimb_id, int32_t latitude, int32_t longitude, float altitude);

private:
    void* _call_every_cookie = nullptr;
};

} // namespace mavsdk
