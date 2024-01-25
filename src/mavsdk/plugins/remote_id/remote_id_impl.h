#pragma once

#include "plugins/remote_id/remote_id.h"

#include "plugin_impl_base.h"

namespace mavsdk {

class RemoteIdImpl : public PluginImplBase {
public:
    explicit RemoteIdImpl(System& system);
    explicit RemoteIdImpl(std::shared_ptr<System> system);

    ~RemoteIdImpl() override;

    void init() override;
    void deinit() override;

    void enable() override;
    void disable() override;

public:
    RemoteId::Result set_basic_id(RemoteId::BasicId basic_id);
    RemoteId::Result set_operator_id(RemoteId::OperatorId operator_id);
    RemoteId::Result set_self_id(RemoteId::SelfId self_id);
    RemoteId::Result set_location(RemoteId::Location location);
    RemoteId::Result set_location_accuracy(RemoteId::LocationAccuracy location_accuracy);
    RemoteId::Result set_system(RemoteId::SystemId system);
    RemoteId::ArmStatusHandle subscribe_arm_status(const RemoteId::ArmStatusCallback& callback);

    void unsubscribe_arm_status(RemoteId::ArmStatusHandle handle);
    RemoteId::ArmStatus arm_status() const;
private:

    struct {
        mutable std::mutex mutex{};
        RemoteId::ArmStatus data{};
        CallbackList<RemoteId::ArmStatus> subscription_callbacks{};
    } _armStatus{};


private:
    void send_drone_location();
    void send_drone_basic_id();
    void send_operator_id();
    void send_drone_id_system();
    void send_drone_self_id();

private:
    void process_arm_status(const mavlink_message_t& message);

private:
    uint32_t get_seconds_after_2019();
    uint64_t get_utc_time_in_milliseconds();
    uint16_t get_seconds_in_full_hour_for_utc_time(uint64_t utc_time);

private:
    std::string _id_or_mac;
    RemoteId::BasicId _basic_id{};
    RemoteId::OperatorId _operator_id{};
    RemoteId::SelfId _self_id{};
    RemoteId::Location _location{};
    RemoteId::LocationAccuracy _location_accuracy{};
    RemoteId::SystemId _system{};

private:
    void* _send_drone_location_call_every_cookie{nullptr};
    void* _send_drone_basic_id_call_every_cookie{nullptr};
    void* _send_operator_id_call_every_cookie{nullptr};
    void* _send_self_id_call_every_cookie{nullptr};
    void* _send_drone_id_system_call_every_cookie{nullptr};
};

} // namespace mavsdk
