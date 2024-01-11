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
    RemoteId::Result set_system(RemoteId::SystemId system);

private:
    void send_drone_location();
    void send_drone_basic_id();
    void send_operator_id();
    void send_drone_id_system();
    void send_drone_self_id();

private:

    uint32_t get_seconds_after_2019();

private:
    std::string _id_or_mac;
    RemoteId::BasicId _basic_id{};
    RemoteId::OperatorId _operator_id{};
    RemoteId::SelfId _self_id{};
    RemoteId::Location _location{};
    RemoteId::SystemId _system{};

private:
    void* _send_drone_location_call_every_cookie{nullptr};
    void* _send_drone_basic_id_call_every_cookie{nullptr};
    void* _send_operator_id_call_every_cookie{nullptr};
    void* _send_self_id_call_every_cookie{nullptr};
    void* _send_drone_id_system_call_every_cookie{nullptr};
};

} // namespace mavsdk
