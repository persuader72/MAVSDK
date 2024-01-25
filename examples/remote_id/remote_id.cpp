//
// Example how to listen to a transponder using MAVSDK.
//

#include <chrono>
#include <cstdint>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/remote_id/remote_id.h>
#include <mavsdk/mavlink/common/mavlink.h>
#include <iostream>
#include <future>
#include <memory>
#include <thread>

using namespace mavsdk;

void usage(const std::string& bin_name)
{
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect to the simulator use URL: udp://:14540\n";
}

int main(int argc, char** argv)
{
    if (argc != 2) {
        usage(argv[0]);
        return 1;
    }

    Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::CompanionComputer}};
    ConnectionResult connection_result = mavsdk.add_any_connection(argv[1]);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    while (mavsdk.systems().size() == 0) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::shared_ptr<System> system = mavsdk.systems()[0];
    system->subscribe_component_discovered_id([](System::ComponentType type, uint8_t id) {
        std::cerr << "Component found: type " << (int)type << " id " << (int)id << '\n';
    });

    /*auto system = mavsdk.first_autopilot(3.0);*/
    if (!system) {
        std::cerr << "Timed out waiting for system\n";
        return 1;
    }

    // Instantiate plugin.
    auto remote_id = RemoteId{system};

    remote_id.set_basic_id(RemoteId::BasicId{
        .id_type = RemoteId::BasicId::IdType::SerialNumber,
        .ua_type = RemoteId::BasicId::UasType::HybridLift,
        .uas_id = std::string({'S',  'R',  'R',  'B',  'T',  '#',  '0',  '1',  0x00, 0x00,
                               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00})});

    remote_id.set_self_id(RemoteId::SelfId{
        .description_type = RemoteId::SelfId::DescType::Text,
        .description = std::string({'T',  'E',  'S',  'T',  '_',  'S',  'E',  'L',  'F',  0x00,
                                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00})});

    remote_id.set_operator_id(RemoteId::OperatorId{
        .operator_id_type = RemoteId::OperatorId::OperatorIdType::Caa,
        .operator_id = std::string({'T',  'E',  'S',  'T',  '_',  'O',  'P',  'E',  'R',  0x00,
                                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00})});

    remote_id.set_location_accuracy(RemoteId::LocationAccuracy{
        .horizontal_accuracy = RemoteId::LocationAccuracy::HorAcc::Meter1});

    float latitude_op = 43.1213;
    float longitude_op = 12.3134;

    {
        const auto now = std::chrono::floor<std::chrono::seconds>(std::chrono::system_clock::now());
        std::tm tm = {
            .tm_sec = 0, .tm_min = 0, .tm_hour = 0, .tm_mday = 1, .tm_mon = 0, 2019 - 1900};
        const auto ref = std::chrono::system_clock::from_time_t(std::mktime(&tm));
        const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now - ref).count();

        remote_id.set_system(RemoteId::SystemId{
            .operator_location_type = RemoteId::SystemId::OperatorLocationType::Fixed,
            .classification_type = RemoteId::SystemId::ClassificationType::Eu,
            .operator_latitude_deg = latitude_op,
            .operator_longitude_deg = longitude_op,
            .area_count = 1,
            .area_radius_m = 0,
            .area_ceiling_m = -1000,
            .area_floor_m = -1000,
            .category_eu = RemoteId::SystemId::CategoryEu::Open,
            .class_eu = RemoteId::SystemId::ClassEu::Class2,
            .operator_altitude_geo_m = -1000,
            .timestamp_s = (uint32_t)seconds});
    }

    int steps = 0;
    // Search for aircraft transponders
    while (steps < 7200) {
        float latitude = latitude_op + 0.005 * sin(M_PI / 60 * steps);
        float longitude = longitude_op + 0.005 * cos(M_PI / 60 * steps);
        float height = 150 + 75 * sin(M_PI / 30 * steps);

        const auto now = std::chrono::floor<std::chrono::seconds>(std::chrono::system_clock::now());
        const auto timeodhour = now - std::chrono::floor<std::chrono::hours>(now);
        const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(timeodhour).count();

        remote_id.set_location(RemoteId::Location{
            .status = RemoteId::Location::Status::Airborne,
            .latitude_deg = latitude,
            .longitude_deg = longitude,
            .height_reference = RemoteId::Location::HeightRef::OverTakeoff,
            .height_m = height,
            .timestamp_s = (float)seconds});

        std::this_thread::sleep_for(std::chrono::seconds(1));
        steps++;
    }
    std::cout << "Finished...\n";

    return 0;
}
