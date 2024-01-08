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
        .id_type = MAV_ODID_ID_TYPE_SERIAL_NUMBER,
        .ua_type = MAV_ODID_UA_TYPE_HYBRID_LIFT,
        .uas_id = std::string({'S',  'R',  'R',  'B',  'T',  '#',  '0',  '1',  0x00, 0x00,
                               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00})});

    int steps = 0;
    // Search for aircraft transponders
    while (steps < 7200) {
        float latitude = 43.1213 + 0.005 * sin(M_PI / 60 * steps);
        float longitude = 12.3134 + 0.005 * cos(M_PI / 60 * steps);
        float height = 150 + 75 * sin(M_PI / 30 * steps);

        const auto now = std::chrono::floor<std::chrono::seconds>(std::chrono::system_clock::now());
        const auto timeodhour = now - std::chrono::floor<std::chrono::hours>(now);
        const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(timeodhour).count();

        remote_id.set_location(RemoteId::Location{
            .status = MAV_ODID_STATUS_AIRBORNE,
            .latitude = (int32_t)(latitude * 1e7),
            .longitude = (int32_t)(longitude * 1e7),
            .height_reference = MAV_ODID_HEIGHT_REF_OVER_TAKEOFF,
            .height = height,
            .timestamp = (float)seconds,
            .timestamp_accuracy = MAV_ODID_TIME_ACC_1_0_SECOND});

        std::this_thread::sleep_for(std::chrono::seconds(1));
        steps++;
    }
    std::cout << "Finished...\n";

    return 0;
}
