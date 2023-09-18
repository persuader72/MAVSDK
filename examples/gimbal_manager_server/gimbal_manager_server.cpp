#include <iostream>
#include <thread>
#include <future>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/gimbal_manager_server/gimbal_manager_server.h>

int main(int argc, char** argv)
{
    mavsdk::Mavsdk mavsdk_client; // VS gimbal device
    mavsdk::Mavsdk::Configuration configuration(mavsdk::Mavsdk::Configuration::UsageType::Custom);
    mavsdk_client.set_configuration(configuration);

    auto result = mavsdk_client.add_any_connection("udp://:14550");
    if (result != mavsdk::ConnectionResult::Success) {
        std::cerr << "Could not establish connection: " << result << std::endl;
        return 1;
    }

    /*std::cerr << "Wait for first autopilot\n";
    auto system = mavsdk_client.first_autopilot(3.0);
    if (!system) {
        std::cerr << "Timed out waiting for system\n";
        return 1;
    }*/

    result = mavsdk_client.add_any_connection("udp://:13030");
    if (result != mavsdk::ConnectionResult::Success) {
        std::cerr << "Could not establish connection: " << result << std::endl;
        return 1;
    }

    /*mavsdk::Mavsdk mavsdk_server; // VS Ground control station
    mavsdk_server.set_configuration(configuration);
    result = mavsdk_server.add_any_connection("udp://:14550");
    if (result != mavsdk::ConnectionResult::Success) {
        std::cerr << "Could not establish connection: " << result << std::endl;
        return 1;
    }*/

    std::cout << "Created gimbal server/client connections" << std::endl;

    auto gimbal_manager = mavsdk::GimbalManagerServer{mavsdk_client.server_component_by_type(
        mavsdk::Mavsdk::ServerComponentType::CompanionComputer)};

    std::promise<uint32_t> prom;
    std::future<uint32_t> fut = prom.get_future();

    gimbal_manager.subscribe_device_discovered(
        [&prom](mavsdk::GimbalManagerServer::Information information) {
            prom.set_value(information.gimbal_device_id);
        });

    if (gimbal_manager.device_discovered().gimbal_device_id) {
        prom.set_value(gimbal_manager.device_discovered().gimbal_device_id);
    }

    if (fut.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
        std::cout << "FAIL\n";
        std::cout << "-> no device found\n";
        return 1;
    }

    std::cout << "Discovered new gimbal device with ID=" << fut.get() << std::endl;

    gimbal_manager.subscribe_device_attitude_status(
        [](mavsdk::GimbalManagerServer::DeviceAttitudeStatus attitude_status) {
            // std::cout << "Received device attitude status " << attitude_status.time_boot_ms <<
            // "\n";
        });

    gimbal_manager.subscribe_set_roi_location(
        [](mavsdk::GimbalManagerServer::RoiLocation roi_location) {
            std::cout << "Received roi location request lat:" << roi_location.latitude_deg
                      << " lon:" << roi_location.longitude_deg << "\n";
        });

    // works as a server and never quit
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}

/*auto ret = gimbal_manager.set_information(
        {.cap_flags = 0,
         .gimbal_device_id = MAV_COMP_ID_GIMBAL,
         .roll_min = 0,
         .roll_max = 0,
         .pitch_min = -110,
         .pitch_max = 30,
         .yaw_min = -170,
         .yaw_max = 170});

    if (ret != mavsdk::GimbalManagerServer::Result::Success) {
        std::cerr << "Failed to set gimbal info, exiting" << std::endl;
        return 2;
    }*/

/*gimbal_manager.subscribe_set_attitude([](mavsdk::GimbalManagerServer::SetAttitude attitude) {
    float q[4] = {
        attitude.attitude_quaternion.w,
        attitude.attitude_quaternion.x,
        attitude.attitude_quaternion.y,
        attitude.attitude_quaternion.z};
    float roll, pitch, yaw;
    mavlink_quaternion_to_euler(q, &roll, &pitch, &yaw);
    std::cout << "gimbal_manager.subscribe_set_attitude ptich " << pitch << " yaw " << yaw
              << std::endl;
});*/