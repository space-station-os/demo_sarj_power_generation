
#include<iostream>
#include<string>
#include <chrono>
#include <fstream>

#include "space_station_simulator.hpp"


using namespace std;

int main() {

    string line2 = "";

    line2 = "2 25544  51.6374 204.6652 0002397  79.3170   4.4179 15.49294231507097";

    line2 = "2 60182  97.9211 215.3545 0001598  99.1275 261.0117 14.79484184 44375";

    //string line1 = "1 25544U 98067A   22095.91869325  .00012930  00000 - 0  23502 - 3 0  9991";
    //string line2 = "2 25544  51.6452 334.5328 0004408 351.0413  99.6998 15.49890618333972";

    Eigen::Vector3d ss_position_eci;
    Eigen::Vector3d ss_velocity_eci;

    OrbitLib::convert_tle_to_eci(line2, ss_position_eci, ss_velocity_eci);

    SpaceStationSimulator::SpaceStationSimulator sss;
    //sss.activate_propagation_j2(OrbitLib::J2*100);
    sss.activate_propagation_air_drag(2.2, 1000, 10);
    sss.initialize(ss_position_eci, ss_velocity_eci, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 1);

    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();

    // UNIX時間（秒単位）に変換
    auto epoch_time = std::chrono::time_point_cast<std::chrono::seconds>(now);

    // UNIX時間を出力
    int64_t current_unix_time = epoch_time.time_since_epoch().count();
    std::cout << "Current UNIX Time: " << current_unix_time << std::endl;

    ofstream out_csv("result.csv");
    out_csv << "unix_time,eci_x,eci_y,eci_z,eci_vx,eci_vy,eci_vz,eci_ax,eci_ay,eci_az,latitude,longitude,altitude" << endl;

    double latitude_deg;
    double longitude_deg;
    double altitude_m;

    double t = 0.0;
    double dt = 10.0;

    cout << "Simulation Start." << endl;

    for (int32_t i = 0; i < 100000; ++i) {
        
        t += dt;
        sss.update(t);
        ss_position_eci = sss.get_ss_position_eci();
        ss_velocity_eci = sss.get_ss_velocity_eci();
        auto ss_acceleration_eci = sss.get_ss_acceleration_eci();

        //cout << "------------------ [" << i << "]--------------------" << endl;
        //cout << "ss_position_eci: " << ss_position_eci.transpose() << endl;
        //cout << "ss_velocity_eci: " << ss_velocity_eci.transpose() << endl;

        int64_t current_simu_unix_time = current_unix_time + int64_t(sss.get_time());
        OrbitLib::eci_to_geodetic(ss_position_eci, current_simu_unix_time, 0, latitude_deg, longitude_deg, altitude_m);

        //cout << "latitude_deg: " << latitude_deg << endl;
        //cout << "longitude_deg: " << longitude_deg << endl;
        //cout << "altitude[k]: " << altitude_m << endl;

        out_csv << current_simu_unix_time << ",";

        out_csv << ss_position_eci[0] << ",";
        out_csv << ss_position_eci[1] << ",";
        out_csv << ss_position_eci[2] << ",";
        out_csv << ss_velocity_eci[0] << ",";
        out_csv << ss_velocity_eci[1] << ",";
        out_csv << ss_velocity_eci[2] << ",";
        out_csv << ss_acceleration_eci[0] << ",";
        out_csv << ss_acceleration_eci[1] << ",";
        out_csv << ss_acceleration_eci[2] << ",";

        out_csv << latitude_deg << ",";
        out_csv << longitude_deg << ",";
        out_csv << altitude_m << endl;
    }

    cout << "Finished." << endl << endl;

	return 1;
}
