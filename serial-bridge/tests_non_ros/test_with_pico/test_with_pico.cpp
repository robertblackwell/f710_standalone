
#include <cstdio>
#include <cstring>
#include <vector>
#include <string>
#include <algorithm>
#include <format>
#include <thread>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <libserial/SerialPort.h>
#include <unittest.h>
#include "jsoncons/json.hpp"
#include "jsoncons_ext/jsonpath/jsonpath.hpp"
#include <bridge_lib/serial_settings.h>
#include <fbcontrol/custom.h>


struct SideStatus {
    double          tick_count;
    uint64_t        sample_elapsedtime_usecs;
    double          wheel_speed_mm_secs;
    double          motor_rpm_estimate;
};

struct TwoSidesStatus {
    SideStatus left;
    SideStatus right;
};

struct PwmStatusRecord {
    double left_pwm;
    double right_pwm;
    std::vector<TwoSidesStatus> pwm_status_list;
};

struct TargetRecord {
    double target_rpm;
    std::vector<PwmStatusRecord> target_status_list;
};

std::vector<TargetRecord> run_record;

void print_run_record()
{
    for(auto r: run_record) {
        printf("{\n");
        printf("    \"target\":%f,\n", r.target_rpm);
        for(auto s: r.target_status_list) {
            printf("    \"left_pwm\" :%5.2f,\n", s.left_pwm);
            printf("    \"right_pwm\":%5.2f,\n", s.right_pwm);
            printf("    [\n");
            for(auto ts: s.pwm_status_list) {
                printf("\"left\": {");
                printf("\"ticks\": %6.2f, ", ts.left.tick_count);
                printf("\"elapsedus\": %ld, ", ts.left.sample_elapsedtime_usecs);
                printf("\"rpm\": %8.3f, ", ts.left.motor_rpm_estimate);
                printf("\"wspeed\": %6.2f,", ts.left.wheel_speed_mm_secs);
                printf("},\n");
                printf("\"right\":{");
                printf("\"ticks\": %6.2f, ", ts.right.tick_count);
                printf("\"elapsedus\": %ld, ", ts.right.sample_elapsedtime_usecs);
                printf("\"rpm\": %8.3f, ", ts.right.motor_rpm_estimate);
                printf("\"wspeed\": %6.2f,", ts.right.wheel_speed_mm_secs);
                printf("}\n},");
            }
        }
    }
}

void send_pwm(LibSerial::SerialPort& serial_port, double left, double right)
{
    printf("send_pwm %f %f\n", left, right);
    serial_port.Write(std::format("pwm {} {}\n", left, right));
}
void read_encoder_status(LibSerial::SerialPort& serial_port, TwoSidesStatus& status)
{
    std::string line{};
    serial_port.Write(std::format("e\n"));
    while(1) {
        serial_port.ReadLine(line);
        if(line.substr(0,2) == "1J") {
            printf("%s", line.c_str());
            std::string json_string = line.substr(2, std::string::npos);
            jsoncons::json j = jsoncons::json::parse(json_string);
            status.left.tick_count = j[0]["ss"].as<int64_t>();
            status.left.sample_elapsedtime_usecs = j[0]["ts"].as<int64_t>();
            status.left.motor_rpm_estimate = j[0]["mr"].as<float>();
            status.left.wheel_speed_mm_secs = j[0]["ws"].as<int8_t>();

            status.right.tick_count = j[1]["ss"].as<int64_t>();
            status.right.sample_elapsedtime_usecs = j[1]["ts"].as<int64_t>();
            status.right.motor_rpm_estimate = j[1]["mr"].as<float>();
            status.right.wheel_speed_mm_secs = j[1]["ws"].as<int8_t>();
            return;
        } else if(line.substr(0, 5) == "Heart") {
            ;
        } else {
//            printf("%s", line.c_str());
        }
    }
}
FILE* json_file;
void print_pwm_status_record(PwmStatusRecord pwm_status_record)
{
    for(auto ps: pwm_status_record.pwm_status_list) {
        fprintf(json_file, "    [{\"pwm\": %5.2f, ", pwm_status_record.left_pwm);

        fprintf(json_file, "\"ticks\": %7.2f, ", ps.left.tick_count);
        fprintf(json_file, "\"elapsedus\": %6lu, ", ps.left.sample_elapsedtime_usecs);
        fprintf(json_file, "\"rpm\": %7.2f, ", ps.left.motor_rpm_estimate);
        fprintf(json_file, "\"wspeed\": %5.2f,", ps.left.wheel_speed_mm_secs);
        fprintf(json_file, "},");

        fprintf(json_file, "{\"pwm\": %5.2f, ", pwm_status_record.right_pwm);

        fprintf(json_file, "\"ticks\": %7.2f, ", ps.right.tick_count);
        fprintf(json_file, "\"elapsedus\": %6lu, ", ps.right.sample_elapsedtime_usecs);
        fprintf(json_file, "\"rpm\": %7.2f, ", ps.right.motor_rpm_estimate);
        fprintf(json_file, "\"wspeed\": %5.2f,", ps.right.wheel_speed_mm_secs);
        fprintf(json_file, "}],\n");
    }
}
void print_target_record(TargetRecord target_record)
{
    fprintf(json_file, "  {\"target\":%f, \n", target_record.target_rpm);
    fprintf(json_file, "[\n");
    for(auto p: target_record.target_status_list) {
        print_pwm_status_record(p);
    }
    fprintf(json_file, "]}\n");
}
void print_run_record(std::vector<TargetRecord> run_record)
{
    fprintf(json_file, "[\n");
    for(auto rr: run_record) {
        print_target_record(rr);
        fprintf(json_file, ",\n");
    }
    fprintf(json_file, "]\n");
}

double calculate_rpm(double ticks, uint64_t elapsed_time_us)
{
    auto t = ticks; //std::round(((ticks+2.4)/5.0))*5.0;
    auto e = (double)elapsed_time_us;// std::round((elapsed_time_us)/10.0) * 10.0;
    auto rvs  = e / 48.0 / t;
    auto rpm = (e * 60.0 / 48.0) / t;
    auto x = (t * 60.0) / ((e / 1000000.0) * 48.0);
    auto y = round((2.0*x)/10.0)*5.0;
    return x;
}

int main() {
#ifdef FBCONTROL_JSON_OUTPUT_PATH
    printf("json output path is %s\n", FBCONTROL_JSON_OUTPUT_PATH);
    json_file = fopen(FBCONTROL_JSON_OUTPUT_PATH, "w");
#endif
    auto devices = list_serial_devices();
    LibSerial::SerialPort serial_port{
            devices[0],
            LibSerial::BaudRate::BAUD_1152000,
            LibSerial::CharacterSize::CHAR_SIZE_8,
            LibSerial::FlowControl::FLOW_CONTROL_NONE,
            LibSerial::Parity::PARITY_NONE,
            LibSerial::StopBits::STOP_BITS_1};
//    while (1) {
//        EncoderStatusRecord status;
//        read_encoder_status(serial_port, status);
//        printf("Stop here\n");
//    }
    serial_port.FlushIOBuffers();
    CustomController left_controller(CustomController::Algorithm::fixed_step, MotorSide::left);
    CustomController right_controller(CustomController::Algorithm::fixed_step, MotorSide::right);

    for(int i = 2500; i <= 7500; i+= 500) {
        TargetRecord  target_record{};
        PwmStatusRecord pwm_status_record{};

        double saved_left_pwm;
        double saved_right_pwm;
        double target = (double)(i + 1 + (rand() % 490));

        target_record.target_rpm = target;

        double previous_left_rpm = 0.0;
        double previous_right_rpm = 0.0;
        auto left_first_pwm = left_controller.set_target(ValueWithDirection{target});
        auto right_first_pwm = right_controller.set_target(ValueWithDirection{target});

        saved_left_pwm = left_first_pwm.signed_value();
        saved_right_pwm= right_first_pwm.signed_value();

        send_pwm(serial_port, left_first_pwm.signed_value(), right_first_pwm.signed_value());
        for(int count = 0; count < 15; count++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            TwoSidesStatus status{};
            read_encoder_status(serial_port, status);

            double left_rpm = status.left.motor_rpm_estimate;
            double right_rpm = status.right.motor_rpm_estimate;

#if 0
            left_rpm = calculate_rpm(status.left.tick_count, status.left.sample_elapsedtime_usecs);
            right_rpm = calculate_rpm(status.right.tick_count, status.right.sample_elapsedtime_usecs);
            status.left.motor_rpm_estimate = left_rpm;
            status.right.motor_rpm_estimate = right_rpm;
#endif
            auto new_left_pwm = left_controller.update(left_rpm);
            auto new_right_pwm = right_controller.update(right_rpm);
            if(new_left_pwm.signed_value() == 0.0) {
                printf("Hello zero");
            }
            pwm_status_record.pwm_status_list.push_back(status);
            pwm_status_record.left_pwm = new_left_pwm.signed_value();
            pwm_status_record.right_pwm = new_right_pwm.signed_value();
            target_record.target_status_list.push_back(pwm_status_record);

            bool left_same = (fabs(fabs(saved_left_pwm) - fabs(new_left_pwm.signed_value())) < 0.1);
            bool right_same = (fabs(fabs(saved_right_pwm) - fabs(new_right_pwm.signed_value())) < 0.1);
            saved_left_pwm = new_left_pwm.signed_value();
            saved_right_pwm = new_right_pwm.signed_value();
            printf("controller output: left_pwm: %f  right_pwm: %f\n", new_left_pwm.signed_value(), new_right_pwm.signed_value());
            if(!(left_same && right_same)) {
                send_pwm(serial_port, new_left_pwm.signed_value(), new_right_pwm.signed_value());
            } else {
                printf("did not send pwm  %f %f \n", new_left_pwm.signed_value(), new_right_pwm.signed_value());
            }
        }
        run_record.push_back(target_record);
    }
    print_run_record(run_record);
    fclose(json_file);
    return 0;
}