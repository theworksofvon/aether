#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <iomanip>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>


// RMC → position, speed, heading
// GGA → position, altitude, satellite count  
// GSA → fix quality, HDOP, PDOP (accuracy metrics)

struct GGA {
    std::string time;
    double lat, lon;
    std::string fix, sats, alt, alt_unit;
};

struct RMC {
    std::string time, status;
    double lat, lon;
    std::string speed, course, date;
};

struct GSA {
    std::string fix_mode, pdop, hdop, vdop;
};

enum class SentenceType { GGA, RMC, GSA, UNKNOWN };

double nmea_to_decimal(const std::string& coord, const std::string& dir) {
    if (coord.empty()) return 0.0;
    auto dot = coord.find('.');
    if (dot == std::string::npos || dot < 2) return 0.0;
    double degrees = std::stod(coord.substr(0, dot - 2));
    double minutes = std::stod(coord.substr(dot - 2));
    double decimal = degrees + minutes / 60.0;
    if (dir == "S" || dir == "W") decimal = -decimal;
    return decimal;
}

std::string strip_checksum(const std::string& s) {
    auto star = s.find('*');
    return (star != std::string::npos) ? s.substr(0, star) : s;
}

const std::string& field(const std::vector<std::string>& f, size_t i) {
    static const std::string empty;
    return i < f.size() ? f[i] : empty;
}

SentenceType get_sentence_type(const std::string& id) {
    std::string s = strip_checksum(id);
    if (!s.empty() && s[0] == '$') s = s.substr(1);
    if (s.size() < 3) return SentenceType::UNKNOWN;
    std::string tag = s.substr(s.size() - 3);

    if (tag == "GGA") return SentenceType::GGA;
    if (tag == "RMC") return SentenceType::RMC;
    if (tag == "GSA") return SentenceType::GSA;
    return SentenceType::UNKNOWN;
}

std::vector<std::string> split(const std::string& line, char delim) {
    std::vector<std::string> fields;
    std::stringstream ss(line);
    std::string f;
    while (std::getline(ss, f, delim))
        fields.push_back(f);
    return fields;
}

GGA parse_gga(const std::vector<std::string>& f) {
    return {
        field(f, 1),
        nmea_to_decimal(field(f, 2), field(f, 3)),
        nmea_to_decimal(field(f, 4), field(f, 5)),
        field(f, 6), field(f, 7), field(f, 9), field(f, 10)
    };
}

RMC parse_rmc(const std::vector<std::string>& f) {
    return {
        field(f, 1), field(f, 2),
        nmea_to_decimal(field(f, 3), field(f, 4)),
        nmea_to_decimal(field(f, 5), field(f, 6)),
        field(f, 7), field(f, 8), field(f, 9)
    };
}

GSA parse_gsa(const std::vector<std::string>& f) {
    return {
        field(f, 2),
        field(f, 15), field(f, 16),
        strip_checksum(field(f, 17))
    };
}


int open_serial(const std::string& port) {
    int fd = open(port.c_str(), O_RDONLY | O_NOCTTY);
    if (fd < 0) {
        std::cerr << "Failed to open " << port << ": " << strerror(errno) << "\n";
        return -1;
    }

    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "tcgetattr failed: " << strerror(errno) << "\n";
        close(fd);
        return -1;
    }

    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_cflag |= CLOCAL | CREAD;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "tcsetattr failed: " << strerror(errno) << "\n";
        close(fd);
        return -1;
    }

    return fd;
}

std::string read_line(int fd) {
    std::string line;
    char c;
    while (true) {
        ssize_t n = read(fd, &c, 1);
        if (n < 0) {
            std::cerr << "Read error: " << strerror(errno) << "\n";
            return "";
        }
        if (n == 0) continue;
        if (c == '\n') return line;
        if (c != '\r') line += c;
    }
}

class GpsNode : public rclcpp::Node {
    public:
        GpsNode() : Node("gps_node") {
            fix_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
            status_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatStatus>("/gps/status", 10);


            const char* env_port = std::getenv("GPS_PORT");
            std::string port = env_port ? env_port : "/dev/ttyUSB0";

            fd_ = open_serial(port);

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&GpsNode::loop, this)
            );

            RCLCPP_INFO(this->get_logger(), "GPS node started on %s", port.c_str());
        }

    private:
        void loop() {
            if (fd_ < 0) return;
            std::string line = read_line(fd_);
            if (line.empty() || line[0] != '$') return;

            auto fields = split(line, ',');
            if (fields.empty()) return;

            switch (get_sentence_type(fields[0])) {
                case SentenceType::GGA: publishGGA(parse_gga(fields)); break;
                case SentenceType::RMC: publishRMC(parse_rmc(fields)); break;
                case SentenceType::GSA: publishGSA(parse_gsa(fields)); break;
                default: break;
            }
        }

        void publishRMC(const RMC& r) {
            if (r.status != "A") return;
            auto msg = sensor_msgs::msg::NavSatFix();
            msg.latitude = r.lat;
            msg.longitude = r.lon;
            msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
            fix_publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "[RMC] Lat: %.6f  Lon: %.6f  Speed: %s kn", r.lat, r.lon, r.speed.c_str());
        }

        void publishGGA(const GGA& g) {
            if (g.fix == "0") return;
            auto msg = sensor_msgs::msg::NavSatFix();

            msg.latitude = g.lat;
            msg.longitude = g.lon;
            msg.altitude = g.alt.empty() ? 0.0 : std::stod(g.alt);
            msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
            fix_publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "[GGA] Lat: %.6f  Lon: %.6f  Alt: %s m  Sats: %s", g.lat, g.lon, g.alt.c_str(), g.sats.c_str());
        }

        void publishGSA(const GSA& g) {
            auto msg = sensor_msgs::msg::NavSatStatus();
            msg.status = (g.fix_mode == "1") ? 
                sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX :
                sensor_msgs::msg::NavSatStatus::STATUS_FIX;
            status_publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "[GSA] Fix: %s  HDOP: %s  PDOP: %s", g.fix_mode.c_str(), g.hdop.c_str(), g.pdop.c_str());
        }

        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatStatus>::SharedPtr status_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        int fd_ = -1;
};


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsNode>());
    rclcpp::shutdown();
    return 0;
};
