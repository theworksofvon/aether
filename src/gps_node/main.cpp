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

struct SatInfo {
    std::string prn, elevation, azimuth, snr;
};

struct GSV {
    std::string total_msgs, msg_num, sats_in_view;
    std::vector<SatInfo> satellites;
};

struct VTG {
    std::string course_true, course_mag, speed_knots, speed_kmh;
};

struct GLL {
    double lat, lon;
    std::string time, status;
};

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

enum class SentenceType { GGA, RMC, GSA, GSV, VTG, GLL, UNKNOWN };

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
    if (tag == "GSV") return SentenceType::GSV;
    if (tag == "VTG") return SentenceType::VTG;
    if (tag == "GLL") return SentenceType::GLL;
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

// --- Parsers ---

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

GSV parse_gsv(const std::vector<std::string>& f) {
    GSV g{field(f, 1), field(f, 2), field(f, 3), {}};
    for (size_t i = 4; i + 3 < f.size(); i += 4) {
        g.satellites.push_back({
            f[i], f[i + 1], f[i + 2], strip_checksum(f[i + 3])
        });
    }
    return g;
}

VTG parse_vtg(const std::vector<std::string>& f) {
    return {field(f, 1), field(f, 3), field(f, 5), field(f, 7)};
}

GLL parse_gll(const std::vector<std::string>& f) {
    return {
        nmea_to_decimal(field(f, 1), field(f, 2)),
        nmea_to_decimal(field(f, 3), field(f, 4)),
        field(f, 5), field(f, 6)
    };
}

// --- Printers ---

void print(const GGA& g) {
    std::cout << std::fixed << std::setprecision(6)
              << "[GGA] Time: " << g.time
              << "  Lat: " << g.lat
              << "  Lon: " << g.lon
              << "  Fix: " << g.fix
              << "  Sats: " << g.sats
              << "  Alt: " << g.alt << " " << g.alt_unit << "\n";
}

void print(const RMC& r) {
    std::cout << std::fixed << std::setprecision(6)
              << "[RMC] Time: " << r.time
              << "  Status: " << r.status
              << "  Lat: " << r.lat
              << "  Lon: " << r.lon
              << "  Speed: " << r.speed << " kn"
              << "  Course: " << r.course
              << "  Date: " << r.date << "\n";
}

void print(const GSA& g) {
    std::string fix;
    if (g.fix_mode == "1") fix = "No fix";
    else if (g.fix_mode == "2") fix = "2D";
    else if (g.fix_mode == "3") fix = "3D";
    else fix = g.fix_mode;
    std::cout << "[GSA] Fix: " << fix
              << "  PDOP: " << g.pdop
              << "  HDOP: " << g.hdop
              << "  VDOP: " << g.vdop << "\n";
}

void print(const GSV& g) {
    std::cout << "[GSV] Msg " << g.msg_num << "/" << g.total_msgs
              << "  Sats in view: " << g.sats_in_view << "\n";
    for (const auto& s : g.satellites) {
        std::cout << "       PRN: " << s.prn
                  << "  Elev: " << s.elevation
                  << "  Az: " << s.azimuth
                  << "  SNR: " << s.snr << "\n";
    }
}

void print(const VTG& v) {
    std::cout << "[VTG] Course(true): " << v.course_true
              << "  Course(mag): " << v.course_mag
              << "  Speed: " << v.speed_knots << " kn"
              << "  Speed: " << v.speed_kmh << " km/h\n";
}

void print(const GLL& g) {
    std::cout << std::fixed << std::setprecision(6)
              << "[GLL] Lat: " << g.lat
              << "  Lon: " << g.lon
              << "  Time: " << g.time
              << "  Status: " << g.status << "\n";
}

// --- Serial ---

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


int main() {
    const char* env_port = std::getenv("GPS_PORT");
    std::string port = env_port ? env_port : "/dev/ttyUSB0";

    std::cout << "Opening " << port << " at 9600 baud...\n";
    int fd = open_serial(port);
    if (fd < 0) return 1;

    std::cout << "Reading NMEA sentences...\n";
    while (true) {
        std::string line = read_line(fd);
        if (line.empty() || line[0] != '$') continue;

        auto fields = split(line, ',');
        if (fields.empty()) continue;

        switch (get_sentence_type(fields[0])) {
            case SentenceType::GGA: print(parse_gga(fields)); break;
            case SentenceType::RMC: print(parse_rmc(fields)); break;
            case SentenceType::GSA: print(parse_gsa(fields)); break;
            case SentenceType::GSV: print(parse_gsv(fields)); break;
            case SentenceType::VTG: print(parse_vtg(fields)); break;
            case SentenceType::GLL: print(parse_gll(fields)); break;
            case SentenceType::UNKNOWN: break;
        }
    }

    close(fd);
    return 0;
}
