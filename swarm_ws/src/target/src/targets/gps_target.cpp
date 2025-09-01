#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "target/target_node.hpp"

/* Constants and Global Definitions */
#define MAX_BUFFER_SIZE 1024
#define QUEUE_LENGTH 10
const char *target_topic = "/target/global_position";
const char *serial_port = "/dev/ttyUSB0";

// Assuming UERE for BU-353S4 is 5 meters
const double UERE = 5.0;

class GPSTarget : public Target
{
public:
    GPSTarget()
        : Target(), bufferIndex(0)
    {
        /* Quality of Service set-up for PX4 topics. */
        // rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        // auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        position_publisher_ = this->create_publisher<px4_msgs::msg::VehicleGlobalPosition>(target_topic, QUEUE_LENGTH);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                         std::bind(&GPSTarget::publish_position, this));

        fd = open(serial_port, O_RDWR | O_NOCTTY | O_SYNC); // Change the ttyUSB0 according to connected serial port

        if (fd < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error opening serial port: %s", strerror(errno));
            rclcpp::shutdown();
            return;
        }

        // Setting up serial port
        struct termios tty;
        memset(&tty, 0, sizeof tty);
        if (tcgetattr(fd, &tty) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr: %s", strerror(errno));
            close(fd);
            rclcpp::shutdown();
            return;
        }

        cfsetospeed(&tty, B38400);
        cfsetispeed(&tty, B38400);
        

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN] = 1;
        tty.c_cc[VTIME] = 5;

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

  

        if (tcsetattr(fd, TCSANOW, &tty) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error from tcsetattr: %s", strerror(errno));
            close(fd);
            rclcpp::shutdown();
            return;
        }
        // Enable $GPGGA
        const char* enable_gpgga = "$PSRF103,00,01,10,01*24\r\n";

        // Enable $GPGSA
        const char* enable_gpgsa = "$PSRF103,02,01,01,01*26\r\n";

        // Disable $GPRMC
        const char* disable_gprmc = "$PSRF103,04,00,00,01*20\r\n";

        // Disable $GPVTG
        const char* disable_gpvtg = "$PSRF103,05,00,00,01*21\r\n";

        // Disable $GPGSV
        const char* disable_gpgsv = "$PSRF103,03,00,00,01*27\r\n";

        // Send each command via the serial port
        const char* commands[] = { enable_gpgga, enable_gpgsa, disable_gprmc, disable_gpvtg, disable_gpgsv};
        
        for (const char* command : commands)
        {
            ssize_t written_bytes = write(fd, command, strlen(command));
            if (written_bytes == -1)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to write GPS command: %s", strerror(errno));
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Sent command: %s", command);
            }
        }
    }

private:
    void convert_GPS_data(const char *line, double *prevLat, double *prevLon, double *prevAlt)
    {
        // NMEA sentence $GPGGA
        char *token;
        char *data[15];
        int index = 0;

        // Tokenize the line by commas
        char lineCopy[256];
        strcpy(lineCopy, line);
        token = strtok(lineCopy, ",");
        while (token != NULL && index < 15)
        {
            data[index++] = token;
            token = strtok(NULL, ",");
        }

        if (index >= 10)
        { // Ensure there are enough tokens
            double latitude = atof(data[2]);
            double longitude = atof(data[4]);
            double altitude = atof(data[9]);

            // Convert latitude and longitude to decimal degrees
            int latDegrees = (int)(latitude / 100);
            double latMinutes = latitude - (latDegrees * 100);
            double latDecimal = latDegrees + (latMinutes / 60);

            if (strcmp(data[3], "S") == 0)
            {
                latDecimal = -latDecimal;
            }

            int lonDegrees = (int)(longitude / 100);
            double lonMinutes = longitude - (lonDegrees * 100);
            double lonDecimal = lonDegrees + (lonMinutes / 60);

            if (strcmp(data[5], "W") == 0)
            {
                lonDecimal = -lonDecimal;
            }

            // DISTANCE TRACKER
            // if (*prevLat != 0.0 && *prevLon != 0.0)
            // {
            //     double distance = calculateDistance(*prevLat, *prevLon, latDecimal, lonDecimal);
            //     *totalDistance += distance;
            //     // printf("Distance from previous point: %.2f km\n", distance); // Print distance between points
            // }

            *prevLat = latDecimal;
            *prevLon = lonDecimal;
            *prevAlt = altitude;
        }
    }
    
    
    
    void convert_HDOP_data(const char *line, double *hdop, double *hdop_in_meters)
    {
        // NMEA sentence $GPGSA
        char *token;
        char *data[18];
        int index = 0;


        char lineCopy[256];
        strcpy(lineCopy, line);
        token = strtok(lineCopy, ",");
        while (token != NULL && index < 18)
        {
            data[index++] = token;
            token = strtok(NULL, ",");
        }

        if (index >= 15)
        { 
            *hdop = atof(data[16]); 
            *hdop_in_meters = (*hdop) * UERE; // Convert HDOP to meters
        }
    }

    void log_GPGGA_data(double latitude, double longitude, double altitude)
    {
 
        std::ofstream gps_log("gps_log.txt", std::ios::app);
        
        if (gps_log.is_open())
        {
            // Write the GPGGA sentence to the log file
            gps_log << latitude << ", " << longitude << ", " << altitude << std::endl;
        
            gps_log.close();
        }
        else
        {
            std::cerr << "Unable to open log file" << std::endl;
        }
    }

    void publish_position(void)
    {
        char c;
        std::cout << "Attempting to read from the serial port..." << std::endl;
        double prevLat = 0.0, prevLon = 0.0, prevAlt = 0.0;

        // while (true)
        // {
            ssize_t n = read(fd, &c, 1);
            if (n == 1)
            {
                if (c == '\n')
                {
                    buffer[bufferIndex] = '\0';
                    // Check if the line starts with $GPGGA (GPS Data)
                    if (strncmp(buffer, "$GPGGA", 6) == 0)
                    {
                        convert_GPS_data(buffer, &prevLat, &prevLon, &prevAlt);
                        
                        // USE TO LOG GPS DATA
                        log_GPGGA_data(prevLon, prevLat, prevAlt);

                        auto msg = px4_msgs::msg::VehicleGlobalPosition();
                        msg.lon = prevLon;
                        msg.lat = prevLat;
                        msg.alt = prevAlt;
                        position_publisher_->publish(msg);
                        

                        // RCLCPP_INFO(this->get_logger(), "LON: '%f' | LAT: '%f' | ALT: '%f' \n", msg.lon, msg.lat, msg.alt);
                    }
                    bufferIndex = 0;
                    
                    // Check if the line starts with $GPGSA (DOP values)
                    if (strncmp(buffer, "$GPGSA", 6) == 0)
                    {
                        convert_HDOP_data(buffer, &hdop, &hdop_in_meters);
                        RCLCPP_INFO(this->get_logger(), "HDOP: '%f' | Position Error: '%f' meters\n", hdop, hdop_in_meters);
                    }


                    
                }
                else if (bufferIndex < MAX_BUFFER_SIZE - 1)
                {
                    buffer[bufferIndex++] = c;
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Error reading from serial port: %s", strerror(errno));
            }
        // }
    }

    int fd;
    char buffer[MAX_BUFFER_SIZE];
    int bufferIndex;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPSTarget>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
