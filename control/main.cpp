#include <detector.hpp>
#include <SerialStream.h>
#include <string>

int main(int argc, char** argv) {
    cv::namedWindow("window");

    if (argc < 2) {
        std::cerr << "Not enough arguments" << std::endl;
        return 1;
    }

    LaserDetector detector(std::stoi(argv[1]));
    detector.set_pipeline_size(5);
    // detector.set_min_size(6.0);
    // detector.debug_enabled = true;

    LibSerial::SerialStream serial;
    serial.Open("/dev/ttyACM0");
    serial.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_9600);

    if (!serial) {
        std::cerr << "Failed to open serial port" << std::endl;
    }

    while (true) {
        int vertical = 1;
        int horizontal = 1;

        auto point = detector.detect();
        if (point) {
            std::cout << point->x << ' ' << point->y << std::endl;

            if (point->x < -0.1) {
                horizontal = 0;
            } else if (point->x > 0.1) {
                horizontal = 2;
            }

            if (point->y < 0.1) {
                vertical = 0;
            } else if (point->y > 0.3) {
                vertical = 2;
            }


            auto f = detector.get_rgb_frame();
            auto p = detector.get_processed_frame();
            cv::merge({p, p, p}, p);

            point->draw(f);
            point->draw(p);
            cv::Mat frame;
            cv::hconcat(f, p, frame);

            cv::imshow("window", frame);
        } else {
            auto f = detector.get_rgb_frame();
            auto p = detector.get_processed_frame();
            cv::merge({p, p, p}, p);
            cv::Mat frame;
            cv::hconcat(f, p, frame);
            cv::imshow("window", frame);
        }

        // std::cout << horizontal << vertical << std::endl;
        // std::cout << vertical << std::endl;
        std::cout << horizontal << std::endl;
        serial << 'a' << horizontal << vertical;

        if (cv::waitKey(30) >= 0) break;
    }
}
