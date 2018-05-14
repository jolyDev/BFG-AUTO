#include <opencv2/opencv.hpp>

#include <experimental/optional>
#include <vector>
#include <utility>
#include <deque>
#include <cstdint>
#include <string>

struct LaserPoint {
    float x;
    float y;
    float size;

    LaserPoint() = default;
    LaserPoint(float x, float y, float size) : x(x), y(y), size(size) {}

    void draw(cv::Mat frame, cv::Scalar color = cv::Scalar(255)) const;
};

class LaserDetector {
    enum class PointStatus {
        Present,
        NotPresent,
        NotProcessed,
    };

    cv::VideoCapture cap;
    cv::SimpleBlobDetector blob_detector;

    std::deque<cv::Mat> pipeline;
    std::deque<std::tuple<LaserPoint, PointStatus>> biggest_points;
    size_t pipeline_size = 1;
    uint8_t min_hue = 40;
    uint8_t max_hue = 70;
    float min_size = 0.0;

    void fill_pipeline();
    void cycle();

    cv::Mat preprocess(cv::Mat, uint8_t thresh_min, uint8_t thresh_max, size_t blur_size);
    std::vector<LaserPoint> get_points(cv::Mat processed, cv::Mat frame, float circle_viccinity);
    std::experimental::optional<LaserPoint> get_biggest(const std::vector<LaserPoint>& points);
    std::experimental::optional<LaserPoint> get_frame_point(size_t index);

    void trace(const std::string& str);
    void debug(const std::string& str);

public:
    bool show_discarded = false;
    bool debug_enabled = false;
    bool trace_enabled = false;

    LaserDetector(size_t cam_id) : cap(cam_id) {
        fill_pipeline();
    }

    cv::Mat get_hsv_frame();
    cv::Mat get_rgb_frame();
    cv::Mat get_processed_frame();

    void set_pipeline_size(size_t size);
    void set_hue_range(uint8_t min, uint8_t max);
    void set_min_size(float min);

    std::experimental::optional<LaserPoint> detect();
};
