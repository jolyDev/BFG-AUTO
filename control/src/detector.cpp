#include <detector.hpp>
#include <iostream>
#include <utility>

using namespace cv;
using namespace std;
using experimental::optional;
using experimental::nullopt;

void LaserPoint::draw(Mat frame, Scalar color) const {
    cv::circle(
        frame,
        cv::Point(
            (x + 1) / 2 * frame.cols,
            (y + 1) / 2 * frame.rows
        ),
        size,
        color,
        2
    );
}


Mat LaserDetector::get_hsv_frame() {
    trace(__func__);

    fill_pipeline();

    Mat frame = pipeline.back();

    Mat rgb;
    cvtColor(pipeline.back(), rgb, CV_RGB2HSV);

    return rgb;
}

Mat LaserDetector::get_rgb_frame() {
    trace(__func__);

    fill_pipeline();

    return pipeline.back().clone();
}

Mat LaserDetector::get_processed_frame() {
    Mat frame = get_hsv_frame();
    array<Mat, 3> hsv;
    split(frame, hsv.data());

    Mat v = hsv[2];
    return preprocess(v, 240, 255, 5);
}

void LaserDetector::fill_pipeline() {
    trace(__func__);

    while (pipeline.size() < pipeline_size) {
        Mat frame;
        cap >> frame;
        Mat dst;
        flip(frame, dst, -1);
        pipeline.push_back(dst);

        biggest_points.emplace_back(LaserPoint(), PointStatus::NotProcessed);
    }
}

void LaserDetector::cycle() {
    trace(__func__);

    pipeline.pop_front();
    biggest_points.pop_front();

    fill_pipeline();
}

Mat LaserDetector::preprocess(Mat mat, uint8_t thresh_min, uint8_t thresh_max, size_t blur_size) {
    trace(__func__);

    Mat res;

    threshold(mat, res, thresh_min, thresh_max, CV_THRESH_BINARY_INV);
    blur(res, res, Size(blur_size, blur_size));

    return res;
}

vector<LaserPoint> LaserDetector::get_points(Mat processed, Mat frame, float circle_viccinity) {
    trace(__func__);

    vector<KeyPoint> blobs;
    blob_detector.detect(processed, blobs);

    debug("got " + to_string(blobs.size()) + " blobs");

    vector<LaserPoint> points;

    Mat mask = frame.clone();
    cvtColor(mask, mask, CV_RGB2GRAY);
    for (auto& kp : blobs) {
        mask.setTo(Scalar(0));
        circle(mask, kp.pt, kp.size * circle_viccinity, Scalar(255), CV_FILLED);
        circle(mask, kp.pt, kp.size, Scalar(0), CV_FILLED);

        Scalar m = mean(frame, mask > 0);
        uint8_t h = m[0];

        if (show_discarded) {
            debug("blob hue: " + to_string(h) + "; blob size: " + to_string(kp.size));
        }

        if (kp.size >= min_size && h >= min_hue && h <= max_hue) {

            if (!show_discarded) {
                debug("blob hue: " + to_string(h) + "; blob size: " + to_string(kp.size));
            }
            circle(processed, kp.pt, kp.size, Scalar(0), 2);

            points.emplace_back(
                kp.pt.x / frame.cols * 2 - 1,
                kp.pt.y / frame.rows * 2 - 1,
                kp.size
            );
        }
    }

    return points;
}

optional<LaserPoint> LaserDetector::get_biggest(const vector<LaserPoint>& points) {
    auto biggest = max_element(
        cbegin(points), cend(points),
        [](const auto& fst, const auto& snd) {
            return fst.size > snd.size;
        }
    );

    if (biggest != cend(points)) {
        return *biggest;
    } else {
        return nullopt;
    }
}

optional<LaserPoint> LaserDetector::get_frame_point(size_t index) {
    auto& pair = biggest_points[index];

    switch(get<1>(pair)) {
        case PointStatus::Present:
            return get<0>(pair);

        case PointStatus::NotPresent:
            return nullopt;

        case PointStatus::NotProcessed:
        {
            Mat frame = pipeline[index];
            Mat hsv_frame;
            cvtColor(frame, hsv_frame, CV_RGB2HSV);

            array<Mat, 3> hsv;
            split(hsv_frame, hsv.data());

            Mat v = hsv[2];

            auto point = get_biggest(
                get_points(
                    preprocess(v, 240, 255, 5),
                    hsv_frame,
                    1.5
                )
            );

            if (point) {
                get<1>(pair) = PointStatus::Present;
            } else {
                get<1>(pair) = PointStatus::NotPresent;
            }

            return point;
        }

        default:
            debug("something went wrong");
            return nullopt;
    }
}

optional<LaserPoint> LaserDetector::detect() {
    trace(__func__);

    cycle();

    vector<LaserPoint> points;

    for (size_t i = 0; i < pipeline.size(); i++) {
        auto point = get_frame_point(i);

        if (point) {
            points.push_back(*point);
        }
    }

    debug("got " + to_string(points.size()) + " laser dots");

    if (points.size() >= ceil(static_cast<float>(pipeline_size)/2)) {
        return *max_element(
            cbegin(points), cend(points),
            [](const auto& fst, const auto& snd) {
                return fst.size < snd.size;
            }
        );

        // float sum_x = 0;
        // float sum_y = 0;
        // float sum_size = 0;
//
        // for (const auto& pt : points) {
            // sum_x += pt.x;
            // sum_y += pt.y;
            // sum_size += pt.size;
        // }
//
        // return LaserPoint {
            // sum_x / points.size(),
            // sum_y / points.size(),
            // sum_size / points.size()
        // };
    }

    return nullopt;
}

void LaserDetector::set_pipeline_size(size_t size) {
    trace(__func__);

    pipeline_size = (size == 0 ? 1 : size);

    while (pipeline.size() > pipeline_size) {
        pipeline.pop_front();
        biggest_points.pop_front();
    }

    if (pipeline.size() < pipeline_size) {
        fill_pipeline();
    }
}

void LaserDetector::set_hue_range(uint8_t min, uint8_t max) {
    trace(__func__);

    if (min > max) {
        swap(min, max);
    }

    min_hue = min;
    max_hue = max;
}

void LaserDetector::trace(const string& str) {
    if (trace_enabled) {
        cerr << "[TRACE] " << str << endl;
    }
}

void LaserDetector::debug(const string& str) {
    if (debug_enabled) {
        cerr << "[DEBUG] " << str << endl;
    }
}

void LaserDetector::set_min_size(float min) {
    min_size = (min <= 0 ? 0 : min);
}
