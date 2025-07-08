#include <NiTE.h>
#include <OpenNI.h>
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // Initialize OpenNI and NiTE
    openni::OpenNI::initialize();
    nite::NiTE::initialize();

    // Open the device
    openni::Device device;
    if (device.open(openni::ANY_DEVICE) != openni::STATUS_OK) {
        std::cerr << "Failed to open device\n";
        return 1;
    }

    // Create a video stream (color)
    openni::VideoStream colorStream;
    if (colorStream.create(device, openni::SENSOR_COLOR) != openni::STATUS_OK) {
        std::cerr << "Failed to create color stream\n";
        return 1;
    }

    colorStream.start();

    // Create and start recorder
    openni::Recorder recorder;
    if (recorder.create("recording.oni") != openni::STATUS_OK) {
        std::cerr << "Failed to create recorder\n";
        return 1;
    }

    if (recorder.attach(colorStream) != openni::STATUS_OK) {
        std::cerr << "Failed to attach color stream to recorder\n";
        return 1;
    }

    if (recorder.start() != openni::STATUS_OK) {
        std::cerr << "Failed to start recorder\n";
        return 1;
    }

    // Create the hand tracker
    nite::HandTracker handTracker;
    if (handTracker.create(&device) != nite::STATUS_OK) {
        std::cerr << "Couldn't create hand tracker\n";
        return 1;
    }

    handTracker.startGestureDetection(nite::GESTURE_WAVE);
    handTracker.startGestureDetection(nite::GESTURE_CLICK);

    cv::namedWindow("Video Feed", cv::WINDOW_AUTOSIZE);

    while (true) {
        // Read color frame
        openni::VideoFrameRef colorFrame;
        if (colorStream.readFrame(&colorFrame) != openni::STATUS_OK) {
            std::cerr << "Failed to read color frame\n";
            continue;
        }

        const openni::RGB888Pixel* imageBuffer = (const openni::RGB888Pixel*)colorFrame.getData();
        cv::Mat image(colorFrame.getHeight(), colorFrame.getWidth(), CV_8UC3, (void*)imageBuffer);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        // Read hand frame
        nite::HandTrackerFrameRef handFrame;
        if (handTracker.readFrame(&handFrame) != nite::STATUS_OK) {
            std::cerr << "Get next frame failed\n";
            continue;
        }

        // Handle gestures
        const nite::Array<nite::GestureData>& gestures = handFrame.getGestures();
        for (int i = 0; i < gestures.getSize(); ++i) {
            const nite::GestureData& gesture = gestures[i];
            if (gesture.isComplete()) {
                nite::HandId newHandId;
                if (handTracker.startHandTracking(gesture.getCurrentPosition(), &newHandId) != nite::STATUS_OK) {
                    std::cerr << "Failed to start hand tracking\n";
                }
            }
        }

        // Draw tracked hands
        const nite::Array<nite::HandData>& hands = handFrame.getHands();
        for (int i = 0; i < hands.getSize(); ++i) {
            const nite::HandData& hand = hands[i];
            if (hand.isTracking()) {
                const auto& pos = hand.getPosition();
                std::cout << "Hand ID " << hand.getId() << ": (" 
                          << pos.x << ", " << pos.y << ", " << pos.z << ")\n";

                float imageX, imageY;
                handTracker.convertHandCoordinatesToDepth(pos.x, pos.y, pos.z, &imageX, &imageY);
                cv::circle(image, cv::Point((int)imageX, (int)imageY), 10, cv::Scalar(0, 255, 0), -1);
            }
        }

        cv::imshow("Video Feed", image);
        if (cv::waitKey(1) == 27) break;  // ESC
    }

    // Cleanup
    recorder.stop();
    recorder.destroy();

    colorStream.stop();
    colorStream.destroy();
    device.close();

    nite::NiTE::shutdown();
    openni::OpenNI::shutdown();

    return 0;
}

