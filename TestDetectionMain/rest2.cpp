#include <NiTE.h>
#include <OpenNI.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

class HandDetector {
private:
    openni::Device device;
    openni::VideoStream colorStream;
    openni::VideoStream depthStream;
    nite::HandTracker handTracker;
    openni::Recorder recorder;
    
    // Paramètres de détection améliorés
    const int SMOOTHING_FACTOR = 3;
    const float MIN_CONFIDENCE = 0.5f;
    const float GESTURE_DETECTION_DISTANCE = 1500.0f; // mm
    
    std::vector<cv::Point2f> handHistory;
    
public:
    bool initialize() {
        // Initialize OpenNI and NiTE
        if (openni::OpenNI::initialize() != openni::STATUS_OK) {
            std::cerr << "Failed to initialize OpenNI\n";
            return false;
        }
        
        if (nite::NiTE::initialize() != nite::STATUS_OK) {
            std::cerr << "Failed to initialize NiTE\n";
            return false;
        }

        // Open device
        if (device.open(openni::ANY_DEVICE) != openni::STATUS_OK) {
            std::cerr << "Failed to open device\n";
            return false;
        }

        // Create color stream avec paramètres optimisés
        if (colorStream.create(device, openni::SENSOR_COLOR) != openni::STATUS_OK) {
            std::cerr << "Failed to create color stream\n";
            return false;
        }
        
        // Configuration optimale pour la détection
        openni::VideoMode colorMode = colorStream.getVideoMode();
        colorMode.setResolution(640, 480);
        colorMode.setFps(30);
        colorMode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
        colorStream.setVideoMode(colorMode);

        // Create depth stream pour améliorer la détection
        if (depthStream.create(device, openni::SENSOR_DEPTH) != openni::STATUS_OK) {
            std::cerr << "Failed to create depth stream\n";
            return false;
        }
        
        openni::VideoMode depthMode = depthStream.getVideoMode();
        depthMode.setResolution(640, 480);
        depthMode.setFps(30);
        depthStream.setVideoMode(depthMode);

        // Start streams
        if (colorStream.start() != openni::STATUS_OK || 
            depthStream.start() != openni::STATUS_OK) {
            std::cerr << "Failed to start streams\n";
            return false;
        }

        // Synchronisation des streams
        device.setDepthColorSyncEnabled(true);
        device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);

        // Create recorder
        if (recorder.create("recording.oni") != openni::STATUS_OK) {
            std::cerr << "Failed to create recorder\n";
            return false;
        }

        if (recorder.attach(colorStream) != openni::STATUS_OK ||
            recorder.attach(depthStream) != openni::STATUS_OK) {
            std::cerr << "Failed to attach streams to recorder\n";
            return false;
        }

        if (recorder.start() != openni::STATUS_OK) {
            std::cerr << "Failed to start recorder\n";
            return false;
        }

        // Create hand tracker avec paramètres optimisés
        if (handTracker.create(&device) != nite::STATUS_OK) {
            std::cerr << "Couldn't create hand tracker\n";
            return false;
        }

        // Configuration du hand tracker
        handTracker.setSmoothingFactor(SMOOTHING_FACTOR);
        
        // Démarrer plusieurs types de gestes pour améliorer la détection
        handTracker.startGestureDetection(nite::GESTURE_WAVE);
        handTracker.startGestureDetection(nite::GESTURE_CLICK);
        handTracker.startGestureDetection(nite::GESTURE_HAND_RAISE);

        return true;
    }

    void run() {
        cv::namedWindow("Hand Detection", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("Depth View", cv::WINDOW_AUTOSIZE);

        std::cout << "Détection des mains active. Faites des gestes devant la caméra.\n";
        std::cout << "Appuyez sur ESC pour quitter.\n";

        while (true) {
            // Read frames
            openni::VideoFrameRef colorFrame, depthFrame;
            if (colorStream.readFrame(&colorFrame) != openni::STATUS_OK ||
                depthStream.readFrame(&depthFrame) != openni::STATUS_OK) {
                std::cerr << "Failed to read frames\n";
                continue;
            }

            // Convert color frame to OpenCV Mat
            const openni::RGB888Pixel* imageBuffer = (const openni::RGB888Pixel*)colorFrame.getData();
            cv::Mat colorImage(colorFrame.getHeight(), colorFrame.getWidth(), CV_8UC3, (void*)imageBuffer);
            cv::cvtColor(colorImage, colorImage, cv::COLOR_RGB2BGR);

            // Convert depth frame pour visualisation
            const openni::DepthPixel* depthBuffer = (const openni::DepthPixel*)depthFrame.getData();
            cv::Mat depthImage(depthFrame.getHeight(), depthFrame.getWidth(), CV_16UC1, (void*)depthBuffer);
            cv::Mat depthDisplay;
            depthImage.convertTo(depthDisplay, CV_8UC1, 255.0/4000.0);
            cv::applyColorMap(depthDisplay, depthDisplay, cv::COLORMAP_JET);

            // Read hand frame
            nite::HandTrackerFrameRef handFrame;
            if (handTracker.readFrame(&handFrame) != nite::STATUS_OK) {
                continue;
            }

            // Process gestures avec détection améliorée
            const nite::Array<nite::GestureData>& gestures = handFrame.getGestures();
            for (int i = 0; i < gestures.getSize(); ++i) {
                const nite::GestureData& gesture = gestures[i];
                
                // Vérifier la distance pour éviter les fausses détections
                if (gesture.getCurrentPosition().z > GESTURE_DETECTION_DISTANCE) {
                    continue;
                }
                
                if (gesture.isComplete()) {
                    std::cout << "Geste détecté: " << gesture.getType() << std::endl;
                    
                    nite::HandId newHandId;
                    if (handTracker.startHandTracking(gesture.getCurrentPosition(), &newHandId) == nite::STATUS_OK) {
                        std::cout << "Suivi de la main démarré (ID: " << newHandId << ")\n";
                    }
                }
                
                // Visualiser les gestes en cours
                if (gesture.isInProgress()) {
                    float x, y;
                    handTracker.convertHandCoordinatesToDepth(
                        gesture.getCurrentPosition().x,
                        gesture.getCurrentPosition().y,
                        gesture.getCurrentPosition().z,
                        &x, &y);
                    
                    cv::circle(colorImage, cv::Point((int)x, (int)y), 15, cv::Scalar(255, 255, 0), 2);
                    cv::putText(colorImage, "Gesture", cv::Point((int)x-30, (int)y-20), 
                               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0));
                }
            }

            // Process tracked hands avec filtrage amélioré
            const nite::Array<nite::HandData>& hands = handFrame.getHands();
            for (int i = 0; i < hands.getSize(); ++i) {
                const nite::HandData& hand = hands[i];
                
                if (hand.isTracking()) {
                    const auto& pos = hand.getPosition();
                    
                    // Filtrer les positions trop éloignées
                    if (pos.z > GESTURE_DETECTION_DISTANCE) {
                        continue;
                    }
                    
                    // Convertir les coordonnées 3D en 2D
                    float imageX, imageY;
                    handTracker.convertHandCoordinatesToDepth(pos.x, pos.y, pos.z, &imageX, &imageY);
                    
                    // Vérifier que les coordonnées sont dans l'image
                    if (imageX >= 0 && imageX < colorImage.cols && 
                        imageY >= 0 && imageY < colorImage.rows) {
                        
                        // Dessiner la main avec informations détaillées
                        cv::Point handPoint((int)imageX, (int)imageY);
                        
                        // Cercle principal
                        cv::circle(colorImage, handPoint, 20, cv::Scalar(0, 255, 0), 3);
                        cv::circle(colorImage, handPoint, 5, cv::Scalar(0, 255, 0), -1);
                        
                        // Informations textuelles
                        std::string info = "ID:" + std::to_string(hand.getId()) + 
                                         " Z:" + std::to_string((int)pos.z) + "mm";
                        cv::putText(colorImage, info, cv::Point((int)imageX-40, (int)imageY-30), 
                                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
                        
                        // Tracer l'historique des positions
                        handHistory.push_back(handPoint);
                        if (handHistory.size() > 20) {
                            handHistory.erase(handHistory.begin());
                        }
                        
                        for (size_t j = 1; j < handHistory.size(); ++j) {
                            cv::line(colorImage, handHistory[j-1], handHistory[j], 
                                   cv::Scalar(0, 255, 255), 2);
                        }
                        
                        // Afficher aussi sur l'image de profondeur
                        cv::circle(depthDisplay, handPoint, 15, cv::Scalar(0, 255, 0), 3);
                        
                        // Log des positions
                        std::cout << "Main ID " << hand.getId() << ": (" 
                                  << pos.x << ", " << pos.y << ", " << pos.z << "mm)\n";
                    }
                }
                else if (hand.isLost()) {
                    std::cout << "Main perdue (ID: " << hand.getId() << ")\n";
                    handHistory.clear();
                }
            }

            // Afficher les instructions
            cv::putText(colorImage, "Faites des gestes pour detecter les mains", 
                       cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255));
            cv::putText(colorImage, "ESC pour quitter", 
                       cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255));

            // Display images
            cv::imshow("Hand Detection", colorImage);
            cv::imshow("Depth View", depthDisplay);
            
            if (cv::waitKey(1) == 27) break;  // ESC
        }
    }

    void cleanup() {
        std::cout << "Nettoyage des ressources...\n";
        
        recorder.stop();
        recorder.destroy();

        colorStream.stop();
        colorStream.destroy();
        
        depthStream.stop();
        depthStream.destroy();
        
        handTracker.destroy();
        device.close();

        nite::NiTE::shutdown();
        openni::OpenNI::shutdown();
    }
};

int main() {
    HandDetector detector;
    
    if (!detector.initialize()) {
        std::cerr << "Échec de l'initialisation\n";
        return 1;
    }
    
    detector.run();
    detector.cleanup();
    
    return 0;
}
