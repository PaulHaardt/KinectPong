#include <OpenNI.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <algorithm>
#include <chrono>
#include <thread>
#include <fstream>

class RealTimeHandTracker {
private:
    openni::Device device;
    openni::VideoStream colorStream;
    openni::VideoStream depthStream;
    
    // Paramètres de détection optimisés
    const int MIN_CONTOUR_AREA = 3000;
    const int MAX_CONTOUR_AREA = 50000;
    const float MIN_DEPTH = 300.0f;  // 30cm
    const float MAX_DEPTH = 1200.0f; // 1.2m
    const int DEPTH_THRESHOLD = 50;   // mm de tolérance
    
    // Tracking des mains
    struct Hand {
        cv::Point2f center;
        cv::Rect boundingBox;
        float depth;
        int trackingAge;
        cv::Scalar color;
        std::vector<cv::Point> contour;
        bool active;
    };
    
    std::vector<Hand> trackedHands;
    std::string oniFile;
    bool useOni;
    
    // Paramètres de couleur de peau (HSV)
    cv::Scalar skinLowerHSV = cv::Scalar(0, 20, 70);
    cv::Scalar skinUpperHSV = cv::Scalar(20, 255, 255);
    
    // ROI pour optimiser le traitement
    cv::Rect processingROI;
    bool roiInitialized = false;
    
public:
    RealTimeHandTracker(const std::string& oni = "") : oniFile(oni), useOni(!oni.empty()) {}
    
    bool initialize() {
        std::cout << "=== DIAGNOSTIC OPENNI ===\n";
        
        if (openni::OpenNI::initialize() != openni::STATUS_OK) {
            std::cerr << "OpenNI init failed: " << openni::OpenNI::getExtendedError() << std::endl;
            return false;
        }
        std::cout << "✓ OpenNI initialisé\n";

        openni::Status rc;
        
        if (useOni) {
            std::cout << "\n=== MODE FICHIER ONI ===\n";
            std::cout << "Fichier: " << oniFile << std::endl;
            
            // Vérifier que le fichier existe
            std::ifstream file(oniFile);
            if (!file.good()) {
                std::cerr << "ERREUR: Fichier " << oniFile << " introuvable!\n";
                return false;
            }
            file.close();
            std::cout << "✓ Fichier ONI trouvé\n";
            
            rc = device.open(oniFile.c_str());
            if (rc != openni::STATUS_OK) {
                std::cerr << "ERREUR ouverture ONI: " << openni::OpenNI::getExtendedError() << std::endl;
                return false;
            }
            std::cout << "✓ Fichier ONI ouvert\n";
            
        } else {
            std::cout << "\n=== MODE KINECT LIVE ===\n";
            
            // Diagnostic des périphériques disponibles
            openni::Array<openni::DeviceInfo> deviceList;
            openni::OpenNI::enumerateDevices(&deviceList);
            
            std::cout << "Périphériques OpenNI détectés: " << deviceList.getSize() << std::endl;
            
            if (deviceList.getSize() == 0) {
                std::cerr << "\n❌ AUCUN PÉRIPHÉRIQUE DÉTECTÉ!\n";
                std::cerr << "\n=== SOLUTIONS ===\n";
                std::cerr << "1. Vérifiez que la Kinect est branchée (LED verte)\n";
                std::cerr << "2. Vérifiez les permissions:\n";
                std::cerr << "   sudo usermod -a -G dialout $USER\n";
                std::cerr << "   newgrp dialout\n";
                std::cerr << "3. Créez les règles udev:\n";
                std::cerr << "   sudo nano /etc/udev/rules.d/51-kinect.rules\n";
                std::cerr << "   Ajoutez: SUBSYSTEM==\"usb\", ATTR{idVendor}==\"045e\", ATTR{idProduct}==\"02ae\", MODE=\"0666\"\n";
                std::cerr << "   sudo udevadm control --reload-rules\n";
                std::cerr << "4. Testez avec: lsusb | grep Microsoft\n";
                std::cerr << "5. Essayez en tant que root: sudo ./votre_programme\n";
                return false;
            }
            
            // Afficher tous les périphériques
            for (int i = 0; i < deviceList.getSize(); ++i) {
                const openni::DeviceInfo& info = deviceList[i];
                std::cout << "\nPériphérique " << i << ":\n";
                std::cout << "  URI: " << info.getUri() << std::endl;
                std::cout << "  Nom: " << info.getName() << std::endl;
                std::cout << "  Vendor: " << info.getVendor() << std::endl;
                std::cout << "  PID: 0x" << std::hex << info.getUsbProductId() << std::dec << std::endl;
                std::cout << "  VID: 0x" << std::hex << info.getUsbVendorId() << std::dec << std::endl;
            }
            
            // Stratégie 1: Essayer le premier périphérique
            std::cout << "\n--- Tentative 1: Premier périphérique ---\n";
            rc = device.open(deviceList[0].getUri());
            if (rc != openni::STATUS_OK) {
                std::cout << "Échec: " << openni::OpenNI::getExtendedError() << std::endl;
                
                // Stratégie 2: Essayer ANY_DEVICE
                std::cout << "\n--- Tentative 2: ANY_DEVICE ---\n";
                rc = device.open(openni::ANY_DEVICE);
                if (rc != openni::STATUS_OK) {
                    std::cout << "Échec: " << openni::OpenNI::getExtendedError() << std::endl;
                    
                    // Stratégie 3: Essayer tous les périphériques un par un
                    std::cout << "\n--- Tentative 3: Tous les périphériques ---\n";
                    bool deviceOpened = false;
                    for (int i = 0; i < deviceList.getSize() && !deviceOpened; ++i) {
                        std::cout << "Essai périphérique " << i << ": " << deviceList[i].getUri() << std::endl;
                        rc = device.open(deviceList[i].getUri());
                        if (rc == openni::STATUS_OK) {
                            deviceOpened = true;
                            std::cout << "✓ Succès avec le périphérique " << i << "!\n";
                        } else {
                            std::cout << "Échec: " << openni::OpenNI::getExtendedError() << std::endl;
                        }
                    }
                    
                    if (!deviceOpened) {
                        std::cerr << "\n❌ IMPOSSIBLE D'OUVRIR LA KINECT!\n";
                        std::cerr << "\n=== DIAGNOSTIC AVANCÉ ===\n";
                        std::cerr << "Vérifiez avec ces commandes:\n";
                        std::cerr << "1. lsusb | grep -i microsoft\n";
                        std::cerr << "2. ls -la /dev/bus/usb/\n";
                        std::cerr << "3. groups $USER (doit contenir dialout)\n";
                        std::cerr << "4. Essayez: sudo " << "votre_programme" << std::endl;
                        std::cerr << "\nSi le problème persiste, essayez avec un fichier .oni:\n";
                        std::cerr << "./votre_programme fichier.oni\n";
                        return false;
                    }
                } else {
                    std::cout << "✓ Succès avec ANY_DEVICE!\n";
                }
            } else {
                std::cout << "✓ Succès avec le premier périphérique!\n";
            }
        }

        std::cout << "\n=== VÉRIFICATION DES CAPTEURS ===\n";
        
        // Vérifier les capteurs disponibles
        bool colorAvailable = device.hasSensor(openni::SENSOR_COLOR);
        bool depthAvailable = device.hasSensor(openni::SENSOR_DEPTH);
        bool irAvailable = device.hasSensor(openni::SENSOR_IR);
        
        std::cout << "Capteurs disponibles:\n";
        std::cout << "  - Couleur: " << (colorAvailable ? "✓ OUI" : "❌ NON") << "\n";
        std::cout << "  - Profondeur: " << (depthAvailable ? "✓ OUI" : "❌ NON") << "\n";
        std::cout << "  - Infrarouge: " << (irAvailable ? "✓ OUI" : "❌ NON") << "\n";

        if (!depthAvailable) {
            std::cerr << "\n❌ ERREUR CRITIQUE: Pas de capteur de profondeur!\n";
            std::cerr << "Le tracking des mains nécessite le depth stream.\n";
            return false;
        }

        std::cout << "\n=== CRÉATION DES STREAMS ===\n";
        
        // Créer et configurer les streams
        if (colorAvailable) {
            rc = colorStream.create(device, openni::SENSOR_COLOR);
            if (rc == openni::STATUS_OK) {
                std::cout << "✓ Color stream créé\n";
                
                if (!useOni) {
                    // Lister les modes disponibles
                    const openni::Array<openni::VideoMode>& colorModes = colorStream.getSensorInfo().getSupportedVideoModes();
                    std::cout << "Modes couleur disponibles:\n";
                    for (int i = 0; i < colorModes.getSize(); ++i) {
                        const openni::VideoMode& mode = colorModes[i];
                        std::cout << "  " << mode.getResolutionX() << "x" << mode.getResolutionY() 
                                  << " @ " << mode.getFps() << "fps\n";
                    }
                    
                    // Configuration
                    openni::VideoMode colorMode = colorStream.getVideoMode();
                    std::cout << "Mode actuel: " << colorMode.getResolutionX() << "x" << colorMode.getResolutionY() 
                              << " @ " << colorMode.getFps() << "fps\n";
                    
                    // Essayer de forcer 640x480@30fps
                    colorMode.setResolution(640, 480);
                    colorMode.setFps(30);
                    colorMode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
                    rc = colorStream.setVideoMode(colorMode);
                    if (rc != openni::STATUS_OK) {
                        std::cout << "⚠️  Impossible de changer le mode couleur, utilisation par défaut\n";
                    } else {
                        std::cout << "✓ Mode couleur configuré: 640x480@30fps\n";
                    }
                }
                
                rc = colorStream.start();
                if (rc == openni::STATUS_OK) {
                    std::cout << "✓ Color stream démarré\n";
                } else {
                    std::cerr << "❌ Échec démarrage color stream: " << openni::OpenNI::getExtendedError() << std::endl;
                    colorAvailable = false;
                }
            } else {
                std::cerr << "❌ Échec création color stream: " << openni::OpenNI::getExtendedError() << std::endl;
                colorAvailable = false;
            }
        }
        
        if (depthAvailable) {
            rc = depthStream.create(device, openni::SENSOR_DEPTH);
            if (rc == openni::STATUS_OK) {
                std::cout << "✓ Depth stream créé\n";
                
                if (!useOni) {
                    // Lister les modes disponibles
                    const openni::Array<openni::VideoMode>& depthModes = depthStream.getSensorInfo().getSupportedVideoModes();
                    std::cout << "Modes profondeur disponibles:\n";
                    for (int i = 0; i < depthModes.getSize(); ++i) {
                        const openni::VideoMode& mode = depthModes[i];
                        std::cout << "  " << mode.getResolutionX() << "x" << mode.getResolutionY() 
                                  << " @ " << mode.getFps() << "fps\n";
                    }
                    
                    openni::VideoMode depthMode = depthStream.getVideoMode();
                    std::cout << "Mode actuel: " << depthMode.getResolutionX() << "x" << depthMode.getResolutionY() 
                              << " @ " << depthMode.getFps() << "fps\n";
                    
                    depthMode.setResolution(640, 480);
                    depthMode.setFps(30);
                    rc = depthStream.setVideoMode(depthMode);
                    if (rc != openni::STATUS_OK) {
                        std::cout << "⚠️  Impossible de changer le mode profondeur, utilisation par défaut\n";
                    } else {
                        std::cout << "✓ Mode profondeur configuré: 640x480@30fps\n";
                    }
                }
                
                rc = depthStream.start();
                if (rc == openni::STATUS_OK) {
                    std::cout << "✓ Depth stream démarré\n";
                } else {
                    std::cerr << "❌ Échec démarrage depth stream: " << openni::OpenNI::getExtendedError() << std::endl;
                    return false;
                }
            } else {
                std::cerr << "❌ Échec création depth stream: " << openni::OpenNI::getExtendedError() << std::endl;
                return false;
            }
        }

        // Synchronisation
        if (colorAvailable && depthAvailable && !useOni) {
            std::cout << "\n=== SYNCHRONISATION ===\n";
            
            if (device.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR)) {
                device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
                std::cout << "✓ Registration depth-to-color activée\n";
            } else {
                std::cout << "⚠️  Registration depth-to-color non supportée\n";
            }
            
            device.setDepthColorSyncEnabled(true);
            std::cout << "✓ Synchronisation depth-color activée\n";
        }

        // Configuration du playback pour ONI
        if (useOni) {
            std::cout << "\n=== CONFIGURATION PLAYBACK ONI ===\n";
            openni::PlaybackControl* playback = device.getPlaybackControl();
            if (playback) {
                playback->setRepeatEnabled(true);
                std::cout << "✓ Répétition automatique activée\n";
                
                int numFrames = playback->getNumberOfFrames(depthStream);
                if (numFrames > 0) {
                    std::cout << "✓ Fichier ONI contient " << numFrames << " frames\n";
                } else {
                    std::cout << "⚠️  Impossible de déterminer le nombre de frames\n";
                }
            }
        }
        
        // ROI initial
        processingROI = cv::Rect(160, 120, 320, 240);
        
        std::cout << "\n✅ INITIALISATION TERMINÉE AVEC SUCCÈS!\n";
        std::cout << "===========================================\n\n";
        
        return true;
    }

    void run() {
        cv::namedWindow("Hand Tracker 30fps", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("Debug View", cv::WINDOW_AUTOSIZE);
        
        auto lastTime = std::chrono::high_resolution_clock::now();
        int frameCount = 0;
        double fps = 0;
        
        // Trackbars pour ajustement en temps réel
        cv::createTrackbar("Skin H Min", "Debug View", nullptr, 180);
        cv::setTrackbarPos("Skin H Min", "Debug View", 0);
        cv::createTrackbar("Skin H Max", "Debug View", nullptr, 180);
        cv::setTrackbarPos("Skin H Max", "Debug View", 20);

        // Variables pour gérer les différents modes
        bool hasColorStream = device.hasSensor(openni::SENSOR_COLOR) && colorStream.isValid();
        bool hasDepthStream = device.hasSensor(openni::SENSOR_DEPTH) && depthStream.isValid();
        
        std::cout << "Streams actifs: Color=" << hasColorStream << " Depth=" << hasDepthStream << std::endl;

        while (true) {
            auto frameStart = std::chrono::high_resolution_clock::now();
            
            // Lecture des frames avec gestion d'erreurs pour ONI
            openni::VideoFrameRef colorFrame, depthFrame;
            cv::Mat colorImage, depthImage;
            bool frameOK = false;
            
            // Lecture du depth (prioritaire)
            if (hasDepthStream) {
                openni::Status depthStatus = depthStream.readFrame(&depthFrame);
                if (depthStatus == openni::STATUS_OK) {
                    const openni::DepthPixel* depthBuffer = (const openni::DepthPixel*)depthFrame.getData();
                    depthImage = cv::Mat(depthFrame.getHeight(), depthFrame.getWidth(), CV_16UC1, (void*)depthBuffer);
                    frameOK = true;
                } else if (useOni && depthStatus == openni::STATUS_NO_DEVICE) {
                    std::cout << "Fin du fichier ONI atteinte\n";
                    break;
                }
            }
            
            // Lecture de la couleur
            if (hasColorStream) {
                openni::Status colorStatus = colorStream.readFrame(&colorFrame);
                if (colorStatus == openni::STATUS_OK) {
                    const openni::RGB888Pixel* colorBuffer = (const openni::RGB888Pixel*)colorFrame.getData();
                    colorImage = cv::Mat(colorFrame.getHeight(), colorFrame.getWidth(), CV_8UC3, (void*)colorBuffer);
                    cv::cvtColor(colorImage, colorImage, cv::COLOR_RGB2BGR);
                } else if (!frameOK) {
                    // Si pas de depth et pas de color, on continue
                    continue;
                }
            }
            
            // Si on n'a que le depth, créer une image couleur factice
            if (!hasColorStream && frameOK) {
                cv::Mat depthDisplay;
                depthImage.convertTo(depthDisplay, CV_8UC1, 255.0/4000.0);
                cv::cvtColor(depthDisplay, colorImage, cv::COLOR_GRAY2BGR);
            }
            
            if (!frameOK) continue;
            
            // Ajuster la taille si nécessaire
            if (colorImage.cols != 640 || colorImage.rows != 480) {
                cv::resize(colorImage, colorImage, cv::Size(640, 480));
            }
            if (depthImage.cols != 640 || depthImage.rows != 480) {
                cv::resize(depthImage, depthImage, cv::Size(640, 480));
            }
            
            // Mise à jour des paramètres de couleur de peau
            int hMin = cv::getTrackbarPos("Skin H Min", "Debug View");
            int hMax = cv::getTrackbarPos("Skin H Max", "Debug View");
            skinLowerHSV[0] = hMin;
            skinUpperHSV[0] = hMax;
            
            // Tracking des mains
            if (hasDepthStream) {
                detectAndTrackHands(colorImage, depthImage);
            }
            
            // Affichage
            cv::Mat debugView;
            if (hasDepthStream) {
                debugView = createDebugView(colorImage, depthImage);
            } else {
                debugView = colorImage.clone();
            }
            
            drawHands(colorImage);
            drawROI(colorImage);
            
            // Calcul FPS réel
            frameCount++;
            auto currentTime = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastTime);
            
            if (elapsed.count() >= 1000) {
                fps = frameCount * 1000.0 / elapsed.count();
                frameCount = 0;
                lastTime = currentTime;
            }
            
            // Informations à l'écran
            cv::putText(colorImage, "FPS: " + std::to_string((int)fps), 
                       cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
            cv::putText(colorImage, "Mains: " + std::to_string(trackedHands.size()), 
                       cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
            
            if (useOni) {
                cv::putText(colorImage, "Mode ONI", cv::Point(10, 90), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
            }
            
            cv::imshow("Hand Tracker 30fps", colorImage);
            cv::imshow("Debug View", debugView);
            
            // Timing adapté pour ONI vs Live
            if (!useOni) {
                auto frameEnd = std::chrono::high_resolution_clock::now();
                auto frameDuration = std::chrono::duration_cast<std::chrono::microseconds>(frameEnd - frameStart);
                auto targetDuration = std::chrono::microseconds(33333); // 1/30 sec
                
                if (frameDuration < targetDuration) {
                    auto sleepTime = targetDuration - frameDuration;
                    std::this_thread::sleep_for(sleepTime);
                }
            }
            
            int key = cv::waitKey(useOni ? 30 : 1); // Plus de délai pour ONI
            if (key == 27) break; // ESC
            if (key == 32) {      // SPACE
                trackedHands.clear();
                std::cout << "Tracking reset\n";
                
                // Relancer le fichier ONI si nécessaire
                if (useOni) {
                    openni::PlaybackControl* playback = device.getPlaybackControl();
                    if (playback) {
                        playback->seek(colorStream, 0);
                        std::cout << "Fichier ONI relancé\n";
                    }
                }
            }
            if (key == 'r') {     // R - reset ROI
                processingROI = cv::Rect(160, 120, 320, 240);
                roiInitialized = false;
                std::cout << "ROI reset\n";
            }
        }
    }
    
private:
    void detectAndTrackHands(const cv::Mat& colorImage, const cv::Mat& depthImage) {
        // Travail sur ROI pour optimiser
        cv::Mat roiColor = colorImage(processingROI);
        cv::Mat roiDepth = depthImage(processingROI);
        
        // Masque de profondeur
        cv::Mat depthMask;
        cv::inRange(roiDepth, MIN_DEPTH, MAX_DEPTH, depthMask);
        
        // Détection couleur de peau
        cv::Mat hsvImage, skinMask;
        cv::cvtColor(roiColor, hsvImage, cv::COLOR_BGR2HSV);
        cv::inRange(hsvImage, skinLowerHSV, skinUpperHSV, skinMask);
        
        // Combinaison des masques
        cv::Mat combinedMask;
        cv::bitwise_and(depthMask, skinMask, combinedMask);
        
        // Nettoyage morphologique rapide
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(combinedMask, combinedMask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(combinedMask, combinedMask, cv::MORPH_CLOSE, kernel);
        
        // Détection de contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(combinedMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        // Traitement des contours
        std::vector<Hand> currentHands;
        
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area < MIN_CONTOUR_AREA || area > MAX_CONTOUR_AREA) continue;
            
            // Calcul du centre et de la boîte englobante
            cv::Moments moments = cv::moments(contour);
            if (moments.m00 == 0) continue;
            
            cv::Point2f center(moments.m10 / moments.m00, moments.m01 / moments.m00);
            cv::Rect bbox = cv::boundingRect(contour);
            
            // Ajustement pour la ROI globale
            center.x += processingROI.x;
            center.y += processingROI.y;
            bbox.x += processingROI.x;
            bbox.y += processingROI.y;
            
            // Calcul de la profondeur moyenne
            float avgDepth = calculateAverageDepth(depthImage, bbox);
            if (avgDepth < MIN_DEPTH || avgDepth > MAX_DEPTH) continue;
            
            // Création de la main
            Hand hand;
            hand.center = center;
            hand.boundingBox = bbox;
            hand.depth = avgDepth;
            hand.trackingAge = 1;
            hand.active = true;
            
            // Contour ajusté pour la position globale
            hand.contour = contour;
            for (auto& pt : hand.contour) {
                pt.x += processingROI.x;
                pt.y += processingROI.y;
            }
            
            currentHands.push_back(hand);
        }
        
        // Association avec les mains existantes (tracking temporel)
        matchHands(currentHands);
        
        // Mise à jour de la ROI basée sur les mains détectées
        updateROI(colorImage.size());
    }
    
    void matchHands(const std::vector<Hand>& currentHands) {
        const float MAX_DISTANCE = 100.0f; // pixels
        
        // Marquer toutes les mains comme inactives
        for (auto& hand : trackedHands) {
            hand.active = false;
        }
        
        // Associer les nouvelles détections
        for (const auto& newHand : currentHands) {
            float minDistance = MAX_DISTANCE;
            int bestMatch = -1;
            
            for (int i = 0; i < trackedHands.size(); i++) {
                float distance = cv::norm(trackedHands[i].center - newHand.center);
                if (distance < minDistance) {
                    minDistance = distance;
                    bestMatch = i;
                }
            }
            
            if (bestMatch >= 0) {
                // Mise à jour de la main existante
                Hand& existingHand = trackedHands[bestMatch];
                
                // Lissage de la position
                float alpha = 0.7f;
                existingHand.center = existingHand.center * alpha + newHand.center * (1.0f - alpha);
                existingHand.boundingBox = newHand.boundingBox;
                existingHand.depth = existingHand.depth * alpha + newHand.depth * (1.0f - alpha);
                existingHand.contour = newHand.contour;
                existingHand.trackingAge++;
                existingHand.active = true;
            } else {
                // Nouvelle main
                Hand newTrackedHand = newHand;
                newTrackedHand.color = generateRandomColor();
                trackedHands.push_back(newTrackedHand);
                std::cout << "Nouvelle main détectée (total: " << trackedHands.size() << ")\n";
            }
        }
        
        // Supprimer les mains perdues
        trackedHands.erase(
            std::remove_if(trackedHands.begin(), trackedHands.end(),
                [](const Hand& hand) { return !hand.active; }),
            trackedHands.end()
        );
    }
    
    float calculateAverageDepth(const cv::Mat& depthImage, const cv::Rect& bbox) {
        cv::Mat roi = depthImage(bbox);
        cv::Scalar meanDepth = cv::mean(roi, roi > 0);
        return meanDepth[0];
    }
    
    void updateROI(const cv::Size& imageSize) {
        if (trackedHands.empty()) return;
        
        // Calculer la boîte englobante de toutes les mains
        cv::Rect overallBbox;
        bool first = true;
        
        for (const auto& hand : trackedHands) {
            if (first) {
                overallBbox = hand.boundingBox;
                first = false;
            } else {
                overallBbox |= hand.boundingBox;
            }
        }
        
        // Expansion de la ROI
        int margin = 100;
        overallBbox.x = std::max(0, overallBbox.x - margin);
        overallBbox.y = std::max(0, overallBbox.y - margin);
        overallBbox.width = std::min(imageSize.width - overallBbox.x, overallBbox.width + 2 * margin);
        overallBbox.height = std::min(imageSize.height - overallBbox.y, overallBbox.height + 2 * margin);
        
        // Lissage de la ROI
        if (roiInitialized) {
            float alpha = 0.8f;
            processingROI.x = processingROI.x * alpha + overallBbox.x * (1.0f - alpha);
            processingROI.y = processingROI.y * alpha + overallBbox.y * (1.0f - alpha);
            processingROI.width = processingROI.width * alpha + overallBbox.width * (1.0f - alpha);
            processingROI.height = processingROI.height * alpha + overallBbox.height * (1.0f - alpha);
        } else {
            processingROI = overallBbox;
            roiInitialized = true;
        }
    }
    
    cv::Scalar generateRandomColor() {
        return cv::Scalar(rand() % 256, rand() % 256, rand() % 256);
    }
    
    void drawHands(cv::Mat& image) {
        for (const auto& hand : trackedHands) {
            // Contour de la main
            std::vector<std::vector<cv::Point>> contours = {hand.contour};
            cv::drawContours(image, contours, -1, hand.color, 2);
            
            // Centre de la main
            cv::circle(image, hand.center, 8, hand.color, -1);
            cv::circle(image, hand.center, 12, cv::Scalar(255, 255, 255), 2);
            
            // Boîte englobante
            cv::rectangle(image, hand.boundingBox, hand.color, 2);
            
            // Informations
            std::string info = "D:" + std::to_string((int)hand.depth) + "mm Age:" + std::to_string(hand.trackingAge);
            cv::putText(image, info, cv::Point(hand.boundingBox.x, hand.boundingBox.y - 10),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, hand.color, 1);
        }
    }
    
    void drawROI(cv::Mat& image) {
        cv::rectangle(image, processingROI, cv::Scalar(0, 255, 255), 2);
        cv::putText(image, "ROI", cv::Point(processingROI.x, processingROI.y - 5),
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
    }
    
    cv::Mat createDebugView(const cv::Mat& colorImage, const cv::Mat& depthImage) {
        cv::Mat roiColor = colorImage(processingROI);
        cv::Mat roiDepth = depthImage(processingROI);
        
        // Masques
        cv::Mat depthMask, skinMask, combinedMask;
        cv::inRange(roiDepth, MIN_DEPTH, MAX_DEPTH, depthMask);
        
        cv::Mat hsvImage;
        cv::cvtColor(roiColor, hsvImage, cv::COLOR_BGR2HSV);
        cv::inRange(hsvImage, skinLowerHSV, skinUpperHSV, skinMask);
        
        cv::bitwise_and(depthMask, skinMask, combinedMask);
        
        // Conversion pour affichage
        cv::Mat depthDisplay, skinDisplay, combinedDisplay;
        cv::cvtColor(depthMask, depthDisplay, cv::COLOR_GRAY2BGR);
        cv::cvtColor(skinMask, skinDisplay, cv::COLOR_GRAY2BGR);
        cv::cvtColor(combinedMask, combinedDisplay, cv::COLOR_GRAY2BGR);
        
        // Composition
        cv::Mat result;
        cv::hconcat(std::vector<cv::Mat>{roiColor, depthDisplay, skinDisplay, combinedDisplay}, result);
        
        return result;
    }

public:
    void cleanup() {
        colorStream.stop();
        colorStream.destroy();
        depthStream.stop();
        depthStream.destroy();
        device.close();
        openni::OpenNI::shutdown();
    }
};

int main(int argc, char* argv[]) {
    std::string oniFile = (argc > 1) ? argv[1] : "";
    
    RealTimeHandTracker tracker(oniFile);
    
    if (!tracker.initialize()) {
        std::cerr << "Initialization failed\n";
        return 1;
    }
    
    tracker.run();
    tracker.cleanup();
    
    return 0;
}