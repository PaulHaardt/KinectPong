#include <iostream>
#include <opencv2/opencv.hpp>
#include <OpenNI.h>
#include <deque>

int main() {
    // Initialize OpenNI
    openni::Status rc = openni::OpenNI::initialize();
    if (rc != openni::STATUS_OK) {
        std::cerr << "Initialize failed: " << openni::OpenNI::getExtendedError()
                  << std::endl;
        return -1;
    }

    // Open device
    openni::Device device;
    rc = device.open(openni::ANY_DEVICE);
    if (rc != openni::STATUS_OK) {
        std::cerr << "Couldn't open device: " << openni::OpenNI::getExtendedError()
                  << std::endl;
        return -1;
    }

    // Create depth stream
    openni::VideoStream depth;
    rc = depth.create(device, openni::SENSOR_DEPTH);
    if (rc != openni::STATUS_OK) {
        std::cerr << "Couldn't create depth stream: "
                  << openni::OpenNI::getExtendedError() << std::endl;
        return -1;
    }

    rc = depth.start();
    if (rc != openni::STATUS_OK) {
        std::cerr << "Couldn't start depth stream: "
                  << openni::OpenNI::getExtendedError() << std::endl;
        return -1;
    }

    cv::Mat imgGray, depth8, depthEq, mask, filtered_mask;
    double minVal, maxVal;
    
    // Buffer pour stocker les frames précédentes et détecter les objets immobiles
    std::deque<cv::Mat> frame_buffer;
    const int BUFFER_SIZE = 10;  // Nombre de frames à garder en mémoire
    const double STABILITY_THRESHOLD = 0.70;  // Seuil de stabilité (95% de pixels identiques)
    
    cv::Mat stable_objects = cv::Mat::zeros(480, 640, CV_8UC1);  // Masque des objets stables
    bool buffer_full = false;

    while (true) {
        // Read frame from Kinect
        openni::VideoFrameRef frame;
        rc = depth.readFrame(&frame);
        if (rc != openni::STATUS_OK) {
            continue;
        }

        // Convert to OpenCV Mat
        cv::Mat rawDepth(frame.getHeight(), frame.getWidth(), CV_16UC1,
                         (void *)frame.getData());

        // Create a copy to avoid issues with frame buffer
        cv::Mat depthMat = rawDepth.clone();

        // Convert depth to grayscale
        depthMat.convertTo(imgGray, CV_8UC1, 255.0 / 65535.0);

        // Find min/max values on the actual depth data
        cv::minMaxLoc(depthMat, &minVal, &maxVal);

        // Avoid divide-by-zero
        if (maxVal - minVal < 1e-6) {
            std::cerr << "Invalid depth range: " << minVal << " - " << maxVal
                      << std::endl;
            continue;
        }

        // Better normalization for depth data
        depthMat.convertTo(depth8, CV_8UC1, 255.0 / (maxVal - minVal),
                          -minVal * 255.0 / (maxVal - minVal));

        // Histogram equalization
        cv::equalizeHist(depth8, depthEq);

        // Threshold pour isoler les objets dans une certaine plage de profondeur
        // Ajustez ces valeurs selon votre terrain de jeu
        cv::inRange(depthEq, 50, 200, mask);

        // Morphological operations pour nettoyer le masque
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

        // Ajouter la frame actuelle au buffer
        frame_buffer.push_back(mask.clone());
        if (frame_buffer.size() > BUFFER_SIZE) {
            frame_buffer.pop_front();
            buffer_full = true;
        }

        // Détection des objets immobiles
        if (buffer_full) {
            // Calculer la stabilité de chaque pixel
            cv::Mat stability_map = cv::Mat::zeros(mask.size(), CV_32FC1);
            
            for (int y = 0; y < mask.rows; y++) {
                for (int x = 0; x < mask.cols; x++) {
                    int stable_count = 0;
                    uchar reference_value = frame_buffer[0].at<uchar>(y, x);
                    
                    // Vérifier si le pixel est stable dans toutes les frames
                    for (size_t i = 0; i < frame_buffer.size(); i++) {
                        if (frame_buffer[i].at<uchar>(y, x) == reference_value) {
                            stable_count++;
                        }
                    }
                    
                    stability_map.at<float>(y, x) = (float)stable_count / frame_buffer.size();
                }
            }

            // Créer un masque pour les objets stables
            cv::Mat temp_stable;
            cv::threshold(stability_map, temp_stable, STABILITY_THRESHOLD, 255, cv::THRESH_BINARY);
            temp_stable.convertTo(stable_objects, CV_8UC1);
            
            // Garder seulement les objets stables qui sont aussi présents dans le masque actuel
            cv::bitwise_and(stable_objects, mask, stable_objects);
        }

        // Filtrer les objets qui ne touchent PAS les bords (objets sur le terrain)
        cv::Mat labels, stats, centroids;
        int num_labels = cv::connectedComponentsWithStats(stable_objects, labels, stats, centroids);

        filtered_mask = cv::Mat::zeros(stable_objects.size(), CV_8UC1);
        int rows = stable_objects.rows;
        int cols = stable_objects.cols;

        // Vérifier chaque composant connecté
        for (int i = 1; i < num_labels; i++) {
            int left = stats.at<int>(i, cv::CC_STAT_LEFT);
            int top = stats.at<int>(i, cv::CC_STAT_TOP);
            int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
            int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            int right = left + width - 1;
            int bottom = top + height - 1;

            bool touches_border = false;

            // Vérifier si l'objet touche les bords
            if (left == 0 || right == cols - 1 || top == 0 || bottom == rows - 1) {
                touches_border = true;
            } else {
                cv::Mat component_mask = (labels == i);

                // Vérifier les bords horizontaux
                for (int x = left; x <= right && !touches_border; x++) {
                    if (component_mask.at<int>(top, x) != 0 ||
                        component_mask.at<int>(bottom, x) != 0) {
                        touches_border = true;
                    }
                }

                // Vérifier les bords verticaux
                for (int y = top; y <= bottom && !touches_border; y++) {
                    if (component_mask.at<int>(y, left) != 0 ||
                        component_mask.at<int>(y, right) != 0) {
                        touches_border = true;
                    }
                }
            }

            // Garder seulement les objets qui NE touchent PAS les bords
            // et qui ont une taille minimale (filtrer le bruit)
            if (!touches_border && area > 100) {
                filtered_mask.setTo(255, labels == i);
            }
        }

        // Visualisation avec les centres des objets
        cv::Mat visualization;
        cv::cvtColor(filtered_mask, visualization, cv::COLOR_GRAY2BGR);

        // Recalculer les composants pour les objets filtrés
        cv::Mat final_labels, final_stats, final_centroids;
        int final_num_labels = cv::connectedComponentsWithStats(filtered_mask, final_labels, final_stats, final_centroids);

        // Afficher le centre de chaque objet immobile détecté
        for (int i = 1; i < final_num_labels; i++) {
            cv::Point2d center(final_centroids.at<double>(i, 0), final_centroids.at<double>(i, 1));
            int area = final_stats.at<int>(i, cv::CC_STAT_AREA);
            
            // Dessiner le centre de l'objet
            cv::circle(visualization, center, 10, cv::Scalar(0, 255, 0), -1);
            
            // Dessiner un cercle autour de l'objet pour mieux le visualiser
            cv::circle(visualization, center, 20, cv::Scalar(255, 0, 0), 2);
            
            // Afficher les coordonnées du centre
            std::cout << "Objet immobile " << i << " - Centre: (" 
                      << (int)center.x << ", " << (int)center.y 
                      << ") Aire: " << area << std::endl;
        }

        // Afficher les résultats
        cv::imshow("Original Frame", depth8);
        cv::imshow("Detected Objects", filtered_mask);
        cv::imshow("Object Centers", visualization);

        // Condition de sortie
        char key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) {
            break;
        }
    }

    depth.stop();
    depth.destroy();
    device.close();
    openni::OpenNI::shutdown();
    return 0;
}