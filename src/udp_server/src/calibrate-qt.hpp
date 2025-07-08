#include <QtGui>
#include <QtWidgets>

// OpenCV includes
#include <opencv2/core.hpp>

class QCalibrationApp : public QMainWindow
{
    public:

        QCalibrationApp(QWidget* parent = nullptr);
        ~QCalibrationApp();

        void setOnDepthFrameChange(std::function<cv::Mat(cv::Mat, int, int)> onDepthFrameChange)
        {
            m_onDepthFrameChange = onDepthFrameChange;
        }

        void setOnRGBFrameChange(std::function<cv::Mat(cv::Mat)> onRGBFrameChange)
        {
            m_onRGBFrameChange = onRGBFrameChange;
        }

        void setPresetName(std::string_view filename);
        void savePresets();
        void loadPresets();


    private:
        friend class QControl;
        struct QCalibrationAppImpl;

        void peek_frame();
        void recompute_homography();
        void onCalibrationMenuChanged(int);

        std::unique_ptr<QCalibrationAppImpl> m_impl;
        std::function<cv::Mat(cv::Mat, int, int)> m_onDepthFrameChange;
        std::function<cv::Mat(cv::Mat)> m_onRGBFrameChange;
};

