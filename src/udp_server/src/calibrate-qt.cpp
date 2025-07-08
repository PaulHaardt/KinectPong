//#include <opencv2/highgui/highgui.hpp>

#include "calibrate-qt.hpp"

#include <QtGui>
#include <QtWidgets>
#include <iostream>
// OpenCV includes

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include "capture-cv.hpp"
#include "calibration-utils.hpp"
#include "utils.hpp"

//using namespace cv;

class QFullscreenView : public QWidget {
    public:

    QFullscreenView(const QImage* image, QWidget* parent = nullptr) : QWidget(parent)
    {
        this->setWindowFlags(Qt::Window);
        this->setWindowState(Qt::WindowFullScreen);

        m_scene = new QGraphicsScene();
        m_item = new QGraphicsPixmapItem();
        if (image != nullptr)
            m_item->setPixmap(QPixmap::fromImage(*image));
        m_scene->addItem(m_item);
        m_view = new QGraphicsView(m_scene);
        m_view->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        QLayout* layout = new QHBoxLayout();
        layout->addWidget(m_view);
        layout->setAlignment(m_view, Qt::AlignLeft);
        layout->setContentsMargins(0, 0, 0, 0);
        this->setLayout(layout);
    }

    void resizeEvent(QResizeEvent* event) override {
        QWidget::resizeEvent(event);
        m_scene->setSceneRect(m_item->boundingRect());
        m_view->resize(this->size());
        m_view->fitInView(m_item, Qt::KeepAspectRatio);
    }

    void setImage(const QImage& image) {
        m_item->setPixmap(QPixmap::fromImage(image));
        m_scene->setSceneRect(m_item->boundingRect());
        m_view->fitInView(m_item, Qt::KeepAspectRatio);
    }


    private:
        QGraphicsScene* m_scene;
        QGraphicsPixmapItem* m_item;
        QGraphicsView* m_view;
};

constexpr static int CONTROL_SIZE = 10;


class QControl : public QGraphicsRectItem
 {
    QCalibrationApp* parent;

    public:
        QControl(QCalibrationApp* parent,  Qt::GlobalColor color, int x, int y) : QGraphicsRectItem(0, 0, CONTROL_SIZE, CONTROL_SIZE) {
            setBrush(QBrush(color));
            setFlags(QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemSendsGeometryChanges);
            this->parent = parent;
            this->setPos(x, y);
        }

        QVariant itemChange(GraphicsItemChange change, const QVariant& value) override {
            if (change == ItemPositionHasChanged) {
                parent->recompute_homography();
            }
            return QGraphicsRectItem::itemChange(change, value);
        }
};

struct QCalibrationApp::QCalibrationAppImpl
{
    QFullscreenView* m_calibration_view = nullptr;
    QFullscreenView* m_output_view = nullptr;

    QGraphicsView* lview;
    QGraphicsView* cview;
    QGraphicsView* rview;
    QGraphicsScene* lscene;
    QGraphicsScene* cscene;
    QGraphicsScene* rscene;
    QGraphicsPixmapItem* rgb;
    QGraphicsPixmapItem* unwrapped;
    QGraphicsPixmapItem* depth;
    QTimer* timer;
    CVKinectCapture capture;
    QCheckBox* m_output_choice;
    QCheckBox* m_output_depth;

    // Control
    std::vector<QControl*> m_control_box;
    std::vector<QControl*> m_control_mire;
    std::vector<QControl*> m_control_depth;

    cv::Mat H1, H2; // Homography matrix

    bool calibrate_depth = false;
    bool mirror_output = false;
    bool saved_requested = false;
    int min_depth, max_depth;
    std::string preset_filename = "calibration.yml";
};

void QCalibrationApp::savePresets()
{
    cv::FileStorage fs(m_impl->preset_filename, cv::FileStorage::WRITE);
    fs.write("H1", m_impl->H1);
    fs.write("H2", m_impl->H2);
    fs.write("min_depth", m_impl->min_depth);
    fs.write("max_depth", m_impl->max_depth);

    cv::Mat points_box(4, 2, CV_32F);
    cv::Mat points_mire(4, 2, CV_32F);
    cv::Mat points_depth(2, 2, CV_32F);
    for (int i = 0; i < 4; ++i)
    {
        points_box.at<float>(i, 0) = m_impl->m_control_box[i]->scenePos().x() + CONTROL_SIZE / 2;
        points_box.at<float>(i, 1) = m_impl->m_control_box[i]->scenePos().y() + CONTROL_SIZE / 2;
        points_mire.at<float>(i, 0) = m_impl->m_control_mire[i]->scenePos().x() + CONTROL_SIZE / 2;
        points_mire.at<float>(i, 1) = m_impl->m_control_mire[i]->scenePos().y() + CONTROL_SIZE / 2;
    }
    for (int i = 0; i < 2; ++i)
    {
        points_depth.at<float>(i, 0) = m_impl->m_control_depth[i]->scenePos().x() + CONTROL_SIZE / 2;
        points_depth.at<float>(i, 1) = m_impl->m_control_depth[i]->scenePos().y() + CONTROL_SIZE / 2;
    }
    fs.write("points_box", points_box);
    fs.write("points_mire", points_mire);
    fs.write("points_depth", points_depth);
}

void QCalibrationApp::loadPresets()
{

    cv::FileStorage fs(m_impl->preset_filename, cv::FileStorage::READ);
    fs["H1"] >> m_impl->H1;
    fs["H2"] >> m_impl->H2;
    fs["min_depth"] >> m_impl->min_depth;
    fs["max_depth"] >> m_impl->max_depth;

    cv::Mat points_box, points_mire, points_depth;
    fs["points_box"] >> points_box;
    fs["points_mire"] >> points_mire;
    fs["points_depth"] >> points_depth;

    for (int i = 0; i < 4; ++i)
    {
        m_impl->m_control_box[i]->setPos(points_box.at<float>(i, 0) - CONTROL_SIZE / 2, points_box.at<float>(i, 1) - CONTROL_SIZE / 2);
        m_impl->m_control_mire[i]->setPos(points_mire.at<float>(i, 0) - CONTROL_SIZE / 2, points_mire.at<float>(i, 1) - CONTROL_SIZE / 2);
    }
    for (int i = 0; i < 2; ++i)
    {
        m_impl->m_control_depth[i]->setPos(points_depth.at<float>(i, 0) - CONTROL_SIZE / 2, points_depth.at<float>(i, 1) - CONTROL_SIZE / 2);
    }
}

void QCalibrationApp::setPresetName(std::string_view filename)
{
    m_impl->preset_filename = filename;
}


void QCalibrationApp::peek_frame()
{
    m_impl->capture.next_loop_event();
}

void QCalibrationApp::recompute_homography()
{
    if (m_impl->rgb->pixmap().isNull())
        return;

    std::vector<cv::Point2f> coordinates_box(4);
    std::vector<cv::Point2f> coordinates_mire(4);

    for (int i = 0; i < 4; ++i)
    {
        coordinates_box[i].x = m_impl->m_control_box[i]->scenePos().x() + CONTROL_SIZE / 2;
        coordinates_box[i].y = m_impl->m_control_box[i]->scenePos().y() + CONTROL_SIZE / 2;
        coordinates_mire[i].x = m_impl->m_control_mire[i]->scenePos().x() + CONTROL_SIZE / 2;
        coordinates_mire[i].y = m_impl->m_control_mire[i]->scenePos().y() + CONTROL_SIZE / 2;
    }
    int w = m_impl->rgb->pixmap().width();
    int h = m_impl->rgb->pixmap().height();

    m_impl->H1 = unwrap_estimate(coordinates_box, w, h);
    auto H2 = unwrap_estimate(coordinates_mire, w, h, m_impl->mirror_output);
    m_impl->H2 = H2 * m_impl->H1.inv();
}

void QCalibrationApp::onCalibrationMenuChanged(int index)
{
    std::for_each(m_impl->m_control_box.begin(), m_impl->m_control_box.end(), [](auto &c) { c->hide(); });
    std::for_each(m_impl->m_control_mire.begin(), m_impl->m_control_mire.end(), [](auto &c) { c->hide(); });
    std::for_each(m_impl->m_control_depth.begin(), m_impl->m_control_depth.end(), [](auto &c) { c->hide(); });
    m_impl->m_calibration_view->hide();

    switch (index)
    {
    case 0:
        std::for_each(m_impl->m_control_box.begin(), m_impl->m_control_box.end(), [](auto &c) { c->show(); });
        break;
    case 1:
        std::for_each(m_impl->m_control_mire.begin(), m_impl->m_control_mire.end(), [](auto &c) { c->show(); });
        m_impl->m_calibration_view->show();
        break;
    case 2:
        std::for_each(m_impl->m_control_depth.begin(), m_impl->m_control_depth.end(), [](auto &c) { c->show(); });
        break;
    }
}

QCalibrationApp::~QCalibrationApp() = default;

QCalibrationApp::QCalibrationApp(QWidget* parent) : QMainWindow(parent)
{
    m_impl = std::make_unique<QCalibrationAppImpl>();
    m_impl->rgb = new QGraphicsPixmapItem();
    m_impl->unwrapped = new QGraphicsPixmapItem();
    m_impl->depth = new QGraphicsPixmapItem();

    m_impl->lview = new QGraphicsView();
    m_impl->cview = new QGraphicsView();
    m_impl->rview = new QGraphicsView();
    m_impl->lscene = new QGraphicsScene();
    m_impl->cscene = new QGraphicsScene();
    m_impl->rscene = new QGraphicsScene();

    m_impl->m_control_box.resize(4);
    m_impl->m_control_box[0] = new QControl(this, Qt::red, 10, 10);
    m_impl->m_control_box[1] = new QControl(this, Qt::red, 630, 10);
    m_impl->m_control_box[2] = new QControl(this, Qt::red, 630, 470);
    m_impl->m_control_box[3] = new QControl(this, Qt::red, 10, 470);
    
    m_impl->m_control_mire.resize(4);
    m_impl->m_control_mire[0] = new QControl(this, Qt::green, 10, 10);
    m_impl->m_control_mire[1] = new QControl(this, Qt::green, 630, 10);
    m_impl->m_control_mire[2] = new QControl(this, Qt::green, 630, 470);
    m_impl->m_control_mire[3] = new QControl(this, Qt::green, 10, 470);

    m_impl->m_control_depth.resize(2);
    m_impl->m_control_depth[0] = new QControl(this, Qt::blue, 50, 230);
    m_impl->m_control_depth[1] = new QControl(this, Qt::blue, 590, 230);

    m_impl->lscene->addItem(m_impl->rgb);
    m_impl->cscene->addItem(m_impl->unwrapped);
    m_impl->rscene->addItem(m_impl->depth);

    for (auto& controlers : {m_impl->m_control_box, m_impl->m_control_mire, m_impl->m_control_depth})
        for (auto& c : controlers) {
            m_impl->lscene->addItem(c);
            c->hide();
        }

    m_impl->lview->setScene(m_impl->lscene);
    m_impl->cview->setScene(m_impl->cscene);
    m_impl->rview->setScene(m_impl->rscene);

    m_impl->lview->setMinimumSize(640, 480);
    m_impl->cview->setMinimumSize(640, 480);
    m_impl->rview->setMinimumSize(640, 480);

    auto central = new QWidget();
    auto layout = new QHBoxLayout(central);
    layout->addWidget(m_impl->lview);
    layout->addWidget(m_impl->cview);
    layout->addWidget(m_impl->rview);
    this->setCentralWidget(central);



    m_impl->capture.set_rgb_callback([&](cv::Mat& input, uint32_t timestamp) {
        QImage image(input.data, input.cols, input.rows, input.step, QImage::Format_RGB888);
        m_impl->rgb->setPixmap(QPixmap::fromImage(image));
        m_impl->lscene->setSceneRect(image.rect());

        cv::Mat W = (m_impl->H1.empty()) ? input : unwrap(input, m_impl->H1);
        cv::Mat transformed = (m_onRGBFrameChange) ? m_onRGBFrameChange(W) : W;
        cv::Mat output = (m_impl->H2.empty()) ? transformed : unwrap(transformed, m_impl->H2);
        {
            auto& to_disp = m_impl->m_output_choice->isChecked() ? output : transformed;
            QImage unwrapped_image(to_disp.data, to_disp.cols, to_disp.rows, to_disp.step, QImage::Format_RGB888);
            m_impl->unwrapped->setPixmap(QPixmap::fromImage(unwrapped_image));
            m_impl->cscene->setSceneRect(unwrapped_image.rect());
        }
        if (!m_impl->m_output_depth->isChecked())
        {
            QImage image(output.data, output.cols, output.rows, output.step, QImage::Format_RGB888);
            m_impl->m_output_view->setImage(image);
        }
    });

    m_impl->capture.set_depth_callback([&](cv::Mat& depth, uint32_t timestamp) {

        if (!m_onDepthFrameChange)
            return;

        auto depth16 = (cv::Mat_<uint16_t>)depth;


        if (m_impl->calibrate_depth)
        {
            m_impl->calibrate_depth = false;
            auto p1 = m_impl->m_control_depth[0]->scenePos();
            auto p2 = m_impl->m_control_depth[1]->scenePos();
            auto q1 = cv::Point2f(p1.x() + CONTROL_SIZE / 2, p1.y() + CONTROL_SIZE / 2);
            auto q2 = cv::Point2f(p2.x() + CONTROL_SIZE / 2, p2.y() + CONTROL_SIZE / 2);
            uint16_t d1 = depth16.at<uint16_t>(q1);
            uint16_t d2 = depth16.at<uint16_t>(q2);
            std::tie(m_impl->min_depth, m_impl->max_depth) = std::minmax(d1, d2);
            std::cout << "Depth calibration: " << m_impl->min_depth << " " << m_impl->max_depth << std::endl;
        }

        // 1. Wrap with H1
        cv::Mat W = (m_impl->H1.empty()) ? depth : unwrap(depth, m_impl->H1);

        if (m_impl->saved_requested)
        {
            m_impl->saved_requested = false;
            cv::imwrite("output.png", W);
        }

        cv::Mat depth_rgb = m_onDepthFrameChange(W, m_impl->min_depth, m_impl->max_depth);
        cv::Mat out = (m_impl->H2.empty()) ? depth_rgb : unwrap(depth_rgb, m_impl->H2);


        {
            QImage image(depth_rgb.data, depth_rgb.cols, depth_rgb.rows, depth_rgb.step, QImage::Format_RGB888);
            m_impl->depth->setPixmap(QPixmap::fromImage(image));
            m_impl->rscene->setSceneRect(image.rect());
        }

        if (m_impl->m_output_depth->isChecked())
        {
            QImage image(out.data, out.cols, out.rows, out.step, QImage::Format_RGB888);
            m_impl->m_output_view->setImage(image);
        }
    });

    QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &QCalibrationApp::peek_frame);
    timer->start(10);

    QToolBar *toolbar = this->addToolBar("Calibration");

    QComboBox* calibrartion_menu = new QComboBox();
    calibrartion_menu->addItem("Box");
    calibrartion_menu->addItem("Mire");
    calibrartion_menu->addItem("Depth");
    m_impl->m_output_choice = new QCheckBox("Real output");
    m_impl->m_output_depth = new QCheckBox("Output Depth Map");
    auto zoom_slider = new QSlider(Qt::Horizontal);
    zoom_slider->setMinimum(1);
    zoom_slider->setMaximum(5);
    zoom_slider->setMaximumWidth(100);


    // Depth calibration button
    auto depth_calibration_button = new QPushButton("Calibrate Depth");
    // Mirror the output
    auto mirror_button = new QCheckBox("Mirror");
    // Save presets button
    auto save_presets_button = new QPushButton("Save Presets");
    // Load presets button
    auto load_presets_button = new QPushButton("Load Presets");
    // Save output button
    auto save_output_button = new QPushButton("Save Output");



    toolbar->addWidget(m_impl->m_output_choice);
    toolbar->addWidget(m_impl->m_output_depth);
    toolbar->addWidget(calibrartion_menu);
    toolbar->addWidget(zoom_slider); 
    toolbar->addWidget(depth_calibration_button);
    toolbar->addWidget(mirror_button);
    toolbar->addWidget(save_presets_button);
    toolbar->addWidget(load_presets_button);
    toolbar->addWidget(save_output_button);



    connect(calibrartion_menu, &QComboBox::currentIndexChanged, this, &QCalibrationApp::onCalibrationMenuChanged);
    connect(zoom_slider, &QSlider::valueChanged, [view=m_impl->lview](int value) {
        view->resetTransform();
        view->scale(value, value);
    });
    connect(depth_calibration_button, &QPushButton::clicked, [this]() {
        m_impl->calibrate_depth = true;
    });
    connect(mirror_button, &QCheckBox::toggled, [this](bool checked) {
        m_impl->mirror_output = checked;
        recompute_homography();
    });
    connect(save_presets_button, &QPushButton::clicked, this, (void(QCalibrationApp::*)()) &QCalibrationApp::savePresets);
    connect(load_presets_button, &QPushButton::clicked, this, (void(QCalibrationApp::*)()) &QCalibrationApp::loadPresets);
    connect(save_output_button, &QPushButton::clicked, [this]() { this->m_impl->saved_requested = true; });


    m_impl->capture.start();

    // Display the calibration image on the second screen
    auto calibration_image = get_calibration_image(640, 480);
    m_impl->m_calibration_view = new QFullscreenView(&calibration_image, this);
    m_impl->m_calibration_view->move(QGuiApplication::screens().last()->geometry().topLeft());
    m_impl->m_calibration_view->hide();

    m_impl->m_output_view = new QFullscreenView(nullptr, this);
    m_impl->m_output_view->move(QGuiApplication::screens().last()->geometry().topLeft());
    m_impl->m_output_view->show();
}



