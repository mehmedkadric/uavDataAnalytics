#ifndef UAV_H
#define UAV_H

#include <QWidget>
#include <QFileDialog>
#include "helpdialog.h"
#include "imageprocessing2d.h"
#include "opencv2/opencv.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/xfeatures2d.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/stitching.hpp"



QT_BEGIN_NAMESPACE
namespace Ui { class uav; }
QT_END_NAMESPACE

class uav : public QWidget
{
    Q_OBJECT

public:
    uav(QWidget *parent = nullptr);
    ~uav();
    void setMainWindowTitle();
    std::vector<cv::Point2f> Points(std::vector<cv::KeyPoint> keypoints);

private slots:
    void on_openApplicationPushButton_clicked();
    void on_help_clicked();
    void on_uploadImageButton_clicked();
    void on_runButton_clicked();
    void on_backButton_clicked();
    void on_homepageButton_clicked();
    void on_gotoSelectConfiguration_clicked();
    void on_goToSceneCompare_clicked();
    void on_backToChooseLayer_clicked();
    void on_backToChooseLayer2_clicked();
    void on_upload2Images_clicked();
    void on_findChanges_clicked();
    void on_imageSegmentation_clicked();
    void on_backButton_3_clicked();
    void on_uploadSnapshot_clicked();
    void on_multipleColorsCheckbox_stateChanged(int colorSpaceStatus);
    void on_pathToFolder_clicked();
    void on_startSegmentation_clicked();
    void calculateThresholdsHSV(cv::Mat src);
    void calculateThresholdsRGB(cv::Mat src);    
    void on_goToTemplateMatching_clicked();
    void on_uploadTemplate_clicked();
    void on_uploadPathTemplate_clicked();
    void on_runTemplateMatching_clicked();
    void on_backTemplate_clicked();
    void calculateHSVmasksMultipleColors(cv::Mat pattern);

    int chooseMethod(int colorSpace);
    int findLowerBoundary(cv::Mat, unsigned maxElementIndex, int boundary);
    int findUpperBoundary(cv::Mat, unsigned maxElementIndex, int boundary, bool H = false);
    int findLowerBoundaryVec(std::vector<float>, unsigned maxElementIndex, int boundary);
    int findUpperBoundaryVec(std::vector<float>, unsigned maxElementIndex, int boundary, bool H = false);
    int getMaxAreaContourId(std::vector <std::vector<cv::Point>> contours);

    cv::Mat preprocessImage(cv::Mat img);
    cv::Mat segmentationRGB(cv::Mat img, cv::Mat pattern, QString filepath, QString filename);
    cv::Mat segmentationHSV(cv::Mat img, cv::Mat pattern, QString filepath, QString filename);
    cv::Mat segmentationHSVmultipleColors(cv::Mat img, cv::Mat pattern, QString filepath, QString filename);
    cv::Mat segmentationGRAY(cv::Mat img);

    std::vector<int> findMinMax(std::vector<float> histValues, bool H);


    void on_goToVehicleDetection_clicked();

    void on_backFromVehicleDetection_clicked();

    void on_uploadVehicleImage_clicked();

    void on_runVehicleDetection_clicked();

    void draw_locations(cv::Mat & img, const std::vector< cv::Rect > & locations, const cv::Scalar & color);

private:
    Ui::uav *ui;
    QString filepath, filepathSegmentation, filepathTemplateMatching, vehicleImage;
    std::vector<QString> differentImageRaw;
    cv::Mat maskSegmentation;
    cv::Scalar minHSV = cv::Scalar(0, 0, 0);
    cv::Scalar maxHSV = cv::Scalar(180, 255, 255);
    std::vector<cv::Scalar> minHSVs;
    std::vector<cv::Scalar> maxHSVs;
    std::vector<float> histValuesH, histValuesS, histValuesV;
    cv::Scalar minRGB = cv::Scalar(0, 0, 0);
    cv::Scalar maxRGB = cv::Scalar(255, 255, 255);
    QStringList snapshotFilename;
    QString templateFilename;

};
#endif // UAV_H
