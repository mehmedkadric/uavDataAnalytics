#ifndef IMAGEPROCESSING2D_H
#define IMAGEPROCESSING2D_H

#include <QObject>
#include <QWidget>
#include "uav.h"
#include "ui_uav.h"
#include <opencv2/core/utility.hpp>

using namespace std;

class ImageProcessing2D : public QObject
{
private:
    Ui::uav* ui;
    QString filepath;
public:
    explicit ImageProcessing2D(Ui::uav* uiForm, QString filepath = "", QObject *parent = nullptr);
    QImage loadImage();
    cv::Mat CannyDetector(int low = 10, int high = 250);
    cv::Mat performFindContour(cv::Mat cannyOutput);
    QImage transformCVtoQImage(cv::Mat cvMat);
    void finishProcess(QImage img);
    cv::Mat watershedAlgorithm();


signals:

};

#endif // IMAGEPROCESSING2D_H
