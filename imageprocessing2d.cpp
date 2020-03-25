#include "imageprocessing2d.h"

ImageProcessing2D::ImageProcessing2D(Ui::uav* uiForm, QString filepath, QObject *parent) : QObject(parent)
{
    this->ui = uiForm;
    this->filepath = filepath;
}


QImage ImageProcessing2D::loadImage(){
    /*This function loads and returns an image and make an icon of it in 2D form*/
    QImage image;
    image.load(filepath);
    this->ui->rawImage->setPixmap(QPixmap::fromImage(image.scaled(350, 400, Qt::KeepAspectRatio)));
    return image;
}

cv::Mat ImageProcessing2D::CannyDetector(int low, int high){
    /*Load image as an OpenCV matrix and perform Canny edge detection*/
    cv::Mat imagecvOrig = cv::imread(this->filepath.toUtf8().data());
    cv::Mat cannyOutput, imagecv;
    cv::cvtColor(imagecvOrig, imagecv, cv::COLOR_BGR2GRAY);
    cv::Canny(imagecv, cannyOutput, low, high);
    return cannyOutput;
}

cv::Mat ImageProcessing2D::performFindContour(cv::Mat cannyOutput){
    /*Find and draw contours*/
    std::vector< std::vector<cv::Point> > contours;
    cv::findContours(cannyOutput, contours, cv::noArray(), cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    cv::drawContours(cannyOutput, contours, -1, cv::Scalar::all(255));
    cv::GaussianBlur(cannyOutput, cannyOutput, cv::Size(3,3), 0);
    return cannyOutput;
}

QImage ImageProcessing2D::transformCVtoQImage(cv::Mat cvMat){
    return QImage((const unsigned char*)(cvMat.data), cvMat.cols, cvMat.rows, QImage::Format_Grayscale8);
}

void ImageProcessing2D::finishProcess(QImage img){
    ui->editedImage->setPixmap(QPixmap::fromImage(img.scaled(350, 400, Qt::KeepAspectRatio)));
}

cv::Mat ImageProcessing2D::watershedAlgorithm(){

    cv::Mat src, raw = cv::imread(this->filepath.toUtf8().data()), dstLAB;

    //cv::GaussianBlur(raw, src, cv::Size(5,5), 5);
    src = raw;
    cv::Mat image, resultHSV;
    cv::Mat maskHSV, maskHSVgrass, maskHSVsky;

    cv::cvtColor(src, image, cv::COLOR_BGR2HSV);

    //Setting up min and max values for HSV masks
    cv::Scalar minHSVgrass = cv::Scalar(22, 76, 0);
    cv::Scalar maxHSVgrass = cv::Scalar(96, 150, 169);
    cv::Scalar minHSVsky = cv::Scalar(90, 54, 175);
    cv::Scalar maxHSVsky = cv::Scalar(115, 255, 255);

    cv::inRange(image, minHSVgrass, maxHSVgrass, maskHSVgrass);
    cv::inRange(image, minHSVsky, maxHSVsky, maskHSVsky);

    maskHSV = maskHSVgrass + maskHSVsky;
    cv::dilate(maskHSV, maskHSV, cv::Mat::ones(3, 3, CV_8UC1));
    cv::dilate(maskHSV, maskHSV, cv::Mat::ones(3, 3, CV_8UC1));
    cv::dilate(maskHSV, maskHSV, cv::Mat::ones(3, 3, CV_8UC1));
    cv::dilate(maskHSV, maskHSV, cv::Mat::ones(3, 3, CV_8UC1));
    cv::bitwise_not(maskHSV, maskHSV);

    resultHSV = cv::Mat::zeros(src.rows, src.cols, CV_8UC3);
    cv::bitwise_and(raw, raw, resultHSV, maskHSV);

    cv::Mat gray_image, temp;
    cv::cvtColor(resultHSV, temp, cv::COLOR_HSV2BGR);
    cv::cvtColor(temp, gray_image, cv::COLOR_BGR2GRAY);

    //cv::threshold(gray_image, gray_image, 127, 255, 0);

    //cv::bitwise_and(resultHSV, resultHSV, resultHSV, cannyOutput);
    //cv::cvtColor(resultHSV, resultHSV, cv::COLOR_HSV2BGR);

    //std::vector< std::vector<cv::Point> > contours;
    //cv::findContours(resultHSV, contours, cv::noArray(), cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    //cv::drawContours(resultHSV, contours, -1, cv::Scalar::all(255));
    cv::imshow("resultHSV", resultHSV);
    return resultHSV;
}


