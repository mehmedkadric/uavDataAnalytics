#include "uav.h"
#include "ui_uav.h"
#include "pointcloudanalyzer.h"
#include <iostream>
#include "opencv2/opencv.hpp"
#include "hsvseparator.h"


/*
 * Index 0: homepage
 * Index 1: landing page
 * Index 2: imgSegmentation
 * Index 3: configuration2D
 * Index 4: result2D
 * Index 5: sceneCompare
 * Index 6: templateMatching
 * */

static int pageHomepage = 0;
static int pageLandingpage = 1;
static int pageImgSegmentation = 2;
static int pageConfiguration2D = 3;
static int pageResult2D = 4;
static int pageSceneCompare = 5;
static int pageTemplateMatching = 6;
static int pageVehicleDetection = 7;



uav::uav(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::uav)
{
    ui->setupUi(this);
    ui->selectAlgorithmComboBox->addItem("Canny edge detection");
    ui->selectAlgorithmComboBox->addItem("Find countour");
    ui->selectAlgorithmComboBox->addItem("Thresholding");

    connect(ui->openApplicationPushButton, SIGNAL(clicked()), this, SLOT(on_openApplicationPushButton_clicked()));
    connect(ui->runButton, SIGNAL(clicked()), this, SLOT(on_runButton_clicked()));
    connect(ui->backButton, SIGNAL(clicked()), this, SLOT(on_backButton_clicked()));
    connect(ui->homepageButton, SIGNAL(clicked()), this, SLOT(on_homepageButton_clicked()));
    connect(ui->goToSceneCompare, SIGNAL(clicked()), this, SLOT(on_goToSceneCompare_clicked()));
    connect(ui->gotoSelectConfiguration, SIGNAL(clicked()), this, SLOT(on_gotoSelectConfiguration_clicked()));
}


uav::~uav()
{
    delete ui;
}


void uav::on_uploadImageButton_clicked()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("Choose"), "", tr("Images (*.png *.jpg *.jpeg *.bmp *.gif)"));

    if(QString::compare(filename, QString()) != 0){
        QImage image;
        bool valid = image.load(filename);

        if(valid){
            this->filepath = filename;
            ui->uploadedImagePath->setText(filename);
            ui->imagePreview->setPixmap(QPixmap::fromImage(image.scaled(100, 100, Qt::KeepAspectRatio)));
        }
        else{
            ui->statusLabel->setText("Image can not be loaded!");
        }
    }
}


void uav::on_runButton_clicked()
{
    if(ui->uploadedImagePath->text() == "")
        ui->statusLabel->setText("Image is not loaded!");
    /*Check if 2D radio button is clicked*/
    else if(ui->radioButton2D->isChecked() && !ui->radioButton3D->isChecked()){
        ui->stackedWidget->setCurrentIndex(pageResult2D);
        ui->statusLabel->setText("");

        ImageProcessing2D image2d(ui, this->filepath);

        //load and return an image and make an icon of it in 2D form
        QImage img2d = image2d.loadImage();
        cv::Mat output;
        /*Check if Canny or Find countour algorithm is selected*/
        if(ui->selectAlgorithmComboBox->currentIndex() == 0 || ui->selectAlgorithmComboBox->currentIndex() == 1){
            int low = 10, high = 250;
            output = image2d.CannyDetector(low, high);

            /*Check if Find countour is selected*/
            if(ui->selectAlgorithmComboBox->currentIndex() == 1){
                output = image2d.performFindContour(output);
            }
        }
        else if(ui->selectAlgorithmComboBox->currentIndex() == 2){
            output = image2d.watershedAlgorithm();
        }
        //Transforms Grayscale CV image to QImage and finish the process
        QImage img = image2d.transformCVtoQImage(output);
        image2d.finishProcess(img);
    }
    else if(!ui->radioButton2D->isChecked() && ui->radioButton3D->isChecked()){
        ui->statusLabel->setText("3D analysis is temporary unavailable.");
    }
    else{
        ui->statusLabel->setText("Invalid input!");
    }

}


void uav::on_upload2Images_clicked()
{
    int y = 1;
    QStringList filename = QFileDialog::getOpenFileNames(this,tr("Open File"), tr("Images (*.png *.jpg *.jpeg *.bmp *.gif)"));
    //std::vector<QImage> differentImageRaw;
    if (!filename.isEmpty()){
        if(filename.length() != 2)
            ui->uploadStatus->setText("Choose two images!");
        else{
            QStringList f0 = filename.at(0).split("/");
            QStringList f1 = filename.at(1).split("/");

            QString img0 = f0.at(f0.length()-1);
            QString img1 = f1.at(f1.length()-1);

            ui->uploadStatus->setText(img0 + "\n" + img1);
            for(int i=0; i<filename.length(); i++){
                differentImageRaw.push_back(filename.at(i));
            }
        }
    }
}


void uav::on_findChanges_clicked()
{
    if(differentImageRaw.size() != 2)
        ui->uploadStatus->setText("Invalid input!");

    else{
        cv::Mat im1 = cv::imread(this->differentImageRaw.at(0).toUtf8().data());
        cv::Mat im2 = cv::imread(this->differentImageRaw.at(1).toUtf8().data());
        differentImageRaw.clear();

        //cv::resize(img1, img1, cv::Size(), 0.75, 0.75);
        //cv::resize(img2, img2, cv::Size(), 0.75, 0.75);

        cv::GaussianBlur(im1, im1, cv::Size(3,3), 0.05);
        cv::GaussianBlur(im2, im2, cv::Size(3,3), 0.05);

        //cv::Mat imgOrig1 = img1, imgOrig2 = img2;
        cv::Mat im1_gray, im2_gray;
        cv::cvtColor(im1, im1_gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(im2, im2_gray, cv::COLOR_BGR2GRAY);

        if(ui->eec->isChecked()){

            const int warp_mode = cv::MOTION_EUCLIDEAN;

            // Set a 2x3 or 3x3 warp matrix depending on the motion model.
            cv::Mat warp_matrix;

            // Initialize the matrix to identity
            if ( warp_mode == cv::MOTION_HOMOGRAPHY )
                warp_matrix = cv::Mat::eye(3, 3, CV_32F);
            else
                warp_matrix = cv::Mat::eye(2, 3, CV_32F);

            // Specify the number of iterations.
            int number_of_iterations = 5;

            // Specify the threshold of the increment
            // in the correlation coefficient between two iterations
            double termination_eps = 1e-10;

            // Define termination criteria
            cv::TermCriteria criteria (cv::TermCriteria::COUNT + cv::TermCriteria::EPS, number_of_iterations, termination_eps);

            // Run the ECC algorithm. The results are stored in warp_matrix.
            cv::findTransformECC(
                        im1_gray,
                        im2_gray,
                        warp_matrix,
                        warp_mode,
                        criteria
                        );

            // Storage for warped image.
            cv::Mat im2_aligned;

            if (warp_mode != cv::MOTION_HOMOGRAPHY){
                // Use warpAffine for Translation, Euclidean and Affine
                //cv::warpAffine(im2, im2_aligned, warp_matrix, im1.size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);
                // Show final result
                cv::Mat imgOut, im2_gray_aligned, imgOut2, warp_matrix2;
                //cv::absdiff(im1, im2_aligned, imgOut);

                cv::warpAffine(im2_gray, im2_gray_aligned, warp_matrix, im1_gray.size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);

                cv::absdiff(im1_gray, im2_gray_aligned, imgOut2);
                cv::Mat finalIm2 = cv::Mat::zeros(im2.rows, im2.cols, CV_8UC3);

                //cv::imshow("Result (pre)", imgOut2);
                cv::threshold(imgOut2, imgOut2, 50, 255, cv::THRESH_BINARY);
                cv::erode(imgOut2, imgOut2, cv::Mat::ones(3, 3, CV_8UC1));

                cv::bitwise_and(im2, im2, finalIm2, imgOut2);
                cv::imshow("Image 1", im1);
                cv::imshow("Image 2", im2);
                cv::imshow("Result", finalIm2);
            }
            else
                // Use warpPerspective for Homography
                cv::warpPerspective (im2, im2_aligned, warp_matrix, im1.size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);
        }
        else if(ui->surf->isChecked()){

            int minHessian = 400;
            //create SURF object
            cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(minHessian);

            std::vector<cv::KeyPoint> keypoints1, keypoints2;
            cv::Mat descriptors1, descriptors2;

            detector->detectAndCompute( im1, cv::noArray(), keypoints1, descriptors1 );
            detector->detectAndCompute( im2, cv::noArray(), keypoints2, descriptors2 );

            //SURF
            cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);

            std::vector< std::vector<cv::DMatch> > knn_matches;
            matcher->knnMatch(descriptors1, descriptors2, knn_matches, 2);

            //Filter matches using the Lowe's ratio test
            const float ratio_thresh = 0.75f;
            std::vector<cv::DMatch> goodMatches;

            for (size_t i = 0; i < knn_matches.size(); i++)
            {
                if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
                {
                    goodMatches.push_back(knn_matches[i][0]);
                }
            }

            //Draw matches
            cv::Mat imgMatches;
            cv::drawMatches( im1_gray, keypoints1, im2_gray, keypoints2, goodMatches, imgMatches, cv::Scalar::all(-1),
                             cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

            //Localize the object
            std::vector<cv::Point2f> kp1;
            std::vector<cv::Point2f> kp2;
            for( size_t i = 0; i < goodMatches.size(); i++ ){
                //Get the keypoints from the good matches
                kp1.push_back( keypoints1[unsigned(goodMatches[i].queryIdx) ].pt );
                kp2.push_back( keypoints2[unsigned(goodMatches[i].trainIdx) ].pt );
            }

            //Finding Homography
            cv::Mat H = findHomography(kp1, kp2, cv::RANSAC);

            //Show detected matches
            //cv::imshow("Good Matches & Object detection", imgMatches );

            cv::Mat img2Transformed, imOut2;
            //Warp source image to destination based on homography
            cv::warpPerspective(im2_gray, img2Transformed, H, im1.size());

            cv::absdiff(im1_gray, img2Transformed, imOut2);
            cv::Mat finalIm2 = cv::Mat::zeros(im2.rows, im2.cols, CV_8UC3);

            //cv::imshow("Result (pre)", imgOut2);
            cv::threshold(imOut2, imOut2, 35, 255, cv::THRESH_BINARY);
            cv::erode(imOut2, imOut2, cv::Mat::ones(3, 3, CV_8UC1));

            cv::bitwise_and(im2, im2, finalIm2, imOut2);
            cv::imshow("Image 1", im1);
            cv::imshow("Image 2", im2);
            cv::imshow("Result", finalIm2);
        }
    }
}


void uav::on_backButton_clicked()
{
    ui->stackedWidget->setCurrentIndex(pageConfiguration2D);
}


void uav::on_homepageButton_clicked()
{
    ui->stackedWidget->setCurrentIndex(pageHomepage);
}


void uav::on_gotoSelectConfiguration_clicked()
{
    ui->stackedWidget->setCurrentIndex(pageConfiguration2D);
}


void uav::on_goToSceneCompare_clicked()
{
    ui->stackedWidget->setCurrentIndex(pageSceneCompare);
}


void uav::on_backToChooseLayer_clicked()
{
    ui->stackedWidget->setCurrentIndex(pageLandingpage);
}


void uav::on_backToChooseLayer2_clicked()
{
    ui->stackedWidget->setCurrentIndex(pageLandingpage);
}


void uav::on_openApplicationPushButton_clicked()
{
    ui->stackedWidget->setCurrentIndex(pageLandingpage);
}


void uav::on_backFromVehicleDetection_clicked()
{
    ui->stackedWidget->setCurrentIndex(pageLandingpage);
}


void uav::setMainWindowTitle(){
    this->setWindowTitle("Semantic Classification and Object Segmentation");
}


void uav::on_help_clicked()
{
    helpDialog h;
    h.setModal(true);
    h.exec();
}


void uav::on_imageSegmentation_clicked()
{
    ui->stackedWidget->setCurrentIndex(pageImgSegmentation);
}


void uav::on_goToVehicleDetection_clicked()
{
    ui->stackedWidget->setCurrentIndex(pageVehicleDetection);
}



void uav::on_backButton_3_clicked()
{
    ui->stackedWidget->setCurrentIndex(pageLandingpage);
}

void uav::on_goToTemplateMatching_clicked()
{
    ui->stackedWidget->setCurrentIndex(pageTemplateMatching);
}


void uav::on_backTemplate_clicked()
{
    ui->stackedWidget->setCurrentIndex(pageLandingpage);
}


void uav::calculateThresholdsHSV(cv::Mat src){
    cv::Mat hsv;
    cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);

    cv::Mat hsvMatrix[3], H, S, V;
    cv::split(hsv, hsvMatrix);

    H = hsvMatrix[0];
    S = hsvMatrix[1];
    V = hsvMatrix[2];

    cv::dilate(H, H, cv::Mat::ones(5, 5, CV_8UC1));
    cv::dilate(H, H, cv::Mat::ones(5, 5, CV_8UC1));
    cv::erode(H, H, cv::Mat::ones(5, 5, CV_8UC1));
    cv::erode(H, H, cv::Mat::ones(5, 5, CV_8UC1));
    cv::GaussianBlur(H, H, cv::Size(15,15), 10);


    int histSizeH = 180, histSize = 255;

    // hue varies from 0 to 179
    float hranges[] = { 0, 180 };
    const float* histRangeH = { hranges };
    float ranges[] = { 0, 256 };
    const float* histRanges = { ranges };

    cv::Mat histH, histS, histV;

    cv::calcHist( &H, 1, nullptr, cv::Mat(), histH, 1, &histSizeH, &histRangeH, true, false );
    cv::calcHist( &S, 1, nullptr, cv::Mat(), histS, 1, &histSize, &histRanges, true, false );
    cv::calcHist( &V, 1, nullptr, cv::Mat(), histV, 1, &histSize, &histRanges, true, false );

    std::vector<float> histValuesH, histValuesS, histValuesV;


    for(int i = 0; i < histH.rows; i++){
        histValuesH.push_back(histH.at<float>(i,0));
        //std::cout<<"H: "<<histH.at<float>(i,0)<<std::endl;
    }
    for(int i = 0; i < histS.rows; i++)
        histValuesS.push_back(histS.at<float>(i,0));
    for(int i = 0; i < histV.rows; i++)
        histValuesV.push_back(histV.at<float>(i,0));


    this->histValuesH = histValuesH;
    this->histValuesS = histValuesS;
    this->histValuesV = histValuesV;


    unsigned maxElementIndexH = unsigned(std::max_element(histValuesH.begin(),histValuesH.end()) - histValuesH.begin());
    unsigned maxElementH = unsigned(*std::max_element(histValuesH.begin(), histValuesH.end()));

    unsigned maxElementIndexS = unsigned(std::max_element(histValuesS.begin(),histValuesS.end()) - histValuesS.begin());
    unsigned maxElementS = unsigned(*std::max_element(histValuesS.begin(), histValuesS.end()));

    unsigned maxElementIndexV = unsigned(std::max_element(histValuesV.begin(),histValuesV.end()) - histValuesV.begin());
    unsigned maxElementV = unsigned(*std::max_element(histValuesV.begin(), histValuesV.end()));

    int boundaryH = int(0*maxElementH);
    int boundaryS = int(0*maxElementS);
    int boundaryV = int(0*maxElementV);

    int lowerBoundaryH, upperBoundaryH;
    int lowerBoundaryS, upperBoundaryS;
    int lowerBoundaryV, upperBoundaryV;

    lowerBoundaryH = findLowerBoundary(histH,maxElementIndexH, boundaryH);
    upperBoundaryH = findUpperBoundary(histH,maxElementIndexH, boundaryH, true);

    lowerBoundaryS = findLowerBoundary(histS,maxElementIndexS, boundaryS);
    upperBoundaryS = findUpperBoundary(histS,maxElementIndexS, boundaryS);

    lowerBoundaryV = findLowerBoundary(histV,maxElementIndexV, boundaryV);
    upperBoundaryV = findUpperBoundary(histV,maxElementIndexV, boundaryV);


    cv::Scalar minHSV = cv::Scalar(lowerBoundaryH + 1, lowerBoundaryS + 1, 0);
    cv::Scalar maxHSV = cv::Scalar(upperBoundaryH + 1, upperBoundaryS + 1, 255);

    /*
    cv::Scalar minHSV = cv::Scalar(lowerBoundaryH + 1, lowerBoundaryS + 1, lowerBoundaryV + 1);
    cv::Scalar maxHSV = cv::Scalar(upperBoundaryH + 1, upperBoundaryS + 1, upperBoundaryV + 1);


    cv::Scalar minHSV = cv::Scalar(lowerBoundaryH + 1, 0, lowerBoundaryV + 1);
    cv::Scalar maxHSV = cv::Scalar(upperBoundaryH + 1, 255, upperBoundaryV + 1);
    */


    this->minHSV = minHSV;
    this->maxHSV = maxHSV;
}


void uav::calculateThresholdsRGB(cv::Mat src){
    cv::Mat rgbMatrix[3], R, G, B;
    cv::split(src, rgbMatrix);

    B = rgbMatrix[0];
    G = rgbMatrix[1];
    R = rgbMatrix[2];

    int histSize = 255;

    float ranges[] = { 0, 256 };
    const float* histRanges = { ranges };

    cv::Mat histR, histG, histB;

    cv::calcHist( &R, 1, nullptr, cv::Mat(), histR, 1, &histSize, &histRanges, true, false );
    cv::calcHist( &G, 1, nullptr, cv::Mat(), histG, 1, &histSize, &histRanges, true, false );
    cv::calcHist( &B, 1, nullptr, cv::Mat(), histB, 1, &histSize, &histRanges, true, false );

    std::vector<float> histValuesR, histValuesG, histValuesB;

    for(int i = 0; i < histR.rows; i++)
        histValuesR.push_back(histR.at<float>(i,0));
    for(int i = 0; i < histG.rows; i++)
        histValuesG.push_back(histG.at<float>(i,0));
    for(int i = 0; i < histB.rows; i++)
        histValuesB.push_back(histB.at<float>(i,0));


    unsigned maxElementIndexR = unsigned(std::max_element(histValuesR.begin(), histValuesR.end()) - histValuesR.begin());
    unsigned maxElementR = unsigned(*std::max_element(histValuesR.begin(), histValuesR.end()));

    unsigned maxElementIndexG = unsigned(std::max_element(histValuesG.begin(), histValuesG.end()) - histValuesG.begin());
    unsigned maxElementG = unsigned(*std::max_element(histValuesG.begin(), histValuesG.end()));

    unsigned maxElementIndexB = unsigned(std::max_element(histValuesB.begin(), histValuesB.end()) - histValuesB.begin());
    unsigned maxElementB = unsigned(*std::max_element(histValuesB.begin(), histValuesB.end()));

    int boundaryR = int(0*maxElementR);
    int boundaryG = int(0*maxElementG);
    int boundaryB = int(0*maxElementB);
    /*
    int boundaryR = int(0.02*maxElementR);
    int boundaryG = int(0.02*maxElementG);
    int boundaryB = int(0.02*maxElementB);
    */

    int lowerBoundaryR, upperBoundaryR;
    int lowerBoundaryG, upperBoundaryG;
    int lowerBoundaryB, upperBoundaryB;

    lowerBoundaryR = findLowerBoundary(histR, maxElementIndexR, boundaryR);
    upperBoundaryR = findUpperBoundary(histR, maxElementIndexR, boundaryR);

    lowerBoundaryG = findLowerBoundary(histG, maxElementIndexG, boundaryG);
    upperBoundaryG = findUpperBoundary(histG, maxElementIndexG, boundaryG);

    lowerBoundaryB = findLowerBoundary(histB, maxElementIndexB, boundaryB);
    upperBoundaryB = findUpperBoundary(histB, maxElementIndexB, boundaryB);

    //Setting up min and max values for RGB mask
    cv::Scalar minRGB = cv::Scalar(lowerBoundaryR + 1, lowerBoundaryG + 1, lowerBoundaryB + 1);
    cv::Scalar maxRGB = cv::Scalar(upperBoundaryR + 1, upperBoundaryG + 1, upperBoundaryB + 1);

    this->minRGB = minRGB;
    this->maxRGB = maxRGB;
}


void uav::on_uploadSnapshot_clicked()
{
    QStringList filename = QFileDialog::getOpenFileNames(this,tr("Open File"), tr("Images (*.png *.jpg *.jpeg *.bmp *.gif)"));
    //std::vector<QImage> differentImageRaw;
    if (!filename.isEmpty()){
        if(filename.length() != 1)
            ui->uploadStatus2->setText("Choose only one image!");
        else{
            QStringList f0 = filename.at(0).split("/");
            QString img0 = f0.at(f0.length()-1);

            ui->uploadStatus2->setText(img0);
            this->snapshotFilename = filename;

            cv::Mat img = cv::imread(filename.at(0).toUtf8().data());
            cv::GaussianBlur( img, img, cv::Size( 7, 7 ), 0, 0 );
            //cv::imshow("Output image 2", img);

            QImage qimage;
            bool valid = qimage.load(filename.at(0).toUtf8().data());

            if(valid){
                ui->histogram->setPixmap(QPixmap::fromImage(qimage.scaled(200, 200, Qt::KeepAspectRatio)));
            }



        }
    }
}


int uav::getMaxAreaContourId(std::vector <std::vector<cv::Point>> contours) {
    double maxArea = 0;
    int maxAreaContourId = -1;
    for (unsigned j = 0; j < contours.size(); j++) {
        double newArea = cv::contourArea(contours.at(j));
        if (newArea > maxArea) {
            maxArea = newArea;
            maxAreaContourId = int(j);
        }
    }
    return maxAreaContourId;
}


int uav::findLowerBoundary(cv::Mat histH, unsigned maxElementIndex, int boundary){
    unsigned i = maxElementIndex;
    unsigned lowerBoundary = 255;
    while(histH.at<float>(int(i),0) > boundary){
        if(i == 0){
            lowerBoundary = 0;
            break;
        }
        else{
            lowerBoundary = i;
            i--;
        }
    }
    return int(lowerBoundary);
}


int uav::findLowerBoundaryVec(std::vector<float> histH, unsigned maxElementIndex, int boundary){
    unsigned i = maxElementIndex;
    unsigned lowerBoundary = 255;
    while(histH[i] > boundary){
        if(i == 0){
            lowerBoundary = 0;
            break;
        }
        else{
            lowerBoundary = i;
            i--;
        }
    }
    return int(lowerBoundary);
}


int uav::findUpperBoundary(cv::Mat histH, unsigned maxElementIndex, int boundary, bool H){
    unsigned i = maxElementIndex;
    unsigned upperBoundary = 0;
    while(histH.at<float>(int(i),0) > boundary){
        if(i == 179 && H){
            upperBoundary = 179;
            break;
        }
        else if(i == 255 && !H){
            upperBoundary = 255;
            break;
        }
        else{
            upperBoundary = i;
            i++;
        }
    }
    return int(upperBoundary);
}


int uav::findUpperBoundaryVec(std::vector<float> histH, unsigned maxElementIndex, int boundary, bool H){
    unsigned i = maxElementIndex;
    unsigned upperBoundary = 0;
    while(histH[i] > boundary){
        if(i == 179 && H){
            upperBoundary = 179;
            break;
        }
        else if(i == 255 && !H){
            upperBoundary = 255;
            break;
        }

        else{
            upperBoundary = i;
            i++;
        }
    }
    return int(upperBoundary);
}


void uav::on_pathToFolder_clicked()
{
    QString filepath = QFileDialog::getExistingDirectory(this, "Get Any File");
    if (!filepath.isEmpty()){
        ui->pathToFolderLabel->setText(filepath);
        this->filepathSegmentation = filepath;
    }
}


int uav::chooseMethod(int colorSpace){
    int selectedMethod;
    if(colorSpace == 1 && ui->multipleColorsCheckbox->isChecked())
        selectedMethod = 3; //RGB (multiple colors)
    else if(colorSpace == 0 && ui->multipleColorsCheckbox->isChecked())
        selectedMethod = 2; //HSV (multiple colors)
    else if(colorSpace == 0)
        selectedMethod = 0; //HSV
    else if(colorSpace == 1)
        selectedMethod = 1; //RGB
    else if(colorSpace == 2)
        selectedMethod = 4; //GRAY
    else
        selectedMethod = 0;
    return selectedMethod;
}


cv::Mat uav::segmentationRGB(cv::Mat img, cv::Mat pattern, QString filepath, QString filename){
    cv::Mat image, mask, maskEpsilon;
    image = img;
    calculateThresholdsRGB(pattern);
    cv::inRange(image, this->minRGB, this->maxRGB, mask);
    //cv::imshow("Mask RGB", mask);
    cv::dilate(mask, mask, cv::Mat::ones(5, 5, CV_8UC1));
    //cv::dilate(mask, mask, cv::Mat::ones(3, 3, CV_8UC1));
    //cv::imshow("Mask RGB (after dilation)", mask);
    cv::erode(mask, mask, cv::Mat::ones(5, 5, CV_8UC1));
    //cv::imshow("Mask RGB (after erodion)", mask);

    cv::Mat resultRGB = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);
    cv::Mat originalImg = cv::imread((filepath + "/" + filename).toUtf8().data());
    cv::bitwise_and(originalImg, originalImg, resultRGB, mask);
    return resultRGB;
}

cv::Mat uav::segmentationHSV(cv::Mat img, cv::Mat pattern, QString filepath, QString filename){
    cv::Mat image, mask;
    cv::cvtColor(img, image, cv::COLOR_BGR2HSV);
    calculateThresholdsHSV(pattern);
    cv::inRange(image, this->minHSV, this->maxHSV, mask);

    //Edition
    cv::Mat image2;
    cv::cvtColor(img, image2, cv::COLOR_BGR2GRAY);
    double thresholdValue = threshold(image2, image2, 0, 255, cv::THRESH_BINARY+cv::THRESH_OTSU);
    cv::Mat mask2;
    cv::threshold(image2, mask2, thresholdValue, 255, 0);
    //End

    cv::Mat finalMask = mask;// + mask2;
    //cv::bitwise_not(finalMask, finalMask);
    //cv::imshow("Mask Final (after)", finalMask);
    //cv::imshow("Mask img (before morphology)", mask);
    cv::dilate(finalMask, finalMask, cv::Mat::ones(5, 5, CV_8UC1));
    cv::dilate(finalMask, finalMask, cv::Mat::ones(5, 5, CV_8UC1));
    cv::dilate(finalMask, finalMask, cv::Mat::ones(5, 5, CV_8UC1));
    cv::erode(finalMask, finalMask, cv::Mat::ones(5, 5, CV_8UC1));
    cv::erode(finalMask, finalMask, cv::Mat::ones(5, 5, CV_8UC1));
    cv::erode(finalMask, finalMask, cv::Mat::ones(5, 5, CV_8UC1));

    cv::Mat resultHSV = cv::Mat::zeros(img.rows, img.cols, CV_8UC3);
    cv::Mat originalImg = cv::imread((filepath + "/" + filename).toUtf8().data());
    cv::bitwise_and(originalImg, originalImg, resultHSV, mask);
    cv::imshow("Result HSV", resultHSV);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours( mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    cv::RNG rng(12345);
    //cv::Mat drawing = cv::Mat::zeros( mask.size(), CV_8UC3 );
    cv::Mat drawing = originalImg;

    double maxArea = 1010;
    for( unsigned i = 0; i<contours.size(); i++ ){
        if (hierarchy[i][3] == -1)
            continue;
        double area = cv::contourArea(contours[i]);
        //std::cout<<"Area: "<< area << std::endl;

        cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );

        if (area > maxArea){
            //cv::drawContours( drawing, contours, i, color, cv::FILLED, 8, hierarchy, 0, cv::Point());
            cv::drawContours( drawing, contours, int(i), color, 4, 8, hierarchy, 0, cv::Point());
        }

    }
    return drawing;
}


cv::Mat uav::segmentationGRAY(cv::Mat img){
    cv::Mat image, mask;
    cv::cvtColor(img, image, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(image, image, cv::Size(5,5), 0, 0);
    double thresholdValue = threshold(image, image, 0, 255, cv::THRESH_BINARY+cv::THRESH_OTSU);
    cv::threshold(image, mask, thresholdValue, 255, 0);
    cv::dilate(mask, mask, cv::Mat::ones(5, 5, CV_8UC1));
    cv::dilate(mask, mask, cv::Mat::ones(5, 5, CV_8UC1));
    cv::dilate(mask, mask, cv::Mat::ones(5, 5, CV_8UC1));
    cv::erode(mask, mask, cv::Mat::ones(5, 5, CV_8UC1));
    cv::erode(mask, mask, cv::Mat::ones(5, 5, CV_8UC1));
    cv::erode(mask, mask, cv::Mat::ones(5, 5, CV_8UC1));
    cv::Mat resultGray;
    cv::bitwise_and(image, image, resultGray, mask);
    return resultGray;
}


cv::Mat uav::segmentationHSVmultipleColors(cv::Mat img, cv::Mat pattern, QString filepath, QString filename){


    cv::Mat image, mask;
    cv::cvtColor(img, image, cv::COLOR_BGR2HSV);

    calculateThresholdsHSV(pattern);
    cv::cvtColor(image, image, cv::COLOR_BGR2HSV);
    cv::inRange(image, this->minHSV, this->maxHSV, mask);

    cv::Mat finalMask;
    std::vector<cv::Mat> masks(this->minHSVs.size());

    if(masks.size() >= 1){
        std::cout<<"Mask size: "<<masks.size()<<std::endl;
        for(int i = 0; i < masks.size(); i++)
            cv::inRange(image, this->minHSVs[i], this->maxHSVs[i], masks[i]);

        finalMask = masks[0];
        for(int i = 1; i < masks.size(); i++)
            finalMask += masks[i];
        finalMask += mask;
    }
    else
        finalMask = mask;


    // Show final mask
    cv::imshow("Mask Final (after)", finalMask);
    cv::Mat resultHSV = cv::Mat::zeros(img.rows, img.cols, CV_8UC3);
    cv::Mat originalImg = cv::imread((filepath + "/" + filename).toUtf8().data());
    cv::bitwise_and(originalImg, originalImg, resultHSV, finalMask);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours( finalMask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    cv::RNG rng(12345);
    //cv::Mat drawing = cv::Mat::zeros( mask.size(), CV_8UC3 );
    cv::Mat drawing = originalImg;

    double maxArea = 1010;
    for( unsigned i = 0; i<contours.size(); i++ ){
        if (hierarchy[i][3] == -1)
            continue;
        double area = cv::contourArea(contours[i]);
        //std::cout<<"Area: "<< area << std::endl;

        cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );

        cv::Rect r = cv::boundingRect(contours[i]);
        unsigned rect_area = unsigned(r.width*r.height);
        double extent = double(area)/rect_area;
        double aspect_ratio = double(r.width)/r.height;

        //Tune parameters!!!
        if (area > maxArea && extent > 0.2 && aspect_ratio > 0.5){
        //if (cv::isContourConvex(contours[i])){
            //cv::drawContours( drawing, contours, i, color, cv::FILLED, 8, hierarchy, 0, cv::Point());
            cv::drawContours( drawing, contours, int(i), color, 4, 8, hierarchy, 0, cv::Point());
        }

    }
    return drawing;
}


void uav::on_startSegmentation_clicked()
{
    if(this->filepathSegmentation == nullptr || this->snapshotFilename.isEmpty()){
        return;
    }

    QString filepath = this->filepathSegmentation;
    QDir directory(filepath);
    QStringList images = directory.entryList(QStringList() << "*.png" << "*.PNG" << "*.jpg" << "*.JPG",QDir::Files);

    /*Color space indexes:
     * Value: 0; Color Space: HSV
     * Value: 1; Color Space: RGB
     * Value: 2; Color Space: GRAY
     */
    int colorSpace = ui->colorSpaceComboBox->currentIndex();
    int selectedMethod = chooseMethod(colorSpace);
    cv::Mat pattern = cv::imread(this->snapshotFilename.at(0).toUtf8().data());
    pattern = preprocessImage(pattern);

    calculateHSVmasksMultipleColors(pattern);

    foreach(QString filename, images) {
        cv::Mat img = cv::imread((filepath + "/" + filename).toUtf8().data());
        cv::Mat imgOriginal;
        img.copyTo(imgOriginal);
        img = preprocessImage(img);
        cv::Mat image, mask, maskEpsilon;

        switch(selectedMethod) {

        case 0: {
            /* HSV Segmentation */
            cv::Mat resultHSV = segmentationHSV(img, pattern, filepath, filename);
            cv::imshow("Result HSV", resultHSV);
            cv::waitKey(0);
            break;
        }

        case 1: {
            /* RGB Segmentation */
            cv::Mat resultRGB = segmentationRGB(img, pattern, filepath, filename);
            cv::imshow("Result RGB", resultRGB);
            cv::waitKey(0);
            break;
        }

        case 2: {
            /* HSV Segmentation (multiple colors) */
            /*
            cv::Mat image, mask, maskEpsilon;
            cv::cvtColor(img, image, cv::COLOR_BGR2HSV);


            cv::Mat hsvMatrix[3], H, S, V;
            cv::split(image, hsvMatrix);

            H = hsvMatrix[0];
            cv::imshow("H Matrix 1", H);
            cv::waitKey(0);
            //cv::dilate(H, H, cv::Mat::ones(5, 5, CV_8UC1));
            //cv::dilate(H, H, cv::Mat::ones(5, 5, CV_8UC1));
            cv::erode(H, H, cv::Mat::ones(5, 5, CV_8UC1));
            cv::erode(H, H, cv::Mat::ones(5, 5, CV_8UC1));
            cv::GaussianBlur(H, H, cv::Size(5,5), 0, 0);

            cv::imshow("H Matrix 2", H);
            cv::waitKey(0);
            */

            //std::vector<cv::Mat> masks = calculateHSVmasksMultipleColors(img, pattern, filepath, filename);
            //cv::imshow("Result HSV (multiple colors)", resultHSV);
            //cv::waitKey(0);
            cv::Mat resultHSV = segmentationHSVmultipleColors(img, pattern, filepath, filename);
            cv::imshow("Result HSV multiple colors", resultHSV);
            cv::waitKey(0);
            break;
        }

        case 4: {
            /* GRAY Thresholding */
            cv::Mat resultGray = segmentationGRAY(img);
            cv::imshow("Result Grayscale thresholding", resultGray);
            cv::waitKey(0);
            break;
        }
        default:
            std::cout<<"***NOT DEFINED STATUS***"<<std::endl;
        }
    }
    cv::destroyAllWindows();
}


void uav::calculateHSVmasksMultipleColors(cv::Mat pattern){
    cv::Mat mask;
    calculateThresholdsHSV(pattern);
    cv::cvtColor(pattern, pattern, cv::COLOR_BGR2HSV);
    cv::inRange(pattern, this->minHSV, this->maxHSV, mask);

    unsigned sum = 0;
    for(unsigned i = 0; i < this->histValuesH.size(); i++){
        sum += unsigned(this->histValuesH[i]);
    }
    unsigned maxElement = unsigned(*max_element(this->histValuesH.begin(), this->histValuesH.end()));
    std::cout<<"Histogram tolerance (%) " << ui->histogramColorTolerance->value() << std::endl;
    float histogramTolerance = float(ui->histogramColorTolerance->value() * 1.0 / 100.0);
    unsigned threshold = unsigned(histogramTolerance * sum);

    while(maxElement > threshold){
        std::cout<<"Max element: " << maxElement << std::endl;
        std::cout<<"Threshold: " << threshold << std::endl;
        std::vector<int> boundariesH = findMinMax(this->histValuesH, true);
        std::vector<int> boundariesS = findMinMax(this->histValuesS, false);
        std::vector<int> boundariesV = findMinMax(this->histValuesV, false);

        this->minHSVs.push_back(cv::Scalar(boundariesH[0] + 1, boundariesS[0] + 1, boundariesV[0] + 1));
        this->maxHSVs.push_back(cv::Scalar(boundariesH[1] + 1, boundariesS[1] + 1, boundariesV[1] + 1));

        std::cout<<"Boundaries H: " << boundariesH[0] << " and " << boundariesH[1] << std::endl;

        for(int i = boundariesH[0]; i <= boundariesH[1]; i++)
            this->histValuesH[unsigned(i)] = 0;
        maxElement = unsigned(*max_element(this->histValuesH.begin(), this->histValuesH.end()));

    }
}


std::vector<int> uav::findMinMax(std::vector<float> histValues, bool H = false){
    unsigned maxElementIndex = unsigned(std::max_element(histValues.begin(),histValues.end()) - histValues.begin());
    unsigned maxElement = unsigned(*std::max_element(histValues.begin(), histValues.end()));

    int boundary = int(0*maxElement);

    int lowerBoundary, upperBoundary;

    lowerBoundary = findLowerBoundaryVec(histValues,maxElementIndex, boundary);
    upperBoundary = findUpperBoundaryVec(histValues,maxElementIndex, boundary, H);

    if(lowerBoundary == upperBoundary){
        lowerBoundary -= 1;
        upperBoundary += 1;
    }

    if(upperBoundary > 255)
        upperBoundary = 255;

    if(lowerBoundary < 0)
        lowerBoundary = 0;


    std::vector<int> boundaries;
    boundaries.push_back(lowerBoundary);
    boundaries.push_back(upperBoundary);
    return boundaries;
}


cv::Mat uav::preprocessImage(cv::Mat img){
    cv::Mat imgCopy = img;

    //Convert the image from BGR to YCrCb color space
    cv::Mat hist_equalized_image;
    cv::cvtColor(imgCopy, hist_equalized_image, cv::COLOR_BGR2YCrCb);

    //Split the image into 3 channels; Y, Cr and Cb channels respectively and store it in a std::vector
    std::vector<cv::Mat> vec_channels;
    cv::split(hist_equalized_image, vec_channels);

    //Equalize the histogram of only the Y channel
    cv::equalizeHist(vec_channels[0], vec_channels[0]);

    //Merge 3 channels in the vector to form the color image in YCrCB color space.
    cv::merge(vec_channels, hist_equalized_image);

    //Convert the histogram equalized image from YCrCb to BGR color space again
    cv::cvtColor(hist_equalized_image, hist_equalized_image, cv::COLOR_YCrCb2BGR);


    cv::Mat imgHist = hist_equalized_image;

    cv::GaussianBlur( imgHist, imgCopy, cv::Size( 3, 3 ), 0, 0 );
    cv::GaussianBlur( imgCopy, imgCopy, cv::Size( 5, 5 ), 0, 0 );
    cv::GaussianBlur( imgCopy, imgCopy, cv::Size( 7, 7 ), 0, 0 );
    //cv::imshow("equalized and blured image", imgCopy);

    return imgCopy;
}



void uav::on_uploadTemplate_clicked()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("Choose"), "", tr("Images (*.png *.jpg *.jpeg *.bmp *.gif)"));

    if(QString::compare(filename, QString()) != 0){
        this->templateFilename = filename;
        ui->templateNameLabel->setText(filename);
    }
}

void uav::on_uploadPathTemplate_clicked()
{
    QString filepath = QFileDialog::getExistingDirectory(this, "Get Any File");
    if (!filepath.isEmpty()){
        ui->templatePathLabel->setText(filepath);
        this->filepathTemplateMatching = filepath;
    }
}


void uav::on_runTemplateMatching_clicked()
{
    if(ui->rotationCheckbox->isChecked()){
        QString filepath = this->filepathTemplateMatching;
        QDir directory(filepath);
        QStringList images = directory.entryList(QStringList() << "*.png" << "*.PNG" << "*.jpg" << "*.JPG",QDir::Files);

        cv::Mat img_object = cv::imread(this->templateFilename.toUtf8().data());
        cv::cvtColor(img_object, img_object, cv::COLOR_BGR2GRAY);
        foreach(QString filename, images) {
            cv::Mat img_scene = cv::imread((filepath + "/" + filename).toUtf8().data());
            cv::cvtColor(img_scene, img_scene, cv::COLOR_BGR2GRAY);

            int minHessian = 400;
            auto detector = cv::xfeatures2d::SURF::create(minHessian);
            std::vector<cv::KeyPoint> keypoints_object, keypoints_scene;

            detector->detect( img_object, keypoints_object );
            detector->detect( img_scene, keypoints_scene );
            auto extractor = cv::xfeatures2d::SurfDescriptorExtractor::create();

            cv::Mat descriptors_object, descriptors_scene;

            extractor->compute( img_object, keypoints_object, descriptors_object );
            extractor->compute( img_scene, keypoints_scene, descriptors_scene );

            cv::FlannBasedMatcher matcher;
            std::vector< cv::DMatch > matches;
            matcher.match( descriptors_object, descriptors_scene, matches );

            double max_dist = 0; double min_dist = 100;

            for( unsigned i = 0; i < unsigned(descriptors_object.rows); i++ )
            { double dist = double(matches[i].distance);
                if( dist < min_dist ) min_dist = dist;
                if( dist > max_dist ) max_dist = dist;
            }


            //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
            std::vector< cv::DMatch > good_matches;

            for( unsigned i = 0; i < unsigned(descriptors_object.rows); i++ )
            { if( matches[i].distance < float(3*min_dist) )
                { good_matches.push_back( matches[i]); }
            }

            cv::Mat img_matches;
            cv::drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
                             good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                             vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );


            std::vector<cv::Point2f> obj;
            std::vector<cv::Point2f> scene;

            for( unsigned i = 0; i < good_matches.size(); i++ ){
                //-- Get the keypoints from the good matches
                obj.push_back( keypoints_object[ unsigned(good_matches[i].queryIdx) ].pt );
                scene.push_back( keypoints_scene[ unsigned(good_matches[i].trainIdx) ].pt );
            }

            cv::Mat H = findHomography( obj, scene, cv::RANSAC );

            std::vector<cv::Point2f> obj_corners(4);
            obj_corners[0] = cv::Point(0,0); obj_corners[1] = cv::Point( img_object.cols, 0 );
            obj_corners[2] = cv::Point( img_object.cols, img_object.rows ); obj_corners[3] = cv::Point( 0, img_object.rows );
            std::vector<cv::Point2f> scene_corners(4);

            cv::perspectiveTransform( obj_corners, scene_corners, H);

            //-- Draw lines between the corners (the mapped object in the scene - image_2 )
            cv::line( img_matches, scene_corners[0] + cv::Point2f( img_object.cols, 0), scene_corners[1] + cv::Point2f( img_object.cols, 0), cv::Scalar(0, 255, 0), 4 );
            cv::line( img_matches, scene_corners[1] + cv::Point2f( img_object.cols, 0), scene_corners[2] + cv::Point2f( img_object.cols, 0), cv::Scalar( 0, 255, 0), 4 );
            cv::line( img_matches, scene_corners[2] + cv::Point2f( img_object.cols, 0), scene_corners[3] + cv::Point2f( img_object.cols, 0), cv::Scalar( 0, 255, 0), 4 );
            cv::line( img_matches, scene_corners[3] + cv::Point2f( img_object.cols, 0), scene_corners[0] + cv::Point2f( img_object.cols, 0), cv::Scalar( 0, 255, 0), 4 );

            //-- Show detected matches
            cv::imshow( "Good Matches & Object detection", img_matches );

            cv::waitKey(0);

        }
        cv::waitKey(0);
        cv::destroyAllWindows();
    }
    else{
        cv::Mat img; cv::Mat templ; cv::Mat result;
        string image_window = "Source Image";
        string result_window = "Result window";

        templ = cv::imread(this->templateFilename.toUtf8().data());
        cv::GaussianBlur(templ, templ, cv::Size(3,3), 0, 0);
        QString filepath = this->filepathTemplateMatching;
        QDir directory(filepath);
        QStringList images = directory.entryList(QStringList() << "*.png" << "*.PNG" << "*.jpg" << "*.JPG",QDir::Files);

        foreach(QString filename, images) {
            img = cv::imread((filepath + "/" + filename).toUtf8().data());
            //cv::GaussianBlur(img, img, cv::Size(3,3), 0, 0);

            cv::Mat img_display;
            img.copyTo( img_display );


            int result_cols =  img.cols - templ.cols + 1;
            int result_rows = img.rows - templ.rows + 1;

            result.create( result_rows, result_cols, CV_32FC1 );
            auto match_method = cv::TM_CCOEFF_NORMED;
            //auto match_method = cv::TM_CCOEFF;

            cv::matchTemplate( img, templ, result, match_method );
            cv::normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );


            double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
            cv::Point matchLoc;

            cv::minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );

            if( match_method  == cv::TM_SQDIFF || match_method == cv::TM_SQDIFF_NORMED ){
                matchLoc = minLoc;
            }
            else{
                matchLoc = maxLoc;
            }


            cv::rectangle( img_display, matchLoc, cv::Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), cv::Scalar::all(0), 2, 8, 0 );
            cv::rectangle( result, matchLoc, cv::Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), cv::Scalar::all(0), 2, 8, 0 );

            cv::imshow( image_window, img_display );
            cv::imshow( result_window, result );
            cv::waitKey(0);

        }
        cv::waitKey(0);
        cv::destroyAllWindows();
    }

}


void uav::on_multipleColorsCheckbox_stateChanged(int colorSpaceStatus)
{
    if(colorSpaceStatus)
        ui->histogramColorTolerance->setEnabled(true);
    else
        ui->histogramColorTolerance->setEnabled(false);
}




void uav::on_uploadVehicleImage_clicked()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("Choose"), "", tr("Images (*.png *.jpg *.jpeg *.bmp *.gif)"));

    if(QString::compare(filename, QString()) != 0){
        QImage image;
        bool valid = image.load(filename);

        if(valid){
            this->vehicleImage = filename;
            ui->uploadVehicleLabel->setText(filename);
            ui->vehicleImagePreview->setPixmap(QPixmap::fromImage(image.scaled(250, 250, Qt::KeepAspectRatio)));
        }
        else{
            ui->uploadVehicleLabel->setText("Image can not be loaded!");
        }
    }
}

void uav::on_runVehicleDetection_clicked()
{
    cv::Mat vehicle, vehicleGray;
    vehicle = cv::imread(this->vehicleImage.toUtf8().data());
    cv::cvtColor(vehicle, vehicleGray, cv::COLOR_BGR2GRAY);



    cv::imshow("Image", vehicleGray);
    cv::waitKey(0);
    cv::destroyAllWindows();
}


void uav::draw_locations(cv::Mat & img, const std::vector< cv::Rect > & locations, const cv::Scalar & color)
{
    if (!locations.empty())
    {
        vector< cv::Rect >::const_iterator loc = locations.begin();
        vector< cv::Rect >::const_iterator end = locations.end();
        for (; loc != end; ++loc)
        {
            cv::rectangle(img, *loc, color, 2);
        }
    }
}



void uav::on_hsvSeparator_clicked()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("Choose"), "", tr("Images (*.png *.jpg *.jpeg *.bmp *.gif)"));

    if(QString::compare(filename, QString()) != 0){
        QImage image;
        cv::Mat imgThreshed;
        bool valid = image.load(filename);

        if(valid){
            cv::Mat imgRGB, imgHSV;
            imgRGB = cv::imread(filename.toUtf8().data());
            cv::cvtColor(imgRGB, imgHSV, cv::COLOR_BGR2HSV);
            cv::inRange(imgHSV, cv::Scalar(40, 100, 0), cv::Scalar(45, 255, 255), imgThreshed);
            cv::imshow("imgThreshed", imgThreshed);
            cv::imshow("imgRGB", imgRGB);
            cv::waitKey(0);
            cv::destroyAllWindows();
        }
    }
}
