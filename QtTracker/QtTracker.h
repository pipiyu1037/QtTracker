#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_QtTracker.h"
#include <QDebug>
#include <QTimer>
#include <QImage>

#include <Kinect.h>
#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <opencv2/opencv.hpp>
#include <chrono>
#include "argument.h"

const std::vector<std::string> jointName = {
    "SpineBase",
    "SpineMid",
    "Neck",
    "Head",
    "ShoulderLeft",
    "ElbowLeft",
    "WristLeft",
    "HandLeft",
    "ShoulderRight",
    "ElbowRight",
    "WristRight",
    "HandRight",
    "HipLeft",
    "KneeLeft",
    "AnkleLeft",
    "FootLeft",
    "HipRight",
    "KneeRight",
    "AnkleRight",
    "FootRight",
    "SpineShoulder",
    "HandTipLeft",
    "ThumbLeft",
    "HandTipRight",
    "ThumbRight"
};

class QtTracker : public QMainWindow
{
    Q_OBJECT

public:
    QtTracker(QWidget *parent = nullptr);
    ~QtTracker();

    static const INT32 bodyCount = 6;
    int width = 800;
    int height = 600;

    QImage Mat2Image(cv::Mat& cvImg);

private slots:
    void processFrame();

private:
    Ui::QtTrackerClass ui;

    bool init();
    //bool update(cv::Mat& colorImg);
    void getJointVelocity(Joint& J, int bodyIndex);
    float getAngle(Joint& Knee, Joint& Hip, Joint& Ankle, int bodyIndex);
    void getAngleV(Joint& Knee, Joint& Hip, Joint& Ankle, int bodyIndex);
    //void getSingleJointArgs(Joint& J, int bodyIndex);

    bool trackHipLeft = true;
    bool trackKneeLeft = true;
    bool trackAnkleLeft = true;
    bool trackFootLeft = true;

    bool trackHipRight = true;
    bool trackKneeRight = true;
    bool trackAnkleRight = true;
    bool trackFootRight = true;

    //void renderUI();
    std::chrono::steady_clock::time_point currentTime;

    int currentFrame = 0;
    UINT uBufferSize = 0;
    unsigned int lengthInPixel = 0;
    IKinectSensor* sensor = nullptr;
    ICoordinateMapper* mpCoordinateMapper = nullptr;
    IBodyFrameReader* mpBodyFrameReader = nullptr;

    IColorFrameReader* mpColorFrameReader = nullptr;

    std::vector<IBody*> mpBodies;
    std::vector<std::vector<Joint>> currentJoints;

    std::array<std::array<Joint, JointType::JointType_Count>, bodyCount> lastJoints;
    std::array<std::array<int64, JointType::JointType_Count>, bodyCount> lastJointFrame;
    std::array<std::array<std::chrono::steady_clock::time_point, JointType::JointType_Count>, bodyCount> lastJointTime;


    Argument args;
    cv::Mat Img;
    cv::Mat colorImg;

    std::shared_ptr<QTimer> mpTimer;
};
