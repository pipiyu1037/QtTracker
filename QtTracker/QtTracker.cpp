#include "QtTracker.h"

//global function
template<class Interface>
inline void SafeRelease(Interface*& pInterfaceToRelease)
{
    if (pInterfaceToRelease != nullptr)
    {
        pInterfaceToRelease->Release();
        pInterfaceToRelease = nullptr;
    }
}

void DrawLine(cv::Mat& Img, const Joint& r1, const Joint& r2, ICoordinateMapper* pMapper)
{
    //if the state is NotTracker, return without do anything
    if (r1.TrackingState == TrackingState_NotTracked || r2.TrackingState == TrackingState_NotTracked)
        return;

    //map point in camera space to color space
    ColorSpacePoint p1, p2;
    pMapper->MapCameraPointToColorSpace(r1.Position, &p1);
    pMapper->MapCameraPointToColorSpace(r2.Position, &p2);

    //draw line
    cv::line(Img, cv::Point(p1.X, p1.Y), cv::Point(p2.X, p2.Y), cv::Vec3b(0, 0, 255), 4);
}

void DrawSphere(cv::Mat& Img, const Joint& J, ICoordinateMapper* pMapper) {
    if (J.TrackingState == TrackingState_NotTracked) {
        return;
    }

    ColorSpacePoint p;
    pMapper->MapCameraPointToColorSpace(J.Position, &p);

    cv::circle(Img, cv::Point(p.X, p.Y), 15, cv::Scalar(0, 0, 255), -1);
}


QtTracker::QtTracker(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);

    if (!init()) {
        std::runtime_error("failed to init kinect");
        exit(0);
    }
    mpBodies = std::vector<IBody*>(bodyCount, nullptr);
}

QtTracker::~QtTracker()
{
    sensor->Close();
    SafeRelease(sensor);
    SafeRelease(mpBodyFrameReader);
    SafeRelease(mpColorFrameReader);
    SafeRelease(mpCoordinateMapper);
}

bool QtTracker::init()
{
    HRESULT hr;
    hr = GetDefaultKinectSensor(&sensor);
    if (FAILED(hr)) {
        std::cout << "failed to get kinect " << std::endl;
        return false;
    }

    if (sensor) {
        hr = sensor->Open();
        IBodyFrameSource* pBodyFrameSource = nullptr;
        if (SUCCEEDED(hr))
            hr = sensor->get_CoordinateMapper(&mpCoordinateMapper);
        if (SUCCEEDED(hr))
            hr = sensor->get_BodyFrameSource(&pBodyFrameSource);
        //if (SUCCEEDED(hr))
            //hr = pBodyFrameSource->get_BodyCount(&bodyCount);
        if (SUCCEEDED(hr))
            hr = pBodyFrameSource->OpenReader(&mpBodyFrameReader);

        currentJoints = std::vector<std::vector<Joint>>(bodyCount, std::vector<Joint>(JointType_Count));

        IColorFrameSource* pColorFrameSource = nullptr;
        IFrameDescription* pFrameDescription = nullptr;
        if (SUCCEEDED(hr)) {
            hr = sensor->get_ColorFrameSource(&pColorFrameSource);

            pColorFrameSource->get_FrameDescription(&pFrameDescription);
            pFrameDescription->get_Width(&width);
            pFrameDescription->get_Height(&height);
            pFrameDescription->get_LengthInPixels(&lengthInPixel);

            std::cout << height << " " << width << std::endl;
            uBufferSize = height * width * 4 * sizeof(BYTE);
        }

        if (SUCCEEDED(hr))
            hr = pColorFrameSource->OpenReader(&mpColorFrameReader);

        sensor->get_CoordinateMapper(&mpCoordinateMapper);
        
        Img = cv::Mat(height, width, CV_8UC4);

        SafeRelease(pBodyFrameSource);
        SafeRelease(pColorFrameSource);
        SafeRelease(pFrameDescription);
    }

    if (!sensor || FAILED(hr)) {
        std::cout << "No ready Kinect found!" << std::endl;
        return false;
    }

    return true;
}

void QtTracker::processFrame() {

}

void QtTracker::getJointVelocity(Joint& J, int bodyIndex)
{
    if (J.TrackingState == TrackingState_NotTracked) return;

    ColorSpacePoint p;
    mpCoordinateMapper->MapCameraPointToColorSpace(J.Position, &p);

    std::cout << jointName[J.JointType] << " Position:" << p.X << " " << p.Y << std::endl;
    if (!args.posInitialized[bodyIndex][J.JointType]) {
        args.lastPos[bodyIndex][J.JointType] = vec2f(p.X, p.Y);
        args.lastJointTime[bodyIndex][J.JointType] = currentTime;
        args.posInitialized[bodyIndex][J.JointType] = true;
        return;
    }

    double durationSeconds = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - lastJointTime[bodyIndex][J.JointType]).count();
    float v_x = static_cast<float>(p.X - args.lastPos[bodyIndex][J.JointType].x) * lengthInPixel / durationSeconds;
    float v_y = static_cast<float>(p.Y - args.lastPos[bodyIndex][J.JointType].y) * lengthInPixel / durationSeconds;

    args.lastPos[bodyIndex][J.JointType] = vec2f(p.X, p.Y);
    args.lastJointTime[bodyIndex][J.JointType] = currentTime;

    std::cout << "v_x: " << v_x << std::endl;
    std::cout << "v_y: " << v_y << std::endl;

    if (!args.velocityInitialied[bodyIndex][J.JointType]) {
        args.lastVelocity[bodyIndex][J.JointType] = vec2f(v_x, v_y);
        args.velocityInitialied[bodyIndex][J.JointType] = true;
        return;
    }

    args.lastA[bodyIndex][J.JointType] = (vec2f(v_x, v_y) - args.lastVelocity[bodyIndex][J.JointType]) / durationSeconds;
    args.lastVelocity[bodyIndex][J.JointType] = vec2f(v_x, v_y);

    std::cout << "a_x" << args.lastA[bodyIndex][J.JointType].x << std::endl;
    std::cout << "a_y" << args.lastA[bodyIndex][J.JointType].y << std::endl;

    return;
}

float QtTracker::getAngle(Joint& Knee, Joint& Hip, Joint& Ankle, int bodyIndex)
{
    ColorSpacePoint KneePos, HipPos, AnklePos;
    mpCoordinateMapper->MapCameraPointToColorSpace(Knee.Position, &KneePos);
    mpCoordinateMapper->MapCameraPointToColorSpace(Hip.Position, &HipPos);
    mpCoordinateMapper->MapCameraPointToColorSpace(Ankle.Position, &AnklePos);

    vec2f kneeToHip(HipPos.X - KneePos.X, HipPos.Y - KneePos.Y);
    vec2f KneeToAnkle(AnklePos.X - KneePos.X, AnklePos.Y - KneePos.Y);

    float angle = angleBetweenTwoVec2f(kneeToHip, KneeToAnkle);

    return angle;
}

void QtTracker::getAngleV(Joint& Knee, Joint& Hip, Joint& Ankle, int bodyIndex)
{
    if (Knee.TrackingState == TrackingState_NotTracked ||
        Hip.TrackingState == TrackingState_NotTracked ||
        Ankle.TrackingState == TrackingState_NotTracked) {
        return;
    }

    if (!args.lastLeftAngleInit) {
        args.lastLeftAngle = getAngle(Knee, Hip, Ankle, bodyIndex);
        args.lastLeftAngleTime = currentTime;
        args.lastLeftAngleInit = true;
        return;
    }

    double durationSeconds = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - args.lastLeftAngleTime).count();
    args.leftAngleV = (getAngle(Knee, Hip, Ankle, bodyIndex) - args.lastLeftAngle) / durationSeconds;
}

QImage QtTracker::Mat2Image(cv::Mat& cvImg)
{
    QImage qImg;
    if (cvImg.channels() == 3)     //3 channels color image
    {
        cv::cvtColor(cvImg, cvImg, cv::COLOR_BGR2RGB);
        qImg = QImage((const unsigned char*)(cvImg.data),
            cvImg.cols, cvImg.rows,
            cvImg.cols * cvImg.channels(),
            QImage::Format_RGB888);
    }
    else if (cvImg.channels() == 1)                    //grayscale image
    {
        qImg = QImage((const unsigned char*)(cvImg.data),
            cvImg.cols, cvImg.rows,
            cvImg.cols * cvImg.channels(),
            QImage::Format_Indexed8);
    }
    else
    {
        qImg = QImage((const unsigned char*)(cvImg.data),
            cvImg.cols, cvImg.rows,
            cvImg.cols * cvImg.channels(),
            QImage::Format_RGB888);
    }
    return qImg;
}
