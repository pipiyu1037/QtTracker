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

    //if the Joint position is invalid, return
    if (isnan(p1.X) || isnan(p1.Y) || isinf(p1.X) || isinf(p1.Y)) {
        return;
    }

    if (isnan(p2.X) || isnan(p2.Y) || isinf(p2.X) || isinf(p2.Y)) {
        return;
    }

    //draw line
    cv::line(Img, cv::Point(p1.X, p1.Y), cv::Point(p2.X, p2.Y), cv::Vec3b(0, 0, 255), 4);
}

void DrawSphere(cv::Mat& Img, const Joint& J, ICoordinateMapper* pMapper) {
    if (J.TrackingState == TrackingState_NotTracked) {
        return;
    }

    ColorSpacePoint p;
    pMapper->MapCameraPointToColorSpace(J.Position, &p);

    if (isnan(p.X) || isnan(p.Y) || isinf(p.X) || isinf(p.Y)) {
        return;
    }

    cv::circle(Img, cv::Point(p.X, p.Y), 15, cv::Scalar(0, 0, 255), -1);
}


QtTracker::QtTracker(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
    mpTimer = std::make_shared<QTimer>(this);
    mpQImg = std::make_shared<QImage>();

    //process frame once time out
    connect(mpTimer.get(), SIGNAL(timeout() ), this, SLOT(processFrame() ));

    //set button click event
    connect(ui.Begin, SIGNAL(clicked()), this, SLOT(begin() ));
    connect(ui.Pause, SIGNAL(clicked()), this, SLOT(pause() ));
    connect(ui.Stop, SIGNAL(clicked()), this, SLOT(stop()));

    if (!init()) {
        std::runtime_error("failed to init kinect");
        exit(0);
    }

    // init Body vector
    mpBodies = std::vector<IBody*>(bodyCount, nullptr);
}

QtTracker::~QtTracker()
{
    //release resource
    sensor->Close();
    SafeRelease(sensor);
    SafeRelease(mpBodyFrameReader);
    SafeRelease(mpColorFrameReader);
    SafeRelease(mpCoordinateMapper);
}

bool QtTracker::init()
{
    HRESULT hr;

    //get kinect sensor
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

            //get frame width and height
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
        colorImg = cv::Mat(height, width, CV_8UC4);

        SafeRelease(pBodyFrameSource);
        SafeRelease(pColorFrameSource);
        SafeRelease(pFrameDescription);
    }

    if (!sensor || FAILED(hr)) {
        std::cout << "No ready Kinect found!" << std::endl;
        return false;
    }

    initCharts();
    return true;
}

void QtTracker::initCharts()
{   
    mpCharts[JointType_KneeLeft] = std::make_shared<Chart>("KneeLeft");
    mpCharts[JointType_AnkleLeft] = std::make_shared<Chart>("AnkleLeft");
    mpCharts[JointType_FootLeft] = std::make_shared<Chart>("FootLeft");

    mpCharts[JointType_KneeRight] = std::make_shared<Chart>("KneeRight");
    mpCharts[JointType_AnkleRight] = std::make_shared<Chart>("AnkleRight");
    mpCharts[JointType_FootRight] = std::make_shared<Chart>("FootRight");

    mpLeftKneeAngleChart = std::make_shared<QChart>();
    mpRightKneeAngleChart = std::make_shared<QChart>();
    mpLeftKneeAngleChart->setTitle("Left Knee Angle");
    mpRightKneeAngleChart->setTitle("Right Knee Angle");

    mpLeftKneeAngleSeries = std::make_shared<QSplineSeries>();
    mpLeftKneeAngleVSeries = std::make_shared<QSplineSeries>();
    mpLeftKneeAngleSeries->setName("angle");
    mpLeftKneeAngleVSeries->setName("angle v");

    mpRightKneeAngleSeries = std::make_shared<QSplineSeries>();
    mpRightKneeAngleVSeries = std::make_shared<QSplineSeries>();
    mpRightKneeAngleSeries->setName("angle");
    mpRightKneeAngleVSeries->setName("angle v");

    mpLeftKneeAxisX = std::make_shared<QValueAxis>();
    mpLeftKneeAxisY = std::make_shared<QValueAxis>();
    mpLeftKneeAxisX->setGridLineVisible(false);
    mpLeftKneeAxisY->setGridLineVisible(false);
    mpLeftKneeAxisX->setTitleText("Time(s)");
    mpLeftKneeAxisY->setTitleText("rad");
    mpLeftKneeAxisX->setRange(0, 10);
    mpLeftKneeAxisY->setRange(-3.5, 3.5);

    mpRightKneeAxisX = std::make_shared<QValueAxis>();
    mpRightKneeAxisY = std::make_shared<QValueAxis>();
    mpRightKneeAxisX->setGridLineVisible(false);
    mpRightKneeAxisY->setGridLineVisible(false);
    mpRightKneeAxisX->setTitleText("Time(s)");
    mpRightKneeAxisY->setTitleText("rad");
    mpRightKneeAxisX->setRange(0, 10);
    mpRightKneeAxisY->setRange(-3.5, 3.5);

    //leftKneeAngle init
    mpLeftKneeAngleChart->addAxis(mpLeftKneeAxisX.get(), Qt::AlignBottom);
    mpLeftKneeAngleChart->addAxis(mpLeftKneeAxisY.get(), Qt::AlignLeft);

    mpLeftKneeAngleChart->addSeries(mpLeftKneeAngleSeries.get());
    mpLeftKneeAngleChart->addSeries(mpLeftKneeAngleVSeries.get());

    mpLeftKneeAngleSeries->attachAxis(mpLeftKneeAxisX.get());
    mpLeftKneeAngleSeries->attachAxis(mpLeftKneeAxisY.get());

    mpLeftKneeAngleVSeries->attachAxis(mpLeftKneeAxisX.get());
    mpLeftKneeAngleVSeries->attachAxis(mpLeftKneeAxisY.get());

    //Right Knee Angle init
    mpRightKneeAngleChart->addAxis(mpRightKneeAxisX.get(), Qt::AlignBottom);
    mpRightKneeAngleChart->addAxis(mpRightKneeAxisY.get(), Qt::AlignLeft);

    mpRightKneeAngleChart->addSeries(mpRightKneeAngleSeries.get());
    mpRightKneeAngleChart->addSeries(mpRightKneeAngleVSeries.get());

    mpRightKneeAngleSeries->attachAxis(mpRightKneeAxisX.get());
    mpRightKneeAngleSeries->attachAxis(mpRightKneeAxisY.get());

    mpRightKneeAngleVSeries->attachAxis(mpRightKneeAxisX.get());
    mpRightKneeAngleVSeries->attachAxis(mpRightKneeAxisY.get());

    ui.graphicsView->setChart(mpCharts[JointType_KneeLeft]->mpQChart.get());
    ui.graphicsView->setRenderHint(QPainter::Antialiasing);

    ui.graphicsView_2->setChart(mpCharts[JointType_AnkleLeft]->mpQChart.get());
    ui.graphicsView_2->setRenderHint(QPainter::Antialiasing);

    ui.graphicsView_3->setChart(mpCharts[JointType_FootLeft]->mpQChart.get());
    ui.graphicsView_3->setRenderHint(QPainter::Antialiasing);

    ui.graphicsView_4->setChart(mpCharts[JointType_KneeRight]->mpQChart.get());

    ui.graphicsView_5->setChart(mpCharts[JointType_AnkleRight]->mpQChart.get());

    ui.graphicsView_6->setChart(mpCharts[JointType_FootRight]->mpQChart.get());

    ui.graphicsView_7->setChart(mpLeftKneeAngleChart.get());
    ui.graphicsView_8->setChart(mpRightKneeAngleChart.get());
}

bool QtTracker::update()
{
    currentFrame++;
    if (!mpBodyFrameReader) {
        return false;
    }
    if (!mpColorFrameReader) {
        return false;
    }

    IColorFrame* pColorFrame = nullptr;

    /*if (mpColorFrameReader->AcquireLatestFrame(&pColorFrame) == S_OK) {
        pColorFrame->CopyConvertedFrameDataToArray(uBufferSize, colorImg.data, ColorImageFormat_Bgra);
    }*/

    Img = cv::Scalar(0.f);
    IBodyFrame* pBodyFrame = nullptr;

    if (mpBodyFrameReader->AcquireLatestFrame(&pBodyFrame) == S_OK && pBodyFrame->GetAndRefreshBodyData(bodyCount, mpBodies.data()) == S_OK) {
        currentTime = std::chrono::steady_clock::now();
        if (!startTrack) {
            startTime = currentTime;
            startTrack = true;
        }
        currentToStart = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - startTime).count();

        int i = 0;
        for (auto& pBody : mpBodies) {
            BOOLEAN bTracked = false;
            if ((pBody->get_IsTracked(&bTracked) == S_OK) && bTracked)
            {
                if (pBody->GetJoints(JointType::JointType_Count, currentJoints[i].data()) == S_OK)
                {
                    getAngleV(currentJoints[i][JointType_KneeLeft], currentJoints[i][JointType_HipLeft], currentJoints[i][JointType_AnkleLeft], i);
                    getAngleV(currentJoints[i][JointType_KneeRight], currentJoints[i][JointType_HipRight], currentJoints[i][JointType_AnkleRight], i);

                    getJointVelocity(currentJoints[i][JointType_SpineBase], i);

                    getJointVelocity(currentJoints[i][JointType_HipLeft], i);
                    getJointVelocity(currentJoints[i][JointType_KneeLeft], i);
                    getJointVelocity(currentJoints[i][JointType_AnkleLeft], i);
                    getJointVelocity(currentJoints[i][JointType_FootLeft], i);

                    getJointVelocity(currentJoints[i][JointType_HipRight], i);
                    getJointVelocity(currentJoints[i][JointType_KneeRight], i);
                    getJointVelocity(currentJoints[i][JointType_AnkleRight], i);
                    getJointVelocity(currentJoints[i][JointType_FootRight], i);

                    DrawSphere(Img, currentJoints[i][JointType_SpineBase], mpCoordinateMapper);

                    DrawSphere(Img, currentJoints[i][JointType_HipLeft], mpCoordinateMapper);
                    DrawSphere(Img, currentJoints[i][JointType_KneeLeft], mpCoordinateMapper);
                    DrawSphere(Img, currentJoints[i][JointType_AnkleLeft], mpCoordinateMapper);
                    DrawSphere(Img, currentJoints[i][JointType_FootLeft], mpCoordinateMapper);

                    DrawSphere(Img, currentJoints[i][JointType_HipRight], mpCoordinateMapper);
                    DrawSphere(Img, currentJoints[i][JointType_KneeRight], mpCoordinateMapper);
                    DrawSphere(Img, currentJoints[i][JointType_AnkleRight], mpCoordinateMapper);
                    DrawSphere(Img, currentJoints[i][JointType_FootRight], mpCoordinateMapper);

                    DrawLine(Img, currentJoints[i][JointType_SpineBase], currentJoints[i][JointType_HipLeft], mpCoordinateMapper);
                    DrawLine(Img, currentJoints[i][JointType_HipLeft], currentJoints[i][JointType_KneeLeft], mpCoordinateMapper);
                    DrawLine(Img, currentJoints[i][JointType_KneeLeft], currentJoints[i][JointType_AnkleLeft], mpCoordinateMapper);
                    DrawLine(Img, currentJoints[i][JointType_AnkleLeft], currentJoints[i][JointType_FootLeft], mpCoordinateMapper);

                    DrawLine(Img, currentJoints[i][JointType_SpineBase], currentJoints[i][JointType_HipRight], mpCoordinateMapper);
                    DrawLine(Img, currentJoints[i][JointType_HipRight], currentJoints[i][JointType_KneeRight], mpCoordinateMapper);
                    DrawLine(Img, currentJoints[i][JointType_KneeRight], currentJoints[i][JointType_AnkleRight], mpCoordinateMapper);
                    DrawLine(Img, currentJoints[i][JointType_AnkleRight], currentJoints[i][JointType_FootRight], mpCoordinateMapper);
                }
            }
        }
    }
    SafeRelease(pColorFrame);
    SafeRelease(pBodyFrame);
    return true;
}

void QtTracker::begin()
{
    //mpTimer->setInterval(30);

    mpTimer->start(40);
}

void QtTracker::pause()
{
    mpTimer->stop();
}

void QtTracker::stop()
{
    mpTimer->stop();
    args.lastLeftAngleInit = false;
    args.lastRightAngleInit = false;

    for (int i = 0; i < bodyCount; i++) {
        for (int j = 0; j < JointType_Count; j++) {
            args.posInitialized[i][j] = false;
            args.velocityInitialied[i][j] = false;
            startTrack = false;
        }
    }

    mpLeftKneeAngleSeries->clear();
}

void QtTracker::processFrame()
{
    //update content of cvImg
    update();

    mpQImg = std::make_shared<QImage>(
        Mat2Image(Img).scaled(ui.imgArea->width(), ui.imgArea->height(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation)
    );

    ui.imgArea->setPixmap(QPixmap::fromImage(*mpQImg.get()));
}

void QtTracker::getJointVelocity(Joint& J, int bodyIndex)
{
    if (J.TrackingState == TrackingState_NotTracked) return;

    CameraSpacePoint p = J.Position;

    if (isnan(p.X) || isnan(p.Y) || isinf(p.X) || isinf(p.Y)) {
        return;
    }

    if (!args.posInitialized[bodyIndex][J.JointType]) {
        args.lastPos[bodyIndex][J.JointType] = vec2f(p.X, p.Y);
        args.lastJointTime[bodyIndex][J.JointType] = currentTime;
        args.posInitialized[bodyIndex][J.JointType] = true;
        return;
    }

    double durationSeconds = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - args.lastJointTime[bodyIndex][J.JointType]).count();
    float v_x = static_cast<float>(p.X - args.lastPos[bodyIndex][J.JointType].x) / durationSeconds;
    float v_y = static_cast<float>(p.Y - args.lastPos[bodyIndex][J.JointType].y) / durationSeconds;

    args.lastPos[bodyIndex][J.JointType] = vec2f(p.X, p.Y);
    args.lastJointTime[bodyIndex][J.JointType] = currentTime;

    if (mpCharts[J.JointType].get() != nullptr) {

        mpCharts[J.JointType]->mpVelocitySplineX->append(currentToStart, v_x);
        mpCharts[J.JointType]->mpVelocitySplineY->append(currentToStart, v_y);

        if (currentToStart > 10.f) {
            mpCharts[J.JointType]->setMinX(currentToStart - 10.f);
            mpCharts[J.JointType]->setMaxX(currentToStart);
        }
    }

    if (!args.velocityInitialied[bodyIndex][J.JointType]) {
        args.lastVelocity[bodyIndex][J.JointType] = vec2f(v_x, v_y);
        args.velocityInitialied[bodyIndex][J.JointType] = true;
        return;
    }

    args.lastA[bodyIndex][J.JointType] = (vec2f(v_x, v_y) - args.lastVelocity[bodyIndex][J.JointType]) / durationSeconds;
    args.lastVelocity[bodyIndex][J.JointType] = vec2f(v_x, v_y);

    if (mpCharts[J.JointType].get() != nullptr) {
        mpCharts[J.JointType]->mpASplineX->append(currentToStart, args.lastA[bodyIndex][J.JointType].x);
        mpCharts[J.JointType]->mpASplineY->append(currentToStart, args.lastA[bodyIndex][J.JointType].y);
    }
    
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

    //process left knee
    if (Knee.JointType == JointType_KneeLeft) {
        if (!args.lastLeftAngleInit) {
            args.lastLeftAngle = getAngle(Knee, Hip, Ankle, bodyIndex);
            args.lastLeftAngleTime = currentTime;
            args.lastLeftAngleInit = true;
            return;
        }

        float angle = getAngle(Knee, Hip, Ankle, bodyIndex);
        if (isnan(angle) || isinf(angle)) {
            return;
        }

        double durationSeconds = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - args.lastLeftAngleTime).count();
        args.leftAngleV = (angle - args.lastLeftAngle) / durationSeconds;

        if (currentToStart > 10.f) {
            mpLeftKneeAxisX->setMax(currentToStart);
            mpLeftKneeAxisX->setMin(currentToStart - 10.f);
        }

        mpLeftKneeAngleSeries->append(currentToStart, angle);
        mpLeftKneeAngleVSeries->append(currentToStart, args.leftAngleV);

        args.lastLeftAngleTime = currentTime;
        args.lastLeftAngle = angle;

    }
    else if (Knee.JointType == JointType_KneeRight)
    {
        if (!args.lastRightAngleInit) {
            args.lastRightAngle = getAngle(Knee, Hip, Ankle, bodyIndex);
            args.lastRightAngleTime = currentTime;
            args.lastRightAngleInit = true;
            return;
        }

        float angle = getAngle(Knee, Hip, Ankle, bodyIndex);
        if (isnan(angle) || isinf(angle)) {
            return;
        }

        double durationSeconds = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - args.lastRightAngleTime).count();
        args.rightAngleV = (angle - args.lastRightAngle) / durationSeconds;

        if (currentToStart > 10.f) {
            mpRightKneeAxisX->setMax(currentToStart);
            mpRightKneeAxisX->setMin(currentToStart - 10.f);
        }

        mpRightKneeAngleSeries->append(currentToStart, angle);
        mpRightKneeAngleVSeries->append(currentToStart, args.rightAngleV);

        args.lastRightAngleTime = currentTime;
        args.lastLeftAngle = angle;
    }

    return;
}

QImage QtTracker::Mat2Image(cv::Mat& cvImg)
{
    cv::Mat temp;
    cv::cvtColor(cvImg, temp, cv::COLOR_BGR2RGB);
    QImage image((const uchar*)temp.data, temp.cols, temp.rows, temp.step, QImage::Format_RGB888);
    image.bits();
    return image;
}