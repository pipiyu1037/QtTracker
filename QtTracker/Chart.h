#pragma once
#include <qsplineseries.h>
#include <qvalueaxis.h>
#include <qchartview.h>
#include <Kinect.h>

class Chart
{
public:
    Chart() {
        init();
    }

    std::shared_ptr<QChart> mpQChart = nullptr;
    void init();
    void setMaxX(qreal m);
    void setMinX(qreal m);

    std::shared_ptr<QSplineSeries> mpVelocitySplineX = nullptr;
    std::shared_ptr<QSplineSeries> mpVelocitySplineY = nullptr;
    std::shared_ptr<QSplineSeries> mpASplineX = nullptr;
    std::shared_ptr<QSplineSeries> mpASplineY = nullptr;
private:
    std::shared_ptr<QValueAxis> mpAxisX = nullptr;
    std::shared_ptr<QValueAxis> mpVelocityAxisY = nullptr;
    std::shared_ptr<QValueAxis> mpRadAxisY = nullptr;
};