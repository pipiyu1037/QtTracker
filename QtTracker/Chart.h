#pragma once
#include <qsplineseries.h>
#include <qvalueaxis.h>
#include <qchartview.h>
#include <Kinect.h>

class Chart
{
public:
    Chart(const QString& title) {
        init(title);
    }

    std::shared_ptr<QChart> mpQChart = nullptr;
    void init(const QString& title);
    void setMaxX(qreal m);
    void setMinX(qreal m);
    void reset();

    std::shared_ptr<QSplineSeries> mpVelocitySplineX = nullptr;
    std::shared_ptr<QSplineSeries> mpVelocitySplineY = nullptr;
    std::shared_ptr<QSplineSeries> mpASplineX = nullptr;
    std::shared_ptr<QSplineSeries> mpASplineY = nullptr;
private:
    std::shared_ptr<QValueAxis> mpAxisX = nullptr;
    std::shared_ptr<QValueAxis> mpVelocityAxisY = nullptr;
    std::shared_ptr<QValueAxis> mpAcceAxisY = nullptr;
};