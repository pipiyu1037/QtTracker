#include "Chart.h"

void Chart::init(const QString& title)
{
    mpQChart = std::make_shared<QChart>();
    
    mpQChart->setTitle(title);
    //init Axis
    mpAxisX = std::make_shared<QValueAxis>();
    mpVelocityAxisY = std::make_shared<QValueAxis>();
    mpAcceAxisY = std::make_shared<QValueAxis>();

    mpVelocitySplineX = std::make_shared<QSplineSeries>();
    mpVelocitySplineY = std::make_shared<QSplineSeries>();

    mpASplineX = std::make_shared<QSplineSeries>();
    mpASplineY = std::make_shared<QSplineSeries>();

    mpQChart->addAxis(mpAxisX.get(), Qt::AlignBottom);
    mpQChart->addAxis(mpVelocityAxisY.get(), Qt::AlignLeft);
    mpQChart->addAxis(mpAcceAxisY.get(), Qt::AlignRight);

    mpQChart->addSeries(mpVelocitySplineX.get());
    mpQChart->addSeries(mpVelocitySplineY.get());

    mpQChart->addSeries(mpASplineX.get());
    mpQChart->addSeries(mpASplineY.get());

    mpAxisX->setRange(0, 20);
    mpAxisX->setGridLineVisible(false);
    mpAxisX->setTitleText("Time(s)");

    mpVelocityAxisY->setRange(-3, 3);
    mpVelocityAxisY->setGridLineVisible(false);
    mpVelocityAxisY->setTitleText("v(m/s)");

    mpAcceAxisY->setRange(-40, 40);
    mpAcceAxisY->setGridLineVisible(false);
    mpAcceAxisY->setTitleText("a(m*m/s)");

    mpVelocitySplineX->setName("v_x");
    mpVelocitySplineY->setName("v_y");
    mpASplineX->setName("a_x");
    mpASplineY->setName("a_y");

    mpVelocitySplineX->attachAxis(mpAxisX.get());
    mpVelocitySplineX->attachAxis(mpVelocityAxisY.get());

    mpVelocitySplineY->attachAxis(mpAxisX.get());
    mpVelocitySplineY->attachAxis(mpVelocityAxisY.get());

    mpASplineX->attachAxis(mpAxisX.get());
    mpASplineX->attachAxis(mpAcceAxisY.get());

    mpASplineY->attachAxis(mpAxisX.get());
    mpASplineY->attachAxis(mpAcceAxisY.get());

    mpASplineX->setOpacity(0.3);
    mpASplineY->setOpacity(0.3);
}

void Chart::setMaxX(qreal m)
{
    mpAxisX->setMax(m);
}

void Chart::setMinX(qreal m)
{
    mpAxisX->setMin(m);
}

void Chart::reset()
{
    mpAxisX->setRange(0, 20);
    mpVelocitySplineX->clear();
    mpVelocitySplineY->clear();
    mpASplineX->clear();
    mpASplineY->clear();
}
