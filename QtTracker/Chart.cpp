#include "Chart.h"

void Chart::init()
{
    mpQChart = std::make_shared<QChart>();
    
    //init Axis
    mpAxisX = std::make_shared<QValueAxis>();
    mpVelocityAxisY = std::make_shared<QValueAxis>();
    mpRadAxisY = std::make_shared<QValueAxis>();

    mpVelocitySplineX = std::make_shared<QSplineSeries>();
    mpVelocitySplineY = std::make_shared<QSplineSeries>();

    mpASplineX = std::make_shared<QSplineSeries>();
    mpASplineY = std::make_shared<QSplineSeries>();

    mpQChart->addAxis(mpAxisX.get(), Qt::AlignBottom);
    mpQChart->addAxis(mpVelocityAxisY.get(), Qt::AlignLeft);
    mpQChart->addAxis(mpRadAxisY.get(), Qt::AlignRight);

    mpQChart->addSeries(mpVelocitySplineX.get());
    mpQChart->addSeries(mpVelocitySplineY.get());
    mpQChart->addSeries(mpASplineX.get());
    mpQChart->addSeries(mpASplineY.get());

    mpAxisX->setRange(0, 10);
    mpAxisX->setGridLineVisible(false);

    mpVelocityAxisY->setRange(-10, 10);
    mpVelocityAxisY->setGridLineVisible(false);

    mpRadAxisY->setRange(-10, 10);
    mpRadAxisY->setGridLineVisible(false);

    mpVelocitySplineX->setName("v_x");
    mpVelocitySplineY->setName("v_y");
    mpASplineX->setName("a_x");
    mpASplineY->setName("a_y");

    mpVelocitySplineX->attachAxis(mpAxisX.get());
    mpVelocitySplineX->attachAxis(mpVelocityAxisY.get());

    mpVelocitySplineY->attachAxis(mpAxisX.get());
    mpVelocitySplineY->attachAxis(mpVelocityAxisY.get());

    mpASplineX->attachAxis(mpAxisX.get());
    mpASplineX->attachAxis(mpVelocityAxisY.get());

    mpASplineY->attachAxis(mpAxisX.get());
    mpASplineY->attachAxis(mpVelocityAxisY.get());
}

void Chart::setMaxX(qreal m)
{
    mpAxisX->setMax(m);
}

void Chart::setMinX(qreal m)
{
    mpAxisX->setMin(m);
}
