/**
******************************************************************************
*
* @file       uavitem.cpp
* @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
*             Parts by Nokia Corporation (qt-info@nokia.com) Copyright (C) 2009.
* @brief      A graphicsItem representing a UAV
* @see        The GNU Public License (GPL) Version 3
* @defgroup   OPMapWidget
* @{
*
*****************************************************************************/
/*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
* for more details.
*
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/
#include "../internals/pureprojection.h"
#include "uavitem.h"
namespace mapcontrol
{
    UAVItem::UAVItem(MapGraphicItem* map,OPMapWidget* parent):map(map),mapwidget(parent),showtrail(true),trailtime(5),traildistance(100)
    {
        pic.load(QString::fromUtf8(":/markers/images/airplane.svg"));
        pic=pic.scaled(30,30,Qt::IgnoreAspectRatio);
        localposition=map->FromLatLngToLocal(mapwidget->CurrentPosition());
        this->setPos(localposition.X(),localposition.Y());
        this->setZValue(4);
        trail=new QGraphicsItemGroup();
        trail->setParentItem(map);


        this->setFlag(QGraphicsItem::ItemIgnoresTransformations,true);
        mapfollowtype=UAVMapFollowType::None;
        trailtype=UAVTrailType::ByDistance;
        timer.start();

       // rect=QRectF(0,0,renderer.defaultSize().width()*0.05,renderer.defaultSize().height()*0.05);

    }
    UAVItem::~UAVItem()
    {
        delete trail;
    }

    void UAVItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
    {
        painter->rotate(-90);
        painter->drawPixmap(-pic.width()/2,-pic.height()/2,pic);
       //   painter->drawRect(QRectF(-pic.width()/2,-pic.height()/2,pic.width()-1,pic.height()-1));
    }
    QRectF UAVItem::boundingRect()const
    {
        return QRectF(-pic.width()/2,-pic.height()/2,pic.width(),pic.height());;
    }
    void UAVItem::SetUAVPos(const internals::PointLatLng &position, const int &altitude)
    {
        if(coord.IsEmpty())
            lastcoord=coord;
        if(coord!=position)
        {

            if(trailtype==UAVTrailType::ByTimeElapsed)
            {
                if(timer.elapsed()>trailtime*1000)
                {
                    trail->addToGroup(new TrailItem(position,altitude,this));
                    timer.restart();
                }

            }
            else if(trailtype==UAVTrailType::ByDistance)
            {
                if(qAbs(internals::PureProjection::DistanceBetweenLatLng(lastcoord,position)*1000)>traildistance)
                {
                    trail->addToGroup(new TrailItem(position,altitude,this));
                    lastcoord=position;
                }
            }
            coord=position;
            this->altitude=altitude;
            RefreshPos();
            if(mapfollowtype==UAVMapFollowType::CenterAndRotateMap||mapfollowtype==UAVMapFollowType::CenterMap)
            {
                mapwidget->SetCurrentPosition(coord);
            }
            this->update();
        }
    }
    void UAVItem::SetUAVHeading(const qreal &value)
    {
        if(mapfollowtype==UAVMapFollowType::CenterAndRotateMap)
        {
            mapwidget->SetRotate(-value);
        }
        else
            this->setRotation(value);
    }
    int UAVItem::type()const
    {
        return Type;
    }
    void UAVItem::RefreshPos()
    {
        localposition=map->FromLatLngToLocal(coord);
        this->setPos(localposition.X(),localposition.Y());
        foreach(QGraphicsItem* i,trail->childItems())
        {
            TrailItem* w=qgraphicsitem_cast<TrailItem*>(i);
            if(w)
                w->setPos(map->FromLatLngToLocal(w->coord).X(),map->FromLatLngToLocal(w->coord).Y());
            //this->update();
        }
    }
    void UAVItem::SetTrailType(const UAVTrailType::Types &value)
    {
        trailtype=value;
        if(trailtype==UAVTrailType::ByTimeElapsed)
            timer.restart();
    }
    void UAVItem::SetShowTrail(const bool &value)
    {
        showtrail=value;
        trail->setVisible(value);
    }
    void UAVItem::DeleteTrail()const
    {
        foreach(QGraphicsItem* i,trail->childItems())
            delete i;
    }
}