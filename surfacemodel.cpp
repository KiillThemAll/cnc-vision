#include <QQmlEngine>

#include "surfacemodel.h"

SurfaceModel::SurfaceModel(QObject *parent)
{

}

int SurfaceModel::rowCount(const QModelIndex &parent) const
{
    Q_UNUSED(parent);
    return m_surface.count();
}

QVariant SurfaceModel::data(const QModelIndex &index, int role) const
{
    if (index.row() < 0 || index.row() >= m_surface.count())
        return QVariant();

    const SurfacePoint& point = m_surface.at(index.row());
    if (role == x) return point.x;
    else if (role == y) return point.y;
    else if (role == z) return point.z;
    return QVariant();
}

void SurfaceModel::createZeroSurface(int width, int height, int step)
{
    surfaceRowNum = height/step+1;
    surfaceColNum = width/step+1;

    beginInsertRows(QModelIndex(), 0, ((surfaceColNum)*(surfaceRowNum)-1));

    SurfacePoint point;

    for (int i=0; i<surfaceRowNum; i++)
    {
        if (i%2)
        {
            point.x = (surfaceColNum-1)*step;
            point.y = step*i;
            point.z = 0;
            m_surface.append(point);
        }
        else
        {
            point.x = 0;
            point.y = step*i;
            point.z = 0;
            m_surface.append(point);
        }

        for (int j=1; j<surfaceColNum; j++)
            if (i%2)
            {
                point.x = (surfaceColNum-1)*step-step*j;
                point.y = step*i;
                point.z = 0;
                m_surface.append(point);
            }
            else
            {
                point.x = step*j;
                point.y = step*i;
                point.z = 0;
                m_surface.append(point);
            }
    }

    endInsertRows();
}

void SurfaceModel::updatePoint(const SurfacePoint &point, int pos)
{
    if (pos < 0 || pos > m_surface.count())
        return;
    QModelIndex modelIndex = createIndex(pos, 0);
    SurfacePoint temp = m_surface.at(pos);
    temp.z = point.z;
    m_surface.replace(pos,temp);
    emit dataChanged(modelIndex, modelIndex);
}

void SurfaceModel::updatePoints(const QVector<SurfacePoint> &points)
{   
    removeAll();
    beginInsertRows(QModelIndex(), 0, points.count()-1);
    m_surface.append(points);
    endInsertRows();
}

QHash<int, QByteArray> SurfaceModel::roleNames() const
{
    QHash<int, QByteArray> roles;
    roles[x] = "x";
    roles[y] = "y";
    roles[z] = "z";
    return roles;
}

void SurfaceModel::removeAll()
{
    beginRemoveRows(QModelIndex(), 0, m_surface.count()-1);
    m_surface.clear();
    m_surface.squeeze();
    endRemoveRows();
}

void SurfaceModel::sortPoints()
{
    if (!m_surface.count())
        return;

    /*int colNum = 1;
    int current = 0;
    QVector<SurfacePoint>::const_iterator it;
    it = m_surface.begin();
    it++;
    while(it!=m_surface.end() && it->x > current) {colNum++; current = it->x; it++;};

    int rowNum = m_surface.count() / colNum;*/

    m_surfaceSorted.clear();
    m_surfaceSorted.squeeze();

    m_surfaceSorted.reserve(m_surface.count());

    for (int i=0; i<surfaceRowNum; i++)
    {
        for (int j=0; j<surfaceColNum; j++)
            if (i%2)
                m_surfaceSorted.append(m_surface.at(surfaceColNum-j+surfaceColNum*i-1));
            else
                m_surfaceSorted.append(m_surface.at(j+surfaceColNum*i));
    }
}
