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

void SurfaceModel::addPoint(const SurfacePoint &point)
{
    beginInsertRows(QModelIndex(), m_surface.count(), m_surface.count());
    m_surface.append(point);
    endInsertRows();
}

void SurfaceModel::updatePoints(const QVector<SurfacePoint> &points)
{
    removeAll();
    beginInsertRows(QModelIndex(), 0, m_surface.count());
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
    beginRemoveRows(QModelIndex(), 0, m_surface.count());
    m_surface.clear();
    endRemoveRows();
}

