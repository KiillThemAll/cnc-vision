#ifndef SURFACEMODEL_H
#define SURFACEMODEL_H

#include <QObject>
#include <QAbstractListModel>
#include "gcodeplayeritem.h"

struct SurfacePoint
{
    //SurfacePoint(float inpX, float inpY, float inpZ){x=inpX;y=inpY;z=inpZ;};

    float x;
    float y;
    float z;
};

class SurfaceModel : public QAbstractListModel
{
    Q_OBJECT
public:
    SurfaceModel(QObject *parent = nullptr);

    enum ItemRoles {
        StatusRole = Qt::UserRole + 1,
        x,
        y,
        z
    };

    int rowCount(const QModelIndex& parent = QModelIndex()) const override;
    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;

    void addPoint(const SurfacePoint &point);
    void updatePoints(const QVector<SurfacePoint> &points);

    void removeAll();

protected:
    QHash<int, QByteArray> roleNames() const override;

signals:

private:

    QVector<SurfacePoint> m_surface;
};


#endif // SURFACEMODEL_H
