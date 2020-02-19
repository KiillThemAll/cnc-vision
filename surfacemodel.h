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
        x = Qt::UserRole + 1,
        y,
        z
    };

    int rowCount(const QModelIndex& parent = QModelIndex()) const override;
    QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;

    void createZeroSurface(int width, int height, int step);
    void updatePoint(const SurfacePoint &point, int pos);
    void updatePoints(const QVector<SurfacePoint> &points);

    void removeAll();

    void sortPoints();

    QVector<SurfacePoint> m_surface;

    QVector<SurfacePoint> m_surfaceSorted;

    int surfaceRowNum;
    int surfaceColNum;

protected:
    QHash<int, QByteArray> roleNames() const override;

signals:

private:


};


#endif // SURFACEMODEL_H
