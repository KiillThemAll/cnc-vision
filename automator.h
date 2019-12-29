#ifndef AUTOMATOR_H
#define AUTOMATOR_H

#include <QObject>
#include <QTimer>
#include <QThread>
#include <QFile>
#include <QTextStream>
#include <QUrl>

#include "rayreceiver.h"
#include "capturecontroller.hpp"
#include "surfacemodel.h"
#include "gcodeplayer.h"


class Automator : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool working READ working NOTIFY workingChanged)
    Q_PROPERTY(bool enabled READ enabled WRITE setEnabled NOTIFY enabledChanged)
    Q_PROPERTY(bool cutModeEnabled READ cutModeEnabled WRITE setCutModeEnabled)
    Q_PROPERTY(bool scanApproved READ scanApproved WRITE setScanApproved)
    Q_PROPERTY(QString message READ message NOTIFY messageChanged)
    Q_PROPERTY(bool autosendPower READ autosendPower WRITE setAutosendPower)
    Q_PROPERTY(float minPower READ minPower WRITE setMinPower)
    Q_PROPERTY(float maxPower READ maxPower WRITE setMaxPower)
    Q_PROPERTY(float lastSentPower READ lastSentPower NOTIFY changePower)
    Q_PROPERTY(SurfaceModel *surfaceModel READ surfaceModel CONSTANT)
public:
    explicit Automator(QObject *parent = nullptr);

    bool working() const;

    bool enabled() const;
    void setEnabled(bool enabled);

    QString message() const;
    Q_INVOKABLE float compensate(float dz) const;

    bool cutModeEnabled() const;
    void setCutModeEnabled(bool cutMode);

    bool scanApproved() const;
    void setScanApproved(bool scanApproved);

    bool autosendPower() const;
    void setAutosendPower(bool autosendPower);

    float maxPower() const;
    void setMaxPower(float maxPower);

    float minPower() const;
    void setMinPower(float minPower);

    float lastSentPower() const;

    Q_INVOKABLE void scanSurface(int width, int height, int step);

    SurfaceModel *surfaceModel() const;

    Q_INVOKABLE void clearSurface() const;

signals:
    void workingChanged();
    void enabledChanged();
    void messageChanged();
    void sendToMC(const QString &command);
    void changePower(float power);
    void startScan(const QUrl &fileUrl);
    void continueScan();

public slots:
    void ondzChanged(float dz);
    void ondzValidChanged(bool valid);
    void onMcConnectionStateChanged(bool connected);
    void onRayConnectionStateChanged(bool connected);
    void onCoordsChanged(float x, float y, float z, float b);
    void onMcStateChanged(RayReceiver::State s);
    void onCameraStateChanged(CaptureController::Status s);
    void compensate();
    void scanSnapshot(GcodePlayer::State s);

private:
    void checkWorkingState();
    bool m_working;
    float m_lastdz;
    bool m_lastdzValid;
    bool m_lastCoordsValid;
    bool m_enabled;
    bool m_mcConnected;
    bool m_cameraConnected;
    float m_mcs_x;
    float m_mcs_y;
    float m_mcs_x_check_state;
    float m_mcs_y_check_state;
    float m_mcs_b;
    QString m_message;
    bool m_autosendPower;
    float m_maxPower;
    float m_minPower;
    float m_lastSentPower;
    QTimer m_powerTimer;
    bool m_compensatorOneShot;
    bool m_scanOneShot;
    bool m_cutModeEnabled;
    bool m_scanApproved;
    SurfaceModel *m_surfaceModel;

private slots:
    void m_scanSnapshot();

};

#endif // AUTOMATOR_H
