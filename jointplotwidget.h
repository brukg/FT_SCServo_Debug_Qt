#ifndef JOINTPLOTWIDGET_H
#define JOINTPLOTWIDGET_H

#include <QWidget>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <map>
#include <vector>
#include "csvrecorder.h"

QT_CHARTS_USE_NAMESPACE

class QComboBox;
class QCheckBox;
class QPushButton;
class QLabel;

// A comprehensive multi-joint time-series plot, reusable on both the Debug and
// Joint Control tabs. One line per joint for the selected signal, with:
//   - a signal selector (position / speed / load / current / temperature / voltage)
//   - an optional dashed "goal" overlay per joint
//   - transport: Play (live) / Pause (freeze) / Clear / Record→CSV / Stop
//   - zoom (rubber-band + buttons + reset) and pan
//   - hover tooltip naming the joint and its value
//   - a clickable legend (toggle a joint's line)
//
// The widget is a view: the owner reads the selected signal from hardware and
// feeds samples in via addSample()/addGoalSample(). It emits signalSelected()
// so the owner knows which register to read.
class JointPlotWidget : public QWidget
{
    Q_OBJECT
public:
    explicit JointPlotWidget(QWidget *parent = nullptr);

    struct JointInfo { uint8_t id; QString name; };
    void setJoints(const std::vector<JointInfo> &joints);   // rebuild series

    // Feed one sample for a joint. t is seconds since the plot started.
    void addSample(uint8_t id, double t, int value);
    void addGoalSample(uint8_t id, double t, int value);

    QString currentSignal() const;   // e.g. "position"
    bool    goalOverlayOn() const;
    bool    isLive() const { return live_; }

signals:
    void signalSelected(const QString &signal);   // owner should read this register
    void goalOverlayToggled(bool on);

private:
    void onSignalChanged();
    void onPlayPause();
    void onClear();
    void onRecordToggle();
    void zoomIn();
    void zoomOut();
    void zoomReset();
    void onSeriesHovered(const QPointF &pt, bool state, uint8_t id);
    void wireLegendToggling();
    QColor colorForIndex(int i) const;
    void trimAndFollow(double t);

    QChart      *chart_ = nullptr;
    QChartView  *view_ = nullptr;
    QValueAxis  *axisX_ = nullptr;
    QValueAxis  *axisY_ = nullptr;

    std::map<uint8_t, QLineSeries*> present_;
    std::map<uint8_t, QLineSeries*> goal_;
    std::map<uint8_t, QString>      names_;

    QComboBox   *signalCombo_ = nullptr;
    QCheckBox   *goalCheck_ = nullptr;
    QPushButton *playPause_ = nullptr;
    QPushButton *record_ = nullptr;
    QLabel      *status_ = nullptr;

    bool   live_ = true;
    bool   following_ = true;    // auto-scroll to latest; off once the user zooms/pans
    double window_s_ = 20.0;     // rolling window shown while live
    double yMin_ = 0, yMax_ = 4095;
    bool   yInit_ = false;
    CsvRecorder recorder_;
    int    color_cursor_ = 0;
};

#endif // JOINTPLOTWIDGET_H
