#include "jointplotwidget.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QComboBox>
#include <QCheckBox>
#include <QPushButton>
#include <QLabel>
#include <QToolTip>
#include <QFileDialog>
#include <QtCharts/QLegendMarker>
#include <QDateTime>
#include <QWheelEvent>
#include <QMouseEvent>
#include <functional>
#include <cmath>

namespace {

// QChartView implementing matplotlib's interactive-navigation standard:
//   - Left-drag  or  Middle-drag : pan
//   - Right-drag                 : zoom (horizontal -> X axis, vertical -> Y axis)
//   - Wheel                      : zoom about the cursor
// (see matplotlib "Interactive navigation" / NavigationToolbar). onInteract fires
// on any view change so the widget can stop auto-following the latest data.
class ChartView : public QChartView
{
public:
    ChartView(QChart *c, QWidget *p) : QChartView(c, p) {}
    std::function<void()> onInteract;
    void setAxes(QValueAxis *ax, QValueAxis *ay) { ax_ = ax; ay_ = ay; }

protected:
    void wheelEvent(QWheelEvent *e) override
    {
        if(onInteract) onInteract();
        const double f = e->angleDelta().y() > 0 ? 1.0 / 1.15 : 1.15;   // span factor
        const QPointF a = dataAt(e->position().toPoint());
        zoomAxis(ax_, a.x(), f);
        zoomAxis(ay_, a.y(), f);
        e->accept();
    }
    void mousePressEvent(QMouseEvent *e) override
    {
        // Only grab presses inside the plot area. Presses on the legend (or any
        // other scene item) must pass through so legend click-to-toggle works.
        if(!chart()->plotArea().contains(e->pos()))
        {
            QChartView::mousePressEvent(e);
            return;
        }

        if(e->button() == Qt::LeftButton || e->button() == Qt::MiddleButton)
            mode_ = Pan;
        else if(e->button() == Qt::RightButton)
            mode_ = Zoom;
        else { QChartView::mousePressEvent(e); return; }

        if(onInteract) onInteract();
        last_ = press_ = e->pos();
        anchor_ = dataAt(press_);
        if(ax_) { px0_ = ax_->min(); px1_ = ax_->max(); }
        if(ay_) { py0_ = ay_->min(); py1_ = ay_->max(); }
        setCursor(mode_ == Pan ? Qt::ClosedHandCursor : Qt::SizeAllCursor);
        e->accept();
    }
    void mouseMoveEvent(QMouseEvent *e) override
    {
        if(mode_ == Pan)
        {
            const QPoint d = e->pos() - last_;
            chart()->scroll(-d.x(), d.y());
            last_ = e->pos();
            e->accept();
            return;
        }
        if(mode_ == Zoom)
        {
            // matplotlib right-drag: drag right zooms X in, drag up zooms Y in;
            // scale grows with drag distance, anchored at the press point.
            const double sx = std::pow(10.0, (e->pos().x() - press_.x()) / 200.0);
            const double sy = std::pow(10.0, (press_.y() - e->pos().y()) / 200.0);
            if(ax_) ax_->setRange(anchor_.x() - (anchor_.x() - px0_) / sx,
                                  anchor_.x() + (px1_ - anchor_.x()) / sx);
            if(ay_) ay_->setRange(anchor_.y() - (anchor_.y() - py0_) / sy,
                                  anchor_.y() + (py1_ - anchor_.y()) / sy);
            e->accept();
            return;
        }
        QChartView::mouseMoveEvent(e);
    }
    void mouseReleaseEvent(QMouseEvent *e) override
    {
        if(mode_ != None)
        {
            mode_ = None;
            unsetCursor();
            e->accept();
            return;
        }
        QChartView::mouseReleaseEvent(e);
    }

private:
    // Widget pixel -> data coordinates using the plot area and axis ranges.
    QPointF dataAt(const QPoint &px) const
    {
        const QRectF pa = chart()->plotArea();
        double x = 0, y = 0;
        if(ax_ && pa.width() > 0)
            x = ax_->min() + (px.x() - pa.left()) / pa.width() * (ax_->max() - ax_->min());
        if(ay_ && pa.height() > 0)
            y = ay_->min() + (pa.bottom() - px.y()) / pa.height() * (ay_->max() - ay_->min());
        return QPointF(x, y);
    }
    // Scale one axis' span by `f` about anchor value `a`.
    static void zoomAxis(QValueAxis *ax, double a, double f)
    {
        if(!ax) return;
        ax->setRange(a - (a - ax->min()) * f, a + (ax->max() - a) * f);
    }

    enum Mode { None, Pan, Zoom } mode_ = None;
    QValueAxis *ax_ = nullptr, *ay_ = nullptr;
    QPoint  last_, press_;
    QPointF anchor_;
    double  px0_ = 0, px1_ = 0, py0_ = 0, py1_ = 0;
};

}

JointPlotWidget::JointPlotWidget(QWidget *parent)
    : QWidget(parent)
{
    // ---- controls ----------------------------------------------------------
    signalCombo_ = new QComboBox(this);
    signalCombo_->addItems({"position", "speed", "load", "current", "temperature", "voltage"});

    goalCheck_ = new QCheckBox("Goal overlay", this);
    goalCheck_->setToolTip("Overlay each joint's commanded goal (dashed) against its actual position.");

    playPause_ = new QPushButton("Pause", this);   // starts live, so button offers Pause
    auto *clearBtn = new QPushButton("Clear", this);
    record_ = new QPushButton("● Record", this);
    auto *zin  = new QPushButton("Zoom +", this);
    auto *zout = new QPushButton("Zoom −", this);
    auto *zrst = new QPushButton("Reset", this);
    status_ = new QLabel(this);

    auto *bar = new QHBoxLayout();
    bar->addWidget(new QLabel("Signal", this));
    bar->addWidget(signalCombo_);
    bar->addWidget(goalCheck_);
    bar->addSpacing(12);
    bar->addWidget(playPause_);
    bar->addWidget(clearBtn);
    bar->addWidget(record_);
    bar->addSpacing(12);
    bar->addWidget(zin);
    bar->addWidget(zout);
    bar->addWidget(zrst);
    bar->addStretch(1);
    bar->addWidget(status_);

    // ---- chart -------------------------------------------------------------
    chart_ = new QChart();
    chart_->setAnimationOptions(QChart::NoAnimation);   // live data: no animation lag
    chart_->legend()->setVisible(true);
    chart_->legend()->setAlignment(Qt::AlignRight);

    axisX_ = new QValueAxis();
    axisX_->setTitleText("time (s)");
    axisX_->setRange(0, window_s_);
    axisY_ = new QValueAxis();
    axisY_->setTitleText("position");
    axisY_->setRange(0, 4095);
    chart_->addAxis(axisX_, Qt::AlignBottom);
    chart_->addAxis(axisY_, Qt::AlignLeft);

    auto *cv = new ChartView(chart_, this);
    cv->setAxes(axisX_, axisY_);
    cv->onInteract = [this]{ following_ = false; };          // user took control of the view
    view_ = cv;
    view_->setRenderHint(QPainter::Antialiasing);
    view_->setToolTip("Navigation (matplotlib-style):\n"
                      "  wheel — zoom about cursor\n"
                      "  left-drag / middle-drag — pan\n"
                      "  right-drag — zoom (horizontal→X, vertical→Y)\n"
                      "  Reset — home view");
    // Navigation (matplotlib standard): wheel = zoom about cursor, left/middle-drag
    // = pan, right-drag = zoom (horizontal→X, vertical→Y), Reset = home.

    auto *outer = new QVBoxLayout(this);
    outer->addLayout(bar);
    outer->addWidget(view_, 1);

    connect(signalCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &JointPlotWidget::onSignalChanged);
    connect(goalCheck_, &QCheckBox::toggled, this, [this](bool on){
        for(auto &kv : goal_) kv.second->setVisible(on && signalCombo_->currentText() == "position");
        emit goalOverlayToggled(on);
    });
    connect(playPause_, &QPushButton::clicked, this, &JointPlotWidget::onPlayPause);
    connect(clearBtn,   &QPushButton::clicked, this, &JointPlotWidget::onClear);
    connect(record_,    &QPushButton::clicked, this, &JointPlotWidget::onRecordToggle);
    connect(zin,  &QPushButton::clicked, this, &JointPlotWidget::zoomIn);
    connect(zout, &QPushButton::clicked, this, &JointPlotWidget::zoomOut);
    connect(zrst, &QPushButton::clicked, this, &JointPlotWidget::zoomReset);
}

QString JointPlotWidget::currentSignal() const { return signalCombo_->currentText(); }
bool JointPlotWidget::goalOverlayOn() const { return goalCheck_->isChecked(); }

// A readable, well-separated categorical palette (distinct hues).
QColor JointPlotWidget::colorForIndex(int i) const
{
    static const QColor palette[] = {
        QColor("#1f77b4"), QColor("#ff7f0e"), QColor("#2ca02c"), QColor("#d62728"),
        QColor("#9467bd"), QColor("#8c564b"), QColor("#e377c2"), QColor("#17becf"),
        QColor("#bcbd22"), QColor("#7f7f7f"), QColor("#393b79"), QColor("#637939"),
        QColor("#8c6d31"), QColor("#843c39"), QColor("#7b4173"), QColor("#3182bd"),
        QColor("#e6550d"), QColor("#31a354"), QColor("#756bb1"), QColor("#636363"),
    };
    return palette[i % (int)(sizeof(palette)/sizeof(palette[0]))];
}

void JointPlotWidget::setJoints(const std::vector<JointInfo> &joints)
{
    for(auto &kv : present_) { chart_->removeSeries(kv.second); delete kv.second; }
    for(auto &kv : goal_)    { chart_->removeSeries(kv.second); delete kv.second; }
    present_.clear(); goal_.clear(); names_.clear();
    color_cursor_ = 0;
    yInit_ = false;

    for(const auto &j : joints)
    {
        names_[j.id] = j.name;
        QColor c = colorForIndex(color_cursor_++);

        auto *s = new QLineSeries();
        s->setName(QString("%1: %2").arg(j.id).arg(j.name));
        s->setColor(c);
        chart_->addSeries(s);
        s->attachAxis(axisX_);
        s->attachAxis(axisY_);
        const uint8_t id = j.id;
        connect(s, &QLineSeries::hovered, this,
                [this, id](const QPointF &pt, bool st){ onSeriesHovered(pt, st, id); });
        present_[j.id] = s;

        auto *g = new QLineSeries();
        g->setName(QString("%1: %2 goal").arg(j.id).arg(j.name));
        QPen gp(c); gp.setStyle(Qt::DashLine); g->setPen(gp);
        chart_->addSeries(g);
        g->attachAxis(axisX_);
        g->attachAxis(axisY_);
        g->setVisible(false);
        goal_[j.id] = g;
    }
    wireLegendToggling();
}

void JointPlotWidget::wireLegendToggling()
{
    // Click a legend entry to show/hide that joint's line.
    const auto markers = chart_->legend()->markers();
    for(auto *m : markers)
    {
        disconnect(m, &QLegendMarker::clicked, nullptr, nullptr);
        connect(m, &QLegendMarker::clicked, this, [m]{
            m->series()->setVisible(!m->series()->isVisible());
            m->setVisible(true);
            qreal a = m->series()->isVisible() ? 1.0 : 0.4;
            QColor lc = m->labelBrush().color(); lc.setAlphaF(a); m->setLabelBrush(lc);
        });
    }
}

void JointPlotWidget::addSample(uint8_t id, double t, int value)
{
    if(!live_) return;
    auto it = present_.find(id);
    if(it == present_.end() || value < 0) return;
    it->second->append(t, value);
    if(recorder_.isRecording())
        recorder_.write(t, id, names_[id], value);
    // bound memory
    if(it->second->count() > 60000)
        it->second->removePoints(0, it->second->count() - 60000);

    if(!yInit_) { yMin_ = yMax_ = value; yInit_ = true; }
    yMin_ = qMin<double>(yMin_, value);
    yMax_ = qMax<double>(yMax_, value);
    trimAndFollow(t);
}

void JointPlotWidget::addGoalSample(uint8_t id, double t, int value)
{
    if(!live_) return;
    auto it = goal_.find(id);
    if(it == goal_.end() || value < 0) return;
    it->second->append(t, value);
    if(it->second->count() > 60000)
        it->second->removePoints(0, it->second->count() - 60000);
}

void JointPlotWidget::trimAndFollow(double t)
{
    // Only auto-scroll while following. Once the user zooms/pans, following is off
    // and their view is left untouched until they press Reset.
    if(!following_)
        return;
    const double lo = qMax(0.0, t - window_s_);
    axisX_->setRange(lo, qMax(t, window_s_));
    const double pad = qMax(1.0, (yMax_ - yMin_) * 0.05);
    axisY_->setRange(yMin_ - pad, yMax_ + pad);
}

void JointPlotWidget::onSignalChanged()
{
    const QString sig = signalCombo_->currentText();
    axisY_->setTitleText(sig);
    // switching signal starts a fresh trace
    onClear();
    // goal overlay only makes sense for position
    const bool goalOk = (sig == "position") && goalCheck_->isChecked();
    for(auto &kv : goal_) kv.second->setVisible(goalOk);
    recorder_.setSignal(sig);
    emit signalSelected(sig);
}

void JointPlotWidget::onPlayPause()
{
    live_ = !live_;
    playPause_->setText(live_ ? "Pause" : "Play");
    status_->setText(live_ ? "" : "paused");
}

void JointPlotWidget::onClear()
{
    for(auto &kv : present_) kv.second->clear();
    for(auto &kv : goal_)    kv.second->clear();
    yInit_ = false;
    following_ = true;                 // resume live follow after a clear
    axisX_->setRange(0, window_s_);
}

void JointPlotWidget::onRecordToggle()
{
    if(recorder_.isRecording())
    {
        recorder_.stop();
        record_->setText("● Record");
        status_->setText("saved");
        return;
    }
    const QString path = QFileDialog::getSaveFileName(
        this, "Record joints to CSV",
        QString("joints_%1.csv").arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss")),
        "CSV (*.csv)");
    if(path.isEmpty()) return;
    if(recorder_.start(path, signalCombo_->currentText()))
    {
        record_->setText("■ Stop");
        status_->setText("recording…");
    }
    else
    {
        status_->setText("record failed");
    }
}

void JointPlotWidget::zoomIn()  { chart_->zoomIn(); }
void JointPlotWidget::zoomOut() { chart_->zoomOut(); }
void JointPlotWidget::zoomReset()
{
    chart_->zoomReset();
    following_ = true;                  // resume auto-follow
    axisX_->setRange(0, window_s_);
}

void JointPlotWidget::onSeriesHovered(const QPointF &pt, bool state, uint8_t id)
{
    if(state)
        QToolTip::showText(QCursor::pos(),
            QString("%1  t=%2s  %3=%4")
                .arg(names_.count(id) ? names_[id] : QString::number(id))
                .arg(pt.x(), 0, 'f', 2)
                .arg(signalCombo_->currentText())
                .arg(qRound(pt.y())));
    else
        QToolTip::hideText();
}
