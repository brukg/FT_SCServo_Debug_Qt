#include "jointcontroltab.h"
#include "jointrow.h"
#include "jointplotwidget.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QSpinBox>
#include <QLabel>
#include <QScrollArea>
#include <QSplitter>
#include <QSlider>
#include <QComboBox>
#include <QTimer>

JointControlTab::JointControlTab(QWidget *parent)
    : QWidget(parent)
{
    // ---- master control bar ------------------------------------------------
    auto *torqueOff = new QPushButton("Torque OFF All", this);
    auto *torqueOn  = new QPushButton("Torque ON All", this);
    auto *selectAll = new QPushButton("Select-all Sync", this);
    selectAll->setCheckable(true);

    goalSlider_ = new QSlider(Qt::Horizontal, this);
    goalSlider_->setRange(0, 4095); goalSlider_->setValue(2048);
    goalSlider_->setMinimumWidth(200);
    goalSlider_->setToolTip("Drag to move every Sync-checked joint to this value, live —\n"
                            "same as an individual joint's slider, but for the whole group.");
    goal_   = new QSpinBox(this); goal_->setRange(0, 4095); goal_->setValue(2048);

    // Slider and spinbox mirror each other; dragging the group slider drives all
    // armed joints LIVE, consistent with the per-joint sliders (no separate button).
    connect(goalSlider_, &QSlider::valueChanged, goal_, &QSpinBox::setValue);
    connect(goal_, QOverload<int>::of(&QSpinBox::valueChanged), goalSlider_, &QSlider::setValue);
    connect(goal_, QOverload<int>::of(&QSpinBox::valueChanged), this, &JointControlTab::onSyncWriteClicked);

    speed_  = new QSpinBox(this); speed_->setRange(0, 65535); speed_->setValue(600);
    acc_    = new QSpinBox(this); acc_->setRange(0, 255);     acc_->setValue(50);
    torque_ = new QSpinBox(this); torque_->setRange(0, 1000); torque_->setValue(500);

    auto *bar = new QHBoxLayout();
    bar->addWidget(torqueOff);
    bar->addWidget(torqueOn);
    bar->addWidget(selectAll);
    bar->addStretch(1);
    bar->addWidget(new QLabel("Armed joints →", this));
    bar->addWidget(goalSlider_);
    bar->addWidget(goal_);
    bar->addWidget(new QLabel("  speed", this)); bar->addWidget(speed_);
    bar->addWidget(new QLabel("acc", this));    bar->addWidget(acc_);
    bar->addWidget(new QLabel("torque", this)); bar->addWidget(torque_);

    // ---- compliance bar (acts on armed joints) -----------------------------
    modeCombo_ = new QComboBox(this);
    modeCombo_->addItems({"Position", "Wheel", "Current (compliant)"});
    modeCombo_->setToolTip("Work mode for all joints. 'Current (compliant)' puts\n"
                           "HLS joints in force mode; 'Position' is normal servo mode.");
    stiffnessSlider_ = new QSlider(Qt::Horizontal, this);
    stiffnessSlider_->setRange(0, 254); stiffnessSlider_->setValue(32);   // Kp; 32 = factory default
    stiffnessSlider_->setMinimumWidth(220);
    stiffnessSlider_->setToolTip("Stiffness = position Kp (SRAM reg 50) of all HLS joints.\n"
                                 "High = stiff/rigid, low = soft/compliant. 32 = default. Live, no EPROM wear.");
    stiffness_ = new QSpinBox(this); stiffness_->setRange(0, 254); stiffness_->setValue(32);

    connect(stiffnessSlider_, &QSlider::valueChanged, stiffness_, &QSpinBox::setValue);
    connect(stiffness_, QOverload<int>::of(&QSpinBox::valueChanged), stiffnessSlider_, &QSlider::setValue);
    connect(stiffness_, QOverload<int>::of(&QSpinBox::valueChanged), this, &JointControlTab::onStiffnessChanged);
    connect(modeCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &JointControlTab::onModeChanged);

    auto *cbar = new QHBoxLayout();
    cbar->addWidget(new QLabel("Compliance — all joints:", this));
    cbar->addWidget(new QLabel("Mode", this)); cbar->addWidget(modeCombo_);
    cbar->addSpacing(12);
    cbar->addWidget(new QLabel("Stiffness (Kp)", this));
    cbar->addWidget(stiffnessSlider_);
    cbar->addWidget(stiffness_);
    cbar->addStretch(1);

    // ---- scrollable row list ----------------------------------------------
    auto *rowHost = new QWidget(this);
    rowLayout_ = new QVBoxLayout(rowHost);
    rowLayout_->setContentsMargins(0, 0, 0, 0);
    rowLayout_->setSpacing(2);
    rowLayout_->addStretch(1);

    auto *scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    scroll->setWidget(rowHost);

    // rows on top, the multi-joint plot below, user-resizable via a splitter
    plot_ = new JointPlotWidget(this);
    auto *split = new QSplitter(Qt::Vertical, this);
    split->addWidget(scroll);
    split->addWidget(plot_);
    split->setStretchFactor(0, 1);
    split->setStretchFactor(1, 2);

    auto *outer = new QVBoxLayout(this);
    outer->addLayout(bar);
    outer->addLayout(cbar);
    outer->addWidget(split, 1);

    connect(torqueOff, &QPushButton::clicked, this, [this]{ emit torqueAllRequested(false); });
    connect(torqueOn,  &QPushButton::clicked, this, [this]{ emit torqueAllRequested(true); });
    connect(selectAll, &QPushButton::toggled, this, &JointControlTab::onSelectAllSync);

    poll_ = new QTimer(this);
    poll_->setInterval(60);
    connect(poll_, &QTimer::timeout, this, [this]{ emit pollTick(); });
}

int JointControlTab::speed() const  { return speed_->value(); }
int JointControlTab::acc() const    { return acc_->value(); }
int JointControlTab::torque() const { return torque_->value(); }

void JointControlTab::setServos(const std::vector<feetech_servo::GroupTarget> &servos)
{
    // tear down old rows
    for(auto *r : rows_)
    {
        rowLayout_->removeWidget(r);
        r->deleteLater();
    }
    rows_.clear();

    // insert before the trailing stretch (last item). s.pos carries the servo's
    // present position (read by the owner), so the adjuster starts there.
    for(const auto &s : servos)
    {
        auto *row = new JointRow(s.id, s.profile, s.pos, this);
        connect(row, &JointRow::torqueToggled, this, &JointControlTab::torqueToggled);
        connect(row, &JointRow::jogged,        this, &JointControlTab::jogged);
        rowLayout_->insertWidget(rowLayout_->count() - 1, row);
        rows_.push_back(row);
    }

    // mirror the joint set into the plot's series
    std::vector<JointPlotWidget::JointInfo> infos;
    for(const auto &s : servos)
        infos.push_back({s.id, s.profile.name});
    plot_->setJoints(infos);
}

std::vector<uint8_t> JointControlTab::rowIds() const
{
    std::vector<uint8_t> ids;
    ids.reserve(rows_.size());
    for(auto *r : rows_)
        ids.push_back(r->id());
    return ids;
}

void JointControlTab::setPresentPosition(uint8_t id, int pos)
{
    for(auto *r : rows_)
        if(r->id() == id)
        {
            r->setPresentPosition(pos);
            return;
        }
}

void JointControlTab::reflectTorque(uint8_t id, bool on)
{
    for(auto *r : rows_)
        if(r->id() == id)
        {
            r->setTorque(on);
            return;
        }
}

void JointControlTab::setPollActive(bool active)
{
    if(active && !rows_.empty())
        poll_->start();
    else
        poll_->stop();
}

void JointControlTab::onSelectAllSync(bool on)
{
    for(auto *r : rows_)
        r->setSyncArmed(on);
}

std::vector<feetech_servo::GroupTarget> JointControlTab::armedTargets(int pos) const
{
    std::vector<feetech_servo::GroupTarget> armed;
    for(auto *r : rows_)
        if(r->isSyncArmed() && r->profile().known)
            armed.push_back({r->id(), r->profile(), pos});
    return armed;
}

std::vector<feetech_servo::GroupTarget> JointControlTab::allTargets() const
{
    std::vector<feetech_servo::GroupTarget> all;
    for(auto *r : rows_)
        if(r->profile().known)
            all.push_back({r->id(), r->profile(), 0});
    return all;
}

void JointControlTab::onSyncWriteClicked()
{
    // Sync Write sends the SAME master Goal value to every armed joint, in one
    // command per series. (Per-joint live positioning is the row sliders.)
    emit syncWriteRequested(armedTargets(goal_->value()));
}

void JointControlTab::onModeChanged()
{
    // Mode/stiffness are per-joint properties, unrelated to the sync-write group.
    emit modeRequested(modeCombo_->currentIndex(), allTargets());
}

void JointControlTab::onStiffnessChanged()
{
    emit stiffnessRequested(stiffness_->value(), allTargets());
}
