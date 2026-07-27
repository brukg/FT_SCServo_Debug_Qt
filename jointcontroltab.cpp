#include "jointcontroltab.h"
#include "jointrow.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QSpinBox>
#include <QLabel>
#include <QScrollArea>
#include <QTimer>

JointControlTab::JointControlTab(QWidget *parent)
    : QWidget(parent)
{
    // ---- master control bar ------------------------------------------------
    auto *torqueOff = new QPushButton("Torque OFF All", this);
    auto *torqueOn  = new QPushButton("Torque ON All", this);
    auto *selectAll = new QPushButton("Select-all Sync", this);
    selectAll->setCheckable(true);

    goal_   = new QSpinBox(this); goal_->setRange(0, 4095); goal_->setValue(2048);
    auto *syncWrite = new QPushButton("Sync Write → Goal", this);
    syncWrite->setToolTip("Write the Goal value to every Sync-checked joint, in one command.");

    speed_  = new QSpinBox(this); speed_->setRange(0, 65535); speed_->setValue(600);
    acc_    = new QSpinBox(this); acc_->setRange(0, 255);     acc_->setValue(50);
    torque_ = new QSpinBox(this); torque_->setRange(0, 1000); torque_->setValue(500);

    auto *bar = new QHBoxLayout();
    bar->addWidget(torqueOff);
    bar->addWidget(torqueOn);
    bar->addWidget(selectAll);
    bar->addStretch(1);
    bar->addWidget(new QLabel("Goal", this));   bar->addWidget(goal_);
    bar->addWidget(syncWrite);
    bar->addWidget(new QLabel("  speed", this)); bar->addWidget(speed_);
    bar->addWidget(new QLabel("acc", this));    bar->addWidget(acc_);
    bar->addWidget(new QLabel("torque", this)); bar->addWidget(torque_);

    // ---- scrollable row list ----------------------------------------------
    auto *rowHost = new QWidget(this);
    rowLayout_ = new QVBoxLayout(rowHost);
    rowLayout_->setContentsMargins(0, 0, 0, 0);
    rowLayout_->setSpacing(2);
    rowLayout_->addStretch(1);

    auto *scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    scroll->setWidget(rowHost);

    auto *outer = new QVBoxLayout(this);
    outer->addLayout(bar);
    outer->addWidget(scroll, 1);

    connect(torqueOff, &QPushButton::clicked, this, [this]{ emit torqueAllRequested(false); });
    connect(torqueOn,  &QPushButton::clicked, this, [this]{ emit torqueAllRequested(true); });
    connect(selectAll, &QPushButton::toggled, this, &JointControlTab::onSelectAllSync);
    connect(syncWrite, &QPushButton::clicked, this, &JointControlTab::onSyncWriteClicked);

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

void JointControlTab::onSyncWriteClicked()
{
    // Sync Write sends the SAME master Goal value to every armed joint, in one
    // command per series. (Per-joint live positioning is the row sliders.)
    const int goal = goal_->value();
    std::vector<feetech_servo::GroupTarget> armed;
    for(auto *r : rows_)
        if(r->isSyncArmed() && r->profile().known)
            armed.push_back({r->id(), r->profile(), goal});
    emit syncWriteRequested(armed);
}
