#include "jointrow.h"
#include "servo/servo_dispatch.h"

#include <QHBoxLayout>
#include <QCheckBox>
#include <QSlider>
#include <QSpinBox>
#include <QLabel>

JointRow::JointRow(uint8_t id, const feetech_servo::ServoProfile &profile, QWidget *parent)
    : QWidget(parent), id_(id), profile_(profile)
{
    const int pos_max = feetech_servo::position_max_for(profile_.series);

    auto *idLabel = new QLabel(QString::number(id_), this);
    idLabel->setMinimumWidth(30);

    auto *nameLabel = new QLabel(profile_.name, this);
    nameLabel->setMinimumWidth(120);

    torque_ = new QCheckBox("Torque", this);

    slider_ = new QSlider(Qt::Horizontal, this);
    slider_->setRange(0, pos_max);
    slider_->setMinimumWidth(160);

    spin_ = new QSpinBox(this);
    spin_->setRange(0, pos_max);
    spin_->setMinimumWidth(70);

    present_ = new QLabel("pos: --", this);
    present_->setMinimumWidth(80);

    sync_ = new QCheckBox("Sync", this);

    auto *lay = new QHBoxLayout(this);
    lay->setContentsMargins(4, 2, 4, 2);
    lay->addWidget(idLabel);
    lay->addWidget(nameLabel);
    lay->addWidget(torque_);
    lay->addWidget(slider_, 1);
    lay->addWidget(spin_);
    lay->addWidget(present_);
    lay->addWidget(sync_);

    // Fail closed: an unknown servo shows the row but cannot be commanded.
    if(!profile_.known)
    {
        torque_->setEnabled(false);
        slider_->setEnabled(false);
        spin_->setEnabled(false);
        sync_->setEnabled(false);
    }

    connect(torque_, &QCheckBox::toggled, this, &JointRow::onTorqueClicked);
    connect(slider_, &QSlider::valueChanged, this, &JointRow::onSliderMoved);
    connect(spin_, QOverload<int>::of(&QSpinBox::valueChanged), this, &JointRow::onSpinChanged);
}

int JointRow::target() const { return spin_->value(); }
bool JointRow::isSyncArmed() const { return sync_->isChecked(); }

void JointRow::setPresentPosition(int pos)
{
    present_->setText(pos < 0 ? "pos: --" : QString("pos: %1").arg(pos));
}

void JointRow::setTorque(bool on)
{
    suppress_ = true;
    torque_->setChecked(on);
    suppress_ = false;
}

void JointRow::setSyncArmed(bool on)
{
    if(sync_->isEnabled())
        sync_->setChecked(on);
}

void JointRow::onTorqueClicked(bool on)
{
    if(suppress_)
        return;
    emit torqueToggled(id_, on);
}

void JointRow::onSliderMoved(int v)
{
    if(suppress_)
        return;
    suppress_ = true;
    spin_->setValue(v);           // keep spin in sync without re-jogging
    suppress_ = false;
    emit jogged(id_, v);
}

void JointRow::onSpinChanged(int v)
{
    if(suppress_)
        return;
    suppress_ = true;
    slider_->setValue(v);
    suppress_ = false;
    emit jogged(id_, v);
}
