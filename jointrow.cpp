#include "jointrow.h"
#include "servo/servo_dispatch.h"

#include <QHBoxLayout>
#include <QCheckBox>
#include <QSlider>
#include <QSpinBox>
#include <QLabel>

JointRow::JointRow(uint8_t id, const feetech_servo::ServoProfile &profile,
                   int initialPos, QWidget *parent)
    : QWidget(parent), id_(id), profile_(profile)
{
    const int pos_max = feetech_servo::position_max_for(profile_.series);
    // Start the adjuster at the servo's current position so nothing jumps to 0.
    const int startPos = (initialPos < 0) ? 0 : qBound(0, initialPos, pos_max);

    auto *idLabel = new QLabel(QString::number(id_), this);
    idLabel->setMinimumWidth(30);

    auto *nameLabel = new QLabel(profile_.name, this);
    nameLabel->setMinimumWidth(120);

    torque_ = new QCheckBox("Torque", this);
    torque_->setToolTip("Enable torque to make this joint controllable.\n"
                        "While off, the joint is limp and its slider is disabled -\n"
                        "no command is sent, so it cannot move. Torque-on joints are\n"
                        "the ones Sync Write moves.");

    slider_ = new QSlider(Qt::Horizontal, this);
    slider_->setRange(0, pos_max);
    slider_->setMinimumWidth(160);

    spin_ = new QSpinBox(this);
    spin_->setRange(0, pos_max);
    spin_->setMinimumWidth(70);

    present_ = new QLabel("pos: --", this);
    present_->setMinimumWidth(80);

    // Initialize the adjuster to the present position without emitting a jog.
    suppress_ = true;
    slider_->setValue(startPos);
    spin_->setValue(startPos);
    suppress_ = false;
    if(initialPos >= 0)
        present_->setText(QString("pos: %1").arg(initialPos));

    auto *lay = new QHBoxLayout(this);
    lay->setContentsMargins(4, 2, 4, 2);
    lay->addWidget(idLabel);
    lay->addWidget(nameLabel);
    lay->addWidget(torque_);
    lay->addWidget(slider_, 1);
    lay->addWidget(spin_);
    lay->addWidget(present_);

    // The joint is only commandable while its torque is on: the slider (and thus
    // any position write) is gated on the torque checkbox. This is what makes
    // "torque off = cannot move" true regardless of the servo's own behaviour.
    updateEnabled();

    // Fail closed: an unknown servo shows the row but cannot be commanded at all.
    if(!profile_.known)
        torque_->setEnabled(false);

    connect(torque_, &QCheckBox::toggled, this, &JointRow::onTorqueClicked);
    connect(slider_, &QSlider::valueChanged, this, &JointRow::onSliderMoved);
    connect(spin_, QOverload<int>::of(&QSpinBox::valueChanged), this, &JointRow::onSpinChanged);
}

int  JointRow::target() const     { return spin_->value(); }
bool JointRow::isTorqueOn() const  { return torque_->isChecked() && profile_.known; }

void JointRow::updateEnabled()
{
    const bool on = torque_->isChecked() && profile_.known;
    slider_->setEnabled(on);
    spin_->setEnabled(on);
}

void JointRow::setPresentPosition(int pos)
{
    present_->setText(pos < 0 ? "pos: --" : QString("pos: %1").arg(pos));
}

void JointRow::setTorque(bool on)
{
    suppress_ = true;
    torque_->setChecked(on);
    suppress_ = false;
    updateEnabled();
}

void JointRow::onTorqueClicked(bool on)
{
    updateEnabled();
    if(suppress_)
        return;
    emit torqueToggled(id_, on);
}

void JointRow::onSliderMoved(int v)
{
    if(suppress_)
        return;
    suppress_ = true;
    spin_->setValue(v);           // keep spin in sync
    suppress_ = false;
    emit jogged(id_, v);          // live jog of this one joint (torque is on, gated above)
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
