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

    sync_ = new QCheckBox("Sync", this);
    sync_->setToolTip("Armed: this joint moves only on the Sync Write button, "
                      "together with other armed joints.\n"
                      "Unarmed: dragging the slider jogs this joint live.");

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
    spin_->setValue(v);           // keep spin in sync
    suppress_ = false;
    // Armed for sync: only set the target; motion waits for the Sync Write button.
    // Unarmed: jog this joint live so you can position it individually.
    if(!isSyncArmed())
        emit jogged(id_, v);
}

void JointRow::onSpinChanged(int v)
{
    if(suppress_)
        return;
    suppress_ = true;
    slider_->setValue(v);
    suppress_ = false;
    if(!isSyncArmed())
        emit jogged(id_, v);
}
