#ifndef JOINTROW_H
#define JOINTROW_H

#include <QWidget>
#include "servo/scserial.h"

class QCheckBox;
class QSlider;
class QSpinBox;
class QLabel;

// One row of the Joint Control tab: a single discovered servo.
//
//   [ID] [name] [Torque check] [====slider====] [target spin] [pos: NNNN] [Sync check]
//
// The row is passive: it emits intent (torque toggled, jogged to a target, sync
// armed/disarmed) and MainWindow performs the actual serial writes through the
// shared dispatch helpers. Present position is pushed in via setPresentPosition().
class JointRow : public QWidget
{
    Q_OBJECT
public:
    // initialPos: the servo's present position at build time, so the adjuster
    // starts where the joint actually is (not at 0). -1 if it could not be read.
    JointRow(uint8_t id, const feetech_servo::ServoProfile &profile,
             int initialPos, QWidget *parent = nullptr);

    uint8_t id() const { return id_; }
    const feetech_servo::ServoProfile &profile() const { return profile_; }
    int  target() const;
    bool isSyncArmed() const;

    void setPresentPosition(int pos);
    void setTorque(bool on);           // reflect state without re-emitting
    void setSyncArmed(bool on);

signals:
    void torqueToggled(uint8_t id, bool on);
    void jogged(uint8_t id, int target);
    void syncArmChanged(uint8_t id, bool armed);

private:
    void onTorqueClicked(bool on);
    void onSliderMoved(int v);
    void onSpinChanged(int v);

    uint8_t                     id_;
    feetech_servo::ServoProfile profile_;

    QCheckBox *torque_ = nullptr;
    QSlider   *slider_ = nullptr;
    QSpinBox  *spin_ = nullptr;
    QLabel    *present_ = nullptr;
    QCheckBox *sync_ = nullptr;
    bool       suppress_ = false;      // guard against slider<->spin feedback loops
};

#endif // JOINTROW_H
