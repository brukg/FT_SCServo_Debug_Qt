#ifndef JOINTCONTROLTAB_H
#define JOINTCONTROLTAB_H

#include <QWidget>
#include <vector>
#include "servo/scserial.h"
#include "servo/servo_dispatch.h"

class QVBoxLayout;
class QSpinBox;
class QTimer;
class JointRow;

// Multi-servo control tab: one JointRow per discovered servo, a master control
// bar (torque all on/off, select-all sync, sync write, shared speed/acc/torque),
// and a poll timer that updates present positions while this tab is active.
//
// The tab is a view. It emits intent; the owner (MainWindow) performs serial IO
// through the shared dispatch helpers and pushes present positions back in.
class JointControlTab : public QWidget
{
    Q_OBJECT
public:
    explicit JointControlTab(QWidget *parent = nullptr);

    // Rebuild all rows from the current discovered-servo set.
    void setServos(const std::vector<feetech_servo::GroupTarget> &servos);

    int  speed() const;
    int  acc() const;
    int  torque() const;

    // Poll support: which servo ids currently have rows (for round-robin reads),
    // and pushing a read-back present position to the matching row.
    std::vector<uint8_t> rowIds() const;
    void setPresentPosition(uint8_t id, int pos);
    void reflectTorque(uint8_t id, bool on);

    // Enable/disable the poll cadence (driven by MainWindow when tab is active).
    void setPollActive(bool active);

signals:
    void torqueToggled(uint8_t id, bool on);
    void jogged(uint8_t id, int target);
    void torqueAllRequested(bool on);                    // master on/off
    void syncWriteRequested(const std::vector<feetech_servo::GroupTarget> &armed);
    void pollTick();                                     // MainWindow reads one/some servos

private:
    void onSyncWriteClicked();
    void onSelectAllSync(bool on);

    QVBoxLayout *rowLayout_ = nullptr;
    std::vector<JointRow*> rows_;
    QSpinBox *goal_ = nullptr;
    QSpinBox *speed_ = nullptr;
    QSpinBox *acc_ = nullptr;
    QSpinBox *torque_ = nullptr;
    QTimer   *poll_ = nullptr;
};

#endif // JOINTCONTROLTAB_H
