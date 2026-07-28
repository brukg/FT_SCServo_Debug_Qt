#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "jointcontroltab.h"
#include "jointplotwidget.h"
#include <QBoxLayout>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTimer>
#include <QTableView>
#include <QTabWidget>
#include <QScrollBar>
#include <QStandardItemModel>
#include <QtCore/QDebug>
#include <QLineEdit>
#include <QIntValidator>
#include <QRegExpValidator>
#include <QDir>
#include <cmath>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("FT SCServo Debug Qt");

    graph_timer_ = new QTimer(this);
    connect(graph_timer_, &QTimer::timeout, this, &MainWindow::onGraphTimerTimeout);
    graph_timer_->start(30);

    serial_ = new QSerialPort(this);
    scserial_ = new feetech_servo::SCSerial(serial_);
    sms_sts_serial_ = new feetech_servo::SMS_STS(scserial_);
    scs_serial_ = new feetech_servo::SCSCL(scserial_);
    hls_serial_ = new feetech_servo::HLSCL(scserial_);

    setupComSettings();
    setupServoLists();

    setupServoControl();
    setupAutoDebug();
    setupDataAnalysis();

    setupProgramming();
    setupJointControl();

    setIntRangeLineEdit(ui->upLimitLineEdit, 0, 1200);
    setIntRangeLineEdit(ui->downLimitLineEdit, 0, 1200);
}

void MainWindow::populateJointTab()
{
    // Read each discovered servo's present position so the Joint Control adjusters
    // start where the joints actually are, not at 0.
    for(auto &d : discovered_)
    {
        int pos = -1;
        if(d.profile.known && serial_->isOpen())
        {
            if(d.profile.series == feetech_servo::HLS)
                pos = hls_serial_->read_pos(d.id);
            else
                pos = scserial_->read_word(d.id, 56);
        }
        d.pos = pos;
    }
    if(joint_tab_)
        joint_tab_->setServos(discovered_);

    // Reflect each servo's ACTUAL torque-enable state (register 40) into the row
    // checkboxes, so the UI never claims torque is off when the servo has it on.
    for(auto &d : discovered_)
    {
        if(!d.profile.known || !serial_->isOpen())
            continue;
        int te = scserial_->read_byte(d.id, 40);
        if(te >= 0)
            joint_tab_->reflectTorque(d.id, te != 0);
    }
}

void MainWindow::setupJointControl()
{
    joint_tab_ = new JointControlTab(this);
    ui->tabWidget->addTab(joint_tab_, "Joint Control");

    connect(joint_tab_, &JointControlTab::torqueToggled,      this, &MainWindow::onJointTorqueToggled);
    connect(joint_tab_, &JointControlTab::jogged,             this, &MainWindow::onJointJogged);
    connect(joint_tab_, &JointControlTab::torqueAllRequested, this, &MainWindow::onJointTorqueAll);
    connect(joint_tab_, &JointControlTab::syncWriteRequested, this, &MainWindow::onJointSyncWrite);
    connect(joint_tab_, &JointControlTab::pollTick,           this, &MainWindow::onJointPollTick);
    connect(ui->tabWidget, &QTabWidget::currentChanged,       this, &MainWindow::onTabChanged);

    // The multi-joint plot lives on the Joint Control tab only. The Debug tab
    // keeps its own single-servo graph; a second plot there was redundant.

    // Timer feeds the Joint Control plot round-robin while that tab is showing.
    plot_clock_ = new QElapsedTimer();
    plot_clock_->start();
    plot_timer_ = new QTimer(this);
    plot_timer_->setInterval(50);
    connect(plot_timer_, &QTimer::timeout, this, &MainWindow::onPlotFeedTick);
    plot_timer_->start();
}

// The Joint Control plot, but only while its tab is visible (else nullptr).
JointPlotWidget *MainWindow::activePlot() const
{
    if(joint_tab_ && ui->tabWidget->currentWidget() == joint_tab_)
        return joint_tab_->plot();
    return nullptr;
}

// Read one signal by name for a servo. Returns -1 on failure.
int MainWindow::readSignal(const feetech_servo::GroupTarget &d, const QString &sig)
{
    if(!d.profile.known)
        return -1;
    if(sig == "position")
        return (d.profile.series == feetech_servo::HLS)
                   ? hls_serial_->read_pos(d.id)
                   : scserial_->read_word(d.id, 56);
    if(sig == "temperature") return scserial_->read_byte(d.id, 63);
    if(sig == "voltage")     return scserial_->read_byte(d.id, 62);

    // speed(58)/load(60)/current(69): 2-byte, sign-magnitude with bit 15.
    uint8_t addr = (sig == "speed") ? 58 : (sig == "load") ? 60 : 69;
    int raw = scserial_->read_word(d.id, addr);
    if(raw < 0) return -1;
    return (raw & (1 << 15)) ? -(raw & ~(1 << 15)) : raw;
}

void MainWindow::onPlotFeedTick()
{
    JointPlotWidget *plot = activePlot();
    if(!plot || discovered_.empty() || !serial_->isOpen() || is_searching_ || !plot->isLive())
        return;
    if(joint_poll_cursor_ >= discovered_.size())
        joint_poll_cursor_ = 0;

    const auto &d = discovered_[joint_poll_cursor_];
    const QString sig = plot->currentSignal();
    const int val = readSignal(d, sig);
    const double t = plot_clock_->elapsed() / 1000.0;
    plot->addSample(d.id, t, val);

    if(sig == "position" && plot->goalOverlayOn())
    {
        auto g = last_goal_.find(d.id);
        if(g != last_goal_.end())
            plot->addGoalSample(d.id, t, g->second);
    }
    joint_poll_cursor_++;
}

const feetech_servo::ServoProfile *MainWindow::profileForId(uint8_t id) const
{
    for(const auto &d : discovered_)
        if(d.id == id)
            return &d.profile;
    return nullptr;
}

void MainWindow::onJointTorqueToggled(uint8_t id, bool on)
{
    if(const auto *p = profileForId(id))
        feetech_servo::enable_torque_for(scs_serial_, sms_sts_serial_, hls_serial_, id, *p, on);
}

void MainWindow::onJointJogged(uint8_t id, int target)
{
    if(const auto *p = profileForId(id))
    {
        feetech_servo::write_goal_for(scs_serial_, sms_sts_serial_, hls_serial_,
                                      id, *p, target,
                                      joint_tab_->speed(), joint_tab_->acc(), joint_tab_->torque());
        last_goal_[id] = target;
    }
}

void MainWindow::onJointTorqueAll(bool on)
{
    for(const auto &d : discovered_)
    {
        feetech_servo::enable_torque_for(scs_serial_, sms_sts_serial_, hls_serial_, d.id, d.profile, on);
        joint_tab_->reflectTorque(d.id, on);
    }
}

void MainWindow::onJointSyncWrite(const std::vector<feetech_servo::GroupTarget> &armed)
{
    if(armed.empty())
    {
        ui->ServoSearchText->setText("Sync Write: no joints armed");
        return;
    }
    feetech_servo::sync_write_group(scs_serial_, sms_sts_serial_, hls_serial_,
                                    armed, joint_tab_->speed(), joint_tab_->acc(), joint_tab_->torque());
    for(const auto &t : armed)
        last_goal_[t.id] = t.pos;
}

void MainWindow::onJointPollTick()
{
    // Round-robin one servo per tick so a large bus is not saturated at 1 Mbaud.
    if(discovered_.empty() || !serial_->isOpen() || is_searching_)
        return;
    if(joint_poll_cursor_ >= discovered_.size())
        joint_poll_cursor_ = 0;

    const auto &d = discovered_[joint_poll_cursor_];
    int pos = -1;
    if(d.profile.known)
    {
        if(d.profile.series == feetech_servo::HLS)
            pos = hls_serial_->read_pos(d.id);
        else
            pos = scserial_->read_word(d.id, 56);   // present position register

        // Reflect the ACTUAL torque-enable state (register 40) into the checkbox
        // every cycle, so it always matches the motor -- if a jog energizes the
        // motor, the box shows ON, never a stale OFF while the motor is driving.
        int te = scserial_->read_byte(d.id, 40);
        if(te >= 0)
            joint_tab_->reflectTorque(d.id, te != 0);
    }
    joint_tab_->setPresentPosition(d.id, pos);
    joint_poll_cursor_++;
}

void MainWindow::onTabChanged(int index)
{
    // Enable present-position polling only while the Joint Control tab is showing.
    const bool jointActive = (joint_tab_ && ui->tabWidget->widget(index) == joint_tab_);
    if(joint_tab_)
        joint_tab_->setPollActive(jointActive);
}

MainWindow::~MainWindow()
{
    delete ui;
    delete graph_timer_;
    delete serial_;
    delete scserial_;
    delete scs_serial_;
    delete sms_sts_serial_;
    delete hls_serial_;
    delete servo_list_model_;
    delete prog_mem_model_;
    delete port_search_timer_;
    delete search_timer_;
    delete servo_read_timer_;
    delete auto_debug_timer_;
    delete prog_timer_;
}

void MainWindow::setupComSettings()
{
    QStringList baudRates = {"1000000", "500000", "250000", "256000", "128000", "115200", "76800", "57600", "38400", "19200", "9600", "4800"};
    ui->BaudComboBox->addItems(baudRates);

    QStringList parity = {"NONE", "ODD", "EVEN"};
    ui->ParityComboBox->addItems(parity);

    setIntRangeLineEdit(ui->timeoutLineEdit, 0, 10000);
    onPortSearchTimerTimeout();
    connect(ui->ComOpenButton, &QPushButton::clicked, this, &MainWindow::onConnectButtonClicked);
    port_search_timer_ = new QTimer(this);
    connect(port_search_timer_, &QTimer::timeout, this, &MainWindow::onPortSearchTimerTimeout);
    port_search_timer_->start(500);
}

void MainWindow::setupServoLists()
{
    connect(ui->SearchButton, &QPushButton::clicked, this, &MainWindow::onSearchButtonClicked);

    ui->ServoSearchText->setText("Stop");
    servo_list_model_ = new QStandardItemModel(0, 2);
    ui->ServoListView->setModel(servo_list_model_);
    clearServoList();

    search_timer_ = new QTimer(this);
    connect(search_timer_, &QTimer::timeout, this, &MainWindow::onSearchTimerTimeout);

    servo_read_timer_ = new QTimer(this);
    connect(servo_read_timer_, &QTimer::timeout, this, &MainWindow::onServoReadTimerTimeout);
    servo_read_timer_->start(10);

    connect(ui->ServoListView->selectionModel(), &QItemSelectionModel::selectionChanged, this, &MainWindow::onServoListSelection);
}

void MainWindow::setupServoControl()
{
    connect(ui->writeRadioButton, &QRadioButton::toggled, this, &MainWindow::onModeRadioButtonsToggled);
    connect(ui->syncWriteRadioButton, &QRadioButton::toggled, this, &MainWindow::onModeRadioButtonsToggled);
    connect(ui->regWriteRadioButton, &QRadioButton::toggled, this, &MainWindow::onModeRadioButtonsToggled);

    connect(ui->goalSlider, &QSlider::valueChanged, this, &MainWindow::onGoalSliderValueChanged);

    setIntRangeLineEdit(ui->accLineEdit, 0, std::numeric_limits<int>::max());
    setIntRangeLineEdit(ui->speedLineEdit, 0, std::numeric_limits<int>::max());
    setIntRangeLineEdit(ui->goalLineEdit, 0, 4095);
    setIntRangeLineEdit(ui->timeLineEdit, 0, std::numeric_limits<int>::max());
    
    connect(ui->setPushButton, &QPushButton::clicked, this, &MainWindow::onSetBuggonClicked);
    connect(ui->torqueEnableCheckBox, &QCheckBox::stateChanged, this, &MainWindow::onTorqueEnableCheckBoxStateChanged);
    connect(ui->actionPushButton, &QPushButton::clicked, this, &MainWindow::onActionButtonClicked);
}

void MainWindow::setupAutoDebug()
{
    setIntRangeLineEdit(ui->startLineEdit, 0, 4095);
    setIntRangeLineEdit(ui->endLineEdit, 0, 4095);
    setIntRangeLineEdit(ui->sweepLineEdit, 0, std::numeric_limits<int>::max());
    setIntRangeLineEdit(ui->setpLineEdit, 1, std::numeric_limits<int>::max());
    setIntRangeLineEdit(ui->setpDelayLineEdit, 1, std::numeric_limits<int>::max());

    connect(ui->sweepButton, &QPushButton::clicked, this, &MainWindow::onSweepButtonClicked);
    connect(ui->setpButton, &QPushButton::clicked, this, &MainWindow::onSetpButtonClicked);

    auto_debug_timer_ = new QTimer(this);
    connect(auto_debug_timer_, &QTimer::timeout, this, &MainWindow::onAutoDebugTimerTimeout);
}

void MainWindow::setupDataAnalysis()
{
    connect(ui->exportPushButton, &QPushButton::clicked, this, &MainWindow::onExportButtonClicked);
    connect(ui->clearPushButton, &QPushButton::clicked, this, &MainWindow::onClearButtonClicked);
    setIntRangeLineEdit(ui->recTimeLineEdit, 0, std::numeric_limits<int>::max());

    data_analysis_timer_ = new QTimer(this);
    connect(data_analysis_timer_, &QTimer::timeout, this, &MainWindow::onDataAnalysisTimerTimeout);
    data_analysis_timer_->start(50);
}

void MainWindow::setupProgramming()
{
    prog_mem_model_ = new QStandardItemModel(0, 5);
    ui->memoryTableView->setModel(prog_mem_model_);
    clearProgMemTable();

    connect(ui->memoryTableView->selectionModel(), &QItemSelectionModel::selectionChanged, this, &MainWindow::onMemoryTableSelection);
    connect(ui->memSetButton, &QPushButton::clicked, this, &MainWindow::onMemSetButtonClicked);

    prog_timer_ = new QTimer(this);
    connect(prog_timer_, &QTimer::timeout, this, &MainWindow::onProgTimerTimeout);
    prog_timer_->start(50);
}

void MainWindow::setEnableComSettings(bool state)
{
    ui->ComComboBox->setEnabled(state);
    ui->BaudComboBox->setEnabled(state);
    ui->ParityComboBox->setEnabled(state);
    ui->timeoutLineEdit->setEnabled(state);
}

void MainWindow::clearServoList()
{
    servo_list_model_->clear();
    servo_list_model_->setHorizontalHeaderLabels(QStringList() << "ID" << "Module");

    ui->ServoListView->setSelectionMode(QAbstractItemView::SingleSelection);
    ui->ServoListView->setSelectionBehavior(QAbstractItemView::SelectionBehavior::SelectRows);
    ui->ServoListView->setHorizontalScrollMode(QAbstractItemView::ScrollPerPixel);
    ui->ServoListView->horizontalScrollBar()->setDisabled(true);
    ui->ServoListView->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ui->ServoListView->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
    ui->ServoListView->setColumnWidth(0, 50);
    ui->ServoListView->verticalHeader()->setVisible(false);
    ui->ServoListView->setEditTriggers(QAbstractItemView::NoEditTriggers);
}

void MainWindow::appendServoList(const int id, const feetech_servo::ServoProfile &profile)
{
    // Stash the resolved profile on the row so reselecting a servo needs no
    // further serial IO, and never has to re-derive the series from its name.
    auto *id_item = new QStandardItem(QString::number(id));
    id_item->setData(static_cast<int>(profile.series), Qt::UserRole + 1);
    id_item->setData(static_cast<int>(profile.end),    Qt::UserRole + 2);
    id_item->setData(profile.known,                    Qt::UserRole + 3);

    servo_list_model_->appendRow(QList<QStandardItem*>() << id_item << new QStandardItem(profile.name));
}

void MainWindow::clearProgMemTable()
{
    prog_mem_model_->clear();
    prog_mem_model_->setHorizontalHeaderLabels(QStringList() << "Address" << "Memory" << "Value" << "Area" << "R/W");
    ui->memoryTableView->setSelectionMode(QAbstractItemView::SingleSelection);
    ui->memoryTableView->setSelectionBehavior(QAbstractItemView::SelectionBehavior::SelectRows);
    ui->memoryTableView->verticalHeader()->setVisible(false);
    ui->memoryTableView->horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);
    ui->memoryTableView->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    ui->memoryTableView->setColumnWidth(0, 70);
    ui->memoryTableView->setColumnWidth(2, 70);
    ui->memoryTableView->setColumnWidth(3, 70);
    ui->memoryTableView->setColumnWidth(4, 70);
    ui->memoryTableView->setEditTriggers(QAbstractItemView::NoEditTriggers);
}

void MainWindow::updatePorgMemTable()
{
    clearProgMemTable();
    auto mem_config = getMemConfig(select_servo_.profile_.series);

    for(auto &item : mem_config)
    {
        auto &[address, name, size, default_value, dir_bit, is_eprom, is_readonly, min_val, max_val] = item;
        QString area = is_eprom ? "EPROM" : "SRAM";
        QString rw = is_readonly ? "R" : "R/W";
        QList<QStandardItem*> rowList;

        rowList << new QStandardItem(QString::number(address)) << new QStandardItem(name) << new QStandardItem(QString::number(default_value)) << new QStandardItem(area) << new QStandardItem(rw);
        prog_mem_model_->appendRow(rowList);
    }
}

void MainWindow::setIntRangeLineEdit(QLineEdit *edit, int min, int max)
{
    edit->setValidator(new QIntValidator(min, max, edit));
}

void MainWindow::setIntLineEdit(QLineEdit *edit)
{
    edit->setValidator(new QRegExpValidator(QRegExp("-?\\d*"), edit));
}

void MainWindow::applyServoProfile(const feetech_servo::ServoProfile &profile)
{
    select_servo_.profile_ = profile;
    scserial_->set_end(profile.end);

    // Fail closed: without a resolved register map any write could land on the
    // wrong register, so show state but refuse to command.
    ui->groupBox_3->setEnabled(profile.known);

    // Addresses 44/45 mean different things per family. On HLS they are Goal
    // Torque (max torque current, 6.5mA units) and a zero there immobilises the
    // servo, so the field is relabelled and given a working default.
    if(profile.series == feetech_servo::ModelSeries::HLS)
    {
        ui->label_16->setText("Torque (x6.5mA)");
        if(ui->timeLineEdit->text().toInt() == 0)
            ui->timeLineEdit->setText("500");
    }
    else
    {
        ui->label_16->setText("Time");
    }

    updatePorgMemTable();
}

const std::vector<feetech_servo::MemoryConfig>& MainWindow::getMemConfig(feetech_servo::ModelSeries series)
{
    switch(series)
    {
        case feetech_servo::ModelSeries::SCS:
            return feetech_servo::SCSMemConfig;
        case feetech_servo::ModelSeries::SCS2:
            return feetech_servo::SCS2MemConfig;
        case feetech_servo::ModelSeries::STS:
            return feetech_servo::STSMemConfig;
        case feetech_servo::ModelSeries::SMBL:
            return feetech_servo::SMBLMemConfig;
        case feetech_servo::ModelSeries::SMCL:
            return feetech_servo::SMCLMemConfig;
        case feetech_servo::ModelSeries::HLS:
            return feetech_servo::HLSMemConfig;
        default:
            return feetech_servo::STSMemConfig;
    }
}

int MainWindow::currentTorqueField() const
{
    return ui->timeLineEdit->text().toInt();
}

// Single dispatch point for every position command. Previously this branch was
// duplicated at nine call sites, which is how HLS came to be driven through the
// SMS/STS write path.
void MainWindow::writeGoal(int pos, int time, int speed, int acc, int torque)
{
    if(!select_servo_.profile_.known)
        return;

    switch(select_servo_.profile_.series)
    {
        case feetech_servo::ModelSeries::SCS:
        case feetech_servo::ModelSeries::SCS2:
            scs_serial_->write_pos(select_servo_.id_, pos, time, speed);
            break;

        case feetech_servo::ModelSeries::HLS:
            hls_serial_->servo_mode(select_servo_.id_);
            hls_serial_->write_pos_ex(select_servo_.id_, pos, speed, acc, torque);
            break;

        default:
            sms_sts_serial_->rotation_mode(select_servo_.id_);
            sms_sts_serial_->write_pos_ex(select_servo_.id_, pos, speed, acc);
            break;
    }
}

void MainWindow::writePos(int pos, int time, int speed, int acc)
{
    writeGoal(pos, time, speed, acc, currentTorqueField());
}

void MainWindow::syncWritePos(int pos, int time, int speed, int acc)
{
    if(!select_servo_.profile_.known)
        return;

    std::vector<uint16_t> times(id_list_.size(), time);
    std::vector<uint16_t> speeds(id_list_.size(), speed);
    std::vector<uint8_t> accs(id_list_.size(), acc);

    switch(select_servo_.profile_.series)
    {
        case feetech_servo::ModelSeries::SCS:
        case feetech_servo::ModelSeries::SCS2:
        {
            std::vector<uint16_t> goals(id_list_.size(), pos);
            scs_serial_->sync_write_pos(id_list_.data(), id_list_.size(), goals.data(), times.data(), speeds.data());
            break;
        }

        case feetech_servo::ModelSeries::HLS:
        {
            std::vector<int16_t> goals(id_list_.size(), pos);
            std::vector<uint16_t> torques(id_list_.size(), currentTorqueField());
            hls_serial_->sync_write_pos_ex(id_list_.data(), id_list_.size(), goals.data(), speeds.data(), accs.data(), torques.data());
            break;
        }

        default:
        {
            std::vector<int16_t> goals(id_list_.size(), pos);
            sms_sts_serial_->sync_write_pos_ex(id_list_.data(), id_list_.size(), goals.data(), speeds.data(), accs.data());
            break;
        }
    }
}

void MainWindow::regWritePos(int pos, int time, int speed, int acc)
{
    if(!select_servo_.profile_.known)
        return;

    switch(select_servo_.profile_.series)
    {
        case feetech_servo::ModelSeries::SCS:
        case feetech_servo::ModelSeries::SCS2:
            scs_serial_->reg_write_pos(select_servo_.id_, pos, time, speed);
            break;

        case feetech_servo::ModelSeries::HLS:
            hls_serial_->servo_mode(select_servo_.id_);
            hls_serial_->reg_write_pos_ex(select_servo_.id_, pos, speed, acc, currentTorqueField());
            break;

        default:
            sms_sts_serial_->rotation_mode(select_servo_.id_);
            sms_sts_serial_->reg_write_pos_ex(select_servo_.id_, pos, speed, acc);
            break;
    }
}

void MainWindow::onPortSearchTimerTimeout()
{
    if(serial_->isOpen())
        return;

    ui->ComComboBox->clear();
    for (const auto &info : QSerialPortInfo::availablePorts()) {
        ui->ComComboBox->addItem(info.portName());
    }
}

void MainWindow::onConnectButtonClicked()
{
    if (serial_->isOpen()) {
        serial_->close();
        ui->ComOpenButton->setText("Open");
        setEnableComSettings(true);
        select_servo_.id_ = -1;
    }
    else {
        serial_->setPortName(ui->ComComboBox->currentText());
        serial_->setBaudRate(ui->BaudComboBox->currentText().toInt());
        uint8_t pidx = ui->ParityComboBox->currentIndex();
        QSerialPort::Parity p = QSerialPort::Parity::NoParity;
        if(pidx == 1)
            p = QSerialPort::Parity::OddParity;
        if(pidx == 2)
            p = QSerialPort::Parity::EvenParity;
        serial_->setParity(p);
        serial_->setDataBits(QSerialPort::DataBits::Data8);
        serial_->setStopBits(QSerialPort::StopBits::OneStop);
        serial_->setFlowControl(QSerialPort::FlowControl::NoFlowControl);
        if (serial_->open(QIODevice::ReadWrite)) {
            ui->ComOpenButton->setText("Close");
            setEnableComSettings(false);
        }
        else {
            ui->ComOpenButton->setText("Open");
        }
        scserial_->set_timeout(ui->timeoutLineEdit->text().toUInt());
    }
}

void MainWindow::onSearchButtonClicked()
{
    if(!serial_->isOpen())
    {
        qDebug() << "serial not open";
        return;
    }

    is_searching_ = !is_searching_;

    if(is_searching_)
    {
        ui->SearchButton->setText("Stop");
        clearServoList();
        id_list_.clear();
        discovered_.clear();
        search_id_ = 0;
        search_timer_->start(10);
        onSearchTimerTimeout();
    }
    else
    {
        ui->SearchButton->setText("Search");
        search_timer_->stop();
        ui->ServoSearchText->setText(QString("Stop"));
        populateJointTab();
    }
}

void MainWindow::onSearchTimerTimeout()
{
    search_timer_->stop();
    if(!is_searching_)
        return;

    if(0xfd < search_id_ || serial_->isOpen() == false)
    {
        is_searching_ = false;
        ui->SearchButton->setText("Search");
        ui->ServoSearchText->setText("Stop");
        populateJointTab();
    }
    else
    {
        ui->ServoSearchText->setText(QString("Ping ID:%1 Servo...").arg(search_id_));
        int ret = scserial_->ping(search_id_);

        if(0 < ret)
        {
            int mid = scserial_->read_model_number(ret);
            int fw  = scserial_->read_firmware_version(ret);
            auto profile = feetech_servo::resolveServo(mid, fw);

            qInfo("ID %d: model=0x%04x firmware=0x%04x -> %s (series=%d, end=%d, known=%d)",
                  ret, mid, fw, qPrintable(profile.name),
                  int(profile.series), int(profile.end), int(profile.known));

            appendServoList(ret, profile);
            id_list_.push_back(ret);
            discovered_.push_back({(uint8_t)ret, profile, 0});
            select_servo_.id_ = ret;
            applyServoProfile(profile);
        }
        search_id_++;
        search_timer_->start(1);
        
    }
}

void MainWindow::onServoListSelection()
{
    QModelIndex selectedRows = ui->ServoListView->selectionModel()->selectedRows()[0];
    std::size_t row = selectedRows.row();
    auto index = servo_list_model_->index(row, 0);
    select_servo_.id_ = servo_list_model_->data(index).toInt();

    // Recover the profile stashed on the row at scan time. Re-deriving it from
    // the displayed name is what caused HLS servos to be driven as SMCL.
    feetech_servo::ServoProfile profile;
    profile.series = static_cast<feetech_servo::ModelSeries>(
        servo_list_model_->data(index, Qt::UserRole + 1).toInt());
    profile.end   = static_cast<uint8_t>(servo_list_model_->data(index, Qt::UserRole + 2).toInt());
    profile.known = servo_list_model_->data(index, Qt::UserRole + 3).toBool();
    profile.name  = servo_list_model_->data(servo_list_model_->index(row, 1)).toString();

    applyServoProfile(profile);
}

void MainWindow::onGoalSliderValueChanged()
{
    int goal = ui->goalSlider->value();
    ui->goalLineEdit->setText(QString::number(goal));

    if(!isServoValidNow())
        return;
    
    if(mode_ == MODE_REG_WRITE)
    {
        regWritePos(goal, 0, 0, 0);
    }
    else if(mode_ == MODE_SYNC_WRITE)
    {
        syncWritePos(goal, 0, 0, 0);
    }
    else if(mode_ == MODE_WRITE)
    {
        writePos(goal, 0, 0, 0);
    }
}

void MainWindow::onSetBuggonClicked()
{
    int goal = ui->goalLineEdit->text().toUInt();
    int speed = ui->speedLineEdit->text().toUInt();
    int acc = ui->accLineEdit->text().toUInt();
    int time = ui->timeLineEdit->text().toUInt();
    ui->goalSlider->setValue(goal);

    if(!isServoValidNow())
        return;
    
    if(mode_ == MODE_REG_WRITE)
    {
        regWritePos(goal, time, speed, acc);
    }
    else if(mode_ == MODE_SYNC_WRITE)
    {
        syncWritePos(goal, time, speed, acc);
    }
    else if(mode_ == MODE_WRITE)
    {
        writePos(goal, time, speed, acc);
    }
}

void MainWindow::onTorqueEnableCheckBoxStateChanged()
{
    if(!isServoValidNow())
        return;
    
    const bool enable = ui->torqueEnableCheckBox->isChecked();
    switch(select_servo_.profile_.series)
    {
        case feetech_servo::ModelSeries::SCS:
        case feetech_servo::ModelSeries::SCS2:
            scs_serial_->enable_torque(select_servo_.id_, enable);
            break;
        case feetech_servo::ModelSeries::HLS:
            hls_serial_->enable_torque(select_servo_.id_, enable);
            break;
        default:
            sms_sts_serial_->enable_torque(select_servo_.id_, enable);
            break;
    }
}

void MainWindow::onModeRadioButtonsToggled(bool checked)
{
    if(checked)
    {
        if(ui->writeRadioButton->isChecked())
        {
            mode_ = MODE_WRITE;
        }
        else if(ui->syncWriteRadioButton->isChecked())
        {
            mode_ = MODE_SYNC_WRITE;
        }
        else if(ui->regWriteRadioButton->isChecked())
        {
            mode_ = MODE_REG_WRITE;
        }

        ui->actionPushButton->setEnabled(mode_ == MODE_REG_WRITE);
    }
}

void MainWindow::onActionButtonClicked()
{
    if(!isServoValidNow())
        return;

    if(mode_ == MODE_REG_WRITE)
    {
        scserial_->reg_write_action(select_servo_.id_);
    }
}

void MainWindow::onSweepButtonClicked()
{
    if(!isServoValidNow())
        return;
    
    if(sweep_running_)
    {
        sweep_running_ = false;
        ui->sweepButton->setText("Sweep");
        ui->setpButton->setEnabled(true);
        auto_debug_timer_->stop();
    }
    else
    {
        sweep_running_ = true;
        ui->sweepButton->setText("Stop");
        ui->setpButton->setEnabled(false);
        latest_auto_debug_goal_ = ui->startLineEdit->text().toUInt();

        writeGoal(latest_auto_debug_goal_, 0, 0, 0, currentTorqueField());
        auto_debug_timer_->start(ui->sweepLineEdit->text().toUInt());
    }
}

void MainWindow::onSetpButtonClicked()
{
    if(!isServoValidNow())
        return;
    
    if(setp_running_)
    {
        setp_running_ = false;
        ui->setpButton->setText("Setp");
        ui->sweepButton->setEnabled(true);
        auto_debug_timer_->stop();
    }
    else
    {
        setp_running_ = true;
        setp_increase_ = true;
        ui->setpButton->setText("Stop");
        ui->sweepButton->setEnabled(false);
        latest_auto_debug_goal_ = ui->startLineEdit->text().toUInt();
        writeGoal(latest_auto_debug_goal_, 0, 0, 0, currentTorqueField());
        auto_debug_timer_->start(ui->setpDelayLineEdit->text().toUInt());
    }
}

void MainWindow::onAutoDebugTimerTimeout()
{
    if(!isServoValidNow())
    {
        auto_debug_timer_->stop();
        sweep_running_ = false;
        setp_running_ = false;
        ui->sweepButton->setText("Sweep");
        ui->sweepButton->setEnabled(true);
        ui->setpButton->setText("Setp");
        ui->setpButton->setEnabled(true);
        return;
    }

    if(sweep_running_)
    {
        int start = ui->startLineEdit->text().toUInt();
        int end = ui->endLineEdit->text().toUInt();
        if(latest_auto_debug_goal_ == start)
        {
            latest_auto_debug_goal_ = end;
        }
        else
        {
            latest_auto_debug_goal_ = start;
        }
        writeGoal(latest_auto_debug_goal_, 0, 0, 0, currentTorqueField());
    }
    else if(setp_running_)
    {
        int start = ui->startLineEdit->text().toUInt();
        int end = ui->endLineEdit->text().toUInt();
        int step = ui->setpLineEdit->text().toUInt();
        
        latest_auto_debug_goal_ += setp_increase_ ? step : -step;
        
        if(end < latest_auto_debug_goal_)
        {
            latest_auto_debug_goal_ = end;
            setp_increase_ = false;
        }
        else if(latest_auto_debug_goal_ < start)
        {
            latest_auto_debug_goal_ = start;
            setp_increase_ = true;
        }

        writeGoal(latest_auto_debug_goal_, 0, 0, 0, currentTorqueField());
    }
    else
    {
        auto_debug_timer_->stop();
    }
}

void MainWindow::onExportButtonClicked()
{
    if(!isServoValidNow())
        return;
    
    if(is_recording_)
    {
        is_recording_ = false;
        ui->exportPushButton->setText("Export");
        ui->clearPushButton->setEnabled(true);

        if(!record_section_data_.isEmpty())
        {
            auto record_file_ = new QFile(record_file_name_);

            if(!record_file_->open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Append))
            {
                qDebug() << "file open error: " << record_file_name_;
                return;
            }

            auto record_stream_ = new QTextStream(record_file_);
            *record_stream_ << record_section_data_;
            record_section_data_.clear();

            record_file_->close();
        }
        ui->recSizeLineEdit->setText(QString::number(record_data_count_));
    }
    else
    {
        is_recording_ = true;
        ui->exportPushButton->setText("Stop");
        ui->clearPushButton->setEnabled(false);
        record_section_data_.clear();
        record_data_count_ = 0;
        file_write_interval_ = ui->recTimeLineEdit->text().toUInt();
        if(file_write_interval_ == 0)
            file_write_interval_ = 1;

        record_file_name_ = ui->recFileNameLineEdit->text();

#if defined(Q_OS_LINUX)
        QString home_path = QDir::homePath();
        if(record_file_name_.startsWith("~"))
        {
            record_file_name_.replace("~", home_path);
        }
#endif // Q_OS_LINUX

        auto record_file_ = new QFile(record_file_name_);
        if(!record_file_->open(QIODevice::WriteOnly | QIODevice::Text))
        {
            qDebug() << "file open error: " << record_file_name_;
            return;
        }
        auto record_stream_ = new QTextStream(record_file_);
        *record_stream_ << "No,Pos,Gol,Ft,V,C,T,Vol\n";
        record_file_->close();
    }
}

void MainWindow::onClearButtonClicked()
{
    record_data_count_ = 0;
    record_section_data_.clear();
    ui->recSizeLineEdit->setText(QString::number(record_data_count_));
}

void MainWindow::onDataAnalysisTimerTimeout()
{
    if(!isServoValidNow())
        return;

    if(!is_recording_)
        return;
    
    record_data_count_++;
    QString line = QString("%1,%2,%3,%4,%5,%6,%7,%8,END\n").arg(record_data_count_).arg(latest_status_.pos).arg(latest_status_.goal).arg(latest_status_.torque).arg(latest_status_.speed).arg(latest_status_.current).arg(latest_status_.temp).arg(latest_status_.voltage);
    record_section_data_ += line;
    const size_t section_size = 20 * file_write_interval_;

    if(record_data_count_%section_size == 0)
    {
        auto record_file_ = new QFile(record_file_name_);

        if(!record_file_->open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Append))
        {
            qDebug() << "file open error: " << record_file_name_;
            return;
        }

        auto record_stream_ = new QTextStream(record_file_);
        *record_stream_ << record_section_data_;
        record_section_data_.clear();

        record_file_->close();

        ui->recSizeLineEdit->setText(QString::number(record_data_count_));
    }
}

void MainWindow::onProgTimerTimeout()
{
    if(ui->tabWidget->currentIndex() != 1)
        return;

    if(!isServoValidNow())
        return;

    if(is_mem_writing_)
        return;

    // 表に表示されている行の値のみ更新する
    int firstVisibleRow = ui->memoryTableView->indexAt(ui->memoryTableView->viewport()->rect().topLeft()).row();
    int lastVisibleRow = ui->memoryTableView->indexAt(ui->memoryTableView->viewport()->rect().bottomLeft()).row();
    if (firstVisibleRow != -1 && lastVisibleRow != -1)
    {
        auto mem_config = getMemConfig(select_servo_.profile_.series);
        for(int i = firstVisibleRow; i <= lastVisibleRow; i++)
        {
            // メモリ更新
            auto &[address, name, size, default_value, dir_bit, is_eprom, is_readonly, min_val, max_val] = mem_config[i];
            
            int val = 0;
            if(size == 2)
            {
                val = scserial_->read_word(select_servo_.id_, address);
            }
            else
            {
                val = scserial_->read_byte(select_servo_.id_, address);
            }

            ui->memoryTableView->model()->setData(ui->memoryTableView->model()->index(i,2), QString::number(val));
        }
    }
}

void MainWindow::onMemoryTableSelection()
{
    QModelIndex selectedRows = ui->memoryTableView->selectionModel()->selectedRows().first();
    std::size_t row = selectedRows.row();
    auto index = prog_mem_model_->index(row, 1);
    ui->memLabel->setText(prog_mem_model_->data(index).toString());
    ui->memSetLineEdit->setText(prog_mem_model_->data(prog_mem_model_->index(row, 2)).toString());
    auto mem_config = getMemConfig(select_servo_.profile_.series);
    bool is_readonly = mem_config[row].is_readonly;
    if(is_readonly)
    {
        ui->memSetLineEdit->setEnabled(false);
        ui->memSetButton->setEnabled(false);
    }
    else
    {
        ui->memSetLineEdit->setEnabled(true);
        ui->memSetButton->setEnabled(true);
    }
}

void MainWindow::onMemSetButtonClicked()
{
    is_mem_writing_ = true;
    QModelIndex selectedRows = ui->memoryTableView->selectionModel()->selectedRows().first();
    auto mem_config = getMemConfig(select_servo_.profile_.series);
    auto &[address, name, size, default_value, dir_bit, is_eprom, is_readonly, min_val, max_val] = mem_config[selectedRows.row()];

    // Todo address参照じゃなくする
    if(address == 5)
    {
        // Toso: STSサーボ以外に対応する
        uint8_t val = ui->memSetLineEdit->text().toShort();
        scserial_->write_byte(select_servo_.id_, 55, 0); // unlock
        scserial_->write_byte(select_servo_.id_, address, val);
        scserial_->write_byte(select_servo_.id_, 55, 1); // lock
        select_servo_.id_ = val;
    }
    if(size == 2)
    {
        int16_t val = ui->memSetLineEdit->text().toShort();
        scserial_->write_word(select_servo_.id_, address, val);
    }
    else
    {
        uint8_t val = ui->memSetLineEdit->text().toShort();
        scserial_->write_byte(select_servo_.id_, address, val);
    }
    is_mem_writing_ = false;
}

void MainWindow::onGraphTimerTimeout()
{
    ui->graphWidget->up_limit = ui->upLimitLineEdit->text().toUInt();
    ui->graphWidget->down_limit = ui->downLimitLineEdit->text().toUInt();
    ui->graphWidget->horizontal = ui->horizontalSlider->value();
    ui->graphWidget->zoom = ui->zoomSlider->value();

    if(is_searching_ || !serial_->isOpen() || select_servo_.id_ < 0)
        return;

    ui->positionLabel->setText(QString::number(latest_status_.pos));
    ui->torqueLabel->setText(QString::number(latest_status_.torque));
    ui->speedLabel->setText(QString::number(latest_status_.speed));
    ui->currentLabel->setText(QString::number(latest_status_.current));
    ui->temperatureLabel->setText(QString::number(latest_status_.temp));
    ui->voltageLabel->setText(QString::number(0.1*latest_status_.voltage, 'f', 1) + QString("V"));
    ui->movingLabel->setText(QString::number(latest_status_.move));
    ui->goalLabel->setText(QString::number(latest_status_.goal));

    ui->graphWidget->pos_visible = ui->posCheckBox->isChecked();
    ui->graphWidget->torque_visible = ui->torqueCheckBox->isChecked();
    ui->graphWidget->speed_visible = ui->speedCheckBox->isChecked();
    ui->graphWidget->current_visible = ui->currentCheckBox->isChecked();
    ui->graphWidget->temp_visible = ui->tempCheckBox->isChecked();
    ui->graphWidget->voltage_visible = ui->voltageCheckBox->isChecked();
}

void MainWindow::onServoReadTimerTimeout()
{
    if(ui->tabWidget->currentIndex() != 0)
        return;
    
    static int count = 0;
    if (isServoValidNow())
    {
        if(select_servo_.profile_.series == feetech_servo::ModelSeries::SCS ||
           select_servo_.profile_.series == feetech_servo::ModelSeries::SCS2)
        {
            switch(count)
            {
                case 0:
                    latest_status_.pos = scs_serial_->read_position(select_servo_.id_);
                    latest_status_.torque = scs_serial_->read_load(select_servo_.id_);
                    break;

                case 1:
                    latest_status_.speed = scs_serial_->read_speed(select_servo_.id_);
                    latest_status_.current = scs_serial_->read_current(select_servo_.id_);
                    break;

                case 2:
                    latest_status_.temp = scs_serial_->read_temperature(select_servo_.id_);
                    latest_status_.voltage = scs_serial_->read_voltage(select_servo_.id_);
                    latest_status_.move = scs_serial_->read_move(select_servo_.id_);
                    latest_status_.goal = scs_serial_->read_goal(select_servo_.id_);
                    ui->graphWidget->append_data(latest_status_.pos, latest_status_.torque, latest_status_.speed, latest_status_.current, latest_status_.temp, latest_status_.voltage);
                    break;
            }
        }
        else
        {
            switch(count)
            {
                case 0:
                    latest_status_.pos = sms_sts_serial_->read_position(select_servo_.id_);
                    latest_status_.torque = sms_sts_serial_->read_load(select_servo_.id_);
                    break;

                case 1:
                    latest_status_.speed = sms_sts_serial_->read_speed(select_servo_.id_);
                    latest_status_.current = sms_sts_serial_->read_current(select_servo_.id_);
                    latest_status_.temp = sms_sts_serial_->read_temperature(select_servo_.id_);
                    break;

                case 2:
                    latest_status_.voltage = sms_sts_serial_->read_voltage(select_servo_.id_);
                    latest_status_.move = sms_sts_serial_->read_move(select_servo_.id_);
                    latest_status_.goal = sms_sts_serial_->read_goal(select_servo_.id_);
                    ui->graphWidget->append_data(latest_status_.pos, latest_status_.torque, latest_status_.speed, latest_status_.current, latest_status_.temp, latest_status_.voltage);
                    break;
            }
        }


        count++;
        count %= 3;
    }
}
