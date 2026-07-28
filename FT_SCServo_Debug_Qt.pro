QT       += core gui serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# QtCharts: use the system module if installed, else the vendored copy fetched
# by tools/fetch_qtcharts.sh (third_party/qt5charts). No sudo required either way.
qtHaveModule(charts) {
    QT += charts
} else {
    INCLUDEPATH += $$PWD/third_party/qt5charts/include
    LIBS += -L$$PWD/third_party/qt5charts/lib -lQt5Charts
    QMAKE_RPATHDIR += $$PWD/third_party/qt5charts/lib
    DEFINES += QT_CHARTS_LIB
}

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    servo/scserial.cpp \
    simpegraphwiget.cpp \
    jointrow.cpp \
    jointcontroltab.cpp \
    jointplotwidget.cpp

HEADERS += \
    mainwindow.h \
    servo/scserial.h \
    servo/servo_types.h \
    servo/sms_sts.h \
    servo/scscl.h \
    servo/hlscl.h \
    servo/servo_dispatch.h \
    simpegraphwiget.h \
    jointrow.h \
    jointcontroltab.h \
    jointplotwidget.h \
    csvrecorder.h

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
