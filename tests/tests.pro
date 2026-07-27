QT       += core serialport testlib
QT       -= gui

CONFIG   += c++17 console testcase
CONFIG   -= app_bundle

TARGET = test_packets

INCLUDEPATH += ..

SOURCES += \
    test_packets.cpp \
    ../servo/scserial.cpp

HEADERS += \
    ../servo/scserial.h \
    ../servo/servo_types.h \
    ../servo/sms_sts.h \
    ../servo/scscl.h \
    ../servo/hlscl.h \
    ../servo/servo_dispatch.h
