QT       += core serialport
QT       -= gui
CONFIG   += c++17 console
CONFIG   -= app_bundle
TARGET    = probe_bus
INCLUDEPATH += ..
SOURCES  += probe_bus.cpp ../servo/scserial.cpp
HEADERS  += ../servo/scserial.h ../servo/servo_types.h ../servo/sms_sts.h ../servo/scscl.h ../servo/hlscl.h
