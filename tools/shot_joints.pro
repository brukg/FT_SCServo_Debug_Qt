QT += core gui widgets serialport
CONFIG += c++17
TARGET = shot_joints
INCLUDEPATH += ..
SOURCES += shot_joints.cpp ../jointrow.cpp ../jointcontroltab.cpp ../servo/scserial.cpp
HEADERS += ../jointrow.h ../jointcontroltab.h ../servo/scserial.h
