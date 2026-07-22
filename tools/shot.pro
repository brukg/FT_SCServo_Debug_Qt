QT += core gui widgets serialport
CONFIG += c++17
TARGET = shot
INCLUDEPATH += ..
SOURCES += shot.cpp ../mainwindow.cpp ../simpegraphwiget.cpp ../servo/scserial.cpp
HEADERS += ../mainwindow.h ../simpegraphwiget.h ../servo/scserial.h
FORMS += ../mainwindow.ui
