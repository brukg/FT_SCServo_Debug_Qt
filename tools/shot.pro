QT += core gui widgets serialport
CONFIG += c++17
TARGET = shot
INCLUDEPATH += ..
SOURCES += shot.cpp ../mainwindow.cpp ../simpegraphwiget.cpp ../servo/scserial.cpp ../jointrow.cpp ../jointcontroltab.cpp
HEADERS += ../mainwindow.h ../simpegraphwiget.h ../servo/scserial.h ../jointrow.h ../jointcontroltab.h
FORMS += ../mainwindow.ui
