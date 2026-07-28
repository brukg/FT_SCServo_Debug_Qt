QT += core gui widgets serialport
qtHaveModule(charts): QT += charts
else {
  INCLUDEPATH += $$PWD/../third_party/qt5charts/include
  LIBS += -L$$PWD/../third_party/qt5charts/lib -lQt5Charts
  QMAKE_RPATHDIR += $$PWD/../third_party/qt5charts/lib
  DEFINES += QT_CHARTS_LIB
}
CONFIG += c++17
TARGET = shot_joints
INCLUDEPATH += ..
SOURCES += shot_joints.cpp ../jointrow.cpp ../jointcontroltab.cpp ../jointplotwidget.cpp ../servo/scserial.cpp
HEADERS += ../jointrow.h ../jointcontroltab.h ../jointplotwidget.h ../csvrecorder.h ../servo/scserial.h
