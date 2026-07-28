QT += core gui widgets serialport
qtHaveModule(charts): QT += charts
else {
  INCLUDEPATH += $$PWD/../third_party/qt5charts/include
  LIBS += -L$$PWD/../third_party/qt5charts/lib -lQt5Charts
  QMAKE_RPATHDIR += $$PWD/../third_party/qt5charts/lib
  DEFINES += QT_CHARTS_LIB
}
CONFIG += c++17
TARGET = shot_plot
INCLUDEPATH += ..
SOURCES += shot_plot.cpp ../jointplotwidget.cpp
HEADERS += ../jointplotwidget.h ../csvrecorder.h
