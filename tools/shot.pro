QT += core gui widgets serialport
qtHaveModule(charts): QT += charts
else {
  INCLUDEPATH += $$PWD/../third_party/qt5charts/include
  LIBS += -L$$PWD/../third_party/qt5charts/lib -lQt5Charts
  QMAKE_RPATHDIR += $$PWD/../third_party/qt5charts/lib
  DEFINES += QT_CHARTS_LIB
}
CONFIG += c++17
TARGET = shot
INCLUDEPATH += ..
SOURCES += shot.cpp ../mainwindow.cpp ../simpegraphwiget.cpp ../servo/scserial.cpp ../jointrow.cpp ../jointcontroltab.cpp ../jointplotwidget.cpp
HEADERS += ../mainwindow.h ../simpegraphwiget.h ../servo/scserial.h ../jointrow.h ../jointcontroltab.h ../jointplotwidget.h ../csvrecorder.h
FORMS += ../mainwindow.ui
