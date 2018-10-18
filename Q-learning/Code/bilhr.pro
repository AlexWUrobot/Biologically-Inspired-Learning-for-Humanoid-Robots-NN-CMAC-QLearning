    TEMPLATE = app
    CONFIG += console
    CONFIG -= qt
    CONFIG += C++11



    SOURCES += main.cpp


    #unix:!macx:!symbian: LIBS += -L$$PWD/../Desktop/Rpi-hw-master/build/ -lrpihw

    #INCLUDEPATH += $$PWD/../Desktop/Rpi-hw-master/include/rpi-hw
    #DEPENDPATH += $$PWD/../Desktop/Rpi-hw-master/include/rpi-hw

    unix:!macx:!symbian: LIBS += -lrpihw
