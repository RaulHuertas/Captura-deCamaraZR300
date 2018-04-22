QT -= gui
#QT -= core

CONFIG +=  console
CONFIG += c++14

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += main.cpp \
    transmitudp.cpp \
    video_utils.cpp \
    storage.cpp
SOURCES += main2.cpp


#macx{
#    INCLUDEPATH += /usr/local/include
#    LIBS += -L/usr/local/lib -lrealsense
#}


macx{
win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../../../usr/local/lib/release/ -lrealsense2.2.8.0
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../../../usr/local/lib/debug/ -lrealsense2.2.8.0
else:unix: LIBS += -L$$PWD/../../../../../../../usr/local/lib/ -lrealsense2.2.8.0
}

linux{
        LIBS += -lrealsense -lrealsense_projection -lrealsense2 -lrealsense_image
        LIBS += -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_video -lopencv_videoio

        INCLUDEPATH += /home/rhuertas/realsense_sdk_zr300/realsense_sdk_zr300/sdk/include
}

INCLUDEPATH += $$PWD/../../../../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../../../../usr/local/include

HEADERS += \
    network_utils.hpp \
    video_utils.hpp
