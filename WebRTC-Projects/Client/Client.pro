TEMPLATE = app
CONFIG += console
CONFIG -= qt

SOURCES += \
    ../../trunk/talk/examples/peerconnection/client/peer_connection_client.cc \
    ../../trunk/talk/examples/peerconnection/client/main_wnd.cc \
    ../../trunk/talk/examples/peerconnection/client/main.cc \
    ../../trunk/talk/examples/peerconnection/client/defaults.cc \
    ../../trunk/talk/examples/peerconnection/client/conductor.cc \
    ../../trunk/talk/examples/peerconnection/client/linux/main_wnd.cc \
    ../../trunk/talk/examples/peerconnection/client/linux/main.cc

HEADERS += \
    ../../trunk/talk/examples/peerconnection/client/peer_connection_client.h \
    ../../trunk/talk/examples/peerconnection/client/main_wnd.h \
    ../../trunk/talk/examples/peerconnection/client/flagdefs.h \
    ../../trunk/talk/examples/peerconnection/client/defaults.h \
    ../../trunk/talk/examples/peerconnection/client/conductor.h \
    ../../trunk/talk/examples/peerconnection/client/linux/main_wnd.h

