TEMPLATE = app
CONFIG += console
CONFIG -= qt

SOURCES += \
    ../../trunk/talk/examples/peerconnection/server/utils.cc \
    ../../trunk/talk/examples/peerconnection/server/peer_channel.cc \
    ../../trunk/talk/examples/peerconnection/server/main.cc \
    ../../trunk/talk/examples/peerconnection/server/data_socket.cc

OTHER_FILES += \
    ../../trunk/talk/examples/peerconnection/server/server_test.html

HEADERS += \
    ../../trunk/talk/examples/peerconnection/server/utils.h \
    ../../trunk/talk/examples/peerconnection/server/peer_channel.h \
    ../../trunk/talk/examples/peerconnection/server/data_socket.h

