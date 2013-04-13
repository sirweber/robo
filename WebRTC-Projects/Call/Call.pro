TEMPLATE = app
CONFIG += console
CONFIG -= qt

SOURCES += \
    ../../trunk/talk/examples/call/presencepushtask.cc \
    ../../trunk/talk/examples/call/mucinvitesendtask.cc \
    ../../trunk/talk/examples/call/mucinviterecvtask.cc \
    ../../trunk/talk/examples/call/mediaenginefactory.cc \
    ../../trunk/talk/examples/call/friendinvitesendtask.cc \
    ../../trunk/talk/examples/call/console.cc \
    ../../trunk/talk/examples/call/call_unittest.cc \
    ../../trunk/talk/examples/call/call_main.cc \
    ../../trunk/talk/examples/call/callclient_unittest.cc \
    ../../trunk/talk/examples/call/callclient.cc

OTHER_FILES += \
    ../../trunk/talk/examples/call/Info.plist

HEADERS += \
    ../../trunk/talk/examples/call/presencepushtask.h \
    ../../trunk/talk/examples/call/mucinvitesendtask.h \
    ../../trunk/talk/examples/call/mucinviterecvtask.h \
    ../../trunk/talk/examples/call/muc.h \
    ../../trunk/talk/examples/call/mediaenginefactory.h \
    ../../trunk/talk/examples/call/friendinvitesendtask.h \
    ../../trunk/talk/examples/call/console.h \
    ../../trunk/talk/examples/call/callclient.h

