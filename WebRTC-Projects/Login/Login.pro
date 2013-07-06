TEMPLATE = app
CONFIG += console
CONFIG -= qt

SOURCES += \
    login_main.cc \
    SessionManagement.cpp \
    Presence.cpp \
    NetworkManager.cpp \
    presencepushtask.cc

HEADERS += \
    SessionManagement.h \
    Presence.h \
    NetworkManager.h \
    sessionmanagertask.h \
    presencepushtask.h

INCLUDEPATH += ../libjingle

QMAKE_CXXFLAGS += -pthread -I/usr/include/glib-2.0 -I/usr/lib/x86_64-linux-gnu/glib-2.0/include \
        -I/usr/include/gtk-2.0 -I/usr/lib/x86_64-linux-gnu/gtk-2.0/include \
        -I/usr/include/atk-1.0 -I/usr/include/cairo -I/usr/include/gdk-pixbuf-2.0 \
        -I/usr/include/pango-1.0 -I/usr/include/gio-unix-2.0/ -I/usr/include/pixman-1 \
        -I/usr/include/freetype2 -I/usr/include/libpng12  \
        -frtti \
        -fstack-protector \
        --param=ssp-buffer-size=4 \
        -Werror \
        -pthread \
        #-fno-exceptions \
        -fno-strict-aliasing \
        -Wall \
        -Wno-unused-parameter \
        #-Wno-missing-field-initializers \
        #-fvisibility=hidden \
        #-pipe \
        -fPIC \
        -Wextra \
        -O0 \
        -g -ggdb \
        #-fno-ident \
        #-fdata-sections \
        #-ffunction-sections \
        '-DWEBRTC_SVNREVISION="Unavailable(issue687)"' \
        '-D_FILE_OFFSET_BITS=64' \
        '-DUSE_LINUX_BREAKPAD' \
        '-DUSE_DEFAULT_RENDER_THEME=1' \
        '-DUSE_LIBJPEG_TURBO=1' \
        '-DUSE_NSS=1' \
        '-DENABLE_ONE_CLICK_SIGNIN' \
        '-DGTK_DISABLE_SINGLE_INCLUDES=1' \
        '-DENABLE_REMOTING=1' \
        '-DENABLE_WEBRTC=1' \
        '-DENABLE_CONFIGURATION_POLICY' \
        '-DENABLE_INPUT_SPEECH' \
        '-DENABLE_NOTIFICATIONS' \
        '-DENABLE_GPU=1' \
        '-DENABLE_EGLIMAGE=1' \
        '-DENABLE_TASK_MANAGER=1' \
        '-DENABLE_EXTENSIONS=1' \
        '-DENABLE_PLUGIN_INSTALLATION=1' \
        '-DENABLE_PLUGINS=1' \
        '-DENABLE_SESSION_SERVICE=1' \
        '-DENABLE_THEMES=1' \
        '-DENABLE_BACKGROUND=1' \
        '-DENABLE_AUTOMATION=1' \
        '-DENABLE_GOOGLE_NOW=1' \
        '-DENABLE_LANGUAGE_DETECTION=1' \
        '-DENABLE_PRINTING=1' \
        '-DENABLE_CAPTIVE_PORTAL_DETECTION=1' \
        '-DENABLE_MANAGED_USERS=1' \
        '-DWEBRTC_LOGGING' \
        '-DWEBRTC_LINUX' \
        '-DUNIT_TEST' \
        '-DGTEST_HAS_RTTI=0' \
        '-D__STDC_CONSTANT_MACROS' \
        '-D__STDC_FORMAT_MACROS' \
        '-DNDEBUG' \
        '-DNVALGRIND' \
        '-DDYNAMIC_ANNOTATIONS_ENABLED=0' \
        '-D_FORTIFY_SOURCE=2' \
        '-DPOSIX' \        
        '-D_DEBUG'

QMAKE_LFLAGS +=  \
        -Wl,-z,now \
        -Wl,-z,relro \
        -pthread \
        -Wl,-z,noexecstack \
        -fPIC \
        -Wl,-O1 \
        -Wl,--as-needed \
        -Wl,--gc-sections

LIBS += -Wl,-Bstatic -Wl,-start-group \
            -L../libjingle/out/Debug/obj.target/talk \
            -ljingle_media \
            -L../libjingle/out/Debug/obj.target/third_party/webrtc/voice_engine -lvoice_engine_core \
            -ljingle_peerconnection -ljingle_p2p  -ljingle_sound -ljingle -ljingle_xmpphelp  \
            -L../libjingle/out/Debug/obj.target/third_party/libyuv -lyuv \
            -L../libjingle/out/Debug/obj.target/third_party/webrtc/modules -lwebrtc_utility \
            -L../libjingle/out/Debug/obj.target/third_party/opus -lopus \
            -L../libjingle/out/Debug/obj.target/third_party/libsrtp -lsrtp \
            -L../libjingle/out/Debug/obj.target/third_party/webrtc/common_audio -lresampler \
            -L../libjingle/out/Debug/obj.target/third_party/jsoncpp -ljsoncpp \
            -L../libjingle/out/Debug/obj.target/third_party/webrtc/common_audio -lvad -lsignal_processing \
            -L../libjingle/out/Debug/obj.target/third_party/webrtc/common_video -lcommon_video \
            -L../libjingle/out/Debug/obj.target/third_party/webrtc/modules/video_coding/codecs/vp8 -lwebrtc_vp8 \
            -L../libjingle/out/Debug/obj.target/third_party/webrtc/modules/video_coding/utility -lvideo_coding_utility \
            -L../libjingle/out/Debug/obj.target/third_party/webrtc/video_engine -lvideo_engine_core \
            -L../libjingle/out/Debug/obj.target/third_party/webrtc/system_wrappers/source -lsystem_wrappers \
            -L../libjingle/out/Debug/obj.target/third_party/webrtc/modules -laudio_processing \
            -lvideo_processing_sse2 \
            -laudio_processing_sse2 \
            -lwebrtc_video_coding \
            -laudio_coding_module \
            -lNetEq \
            -lG722 \
            -lPCM16B \
            -liSAC \
            -lremote_bitrate_estimator \
            -liLBC \
            -lCNG \
            -lvideo_capture_module \
            -lmedia_file \
            -laudio_conference_mixer \
            -lwebrtc_i420 \
            -laudioproc_debug_proto \
            -lwebrtc_opus \
            -liSACFix \
            -laudio_device \
            -lbitrate_controller \
            -lG711 \
            -lvideo_render_module \
            -lvideo_processing \
            -lrtp_rtcp \
            -lpaced_sender \
            -L../libjingle/out/Debug/obj.target/third_party/libvpx -lvpx -lvpx_intrinsics_sse2 -lvpx_intrinsics_ssse3 \
            -lvpx_intrinsics_mmx -lvpx_asm_offsets_vp8 \
            -L../libjingle/out/Debug/obj.target/third_party/protobuf -lprotobuf_lite \
            -L../libjingle/out/Debug/obj.target/third_party/libjpeg_turbo -ljpeg_turbo \
        -Wl,--end-group -Wl,-Bdynamic \
        \
        -lgthread-2.0 -lrt -latk-1.0 -lgio-2.0 -lpangoft2-1.0 \
        -lpangocairo-1.0 -lgdk_pixbuf-2.0 -lcairo -lpango-1.0 -lfreetype -lfontconfig -lgobject-2.0 \
        -lglib-2.0 -lX11 -lXcomposite -lXext -lXrender \
        -ldl \
        -lcrypto \
        -lssl \
        -lnss3 \
        -lnssutil3 \
        -lsmime3 \
        -lplds4 \
        -lplc4 \
        -lnspr4 \
        -lexpat
