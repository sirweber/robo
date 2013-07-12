#pragma once
#include <iostream>

#include <talk/base/sigslot.h>
#include <talk/xmpp/xmppclient.h>
#include <talk/media/base/mediaengine.h>
#include <talk/media/base/rtpdataengine.h>
#include <talk/session/media/mediasessionclient.h>

#include <talk/p2p/client/basicportallocator.h>
#include <talk/p2p/base/sessionclient.h>
#include "sessionmanagertask.h"

class SessionClientImpl : public cricket::SessionClient, public sigslot::has_slots<>
{
public:
    // Notifies the client of the creation / destruction of sessions of this type.
    //
    // IMPORTANT: The SessionClient, in its handling of OnSessionCreate, must
    // create whatever channels are indicate in the description.  This is because
    // the remote client may already be attempting to connect those channels. If
    // we do not create our channel right away, then connection may fail or be
    // delayed.

    // This is called when a Session is created (in- or outbound)
    void OnSessionCreate(cricket::Session* session, bool received_initiate);
    void OnMediaStreamsUpdate(cricket::Call* call,
                              cricket::Session* session,
                              const cricket::MediaStreams& added,
                              const cricket::MediaStreams& removed);
    void OnSessionDestroy(cricket::Session* session);

    bool ParseContent(cricket::SignalingProtocol protocol,
                              const buzz::XmlElement* elem,
                              cricket::ContentDescription** content,
                              cricket::ParseError* error);
    bool WriteContent(cricket::SignalingProtocol protocol,
                              const cricket::ContentDescription* content,
                              buzz::XmlElement** elem,
                              cricket::WriteError* error);

    void OnSessionState(cricket::Call* call,
                        cricket::Session* session,
                        cricket::Session::State state);
};

class SessionManagement : public sigslot::has_slots<> {
public:
    SessionManagement();
    ~SessionManagement();

    void Init(buzz::XmppClient* xmpp_client);

    void OnRequestSignaling();
    void OnSessionCreate(cricket::Session* session, bool initiate);
    void OnCallCreate(cricket::Call* call);
    void OnCallDestroy(cricket::Call* call);
    void OnDevicesChange();
    cricket::MediaSessionClient* GetClient(){ return m_media_client; }

private:

    /*void OnJingleInfo(const std::string &relay_token,
                      const std::vector<std::string> &relay_addresses,
                      const std::vector<talk_base::SocketAddress> &stun_addresses)
    {
        std::cerr << "PING" << std::endl;
    }*/

    cricket::BasicPortAllocator* m_port_allocator;
    cricket::SessionManager* m_session_manager;
    cricket::SessionManagerTask* m_session_manager_task;
    //buzz::JingleInfoTask* m_jingle_info_task;
    talk_base::Thread* m_worker;
    cricket::SignalingProtocol m_signaling_protocol;
    cricket::TransportProtocol m_transport_protocol;
    cricket::DataEngineInterface* m_data_engine;
    cricket::MediaEngineInterface* m_media_engine;
    cricket::MediaSessionClient* m_media_client;
    SessionClientImpl* m_session_client;

};
