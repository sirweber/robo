#include "SessionManagement.h"
#include "NetworkManager.h"

void SessionClientImpl::OnSessionState(cricket::Call* call,
                                       cricket::Session* session,
                                       cricket::Session::State state)
{

  std::cout << "OnSessionState: " << state << std::endl;

}
void SessionClientImpl::OnMediaStreamsUpdate(cricket::Call* call,
                                             cricket::Session* session,
                                             const cricket::MediaStreams& added,
                                             const cricket::MediaStreams& removed)
{

  std::cout << "OnMediaStreamsUpdate" << std::endl;

}
/*Your implementation should determine whether the session represents an incoming or an outgoing
    connection request. If this is incoming, received_initiate will be True, and your application
    should connect to the Session's signals and perform any other session-specific tasks, such as
    adding it to a list of active sessions and instantiating helper objects or resources.*/
void SessionClientImpl::OnSessionCreate(cricket::Session *session, bool received_initiate)
{
    if (received_initiate) {

    }
    std::cout << "New session created!" << std::endl;
}
void SessionClientImpl::OnSessionDestroy(cricket::Session* session)
{
    std::cout << "OnSessionDestroy!" << std::endl;

    // TODO Something along the line of
    /*
      // Find the call this session is in, remove it
      SessionMap::iterator it = session_map_.find(session->id());
      ASSERT(it != session_map_.end());
      if (it != session_map_.end()) {
        Call *call = (*it).second;
        session_map_.erase(it);
        call->RemoveSession(session);
      }
    */
}

bool SessionClientImpl::ParseContent(cricket::SignalingProtocol protocol,
                          const buzz::XmlElement* elem,
                          cricket::ContentDescription** content,
                          cricket::ParseError* error)
{
    const std::string& content_type = elem->Name().Namespace();
    std::cout << "Got some content with type = " << content_type << std::endl;

    return true;
}
bool SessionClientImpl::WriteContent(cricket::SignalingProtocol protocol,
                          const cricket::ContentDescription* content,
                          buzz::XmlElement** elem,
                          cricket::WriteError* error)
{
    *elem = new buzz::XmlElement(cricket::QN_GINGLE_AUDIO_CONTENT, true);
    //elem->AddElement(CreateGingleAudioCodecElem(*codec));

    return true;
}

//////////////////////////

SessionManagement::SessionManagement() :
    m_signaling_protocol(cricket::PROTOCOL_HYBRID),
    m_transport_protocol(cricket::ICEPROTO_HYBRID),
    m_data_engine(NULL),
    m_media_engine(NULL),
    m_session_client(NULL)
{
}
SessionManagement::~SessionManagement()
{
    delete m_session_manager;
    delete m_port_allocator;
    delete m_session_manager_task;
    //delete m_jingle_info_task;
    m_worker->Stop();
    delete m_worker;
    delete m_media_engine;
    delete m_data_engine;
    delete m_media_client;
    delete m_session_client;
}

void SessionManagement::Init(buzz::XmppClient* xmpp_client)
{
    NetworkManager network_manager;
    m_worker = new talk_base::Thread();
    m_worker->Start();

    // TODO also takes relay- and stun-server info in the ctor. not for now.
    m_port_allocator = new cricket::BasicPortAllocator(&network_manager);
    m_session_manager = new cricket::SessionManager(m_port_allocator, m_worker);
    //m_session_manager->SignalRequestSignaling.connect(this, &SessionManagement::OnRequestSignaling);
    //m_session_manager->SignalSessionCreate.connect(this, &SessionManagement::OnSessionCreate);
    m_session_manager->OnSignalingReady();

    m_session_manager_task = new cricket::SessionManagerTask(xmpp_client, m_session_manager);
    m_session_manager_task->EnableOutgoingMessages();
    m_session_manager_task->Start();

    // Query for the STUN and relay ports being used. This is an asynchronous call,
    // so we need to register for SignalJingleInfo, which sends the response.
    /*m_jingle_info_task = new buzz::JingleInfoTask(xmpp_client);
    m_jingle_info_task->RefreshJingleInfoNow();
    m_jingle_info_task->SignalJingleInfo.connect(this, &SessionManagement::OnJingleInfo);
    m_jingle_info_task->Start();*/

    m_session_client = new SessionClientImpl;
    m_session_manager->AddClient(cricket::NS_GINGLE_RAW, m_session_client);

    if (!m_media_engine) {
      m_media_engine = cricket::MediaEngineFactory::Create();
    }
    if (!m_data_engine) {
      m_data_engine = new cricket::RtpDataEngine();
    }

    m_media_client = new cricket::MediaSessionClient(
        xmpp_client->jid(),
        m_session_manager,
        m_media_engine,
        m_data_engine,
        cricket::DeviceManagerFactory::Create());
    m_media_client->SignalCallCreate.connect(this, &SessionManagement::OnCallCreate);
    m_media_client->SignalCallDestroy.connect(this, &SessionManagement::OnCallDestroy);
    m_media_client->SignalDevicesChange.connect(this, &SessionManagement::OnDevicesChange);
    m_media_client->set_secure(cricket::SEC_DISABLED);
    m_media_client->set_multisession_enabled(false);
}

void SessionManagement::OnRequestSignaling() {
  m_session_manager->OnSignalingReady();
}
void SessionManagement::OnSessionCreate(cricket::Session* session, bool initiate) {
  session->set_current_protocol(m_signaling_protocol);
}
void SessionManagement::OnCallCreate(cricket::Call* call)
{
    std::cerr << "SESSION CREATE" << std::endl;
    call->SignalSessionState.connect(m_session_client, &SessionClientImpl::OnSessionState);
    call->SignalMediaStreamsUpdate.connect(m_session_client, &SessionClientImpl::OnMediaStreamsUpdate);
}
void SessionManagement::OnCallDestroy(cricket::Call* call)
{
    std::cerr << "SESSION DESTROY" << std::endl;
}
void SessionManagement::OnDevicesChange()
{
    std::cerr << "SESSION CHANGE" << std::endl;
}
