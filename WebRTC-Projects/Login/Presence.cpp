#include "Presence.h"


Presence::~Presence()
{
    delete m_pushTask;
    delete m_Presence_out;
}

void Presence::Init(buzz::XmppClient* xmpp_client, SessionManagement* session, buzz::XmppClientSettings* client)
{
    m_Session = session;
    m_Client = client;

    // Set up debugging.
    talk_base::LogMessage::LogToDebug(talk_base::LS_VERBOSE);

    // Create a PresencePushTask object to retrieve presence information from the server.
    // Create this first to avoid missing a notification received by the sign in process.
    m_pushTask = new PresencePushTask(xmpp_client);

    // Hook up to the notification signal sent when the stanza is received.
    m_pushTask->SignalStatusUpdate.connect(this, &Presence::OnStatusUpdate);
    // Start listening for presence stanzas.
    m_pushTask->Start();

    m_Presence_out = new buzz::PresenceOutTask(xmpp_client);
    m_Status.set_jid(xmpp_client->jid());
    m_Status.set_available(true);
    m_Status.set_show(buzz::PresenceStatus::SHOW_ONLINE);

    m_Presence_out->Send(m_Status);
    m_Presence_out->Start();
}

// Status callbacks
void Presence::OnStatusUpdate(const buzz::PresenceStatus& status) {

  if (status.available())
  {
        std::cout << status.jid().node() << " is available and ready to chat." << std::endl;

        //if (status.jid().node() != m_Client->user())
        if (m_Client->user() == "blyth")
        {
            // TODO Make connection
            cricket::Call* call = m_Session->GetClient()->CreateCall();

            cricket::CallOptions opt;
            cricket::Session* session = call->InitiateSession(status.jid(), buzz::Jid(m_Client->user(), m_Client->host(), m_Client->resource()), opt);

            cricket::SendDataParams params;
            talk_base::Buffer payload("text", 5);
            cricket::SendDataResult result;
            bool sent = call->SendData(session, params, payload, &result);

            // TODO
            // Could not send data: no data channel.

            std::cout << sent << std::endl;
        }
  }
  else
  {
    std::cout << status.jid().node() << " is no longer able to chat." << std::endl;
  }
}
