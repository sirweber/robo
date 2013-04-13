/*
 * libjingle
 * Copyright 2004--2005, Google Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *  3. The name of the author may not be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstdio>
#include <iostream>

#include "talk/base/ssladapter.h"
#include "talk/base/thread.h"
#include "talk/xmpp/constants.h"
#include "talk/xmpp/xmppclientsettings.h"
#include "talk/xmpp/xmppengine.h"
#include "talk/xmpp/xmppthread.h"
#include "talk/xmpp/xmppauth.h"
#include "talk/xmpp/presenceouttask.h"
#include "talk/base/logging.h"

#include "presencepushtask.h"

class Presence : public sigslot::has_slots<>
{
public:
    Presence(buzz::XmppClient* xmpp_client):
        m_xmpp_client(xmpp_client)
    {}

    int start()
    {
        // Set up debugging.
        talk_base::LogMessage::LogToDebug(talk_base::LS_VERBOSE);

        // Create a PresencePushTask object to retrieve presence information from the server.
        // Create this first to avoid missing a notification received by the sign in process.
        m_pushTask = new PresencePushTask(m_xmpp_client);

        // Hook up to the notification signal sent when the stanza is received.
        m_pushTask->SignalStatusUpdate.connect(this, &Presence::OnStatusUpdate);
        // Start listening for presence stanzas.
        m_pushTask->Start();

        m_Presence_out = new buzz::PresenceOutTask(m_xmpp_client);
        m_Status.set_jid(m_xmpp_client->jid());
        m_Status.set_available(true);
        m_Status.set_show(buzz::PresenceStatus::SHOW_ONLINE);

        m_Presence_out->Send(m_Status);
        m_Presence_out->Start();
    }

private:

    // Status callbacks
    void OnStatusUpdate(const buzz::PresenceStatus& status) {

      if (status.available()) {
         std::cout << status.jid().node() << " is available and ready to chat." << std::endl;
      } else {
        std::cout << status.jid().node() << " is no longer able to chat." << std::endl;
      }
    }

    buzz::XmppClient* m_xmpp_client;
    PresencePushTask* m_pushTask;
    buzz::PresenceStatus m_Status;
    buzz::PresenceOutTask* m_Presence_out;
};

class MyLoginClass : public sigslot::has_slots<> {
public:
  int Login(std::string username, std::string password) {
      // Not using SSL for now, since there is a problem with
      // using self-signed certificates
      //talk_base::InitializeSSL();

      // Set up debugging.
      talk_base::LogMessage::LogToDebug(talk_base::LS_VERBOSE);

      buzz::XmppClientSettings xcs;
      buzz::Jid jid;
      jid = buzz::Jid(username);
      if (!jid.IsValid() || jid.node() == "") {
        printf("Invalid JID. JIDs should be in the form user@domain\n");
        return 1;
      }

      talk_base::InsecureCryptStringImpl pass;
      pass.password() = password;

      xcs.set_user(jid.node());
      xcs.set_resource("telepresence");  // Arbitrary resource name.
      xcs.set_host(jid.domain());
      xcs.set_allow_plain(false);
      xcs.set_use_tls(buzz::TLS_DISABLED);
      xcs.set_pass(talk_base::CryptString(pass));
      xcs.set_server(talk_base::SocketAddress("liteart.dyndns.org", 5222));

      talk_base::Thread* main_thread = talk_base::Thread::Current();

      // Sign up to receive signals from the XMPP pump to track sign in progress.
      buzz::XmppPump pump;
      pump.client()->SignalStateChange.connect(this, &MyLoginClass::OnStateChange);

      // Queue up the sign in request.
      pump.DoLogin(xcs, new buzz::XmppSocket(buzz::TLS_DISABLED), new XmppAuth());

      // Signal our presence on the server
      Presence p(pump.client());
      p.start();

      // Start the thread and run indefinitely.
      main_thread->Run();
      pump.DoDisconnect();
      return 0;
  }

  // Status callbacks
  void OnStateChange(buzz::XmppEngine::State state) {
      switch (state) {
        case buzz::XmppEngine::STATE_START:
          //
          std::cout << "======>> Attempting sign in." << std::endl;
          break;
        case buzz::XmppEngine::STATE_OPENING:
          //
          std::cout << "======>> Negotiating with server." << std::endl;
          break;
        case buzz::XmppEngine::STATE_OPEN:
          //  Send your presence information.
          // and sign up to receive presence notifications.
          std::cout << "======>> Connection succeeded." << std::endl;
          break;
        case buzz::XmppEngine::STATE_CLOSED:
          //
          std::cout << "======>> Connection ended." << std::endl;
          break;
      }
  }
};

int main(int argc, char **argv) {
    if (argc>2)
    {
        MyLoginClass log;
        return log.Login(argv[1], argv[2]);
    }
    else
    {
        std::cerr << "Invalid arguments. Taking username and password." << std::endl;
        return 1;
    }
}

