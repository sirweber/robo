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

#include "talk/p2p/base/session.h"

#include "Presence.h"
#include "SessionManagement.h"

class Login : public sigslot::has_slots<> {
public:
  int DoLogin(std::string username, std::string password)
  {
      // Not using SSL for now, since there is a problem with
      // using self-signed certificates
      //talk_base::InitializeSSL();

      // Set up debugging.
      talk_base::LogMessage::LogToDebug(talk_base::LS_VERBOSE);

      buzz::Jid jid;
      jid = buzz::Jid(username);
      if (!jid.IsValid() || jid.node() == "") {
        printf("Invalid JID. JIDs should be in the form user@domain\n");
        return 1;
      }

      talk_base::InsecureCryptStringImpl pass;
      pass.password() = password;

      m_xcs.set_user(jid.node());
      m_xcs.set_resource("telepresence");  // Arbitrary resource name.
      m_xcs.set_host(jid.domain());
      m_xcs.set_allow_plain(false);
      m_xcs.set_use_tls(buzz::TLS_DISABLED);
      m_xcs.set_pass(talk_base::CryptString(pass));

      // TODO Server change
      //xcs.set_server(talk_base::SocketAddress("liteart.dyndns.org", 5222));
      m_xcs.set_server(talk_base::SocketAddress("localhost", 5222));

      talk_base::Thread* main_thread = talk_base::Thread::Current();

      // Sign up to receive signals from the XMPP pump to track sign in progress.
      m_pump.client()->SignalStateChange.connect(this, &Login::OnStateChange);

      // Queue up the sign in request.
      m_pump.DoLogin(m_xcs, new buzz::XmppSocket(buzz::TLS_DISABLED), new XmppAuth());

      // Start the thread and run indefinitely. This call blocks.
      main_thread->Run();
      m_pump.DoDisconnect();
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

          // Open a session
          m_session.Init(m_pump.client());
          // Signal our presence on the server
          m_presence.Init(m_pump.client(), &m_session, &m_xcs);

          break;
        case buzz::XmppEngine::STATE_CLOSED:
          //
          std::cout << "======>> Connection ended." << std::endl;
          break;
      default:
          //
          std::cout << "======>> Unsupported Status Change." << std::endl;
          break;
      }
  }

private:
  buzz::XmppClientSettings m_xcs;
  buzz::XmppPump m_pump;
  SessionManagement m_session;
  Presence m_presence;
};

int main(int argc, char **argv) {
    if (argc>2)
    {
        Login log;
        return log.DoLogin(argv[1], argv[2]);
    }
    else
    {
        std::cerr << "Invalid arguments. Taking username and password." << std::endl;
        return 1;
    }
}

