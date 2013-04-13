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

#include "presencepushtask.h"

#include "talk/base/stringencode.h"
#include "talk/xmpp/constants.h"


bool PresencePushTask::HandleStanza(const buzz::XmlElement * stanza) {
  if (stanza->Name() != buzz::QN_PRESENCE)
    return false;
  QueueStanza(stanza);
  return true;
}

int PresencePushTask::ProcessStart() {
  const buzz::XmlElement * stanza = NextStanza();
  if (stanza == NULL)
    return STATE_BLOCKED;

  buzz::Jid from(stanza->Attr(buzz::QN_FROM));
  buzz::PresenceStatus s;
  FillStatus(from, stanza, &s);
  SignalStatusUpdate(s);

  return STATE_START;
}

bool PresencePushTask::IsXmlSpace(int ch) {
  return ch == ' ' || ch == '\n' || ch == '\r' || ch == '\t';
}
bool PresencePushTask::ListContainsToken(const std::string & list,
                              const std::string & token) {
  size_t i = list.find(token);
  if (i == std::string::npos || token.empty())
    return false;
  bool boundary_before = (i == 0 || IsXmlSpace(list[i - 1]));
  bool boundary_after = (i == list.length() - token.length() ||
                         IsXmlSpace(list[i + token.length()]));
  return boundary_before && boundary_after;
}
bool PresencePushTask::IsUtf8FirstByte(int c) {
  return (((c)&0x80)==0) || // is single byte
    ((unsigned char)((c)-0xc0)<0x3e); // or is lead byte
}
void PresencePushTask::FillStatus(const buzz::Jid& from, const buzz::XmlElement* stanza,
                                  buzz::PresenceStatus* s) {
  s->set_jid(from);
  if (stanza->Attr(buzz::QN_TYPE) == buzz::STR_UNAVAILABLE) {
    s->set_available(false);
  } else {
    s->set_available(true);
    const buzz::XmlElement * status = stanza->FirstNamed(buzz::QN_STATUS);
    if (status != NULL) {
      s->set_status(status->BodyText());

      // Truncate status messages longer than 300 bytes
      if (s->status().length() > 300) {
        size_t len = 300;

        // Be careful not to split legal utf-8 chars in half
        while (!IsUtf8FirstByte(s->status()[len]) && len > 0) {
          len -= 1;
        }
        std::string truncated(s->status(), 0, len);
        s->set_status(truncated);
      }
    }

    const buzz::XmlElement * priority = stanza->FirstNamed(buzz::QN_PRIORITY);
    if (priority != NULL) {
      int pri;
      if (talk_base::FromString(priority->BodyText(), &pri)) {
        s->set_priority(pri);
      }
    }

    const buzz::XmlElement * show = stanza->FirstNamed(buzz::QN_SHOW);
    if (show == NULL || show->FirstChild() == NULL) {
      s->set_show(buzz::PresenceStatus::SHOW_ONLINE);
    }
    else {
      if (show->BodyText() == "away") {
        s->set_show(buzz::PresenceStatus::SHOW_AWAY);
      }
      else if (show->BodyText() == "xa") {
        s->set_show(buzz::PresenceStatus::SHOW_XA);
      }
      else if (show->BodyText() == "dnd") {
        s->set_show(buzz::PresenceStatus::SHOW_DND);
      }
      else if (show->BodyText() == "chat") {
        s->set_show(buzz::PresenceStatus::SHOW_CHAT);
      }
      else {
        s->set_show(buzz::PresenceStatus::SHOW_ONLINE);
      }
    }

    const buzz::XmlElement * caps = stanza->FirstNamed(buzz::QN_CAPS_C);
    if (caps != NULL) {
      std::string node = caps->Attr(buzz::QN_NODE);
      std::string ver = caps->Attr(buzz::QN_VER);
      std::string exts = caps->Attr(buzz::QN_EXT);

      s->set_know_capabilities(true);
      s->set_caps_node(node);
      s->set_version(ver);

      if (ListContainsToken(exts, "voice-v1")) {
        s->set_voice_capability(true);
      }
      if (ListContainsToken(exts, "video-v1")) {
        s->set_video_capability(true);
      }
    }

    const buzz::XmlElement* delay = stanza->FirstNamed(buzz::kQnDelayX);
    if (delay != NULL) {
      // Ideally we would parse this according to the Psuedo ISO-8601 rules
      // that are laid out in JEP-0082:
      // http://www.jabber.org/jeps/jep-0082.html
      std::string stamp = delay->Attr(buzz::kQnStamp);
      s->set_sent_time(stamp);
    }

    const buzz::XmlElement* nick = stanza->FirstNamed(buzz::QN_NICKNAME);
    if (nick) {
      s->set_nick(nick->BodyText());
    }
  }
}
