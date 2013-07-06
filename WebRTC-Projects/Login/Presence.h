#include <iostream>

#include <talk/base/sigslot.h>
#include <talk/xmpp/xmppclient.h>
#include <talk/xmpp/presencestatus.h>
#include <talk/xmpp/presenceouttask.h>
#include <talk/base/logging.h>


#include "presencepushtask.h"

class Presence : public sigslot::has_slots<>
{
public:
    ~Presence();

    void Init(buzz::XmppClient* xmpp_client);

private:

    // Status callbacks
    void OnStatusUpdate(const buzz::PresenceStatus& status);

    PresencePushTask* m_pushTask;
    buzz::PresenceStatus m_Status;
    buzz::PresenceOutTask* m_Presence_out;
};
