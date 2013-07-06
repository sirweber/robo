#include <iostream>

#include <talk/base/network.h>

class NetworkManager : public talk_base::NetworkManagerBase
{
public:
    // Start/Stop monitoring of network interfaces
    // list. SignalNetworksChanged or SignalError is emitted immidiately
    // after StartUpdating() is called. After that SignalNetworksChanged
    // is emitted wheneven list of networks changes.
    void StartUpdating();
    void StopUpdating();
};
