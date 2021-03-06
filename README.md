ndnSIM
======

- This project is a fork ndnSIM (https://github.com/named-data/ndnSIM)
- To clone the project use the following command which will clone the submodules (ndn-cxx and NFD): 

``git clone --recursive https://github.com/named-data-ndnSIM/ndnSIM.git ns-3/src/ndnSIM``

- To install use the guideline from ndnSIM website: https://ndnsim.net/current/getting-started.html#compiling-and-running-ndnsim
- [NDN Packet Specification](http://named-data.net/doc/NDN-packet-spec/current/)

- ndnSIM uses implementation of basic NDN primitives from
  [ndn-cxx library (NDN C++ library with eXperimental eXtensions)](http://named-data.net/doc/ndn-cxx/)

  Based on version `0.6.5`

- All NDN forwarding and management is implemented directly using source code of
  [Named Data Networking Forwarding Daemon (NFD)](http://named-data.net/doc/NFD/)

  Based on version `0.6.5`

- Allows [simulation of real applications](http://ndnsim.net/guide-to-simulate-real-apps.html)
  written against ndn-cxx library

- Requires a modified version of NS-3 based on version `ns-3.29`

[ndnSIM documentation](http://ndnsim.net)
---------------------------------------------

For more information, including downloading and compilation instruction, please refer to
http://ndnsim.net or documentation in `docs/` folder.
