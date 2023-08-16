/*
 * Copyright (c) 2020 Fraunhofer FOKUS and others. All rights reserved.
 *
 * Contact: mosaic@fokus.fraunhofer.de
 *
 * This class is developed for the MOSAIC-NS-3 coupling.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "ns3/yans-wifi-phy.h"
#include "mosaic-node-manager.h"

#include "mosaic-ns3-server.h"

#include "ns3/wave-net-device.h"
#include "mosaic-proxy-app.h"
#include "ns3/string.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/log.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/log.h"
#include "ns3/wifi-net-device.h"
#include "ns3/node-list.h"

NS_LOG_COMPONENT_DEFINE("MosaicNodeManager");

namespace ns3 {
    
    NS_OBJECT_ENSURE_REGISTERED(MosaicNodeManager);

    TypeId MosaicNodeManager::GetTypeId(void) {
        static TypeId tid = TypeId("ns3::MosaicNodeManager")
                .SetParent<Object>()
                .AddConstructor<MosaicNodeManager>()
                .AddAttribute("LossModel", "The used loss model",
                StringValue("ns3::FriisPropagationLossModel"),
                MakeStringAccessor(&MosaicNodeManager::m_lossModel),
                MakeStringChecker())
                .AddAttribute("DelayModel", "The used delay model",
                StringValue("ns3::ConstantSpeedPropagationDelayModel"),
                MakeStringAccessor(&MosaicNodeManager::m_delayModel),
                MakeStringChecker());
        return tid;
    }

    MosaicNodeManager::MosaicNodeManager() : m_ipAddressHelper("10.1.0.0", "255.255.0.0") {
    }

    void MosaicNodeManager::Configure(MosaicNs3Server* serverPtr) {
        m_serverPtr = serverPtr;
        m_wifiChannelHelper.AddPropagationLoss(m_lossModel);
        m_wifiChannelHelper.SetPropagationDelay(m_delayModel);
        m_channel = m_wifiChannelHelper.Create();
        m_wifiPhyHelper.SetChannel(m_channel);
    }

    void MosaicNodeManager::CreateMosaicNode(int ID, Vector position) {
        if (m_isDeactivated[ID]) {
            return;
        }
        Ptr<Node> singleNode = CreateObject<Node>();
        
        NS_LOG_INFO("Created node " << singleNode->GetId());
        m_mosaic2ns3ID[ID] = singleNode->GetId();

        //Install Wave device
        NS_LOG_INFO("Install WAVE on node " << singleNode->GetId());
        InternetStackHelper internet;   
        internet.Install(singleNode);
        NetDeviceContainer netDevices = m_wifi80211pHelper.Install(m_wifiPhyHelper, m_waveMacHelper, singleNode);
        m_ipAddressHelper.Assign(netDevices);

        //Install app
        NS_LOG_INFO("Install MosaicProxyApp application on node " << singleNode->GetId());
        Ptr<MosaicProxyApp> app = CreateObject<MosaicProxyApp>();
        app->SetNodeManager(this);
        singleNode->AddApplication(app);
        app->SetSockets();

        //Install mobility model
        NS_LOG_INFO("Install MosaicMobilityModel on node " << singleNode->GetId());
        Ptr<ConstantVelocityMobilityModel> mobModel = CreateObject<ConstantVelocityMobilityModel>();
        mobModel->SetPosition(position);
        singleNode->AggregateObject(mobModel);

        return;
    }

    uint32_t MosaicNodeManager::GetNs3NodeId(uint32_t nodeId) {
        return m_mosaic2ns3ID[nodeId];
    }

    void MosaicNodeManager::SendMsg(uint32_t nodeId, uint32_t protocolID, uint32_t msgID, uint32_t payLength, Ipv4Address ipv4Add) {
        if (m_isDeactivated[nodeId]) {
            return;
        }
        NS_LOG_INFO("Mosaic MosaicNodeManager::SendMsg " << nodeId);

        Ptr<Node> node = NodeList::GetNode(m_mosaic2ns3ID[nodeId]);
        Ptr<MosaicProxyApp> app = DynamicCast<MosaicProxyApp> (node->GetApplication(0));
        if (app == nullptr) {
            NS_LOG_ERROR("Node " << nodeId << " was not initialized properly, MosaicProxyApp is missing");
            return;
        }
        app->TransmitPacket(protocolID, msgID, payLength, ipv4Add);
    }

    void MosaicNodeManager::AddRecvPacket(unsigned long long recvTime, Ptr<Packet> pack, int nodeID, int msgID) {
        if (m_isDeactivated[nodeID]) {
            return;
        }
        
        m_serverPtr->AddRecvPacket(recvTime, pack, nodeID, msgID);
    }

    void MosaicNodeManager::UpdateNodePosition(uint32_t nodeId, Vector position) {
        if (m_isDeactivated[nodeId]) {
            return;
        }
        
        Ptr<Node> node = NodeList::GetNode(nodeId);
        Ptr<MobilityModel> mobModel = node->GetObject<MobilityModel> ();
        mobModel->SetPosition(position);
    }

    void MosaicNodeManager::DeactivateNode(uint32_t nodeId) {
        if (m_isDeactivated[nodeId]) {
            return;
        }
        
        Ptr<Node> node = NodeList::GetNode(nodeId);
        Ptr<WifiNetDevice> netDev = DynamicCast<WifiNetDevice> (node->GetDevice(1));
        
        if (netDev == nullptr) {
            NS_LOG_ERROR("Node " << nodeId << " has no WifiNetDevice");
            return;
        }
        //Workaround: set a channel number, which no other phy uses. Channel will this way not let the phy
        //receive. Unfortunately, phys cannot be removed from channel, once added.
        netDev->GetPhy()->SetChannelNumber(0x0);
        netDev->GetPhy()->SetSleepMode();
        
        m_isDeactivated[nodeId] = true;
    }

    /**
     * @brief Evaluates configuration message and applies it to the node
     */
    void MosaicNodeManager::ConfigureNodeRadio(uint32_t nodeId, bool radioTurnedOn, int transmitPower) {
        if (m_isDeactivated[nodeId]) {
            return;
        }
        
        Ptr<Node> node = NodeList::GetNode(nodeId);

        Ptr<Application> app = node->GetApplication(0);
        Ptr<MosaicProxyApp> ssa = app->GetObject<MosaicProxyApp>();
        if (!ssa) {
            NS_LOG_ERROR("No app found on node " << nodeId << " !");
            return;
        }
        if (radioTurnedOn) {
            ssa->Enable();
            if (transmitPower > -1) {
                Ptr<WifiNetDevice> netDev = DynamicCast<WifiNetDevice> (node->GetDevice(1));
                if (netDev == nullptr) {
                    NS_LOG_ERROR("Inconsistency: no matching NetDevice found on node while configuring");
                    return;
                }                        
                Ptr<YansWifiPhy> wavePhy = DynamicCast<YansWifiPhy> (netDev->GetPhy());
                if (wavePhy != 0) {
                    double txDBm = 10 * log10((double) transmitPower);
                    wavePhy->SetTxPowerStart(txDBm);
                    wavePhy->SetTxPowerEnd(txDBm);
                }
            }
        } else {
            ssa->Disable();
        }
    }
}
