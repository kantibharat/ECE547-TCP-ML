/*
 * Copyright (c) 2013 ResiliNets, ITTC, University of Kansas
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
 * Authors: Justin P. Rohrer, Truc Anh N. Nguyen <annguyen@ittc.ku.edu>, Siddharth Gangadhar
 * <siddharth@ittc.ku.edu>
 *
 * James P.G. Sterbenz <jpgs@ittc.ku.edu>, director
 * ResiliNets Research Group  https://resilinets.org/
 * Information and Telecommunication Technology Center (ITTC)
 * and Department of Electrical Engineering and Computer Science
 * The University of Kansas Lawrence, KS USA.
 *
 * Work supported in part by NSF FIND (Future Internet Design) Program
 * under grant CNS-0626918 (Postmodern Internet Architecture),
 * NSF grant CNS-1050226 (Multilayer Network Resilience Analysis and Experimentation on GENI),
 * US Department of Defense (DoD), and ITTC at The University of Kansas.
 *
 * "TCP Westwood(+) Protocol Implementation in ns-3"
 * Siddharth Gangadhar, Trúc Anh Ngọc Nguyễn , Greeshma Umapathi, and James P.G. Sterbenz,
 * ICST SIMUTools Workshop on ns-3 (WNS3), Cannes, France, March 2013
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/enum.h"
#include "ns3/error-model.h"
#include "ns3/event-id.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/tcp-header.h"
#include "ns3/traffic-control-module.h"
#include "ns3/udp-header.h"

#include <fstream>
#include <iostream>
#include <string>
#include <queue>

using namespace ns3;
std::string dir = "data/";

NS_LOG_COMPONENT_DEFINE("TcpVariantsComparison");

#define TS_NUM_ELEMENTS 8
float EWMAalpha = 0.3;
Time lastSendingTime = Simulator::Now();
Time lastArrivalTime = Simulator::Now();
double intersendEWMA = 0;
double interarrivalEWMA = 0;
double interarrivalTimeTS = 0;
Time lastInterarrivalTime;
double lastUtility = 0;
SequenceNumber32 lastSeqNum;
uint32_t packetsSent = 0;
uint32_t packetsLost = 0;
float B = 10;
SequenceNumber32 nextRxSeqNum;

static std::map<uint32_t, bool> firstAdvWnd;
static std::map<uint32_t, bool> firstCwnd;                      //!< First congestion window.
static std::map<uint32_t, bool> firstSshThr;                    //!< First SlowStart threshold.
static std::map<uint32_t, bool> firstRtt;                       //!< First RTT.
static std::map<uint32_t, Ptr<OutputStreamWrapper>> cWndStream; //!< Congstion window output stream.
static std::map<uint32_t, Ptr<OutputStreamWrapper>> advWndStream;
static std::map<uint32_t, Ptr<OutputStreamWrapper>>
    ssThreshStream; //!< SlowStart threshold output stream.
static std::map<uint32_t, Ptr<OutputStreamWrapper>> rttStream;      //!< RTT output stream.
static std::map<uint32_t, Ptr<OutputStreamWrapper>> nextTxStream;   //!< Next TX output stream.
static std::map<uint32_t, Ptr<OutputStreamWrapper>> nextRxStream;   //!< Next RX output stream.
static std::map<uint32_t, uint32_t> cWndValue;                      //!< congestion window value.
static std::map<uint32_t, std::queue<Time>> rttValues;
static std::map<uint32_t, std::queue<Time>> intersendingTimes;
static std::map<uint32_t, std::queue<Time>> interarrivalTimes;
static std::map<uint32_t, uint32_t> ssThreshValue;                  //!< SlowStart threshold value.
// static std::map<uint32_t, uint32_t> advWndSize;                     //!< congestion window size

/**
 * Get the Node Id From Context.
 *
 * \param context The context.
 * \return the node ID.
 */
static uint32_t
GetNodeIdFromContext(std::string context)
{
    const std::size_t n1 = context.find_first_of('/', 1);
    const std::size_t n2 = context.find_first_of('/', n1 + 1);
    return std::stoul(context.substr(n1 + 1, n2 - n1 - 1));
}

/**
 * Congestion window tracer.
 *
 * \param context The context.
 * \param oldval Old value.
 * \param newval New value.
 */
static void
CwndTracer(std::string context, uint32_t oldval, uint32_t newval)
{
    uint32_t nodeId = GetNodeIdFromContext(context);
    cWndValue[nodeId] = newval;
}

// static void
// AdvWndTracer(std::string context, uint32_t oldval, uint32_t newval)
// {
//     uint32_t nodeId = GetNodeIdFromContext(context);

//     if (firstAdvWnd[nodeId])
//     {
//         *advWndStream[nodeId]->GetStream() << "0.0 " << oldval << std::endl;
//         firstAdvWnd[nodeId] = false;
//     }
//     *advWndStream[nodeId]->GetStream() << Simulator::Now().GetSeconds() << " " << newval << std::endl;
//     advWndSize[nodeId] = newval;

//     // NS_LOG_INFO("Initialize Global Routing.");
// }

/**
 * Slow start threshold tracer.
 *
 * \param context The context.
 * \param oldval Old value.
 * \param newval New value.
 */
static void
SsThreshTracer(std::string context, uint32_t oldval, uint32_t newval)
{
    uint32_t nodeId = GetNodeIdFromContext(context);
    ssThreshValue[nodeId] = newval;
}

/**
 * RTT tracer.
 *
 * \param context The context.
 * \param oldval Old value.
 * \param newval New value.
 */
static void
RttTracer(std::string context, Time oldval, Time newval)
{
    uint32_t nodeId = GetNodeIdFromContext(context);
    // rttValue[nodeId] = newval.GetSeconds();

    rttValues[nodeId].push(newval);
    if (rttValues[nodeId].size() > TS_NUM_ELEMENTS) {
        rttValues[nodeId].pop();
    }

    if (firstRtt[nodeId]) {
        firstRtt[nodeId] = false;
    }
}

static double calcTS(std::queue<Time> values) {
    uint8_t size = values.size();
    double sum = 0;
    while (!values.empty()) {
        sum += values.front().GetSeconds();
        values.pop();
    }
    double TS = sum / size;

    return TS;
}

static double updateEWMA(double EWMA, std::queue<Time> values) {
    EWMA = values.back().GetSeconds() * EWMAalpha + EWMA * (1-EWMAalpha);

    return EWMA;
}

/**
 * Next TX tracer.
 *
 * \param context The context.
 * \param old Old sequence number.
 * \param nextTx Next sequence number.
 */
static void
NextTxTracer(std::string context, SequenceNumber32 old [[maybe_unused]], SequenceNumber32 nextTx)
{
    uint32_t nodeId = GetNodeIdFromContext(context);
    SequenceNumber32 currentSeqNum = nextTx;

    // std::ostream *stream = nextTxStream[nodeId]->GetStream();
    // *stream << std::fixed << std::setprecision(10);

    if (firstCwnd[nodeId])
    {
        *nextTxStream[nodeId]->GetStream() << "time " << "nextTx " << "intersendTime " << "intersendTS " 
        << "intersendEWMA " << "cWnd " << "ssThreshold " << "rtt " << "rttTS " 
        << "nextRx " << "interarrivalTime " << "interarrivalTS " << "interarrivalEWMA "
        << "numSent " << "numLost " << "reward" << std::endl;
        firstCwnd[nodeId] = false;
        lastSendingTime = Simulator::Now();
        lastArrivalTime = Simulator::Now();
        lastSeqNum = nextTx;
    }

    Time currentSendingTime = Simulator::Now();

    intersendingTimes[nodeId].push(currentSendingTime - lastSendingTime);
    if (intersendingTimes[nodeId].size() > TS_NUM_ELEMENTS-1) {
        intersendingTimes[nodeId].pop();
    }
    
    double intersendingTimeTS = calcTS(intersendingTimes[nodeId]);
    intersendEWMA = updateEWMA(intersendEWMA, intersendingTimes[nodeId]);

    if (firstRtt[nodeId]) {
        firstRtt[nodeId] = false;
    }

    double rttTS = calcTS(rttValues[nodeId]);

    if (currentSeqNum < lastSeqNum) {
        packetsLost++;
    }
    packetsSent++;

    float delta1 = 1;
    float delta2 = 1;
    float minRtt = 0.15;
    double d = rttValues[nodeId].back().GetSeconds() - minRtt;
    double p = packetsLost/packetsSent;
    double throughput = (1-p) / rttTS; // probability of packet send success / time for one packet cycle
    double currentUtility = log10(throughput/B) - delta1*log10(d) - delta2*log10(1-p);
    double r = (currentUtility - lastUtility) / (currentSendingTime.GetSeconds() - lastSendingTime.GetSeconds() + 0.00000001); // avoid nan

    // NS_LOG_INFO("EWMA[nodeId].back(): " << intersendingTimes[nodeId].back().GetSeconds() << ". EWMA: " << intersendEWMA);

    *nextTxStream[nodeId]->GetStream()
        << currentSendingTime.GetSeconds() << " " << nextTx << " " << intersendingTimes[nodeId].back().GetSeconds() << " " 
        << intersendingTimeTS << " " << intersendEWMA << " " << cWndValue[nodeId] << " " << ssThreshValue[nodeId]
        << " " << rttValues[nodeId].back().GetSeconds() << " " << rttTS << " " << nextRxSeqNum << " " << lastInterarrivalTime.GetSeconds()
        << " " << interarrivalTimeTS << " " << interarrivalEWMA << " " << packetsSent << " " << packetsLost << " " << r << " " << std::endl;

    lastUtility = currentUtility;
    lastSendingTime = currentSendingTime;
    lastSeqNum = nextTx;
}

/**
 * Next RX tracer.
 *
 * \param context The context.
 * \param old Old sequence number.
 * \param nextRx Next sequence number.
 */
static void
NextRxTracer(std::string context, SequenceNumber32 old [[maybe_unused]], SequenceNumber32 nextRx)
{
    uint32_t nodeId = GetNodeIdFromContext(context);
    nextRxSeqNum = nextRx;

    if (firstCwnd[nodeId]) {
        *nextTxStream[nodeId]->GetStream() << "time (s) " << "nextRx " << "interarrivalTime " << "interarrivalTS "
        << "interarrivalEWMA" << std::endl;
    }

    Time currentArrivalTime = Simulator::Now();

    lastInterarrivalTime = currentArrivalTime - lastArrivalTime;
    interarrivalTimes[nodeId].push(lastInterarrivalTime);
    if (interarrivalTimes[nodeId].size() > TS_NUM_ELEMENTS) {
        interarrivalTimes[nodeId].pop();
    }
    interarrivalTimeTS = calcTS(interarrivalTimes[nodeId]);
    interarrivalEWMA = updateEWMA(interarrivalEWMA, interarrivalTimes[nodeId]);

    *nextRxStream[nodeId]->GetStream()
        << currentArrivalTime.GetSeconds() << " " << nextRx << " " << interarrivalTimes[nodeId].back().GetSeconds() << " " 
        << interarrivalTimeTS << " " << interarrivalEWMA << " " << std::endl;

    lastArrivalTime = currentArrivalTime;
}

/**
 * Congestion window trace connection.
 *
 * \param cwnd_tr_file_name Congestion window trace file name.
 * \param nodeId Node ID.
 */
static void
TraceCwnd(std::string cwnd_tr_file_name, uint32_t nodeId)
{
    AsciiTraceHelper ascii;
    cWndStream[nodeId] = ascii.CreateFileStream(cwnd_tr_file_name);
    Config::Connect("/NodeList/" + std::to_string(nodeId) +
                        "/$ns3::TcpL4Protocol/SocketList/0/CongestionWindow",
                    MakeCallback(&CwndTracer));
}

// static void
// TraceAdvWnd(std::string advWnd_tr_file_name, uint32_t nodeId)
// {
//     AsciiTraceHelper ascii;
//     advWndStream[nodeId] = ascii.CreateFileStream(advWnd_tr_file_name);
//     Config::Connect("/NodeList/" + std::to_string(nodeId) +
//                         "/$ns3::TcpL4Protocol/SocketList/0/AdvWND",
//                     MakeCallback(&AdvWndTracer));
// }

/**
 * Slow start threshold trace connection.
 *
 * \param ssthresh_tr_file_name Slow start threshold trace file name.
 * \param nodeId Node ID.
 */
static void
TraceSsThresh(std::string ssthresh_tr_file_name, uint32_t nodeId)
{
    AsciiTraceHelper ascii;
    ssThreshStream[nodeId] = ascii.CreateFileStream(ssthresh_tr_file_name);
    Config::Connect("/NodeList/" + std::to_string(nodeId) +
                        "/$ns3::TcpL4Protocol/SocketList/0/SlowStartThreshold",
                    MakeCallback(&SsThreshTracer));
}

/**
 * RTT trace connection.
 *
 * \param rtt_tr_file_name RTT trace file name.
 * \param nodeId Node ID.
 */
static void
TraceRtt(std::string rtt_tr_file_name, uint32_t nodeId)
{
    AsciiTraceHelper ascii;
    rttStream[nodeId] = ascii.CreateFileStream(rtt_tr_file_name);
    Config::Connect("/NodeList/" + std::to_string(nodeId) + "/$ns3::TcpL4Protocol/SocketList/0/RTT",
                    MakeCallback(&RttTracer));
}

/**
 * Next TX trace connection.
 *
 * \param next_tx_seq_file_name Next TX trace file name.
 * \param nodeId Node ID.
 */
static void
TraceNextTx(std::string& next_tx_seq_file_name, uint32_t nodeId)
{
    AsciiTraceHelper ascii;
    nextTxStream[nodeId] = ascii.CreateFileStream(next_tx_seq_file_name);
    Config::Connect("/NodeList/" + std::to_string(nodeId) +
                        "/$ns3::TcpL4Protocol/SocketList/0/NextTxSequence",
                    MakeCallback(&NextTxTracer));
}

/**
 * Next RX trace connection.
 *
 * \param next_rx_seq_file_name Next RX trace file name.
 * \param nodeId Node ID.
 */
static void
TraceNextRx(std::string& next_rx_seq_file_name, uint32_t nodeId)
{
    AsciiTraceHelper ascii;
    nextRxStream[nodeId] = ascii.CreateFileStream(next_rx_seq_file_name);
    Config::Connect("/NodeList/" + std::to_string(nodeId) +
                        "/$ns3::TcpL4Protocol/SocketList/1/RxBuffer/NextRxSequence",
                    MakeCallback(&NextRxTracer));
}

int
main(int argc, char* argv[])
{
    std::string transport_prot = "TcpNewReno";
    double error_p = 0;
    std::string bandwidth = "10Mbps";
    // also = B defined as global variable - changing this mid-test will result in incorrect reward r calc.
    std::string delay = "0.1ms";
    std::string access_bandwidth = "20Mbps";
    std::string access_delay = "74.9ms";
    bool tracing = true;
    std::string prefix_file_name = "model";
    uint64_t data_mbytes = 0;
    uint32_t mtu_bytes = 1250;
    uint16_t buffer_size = 50;
    uint16_t num_flows = 1;
    double duration = 10.0;
    uint32_t run = 0;
    bool flow_monitor = false;
    bool pcap = false;
    bool sack = true;
    std::string queue_disc_type = "ns3::PfifoFastQueueDisc";
    std::string recovery = "ns3::TcpClassicRecovery";

    CommandLine cmd(__FILE__);
    cmd.AddValue("transport_prot",
                 "Transport protocol to use: TcpNewReno, TcpLinuxReno, "
                 "TcpHybla, TcpHighSpeed, TcpHtcp, TcpVegas, TcpScalable, TcpVeno, "
                 "TcpBic, TcpYeah, TcpIllinois, TcpWestwoodPlus, TcpLedbat, "
                 "TcpLp, TcpDctcp, TcpCubic, TcpBbr",
                 transport_prot);
    cmd.AddValue("error_p", "Packet error rate", error_p);
    cmd.AddValue("bandwidth", "Bottleneck bandwidth", bandwidth);
    cmd.AddValue("delay", "Bottleneck delay", delay);
    cmd.AddValue("access_bandwidth", "Access link bandwidth", access_bandwidth);
    cmd.AddValue("access_delay", "Access link delay", access_delay);
    cmd.AddValue("tracing", "Flag to enable/disable tracing", tracing);
    cmd.AddValue("prefix_name", "Prefix of output trace file", prefix_file_name);
    cmd.AddValue("data", "Number of Megabytes of data to transmit", data_mbytes);
    cmd.AddValue("mtu", "Size of IP packets to send in bytes", mtu_bytes);
    cmd.AddValue("buffer_size", "Buffer size at router 1", buffer_size);
    cmd.AddValue("num_flows", "Number of flows", num_flows);
    cmd.AddValue("duration", "Time to allow flows to run in seconds", duration);
    cmd.AddValue("run", "Run index (for setting repeatable seeds)", run);
    cmd.AddValue("flow_monitor", "Enable flow monitor", flow_monitor);
    cmd.AddValue("pcap_tracing", "Enable or disable PCAP tracing", pcap);
    cmd.AddValue("queue_disc_type",
                 "Queue disc type for gateway (e.g. ns3::CoDelQueueDisc)",
                 queue_disc_type);
    cmd.AddValue("sack", "Enable or disable SACK option", sack);
    cmd.AddValue("recovery", "Recovery algorithm type to use (e.g., ns3::TcpPrrRecovery", recovery);
    cmd.Parse(argc, argv);

    transport_prot = std::string("ns3::") + transport_prot;

    SeedManager::SetSeed(1);
    SeedManager::SetRun(run);

    // User may find it convenient to enable logging
    LogComponentEnable("TcpVariantsComparison", LOG_LEVEL_ALL);
    // LogComponentEnable("OnOffApplication", LOG_LEVEL_INFO);
    // LogComponentEnable("PfifoFastQueueDisc", LOG_LEVEL_ALL);

    // Set the simulation start and stop time
    double start_time = 0.1;
    double stop_time = start_time + duration;

    // 2 MB of TCP buffer
    Config::SetDefault("ns3::TcpSocket::RcvBufSize", UintegerValue(1 << 21));
    Config::SetDefault("ns3::TcpSocket::SndBufSize", UintegerValue(1 << 21));
    Config::SetDefault("ns3::TcpSocketBase::Sack", BooleanValue(sack));

    Config::SetDefault("ns3::TcpL4Protocol::RecoveryType",
                       TypeIdValue(TypeId::LookupByName(recovery)));
    // Select TCP variant
    TypeId tcpTid;
    NS_ABORT_MSG_UNLESS(TypeId::LookupByNameFailSafe(transport_prot, &tcpTid),
                        "TypeId " << transport_prot << " not found");
    Config::SetDefault("ns3::TcpL4Protocol::SocketType",
                       TypeIdValue(TypeId::LookupByName(transport_prot)));

    // Create gateways, sources, and sinks
    NodeContainer gateways;
    gateways.Create(1);
    NodeContainer sources;
    sources.Create(num_flows);
    NodeContainer sinks;
    sinks.Create(num_flows);

    // Configure the error model
    // Here we use RateErrorModel with packet error rate
    Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
    uv->SetStream(50);
    RateErrorModel error_model;
    error_model.SetRandomVariable(uv);
    error_model.SetUnit(RateErrorModel::ERROR_UNIT_PACKET);
    error_model.SetRate(error_p);

    PointToPointHelper UnReLink;
    UnReLink.SetDeviceAttribute("DataRate", StringValue(bandwidth));
    UnReLink.SetChannelAttribute("Delay", StringValue(delay));
    UnReLink.SetDeviceAttribute("ReceiveErrorModel", PointerValue(&error_model));

    InternetStackHelper stack;
    stack.InstallAll();

    TrafficControlHelper tchPfifo;
    tchPfifo.SetRootQueueDisc("ns3::PfifoFastQueueDisc");

    Ipv4AddressHelper address;
    address.SetBase("10.0.0.0", "255.255.255.0");

    // Configure the sources and sinks net devices
    // and the channels between the sources/sinks and the gateways
    PointToPointHelper LocalLink;
    LocalLink.SetDeviceAttribute("DataRate", StringValue(access_bandwidth));
    LocalLink.SetChannelAttribute("Delay", StringValue(access_delay));

    Ipv4InterfaceContainer sink_interfaces;


    Config::SetDefault("ns3::PfifoFastQueueDisc::MaxSize",
                       QueueSizeValue(QueueSize(QueueSizeUnit::PACKETS, buffer_size)));

    for (uint32_t i = 0; i < num_flows; i++)
    {
        NetDeviceContainer devices;
        devices = LocalLink.Install(sources.Get(i), gateways.Get(0));
        tchPfifo.Install(devices);
        address.NewNetwork();
        Ipv4InterfaceContainer interfaces = address.Assign(devices);

        devices = UnReLink.Install(gateways.Get(0), sinks.Get(i));
        tchPfifo.Install(devices);

        address.NewNetwork();
        interfaces = address.Assign(devices);
        sink_interfaces.Add(interfaces.Get(1));
    }

    NS_LOG_INFO("Initialize Global Routing.");
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    uint16_t port = 50000;
    Address sinkLocalAddress(InetSocketAddress(Ipv4Address::GetAny(), port));
    PacketSinkHelper sinkHelper("ns3::TcpSocketFactory", sinkLocalAddress);

    for (uint32_t i = 0; i < sources.GetN(); i++)
    {
        AddressValue remoteAddress(InetSocketAddress(sink_interfaces.GetAddress(i, 0), port));
        Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(mtu_bytes));
        OnOffHelper ftp("ns3::TcpSocketFactory", Address());
        // BulkSendHelper ftp("ns3::TcpSocketFactory", Address());
        ftp.SetAttribute("Remote", remoteAddress);
        // ftp.SetAttribute("SendSize", UintegerValue(mtu_bytes));
        ftp.SetAttribute("PacketSize", UintegerValue(mtu_bytes));
        ftp.SetAttribute("MaxBytes", UintegerValue(data_mbytes * 1000000));
        ftp.SetAttribute("DataRate", DataRateValue(access_bandwidth));
        ftp.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));

        if (sources.GetN() > 1) {
            Ptr<ExponentialRandomVariable> off_time = CreateObject<ExponentialRandomVariable> ();
            off_time->SetAttribute("Mean", DoubleValue(0.5));  // mean of 0.5 s

            Ptr<ExponentialRandomVariable> on_time = CreateObject<ExponentialRandomVariable> ();
            on_time->SetAttribute("Mean", DoubleValue(1));  // mean of 10 MB = data rate MBps * (10 MB/data rate MBps) s -> 1 s

            ftp.SetAttribute("OffTime", PointerValue(off_time));
            ftp.SetAttribute("OnTime", PointerValue(on_time));
        }
        else {
            Ptr<ConstantRandomVariable> time = CreateObject<ConstantRandomVariable> ();
            time->SetAttribute("Constant", DoubleValue(0));
            ftp.SetAttribute("OffTime", PointerValue(time));
            time->SetAttribute("Constant", DoubleValue(1));
            ftp.SetAttribute("OnTime", PointerValue(time));
        }

        ApplicationContainer sourceApp = ftp.Install(sources.Get(i));
        sourceApp.Start(Seconds(start_time * i));
        sourceApp.Stop(Seconds(stop_time - 3));

        sinkHelper.SetAttribute("Protocol", TypeIdValue(TcpSocketFactory::GetTypeId()));
        ApplicationContainer sinkApp = sinkHelper.Install(sinks.Get(i));
        sinkApp.Start(Seconds(start_time * i));
        sinkApp.Stop(Seconds(stop_time));
    }

    // Set up tracing if enabled
    if (tracing)
    {
        std::string dirToRemove = "rm -rf " + dir;
        system(dirToRemove.c_str());
        std::string dirToSave = "mkdir -p " + dir;
        system(dirToSave.c_str());

        std::ofstream ascii;
        Ptr<OutputStreamWrapper> ascii_wrap;
        ascii.open(dir + prefix_file_name + "-ascii");
        ascii_wrap = new OutputStreamWrapper(dir + prefix_file_name + "-ascii", std::ios::out);
        stack.EnableAsciiIpv4All(ascii_wrap);

        for (uint16_t index = 0; index < num_flows; index++)
        {
            std::string flowString;
            if (num_flows > 1)
            {
                flowString = "-flow" + std::to_string(index);
                flowString = "";
            }

            firstCwnd[index + 1] = true;
            firstSshThr[index + 1] = true;
            firstRtt[index + 1] = true;

            // Simulator::Schedule(Seconds(start_time * index + 0.00001),
            //                     &TraceAdvWnd,
            //                     dir + prefix_file_name + flowString + "-advwnd.data",
            //                     index + 1);
            Simulator::Schedule(Seconds(start_time * index + 0.00001),
                                &TraceCwnd,
                                dir + prefix_file_name + flowString + "-cwnd.data",
                                index + 1);
            Simulator::Schedule(Seconds(start_time * index + 0.00001),
                                &TraceSsThresh,
                                dir + prefix_file_name + flowString + "-ssth.data",
                                index + 1);
            Simulator::Schedule(Seconds(start_time * index + 0.00001),
                                &TraceRtt,
                                dir + prefix_file_name + flowString + "-rtt.data",
                                index + 1);
            Simulator::Schedule(Seconds(start_time * index + 0.1),
                                &TraceNextRx,
                                dir + prefix_file_name + flowString + "-next-rx.data",
                                num_flows + index + 1);
            Simulator::Schedule(Seconds(start_time * index + 0.1),
                                &TraceNextTx,
                                dir + prefix_file_name + flowString + ".data",
                                index + 1);
        }
    }

    if (pcap)
    {
        UnReLink.EnablePcapAll(prefix_file_name, true);
        LocalLink.EnablePcapAll(prefix_file_name, true);
    }

    // Flow monitor
    FlowMonitorHelper flowHelper;
    if (flow_monitor)
    {
        flowHelper.InstallAll();
    }

    Simulator::Stop(Seconds(stop_time));
    Simulator::Run();

    if (flow_monitor)
    {
        flowHelper.SerializeToXmlFile(prefix_file_name + ".flowmonitor", true, true);
    }

    Simulator::Destroy();
    return 0;
}