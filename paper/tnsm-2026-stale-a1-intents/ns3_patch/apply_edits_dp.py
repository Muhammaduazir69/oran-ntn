#!/usr/bin/env python3
"""Second patch: data plane with a best-effort slice, demand-linked intents, oracle control.
Reads oran-ntn-feeder-mgmt.cc (round 1 and 2 patch), writes oran-ntn-feeder-mgmt_dp.cc."""
import os
H = os.path.dirname(os.path.abspath(__file__))
s = open(os.path.join(H, "oran-ntn-feeder-mgmt.cc")).read()


def rep(a, b):
    global s
    assert s.count(a) == 1, a
    s = s.replace(a, b)


rep("""    std::string gwList{};""", """    bool oracleIntent{false};  // control arm: on-board decisions always see the current intent
    bool surge{false};         // intents linked to demand through a two-state regime
    double surgeLeadS{2.0};    // the rApp knows the regime this long ahead
    double surgeDwellS{-1.0};  // regime dwell apart from the update spacing; <0 ties it to a1MeanS
    double surgeDuty{0.5};     // share of time in a surge
    bool surgeUnlinked{false}; // surges on, intents redrawn independently of them
    double initG{0.3};         // first intent (version 1), which every satellite holds from launch
    double initH{0.2};
    uint32_t surgeSessions{10};
    double surgeHighG{0.8};
    double surgeLowG{0.2};
    double surgeH{0.1};
    bool dataPlane{false};
    bool workConserving{false}; // BE may use critical budget left idle in a TTI
    double cellMbps{10.0};
    uint32_t pktBytes{1250};
    double critDeadlineS{0.1};
    std::string gwList{};""")

# oracle intent at the on-board decision
rep("""        if (cls == kNS)
        {
            const Policy p = FromA1(*sat.a1);""", """        if (cls == kNS)
        {
            const Policy p = g.oracleIntent ? smoCur : FromA1(*sat.a1);""")

# surge: regime process and the intents it implies
rep("""void
SmoIssue()
{
    Simulator::Schedule(Seconds(NextA1Gap()), &SmoIssue);
""", """bool regimeHigh = false;
uint32_t surgeActive = 0;

void SmoPublish(double gv, double hv);

void
RegimeSwitch()
{
    regimeHigh = !regimeHigh;
    surgeActive = regimeHigh ? g.surgeSessions : 0;
    const bool split = g.surgeDwellS > 0 || g.surgeUnlinked;
    double raw = rvA1->GetValue();
    if (g.surgeDwellS > 0 || g.surgeDuty != 0.5)
    {
        const double base = g.surgeDwellS > 0 ? g.surgeDwellS : g.a1MeanS;
        raw = raw / g.a1MeanS * base * 2.0 * (regimeHigh ? g.surgeDuty : 1.0 - g.surgeDuty);
    }
    const double dwell = std::max(g.surgeLeadS + 0.1, raw);
    regimeNextS = Simulator::Now().GetSeconds() + dwell;
    if (!split)
    {
        // the rApp issues the next regime's intent ahead of it
        Simulator::Schedule(Seconds(dwell - g.surgeLeadS), []() {
            SmoPublish(regimeHigh ? g.surgeLowG : g.surgeHighG, g.surgeH);
        });
    }
    Simulator::Schedule(Seconds(dwell), &RegimeSwitch);
}

// Update spacing apart from the dwell: the rApp publishes at Poisson times the regime it
// knows surgeLeadS ahead, or a draw independent of the regime for the unlinked control.
void
SurgePublishLoop()
{
    Simulator::Schedule(Seconds(rvUpd->GetValue()), &SurgePublishLoop);
    if (g.surgeUnlinked)
    {
        SmoPublish(rvIntent->GetValue() < 0.5 ? g.surgeHighG : g.surgeLowG, g.surgeH);
        return;
    }
    const bool flipped = Simulator::Now().GetSeconds() + g.surgeLeadS >= regimeNextS;
    const bool high = flipped ? !regimeHigh : regimeHigh;
    SmoPublish(high ? g.surgeHighG : g.surgeLowG, g.surgeH);
}

void
SmoIssue()
{
    if (g.surge)
    {
        return;
    }
    Simulator::Schedule(Seconds(NextA1Gap()), &SmoIssue);
""")
rep("""// SMO -> ground RIC over the cloud link (10 ms), modeled through the A1
// manager's delivery delay; the RIC then mirrors to satellites per strategy.""", """void
SmoPublish(double gv, double hv)
{
    Policy p;
    p.ver = smoCur.ver + 1;
    p.g = gv;
    p.h = hv;
    smoCur = p;
    verIssued[p.ver] = Simulator::Now().GetSeconds();
    smoMgr->UpdatePolicy(smoPolicyId, ToA1(p));
    if (IsOnboardLike() && servingIdx >= 0 && gview[servingIdx].e2)
    {
        MgmtHeader a;
        a.type = A1_PUT;
        a.ver = p.ver;
        a.v1 = p.g;
        a.v2 = p.h;
        a.untilNs = OnbLeaseUntilNs();
        SendToSat(servingIdx, a, PayloadSize(A1_PUT, 0), true);
    }
}

// SMO -> ground RIC over the cloud link (10 ms), modeled through the A1
// manager's delivery delay; the RIC then mirrors to satellites per strategy.""")
rep("""    return std::min(0.9, g.sessionShare * sessions);
""", """    return std::min(0.9, g.sessionShare * (sessions + surgeActive));
""")
rep("""uint32_t sessions = 0;
""", """uint32_t sessions = 0;
extern uint32_t surgeActive;
""")
rep("""bool regimeHigh = false;
uint32_t surgeActive = 0;
""", """bool regimeHigh = false;
double regimeNextS = 1e18;
Ptr<ExponentialRandomVariable> rvUpd;
""")
rep("""int servingIdx = -1; // sats[] index currently serving
""", """int servingIdx = -1; // sats[] index currently serving
uint32_t surgeActive = 0;
""")
rep("""uint32_t sessions = 0;
extern uint32_t surgeActive;
""", """uint32_t sessions = 0;
""")

# data plane
rep("""// Demand: M/M/inf sessions on the critical slice.
void
Arrival()""", """// ---------------------------------------------------------------------------
// Data plane (round 3): one cell of capacity C shared by the critical slice and a
// saturated best-effort slice. Real packets, a 1 ms scheduler with a hard partition
// x C / (1 - x) C, sinks over point-to-point devices.
// ---------------------------------------------------------------------------
struct DpStat
{
    uint64_t critOffered{0}, critDelivered{0}, critDropped{0}, beDelivered{0};
    double servedS{0};
    std::array<uint64_t, 1001> latHist{};
    double surgeOffered{0}, surgeDelivered{0};
} dp;
struct CritPkt
{
    Ptr<Packet> p;
    Time born;
    bool surge;
};
std::deque<CritPkt> critQ;
std::deque<std::pair<Time, bool>> critInFlight;
Ptr<NetDevice> dpCritTx, dpBeTx;
double dpBudgetCrit = 0, dpBudgetBe = 0;
Ptr<ExponentialRandomVariable> rvPkt;

bool
DpCritRx(Ptr<NetDevice>, Ptr<const Packet> p, uint16_t, const Address&)
{
    const auto e = critInFlight.front();
    critInFlight.pop_front();
    dp.critDelivered += p->GetSize();
    if (e.second)
    {
        dp.surgeDelivered += p->GetSize();
    }
    const double ms = (Simulator::Now() - e.first).GetMilliSeconds();
    ++dp.latHist[std::min<size_t>(1000, static_cast<size_t>(ms))];
    return true;
}

bool
DpBeRx(Ptr<NetDevice>, Ptr<const Packet> p, uint16_t, const Address&)
{
    dp.beDelivered += p->GetSize();
    return true;
}

// Offered critical traffic: Poisson packets at the rate of the active sessions. The
// generator draws at the peak rate and thins, so no event depends on the strategy.
void
DpArrival()
{
    const double peak = 1.0 * g.cellMbps * 1e6 / (8.0 * g.pktBytes);
    Simulator::Schedule(Seconds(rvPkt->GetValue() / peak), &DpArrival);
    const double rate = g.sessionShare * (sessions + surgeActive);
    if (servingIdx < 0 || rvPkt->GetValue() >= rate)
    {
        return;
    }
    dp.critOffered += g.pktBytes;
    const bool sg = surgeActive > 0;
    if (sg)
    {
        dp.surgeOffered += g.pktBytes;
    }
    critQ.push_back({Create<Packet>(g.pktBytes), Simulator::Now(), sg});
}

void
DpTti()
{
    Simulator::Schedule(MilliSeconds(1), &DpTti);
    if (servingIdx < 0)
    {
        return;
    }
    dp.servedS += 0.001;
    const Sat& sat = sats[servingIdx];
    const double x = sat.share < 0 ? g.defaultShare : sat.share;
    const double perTti = g.cellMbps * 1e6 / 8.0 * 0.001;
    const double L = g.pktBytes;
    dpBudgetCrit = std::min(dpBudgetCrit, L) + x * perTti;
    dpBudgetBe = std::min(dpBudgetBe, L) + (1.0 - x) * perTti;
    const Time now = Simulator::Now();
    while (!critQ.empty() && now - critQ.front().born > Seconds(g.critDeadlineS))
    {
        dp.critDropped += g.pktBytes;
        critQ.pop_front();
    }
    while (!critQ.empty() && dpBudgetCrit >= L)
    {
        dpBudgetCrit -= L;
        critInFlight.push_back({critQ.front().born, critQ.front().surge});
        dpCritTx->Send(critQ.front().p, dpCritTx->GetBroadcast(), 0x0800);
        critQ.pop_front();
    }
    if (g.workConserving && critQ.empty() && dpBudgetCrit >= L)
    {
        dpBudgetBe += dpBudgetCrit;
        dpBudgetCrit = 0;
    }
    while (dpBudgetBe >= L)
    {
        dpBudgetBe -= L;
        dpBeTx->Send(Create<Packet>(g.pktBytes), dpBeTx->GetBroadcast(), 0x0800);
    }
}

void
DpSetup()
{
    PointToPointHelper h;
    h.SetDeviceAttribute("DataRate", StringValue("1Gbps"));
    h.SetChannelAttribute("Delay", StringValue("1ms"));
    h.SetQueue("ns3::DropTailQueue<Packet>", "MaxSize", StringValue("100000p"));
    NodeContainer n;
    n.Create(4);
    NetDeviceContainer c = h.Install(n.Get(0), n.Get(1));
    NetDeviceContainer b = h.Install(n.Get(2), n.Get(3));
    dpCritTx = c.Get(0);
    dpBeTx = b.Get(0);
    c.Get(1)->SetReceiveCallback(MakeCallback(&DpCritRx));
    b.Get(1)->SetReceiveCallback(MakeCallback(&DpBeRx));
    rvPkt = CreateObject<ExponentialRandomVariable>();
    rvPkt->SetAttribute("Mean", DoubleValue(1.0));
    rvPkt->SetStream(30);
    Simulator::Schedule(MilliSeconds(1), &DpTti);
    Simulator::Schedule(MilliSeconds(1), &DpArrival);
}

// Demand: M/M/inf sessions on the critical slice.
void
Arrival()""")
# thinning: use a uniform on its own stream
rep("""    if (servingIdx < 0 || rvPkt->GetValue() >= rate)
""", """    if (servingIdx < 0 || rvPktU->GetValue() >= rate)
""")
rep("""Ptr<ExponentialRandomVariable> rvPkt;
""", """Ptr<ExponentialRandomVariable> rvPkt;
Ptr<UniformRandomVariable> rvPktU;
""")
rep("""    rvPkt->SetStream(30);
""", """    rvPkt->SetStream(30);
    rvPktU = CreateObject<UniformRandomVariable>();
    rvPktU->SetStream(31);
""")
# start
rep("""    Simulator::Schedule(MilliSeconds(50), &SlaTick);
""", """    Simulator::Schedule(MilliSeconds(50), &SlaTick);
    if (g.dataPlane)
    {
        DpSetup();
    }
    if (g.surge)
    {
        double first = rvA1->GetValue();
        if (g.surgeDwellS > 0)
        {
            first = first / g.a1MeanS * g.surgeDwellS;
        }
        first = std::max(g.surgeLeadS + 0.1, first);
        regimeNextS = first;
        Simulator::Schedule(Seconds(first), &RegimeSwitch);
        if (g.surgeDwellS > 0 || g.surgeUnlinked)
        {
            rvUpd = CreateObject<ExponentialRandomVariable>();
            rvUpd->SetAttribute("Mean", DoubleValue(g.a1MeanS));
            rvUpd->SetStream(33);
            Simulator::Schedule(Seconds(rvUpd->GetValue()), &SurgePublishLoop);
        }
    }
""")
rep("""    cmd.AddValue("outputDir", "output directory", g.outDir);
""", """    cmd.AddValue("oracleIntent", "control arm: on-board decisions always use the current intent", g.oracleIntent);
    cmd.AddValue("surge", "intents linked to demand through a two-state regime", g.surge);
    cmd.AddValue("surgeSessions", "sessions added during a surge", g.surgeSessions);
    cmd.AddValue("surgeHighG", "guaranteed share of the surge intent", g.surgeHighG);
    cmd.AddValue("surgeLowG", "guaranteed share of the quiet intent", g.surgeLowG);
    cmd.AddValue("surgeH", "headroom of both intents", g.surgeH);
    cmd.AddValue("dataPlane", "packet data plane with a best-effort slice", g.dataPlane);
    cmd.AddValue("cellMbps", "cell capacity (Mb/s)", g.cellMbps);
    cmd.AddValue("initG", "guaranteed share of the first intent, held from launch", g.initG);
    cmd.AddValue("initH", "headroom of the first intent, held from launch", g.initH);
    cmd.AddValue("surgeLeadS", "rApp knowledge of the regime ahead (s)", g.surgeLeadS);
    cmd.AddValue("surgeDwellS", "regime dwell apart from a1MeanS (s)", g.surgeDwellS);
    cmd.AddValue("surgeDuty", "share of time in a surge", g.surgeDuty);
    cmd.AddValue("surgeUnlinked", "surges with intents independent of them", g.surgeUnlinked);
    cmd.AddValue("nsPeriodS", "period and deadline of the slicing class (s)", g.periodS[kNS]);
    cmd.AddValue("workConserving", "BE may use critical budget left idle in a TTI", g.workConserving);
    cmd.AddValue("outputDir", "output directory", g.outDir);
""")
# output
rep("""    {
        std::ofstream f(g.outDir + "/outages.csv");""", """    if (g.dataPlane)
    {
        std::ofstream f(g.outDir + "/dataplane.csv");
        uint64_t n = 0, c = 0;
        for (auto v : dp.latHist)
        {
            n += v;
        }
        double p99 = 0;
        for (size_t i = 0; i < dp.latHist.size(); ++i)
        {
            c += dp.latHist[i];
            if (n && c >= 0.99 * n)
            {
                p99 = i;
                break;
            }
        }
        const double cap = g.cellMbps * 1e6 / 8.0 * dp.servedS;
        f << "crit_offered,crit_delivered,crit_dropped,crit_queued,crit_inflight,be_delivered,served_s,cap_bytes,"
             "crit_ratio,be_share,total_share,crit_p99_ms,surge_offered,surge_delivered\\n";
        f << dp.critOffered << "," << dp.critDelivered << "," << dp.critDropped << "," << critQ.size() * g.pktBytes << ","
          << critInFlight.size() * g.pktBytes << "," << dp.beDelivered << "," << dp.servedS << "," << cap << ","
          << (dp.critOffered ? double(dp.critDelivered) / dp.critOffered : 0.0) << "," << dp.beDelivered / cap << ","
          << (dp.beDelivered + dp.critDelivered) / cap << "," << p99 << "," << dp.surgeOffered << ","
          << dp.surgeDelivered << "\\n";
    }
    {
        std::ofstream f(g.outDir + "/outages.csv");""")
rep("""    NS_ABORT_MSG_IF(g.numGw < 1 || g.numGw > 4, "numGw must be 1..4");
""", """    NS_ABORT_MSG_IF(g.numGw < 1 || g.numGw > 4, "numGw must be 1..4");
    g.deadlineS[kNS] = g.periodS[kNS];
""")
rep("""    smoCur = Policy{1, 0.3, 0.2, Seconds(0)};""", """    smoCur = Policy{1, g.initG, g.initH, Seconds(0)};""")
# Round 8 (post hoc): default-off provisioning of every reachable satellite. The SMO pushes each new
# A1 version to every satellite of the serving set whose feeder is up, through whichever gateway it
# sees, and the retry loop repeats it until acknowledged; the default pushes to the serving satellite only.
rep("""    double initH{0.2};""", """    double initH{0.2};
    bool provisionAll{false};  // push A1 versions to every satellite whose feeder is up, not only the serving one""")
rep("""    cmd.AddValue("initG", "guaranteed share of the first intent, held from launch", g.initG);""",
    """    cmd.AddValue("initG", "guaranteed share of the first intent, held from launch", g.initG);
    cmd.AddValue("provisionAll", "push A1 versions to every satellite whose feeder is up (on-board strategies)", g.provisionAll);""")
rep("""void
SmoPublish(double gv, double hv)
{""", """// Provisioning through any gateway: every satellite with its feeder up that has not acknowledged the
// current version gets it, with the same lease stamp as the serving satellite.
void
ProvisionAll()
{
    if (!g.provisionAll || !IsOnboardLike())
    {
        return;
    }
    for (int s = 0; s < static_cast<int>(sats.size()); ++s)
    {
        if (s == servingIdx || !sats[s].linkUp || satAckedVer[s] >= smoCur.ver)
        {
            continue;
        }
        MgmtHeader a;
        a.type = A1_PUT;
        a.ver = smoCur.ver;
        a.v1 = smoCur.g;
        a.v2 = smoCur.h;
        a.untilNs = OnbLeaseUntilNs();
        SendToSat(s, a, PayloadSize(A1_PUT, 0), true);
    }
}

void
SmoPublish(double gv, double hv)
{""")
rep("""        SendToSat(servingIdx, a, PayloadSize(A1_PUT, 0), true);
    }
}

// SMO -> ground RIC over the cloud link""", """        SendToSat(servingIdx, a, PayloadSize(A1_PUT, 0), true);
    }
    ProvisionAll();
}

// SMO -> ground RIC over the cloud link""")
rep("""    Simulator::Schedule(Seconds(g.a1RetryS), &SmoRetryLoop);
    if (!IsOnboardLike() || servingIdx < 0)""", """    Simulator::Schedule(Seconds(g.a1RetryS), &SmoRetryLoop);
    ProvisionAll();
    if (!IsOnboardLike() || servingIdx < 0)""")
open(os.path.join(H, "oran-ntn-feeder-mgmt_dp.cc"), "w").write(s)
