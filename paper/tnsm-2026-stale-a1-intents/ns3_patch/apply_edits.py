#!/usr/bin/env python3
"""Produces ../ns3_patch/oran-ntn-feeder-mgmt.cc from orig/ (Paper 3 revision, Nov 2026)."""
import os
H = os.path.dirname(os.path.abspath(__file__))
s = open(os.path.join(H, "orig", "oran-ntn-feeder-mgmt.cc")).read()

def rep(a, b, n=1):
    global s
    assert s.count(a) >= 1 and (n == 0 or s.count(a) == n), a
    s = s.replace(a, b)

# --- configuration
rep("    double maxLeaseS{300.0};\n", "    double maxLeaseS{3600.0};  // large enough not to bind in the T_A1 sweep\n"
    "    double fixedLeaseS{-1.0};  // fixed lease timer (s); <0 uses the rule\n")
rep("    uint32_t numBeams{16};\n", """    uint32_t numBeams{16};
    bool nfPinOnboard{false};  // PAL with the host test of the break-even: NF stays on board
    double beamCv2{1.0};       // squared CV of the dwell of the best beam (1 = Poisson)
    double beamDrift{0.5};     // probability that a jump repeats the last direction
    double errRho{0.0};        // correlation of consecutive decision outcomes
    bool a1Step{false};        // a new intent moves one step in the alphabet, not a redraw
    std::string gwList{};      // "lat:lon,lat:lon,...": replaces the gateway list; RIC at the first
""")
rep("const GwSite kGw[4] = {", "GwSite kGw[4] = {")
rep("const double kRicLat = 24.86;", "double kRicLat = 24.86;")
rep("const double kRicLon = 67.01;", "double kRicLon = 67.01;")

# --- event-driven beam process (no sampling grid)
a = s.index("void\nBeamTick()")
b = s.index("// Ground (full xApp) decision for an NF epoch")
s = s[:a] + """// Event-driven beam outcome: the applied and the best beam change only at
// events, and the misaligned time is integrated exactly between them, so no
// grid couples the age difference between hosts to the phase of the NF epochs.
void
BeamAccrue()
{
    const int64_t now = Simulator::Now().GetNanoSeconds();
    const double dt = (now - beamLastNs) * 1e-9;
    beamLastNs = now;
    if (servingIdx < 0 || dt <= 0)
    {
        return;
    }
    beamServedS += dt;
    for (int i = 0; i < kNT; ++i)
    {
        for (int j = 0; j < kNG; ++j)
        {
            if (appliedBeam[i][j] != bestBeam[i])
            {
                beamMisS[i][j] += dt;
            }
        }
    }
}

// Next dwell of the best beam, mean kDwell[i]: exponential by default, Erlang
// (squared CV below 1) or balanced hyperexponential (above 1) as stress cases.
double
BeamGap(int i)
{
    const double T = kDwell[i];
    auto ex = [&]() { return -std::log(1.0 - rvBeam->GetValue()); };
    if (g.beamCv2 < 1.0)
    {
        const int k = std::max(1, static_cast<int>(std::lround(1.0 / g.beamCv2)));
        double x = 0;
        for (int n = 0; n < k; ++n)
        {
            x += ex();
        }
        return T * x / k;
    }
    if (g.beamCv2 > 1.0)
    {
        const double p1 = 0.5 * (1.0 + std::sqrt((g.beamCv2 - 1.0) / (g.beamCv2 + 1.0)));
        const bool first = rvBeam->GetValue() < p1;
        return ex() * (first ? T / (2.0 * p1) : T / (2.0 * (1.0 - p1)));
    }
    return T * ex();
}

void
BeamJump(int i)
{
    BeamAccrue();
    const int B = static_cast<int>(g.numBeams);
    if (rvBeam->GetValue() >= g.beamDrift)
    {
        beamDir[i] = -beamDir[i];
    }
    bestBeam[i] = (bestBeam[i] + (beamDir[i] > 0 ? 1 : B - 1)) % B;
    Simulator::Schedule(Seconds(BeamGap(i)), &BeamJump, i);
}

// Forget decisions of epochs that can no longer be applied in time.
void
BeamCleanup()
{
    Simulator::Schedule(Seconds(1), &BeamCleanup);
    const int64_t cut = (Simulator::Now() - Seconds(5)).GetNanoSeconds();
    while (!nfMeasured.empty() && nfMeasured.begin()->first < cut)
    {
        nfMeasured.erase(nfMeasured.begin());
    }
    while (!rcBeam.empty() && rcBeam.begin()->first < cut)
    {
        rcBeam.erase(rcBeam.begin());
    }
}

// Decision outcome: uniform marginals, reused with probability errRho, so a
// wrong pick can persist while the accuracy of each decision stays the same.
std::pair<double, double>
DecisionDraw()
{
    static double lastX = 0.5;
    static double lastU = 0.5;
    if (g.errRho <= 0.0 || rvDec->GetValue() >= g.errRho)
    {
        lastX = rvDec->GetValue();
        lastU = rvDec->GetValue();
    }
    return {lastX, lastU};
}

""" + s[b:]
rep("uint64_t beamMis[kNT][kNG];\nuint64_t beamSamp = 0;\n",
    "double beamMisS[kNT][kNG];\ndouble beamServedS = 0;\nint64_t beamLastNs = 0;\nint beamDir[kNT] = {1, 1, 1, 1};\n")
rep("""void
ResetAppliedBeams()
{
""", """void BeamAccrue();

void
ResetAppliedBeams()
{
    BeamAccrue();
""")
rep("""    const double x = rvDec->GetValue();
    const double u = rvDec->GetValue();
    std::array<int, kNT> d{};""", """    const auto [x, u] = DecisionDraw();
    std::array<int, kNT> d{};""")
rep("""        const double x = rvDec->GetValue();
        const double u = rvDec->GetValue();
        for (int i = 0; i < kNT; ++i)
        {
            for (int j = 0; j < kNG; ++j)
            {
                appliedBeam[i][j] =""", """        const auto [x, u] = DecisionDraw();
        BeamAccrue();
        for (int i = 0; i < kNT; ++i)
        {
            for (int j = 0; j < kNG; ++j)
            {
                appliedBeam[i][j] =""")
rep("""            return;
        }
        for (int i = 0; i < kNT; ++i)
        {
            for (int j = 0; j < kNG; ++j)
            {
                appliedBeam[i][j] = it->second[i];""", """            return;
        }
        BeamAccrue();
        for (int i = 0; i < kNT; ++i)
        {
            for (int j = 0; j < kNG; ++j)
            {
                appliedBeam[i][j] = it->second[i];""")
rep("    Simulator::Schedule(MilliSeconds(5), &BeamTick);\n", """    for (int i = 0; i < kNT; ++i)
    {
        beamDir[i] = rvBeam->GetValue() < 0.5 ? 1 : -1;
        Simulator::Schedule(Seconds(BeamGap(i)), &BeamJump, i);
    }
    Simulator::Schedule(Seconds(1), &BeamCleanup);
""")
rep("        servingIdx = serveTl[serveSeg].sat;\n", "        BeamAccrue();\n        servingIdx = serveTl[serveSeg].sat;\n")
rep("""    Simulator::Run();
""", """    Simulator::Run();
    BeamAccrue();
""")
rep('softSwitches << "," << beamSamp;', 'softSwitches << "," << beamServedS;')
rep("(beamSamp ? double(beamMis[i][j]) / beamSamp : 0.0)", "(beamServedS > 0 ? beamMisS[i][j] / beamServedS : 0.0)")
rep('soft_switches,beam_samples"', 'soft_switches,beam_served_s"')

# --- lease: fixed timer and a posterior mean over the whole exposure
rep("""    const double meanS = (g.priorUpdateS + a1IntervalSum) / (1.0 + a1IntervalN);
    return std::min(""", """    if (g.fixedLeaseS >= 0)
    {
        return Seconds(g.fixedLeaseS);
    }
    // Gamma-exponential posterior mean over all exposure so far, so a quiet
    // spell raises the estimate; the prior weighs as one observation.
    const double seen = a1IntervalN + (a1LastAt >= 0 ? 1.0 : 0.0);
    const double meanS = (g.priorUpdateS + Simulator::Now().GetSeconds()) / (1.0 + seen);
    return std::min(""")

# --- intents that move one step, as a variant
rep("""    p.g = gs[rvIntent->GetInteger(0, 3)];
    p.h = hs[rvIntent->GetInteger(0, 2)];
""", """    if (g.a1Step)
    {
        static int ig = 1;
        static int ih = 1;
        const bool moveG = rvIntent->GetValue() < 0.5;
        const int step = rvIntent->GetValue() < 0.5 ? 1 : -1;
        if (moveG)
        {
            ig = (ig + step < 0 || ig + step > 3) ? ig - step : ig + step;
        }
        else
        {
            ih = (ih + step < 0 || ih + step > 2) ? ih - step : ih + step;
        }
        p.g = gs[ig];
        p.h = hs[ih];
    }
    else
    {
        p.g = gs[rvIntent->GetInteger(0, 3)];
        p.h = hs[rvIntent->GetInteger(0, 2)];
    }
""")

# --- host rule: NF pinned on board when the gap is below the break-even
rep("        return cls == kRT || s.leased[cls] || s.autonomous;",
    "        return cls == kRT || (g.nfPinOnboard && cls == kNF) || s.leased[cls] || s.autonomous;")
rep("        return cls != kRT; // pal: RT is never placed",
    "        return cls != kRT && !(g.nfPinOnboard && cls == kNF); // pal: RT is never placed")
rep("""        for (int c = kNF; c < kClasses; ++c)
        {
            const Time loop = PredictedGroundLoop(s) + Seconds(g.procGroundS[c]);""",
"""        for (int c = kNF; c < kClasses; ++c)
        {
            if (g.nfPinOnboard && c == kNF)
            {
                continue;
            }
            const Time loop = PredictedGroundLoop(s) + Seconds(g.procGroundS[c]);""")

# --- command line
rep("""    cmd.AddValue("outputDir", "output directory", g.outDir);
""", """    cmd.AddValue("fixedLeaseS", "fixed lease timer (s); <0 uses the closed-form rule", g.fixedLeaseS);
    cmd.AddValue("slack", "deadline slack kappa of the placement test", g.slack);
    cmd.AddValue("cellLat", "cell latitude (deg)", g.cellLat);
    cmd.AddValue("cellLon", "cell longitude (deg)", g.cellLon);
    cmd.AddValue("gwList", "gateways lat:lon,lat:lon,... (RIC at the first)", g.gwList);
    cmd.AddValue("nfPinOnboard", "PAL host test: keep the NF class on board", g.nfPinOnboard);
    cmd.AddValue("beamCv2", "squared CV of the best-beam dwell", g.beamCv2);
    cmd.AddValue("beamDrift", "probability a beam jump repeats the last direction", g.beamDrift);
    cmd.AddValue("errRho", "correlation of consecutive NF decision outcomes", g.errRho);
    cmd.AddValue("a1Step", "intents move one step in the alphabet instead of a redraw", g.a1Step);
    cmd.AddValue("outputDir", "output directory", g.outDir);
""")
rep("""    NS_ABORT_MSG_IF(g.numGw < 1 || g.numGw > 4, "numGw must be 1..4");
""", """    NS_ABORT_MSG_IF(g.numGw < 1 || g.numGw > 4, "numGw must be 1..4");
    if (!g.gwList.empty())
    {
        std::stringstream ss(g.gwList);
        std::string item;
        int n = 0;
        while (std::getline(ss, item, ',') && n < 4)
        {
            const auto c = item.find(':');
            kGw[n].name = "custom";
            kGw[n].lat = std::stod(item.substr(0, c));
            kGw[n].lon = std::stod(item.substr(c + 1));
            ++n;
        }
        NS_ABORT_MSG_IF(n < static_cast<int>(g.numGw), "gwList shorter than numGw");
        kRicLat = kGw[0].lat;
        kRicLon = kGw[0].lon;
    }
""")
rep("#include <set>\n", "#include <set>\n#include <sstream>\n#include <utility>\n")
open(os.path.join(H, "oran-ntn-feeder-mgmt.cc"), "w").write(s)
