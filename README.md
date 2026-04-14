# O-RAN NTN Module for ns-3

A comprehensive O-RAN (Open RAN) Non-Terrestrial Network (NTN) simulation module for ns-3, deeply integrating with mmWave, satellite (SNS3), and ns3-ai modules.

## Overview

This module implements the complete Space-O-RAN architecture for LEO satellite networks with:

- **Near-RT RIC** with E2/A1 interfaces and Shared Data Layer (SDL)
- **Non-RT RIC** with orbit-aware A1 policy lifecycle management
- **Space RIC** with on-board autonomous decision-making and ISL coordination
- **9 xApps** covering handover, beam hopping, slicing, Doppler compensation, TN-NTN steering, interference management, energy harvesting, predictive allocation, and multi-connectivity
- **5 OpenGymEnv** environments for RL-based xApp training via ns3-ai
- **Federated Learning** coordinator with FedAvg/FedProx/FedNova aggregation
- **Deep satellite integration** with Markov fading, DVB-S2X ModCod selection, inter-beam interference
- **mmWave NTN PHY** with elevation-aware beamforming, NTN channel model, RTT-aware scheduler
- **Dual Connectivity** manager for simultaneous TN + NTN bearer support

## Architecture

```
Non-RT RIC (SMO)
    |  A1 policies (orbit-aware)
    v
Near-RT RIC
    |  E2 interface (KPM reports, RC actions)
    |  xApp conflict resolution
    |  Shared Data Layer
    v
+---+---+---+---+---+---+---+---+---+
| HO  |Beam|Slice|Dopp|Steer|Intf|Enrg|Pred|Multi|
|Pred |Hop |Mgr  |Comp|     |Mgmt|Harv|Allc|Conn |
+---+---+---+---+---+---+---+---+---+
    |  OpenGymEnv (ns3-ai)
    v
Space RIC (on-orbit)
    |  ISL coordination
    |  LibTorch / Rule-based inference
    |  Federated Learning
    v
Satellite Bridge
    |  SGP4 orbit, fading, ModCod, interference
    v
mmWave PHY + Satellite Channel
```

## Module Structure

```
oran-ntn/
├── model/                    # Core implementation (32 .h + 32 .cc)
│   ├── oran-ntn-types.h                    # All data structures and enums
│   ├── oran-ntn-e2-interface.*             # E2 KPM/RC interface
│   ├── oran-ntn-a1-interface.*             # A1 policy management
│   ├── oran-ntn-conflict-manager.*         # Multi-xApp conflict resolution
│   ├── oran-ntn-xapp-base.*               # Abstract xApp base class
│   ├── oran-ntn-near-rt-ric.*             # Near-RT RIC controller
│   ├── oran-ntn-space-ric.*               # On-orbit Space RIC
│   ├── oran-ntn-sat-bridge.*              # Deep satellite integration
│   ├── oran-ntn-xapp-ho-predict.*         # HO Prediction xApp (LSTM/DQN)
│   ├── oran-ntn-xapp-beam-hop.*           # Beam Hopping xApp
│   ├── oran-ntn-xapp-slice-manager.*      # Network Slice Manager xApp
│   ├── oran-ntn-xapp-doppler-comp.*       # Doppler Compensation xApp
│   ├── oran-ntn-xapp-tn-ntn-steering.*    # TN-NTN Traffic Steering xApp
│   ├── oran-ntn-mmwave-beamforming.*      # NTN-aware beamforming model
│   ├── oran-ntn-channel-model.*           # Composite NTN channel model
│   ├── oran-ntn-ntn-scheduler.*           # RTT-aware MAC scheduler
│   ├── oran-ntn-phy-kpm-extractor.*       # Real PHY KPM extraction
│   ├── oran-ntn-dual-connectivity.*       # TN-NTN dual connectivity
│   ├── oran-ntn-gym-handover.*            # Gymnasium env: handover RL
│   ├── oran-ntn-gym-beam-hop.*            # Gymnasium env: beam hopping RL
│   ├── oran-ntn-gym-slice.*               # Gymnasium env: slice mgmt RL
│   ├── oran-ntn-gym-steering.*            # Gymnasium env: steering RL
│   ├── oran-ntn-gym-predictive.*          # Gymnasium env: prediction RL
│   ├── oran-ntn-msg-interface.h           # ns3-ai shared memory structs
│   ├── oran-ntn-federated-learning.*      # FL coordinator (FedAvg/FedProx)
│   ├── oran-ntn-xapp-interference-mgmt.* # Interference Management xApp
│   ├── oran-ntn-xapp-energy-harvest.*     # Energy Harvesting xApp
│   ├── oran-ntn-xapp-predictive-alloc.*   # Predictive Allocation xApp
│   ├── oran-ntn-xapp-multi-conn.*         # Multi-Connectivity xApp
│   ├── oran-ntn-space-ric-inference.*     # On-board inference engine
│   └── oran-ntn-isl-header.*             # ISL protocol header
├── helper/
│   └── oran-ntn-helper.*                  # Top-level scenario builder
├── test/
│   └── oran-ntn-test-suite.cc             # 18 unit tests
├── examples/
│   └── oran-ntn-full-scenario.cc          # Full constellation example
├── tools/
│   ├── oran_ntn_ai_agent.py               # Core AI architectures
│   ├── oran_ntn_gym_agents.py             # Gymnasium RL agents
│   └── oran_ntn_space_ric_agent.py        # Space RIC AI agent
└── CMakeLists.txt
```

## Key Features

### Deep Satellite Integration (Phase 1)
- **Markov fading model**: 3-state (clear/shadow/blocked) with elevation-dependent transition probabilities
- **DVB-S2X ModCod selection**: 28 SINR thresholds from QPSK 1/4 to 32APSK 9/10
- **Inter-beam interference**: Intra-satellite sidelobe + inter-satellite co-channel
- **ISL topology**: Intra-plane and inter-plane links with realistic delays

### mmWave NTN PHY (Phase 2)
- **NTN beamforming**: Elevation-dependent steering, Doppler beam squint correction, predictive steering
- **Composite channel**: FSPL + ITU-R atmospheric (P.676/P.618/P.531) + Loo fading + Markov + clutter
- **NTN scheduler**: RTT-aware HARQ (5-40ms), beam-hopping slots, TTE-priority, slice PRB ranges
- **Dual connectivity**: McUeNetDevice management, split bearer, primary path switching

### AI/ML Integration (Phase 3)
- **5 OpenGymEnv subclasses** with observation/action/reward definitions for each xApp
- **ns3-ai msg-interface** structs for high-performance Space RIC inference
- **Federated learning**: FedAvg, FedProx, FedNova with top-k gradient compression
- **Python agents**: DQN+PER, PPO, MAPPO, LSTM architectures

### Advanced xApps (Phase 4)
- **Interference Management**: Graph-based detection, power control, null steering, graph coloring
- **Energy Harvesting**: Solar model, eclipse detection, battery SoC, SLA-aware beam shutdown
- **Predictive Allocation**: LSTM traffic prediction, anomaly detection, proactive PRB reservation
- **Multi-Connectivity**: DC activation/teardown, optimal split ratio, predictive DC

### Enhanced Space RIC (Phase 5)
- **Multi-backend inference**: LibTorch -> msg-interface IPC -> rule-based fallback
- **ISL packet exchange**: Custom 27-byte protocol header, realistic delay-based delivery
- **Model management**: Federated learning weight distribution and version tracking

## Dependencies

- ns-3.43+
- SNS3 satellite module (contrib/satellite)
- mmWave module (contrib/mmwave)
- ns3-ai module (contrib/ai)
- Boost (for ns3-ai shared memory)
- Protobuf (for ns3-ai gym interface)

## References

- O-RAN.WG3.E2SM-KPM-v03.00
- O-RAN.WG3.E2SM-RC-v01.03
- O-RAN.WG2.A1-v03.01
- Space-O-RAN (IEEE CommMag 2026)
- 3GPP TR 38.811 (NTN channel models)
- 3GPP TS 38.821 (NTN solutions)
- DVB-S2X (ETSI EN 302 307-2)

## Author

Muhammad Uzair

## License

GPL-2.0-only
