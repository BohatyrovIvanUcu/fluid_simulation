# Autonomous Interactive Fluid Simulation Pendant
> **UCU Robotics and Embedded Systems — Defense 1 Proof of Concept (October 2026)**

An autonomous, wearable smart pendant ($\varnothing 45$\,mm, 11.5\,mm depth) that simulates real-time interactive fluid dynamics responding to physical tilting and acceleration. Built on an ARM Cortex-M microcontroller driving a custom circular matrix of 240 Charlieplexed LEDs with zero dynamic allocations.

---

## Team & Roles
* **Ivan Bohatyrov** (`bohatyrov.pn@ucu.edu.ua`) — Mathematical Formulation, C++ Physics Engine & Simulation Benchmarks
* **Ostap Sukhomlyn** (`sukhomlyn.pn@ucu.edu.ua`) — STM32 Architecture, Charlieplexing Matrix Driver & Timers
* **Artem Serhiichuk** (`serhiichuk.pn@ucu.edu.ua`) — System Architecture, CAD Enclosure Design (Fusion 360) & Requirements
* **Andriy Antoniuk** (`a.antoniuk.pn@ucu.edu.ua`) — Power Subsystem (Battery/LDO/Charger), Bill of Materials & Procurement
* **Daryna Shevchuk** (`shevchuk.d@ucu.edu.ua`) — Hardware Testing & Quality Assurance
* **Mentor**: Oleksiy Hoyev (Software Engineer, Sigma Software)

---

## Key Engineering Specifications
* **Display Matrix**: 240 discrete SMD 0603 Blue LEDs arranged in a circular grid ($\varnothing 42$\,mm active PCB).
* **Display Driving**: **Charlieplexing** ($N(N-1) = 16 \times 15 = 240$ LEDs using only 16 MCU GPIO pins).
* **Target Refresh Rate**: 40–60 FPS.
* **Microcontroller Strategy**:
  * *Prototyping / Lab bring-up:* NUCLEO-L432KC (ARM Cortex-M4 @ 80\,MHz, 64\,KB SRAM).
  * *Final Custom PCB:* Standalone STM32F401CCU6 (UFQFPN-48, Cortex-M4 @ 84\,MHz with hardware single-precision FPU).
* **Sensor**: LSM6DS3 6-axis ultra-low-power IMU (0.9\,mA active, I2C @ 400\,kHz, 104\,Hz sampling).
* **Power Subsystem**: Single 3.7\,V 300\,mAh Li-Po battery (model 502030) with TP4056 USB Type-C charging and XC6206 3.3\,V LDO regulator.
* **Battery Runtime**: $\ge 6.2$ hours continuous runtime at 40.95\,mA average draw.

---

## Physics Engine Architecture (Position-Based Fluids)
To operate within strict embedded constraints ($\le 64$\,KB RAM, low CPU budget), fluid behavior is simulated as discrete 2D particles using distance constraints:
1. **Force Integration & Prediction**: Velocity updated with IMU gravity vector $\mathbf{g}$ and damping $\alpha = 0.988$.
2. **Incompressibility Constraint**: Neighboring particles closer than $2r$ are pushed apart along their contact normal.
3. **Boundary Clamping**: Particles are constrained strictly inside normalized circle radius $R = 0.97$.
4. **Velocity from Displacement**: $\mathbf{v} = (\mathbf{x}^* - \mathbf{x})/\Delta t$, naturally generating lateral splash waves upon impact.
5. **XSPH Viscosity Smoothing**: Cohesive liquid surface tension and wave damping.

### Benchmark Results (PC vs. STM32 Cortex-M4)
| Fill Level | Particles ($N$) | Active LEDs | Latency per Step | PC Frame Rate | Estimated MCU Load (84\,MHz @ 40 FPS) |
| :---: | :---: | :---: | :---: | :---: | :---: |
| 20% | 8 | 36 | 0.002\,ms | 415,000 FPS | $<1\%$ |
| 40% | 18 | 69 | 0.006\,ms | 170,000 FPS | $<1\%$ |
| **60% (Nominal)** | **28** | **95** | **0.010\,ms** | **103,000 FPS** | **$\approx 1\%$** |
| 80% | 40 | 125 | 0.016\,ms | 64,000 FPS | $\approx 2\%$ |
| 100% (Full) | 52 | 150 | 0.023\,ms | 43,000 FPS | $\approx 3\%$ |

* **SRAM Footprint**: **5.95\,KB** (only 9.3% of STM32F401 64\,KB SRAM).
* **Heap Usage**: **0 bytes ($O(1)$ static memory)**.
* **Stability**: Zero lost particles or NaN errors over 10,000 steps of 5G shocks.

---

## Quickstart & Interactive CLI

### Build the Project:
```bash
make
```

### Run the Interactive Terminal Simulation:
```bash
./build/fluid_sim_cli
```

**Controls:**
* `W` `A` `S` `D` / `Arrow Keys` — Tilt the pendant (adjust gravity vector).
* `Space` — Toggle automatic circular rotation.
* `R` / `E` — Spin the pendant counter-clockwise (`↺`) / clockwise (`↻`).
* `F` / `V` — Increase / decrease rotation speed.
* `1` .. `5` — Switch fluid fill level (15% .. 80%).
* `+` / `-` — Add / remove fluid particles dynamically.
* `X` — Shake the pendant (shock impulse).
* `C` — Reset fluid to resting bottom state.
* `Q` — Quit.

### Run Verification Test Suite & Benchmarks:
```bash
make benchmark
```
*(or `./build/fluid_sim_cli --test`)*

---

## Documentation & Reports
* **IEEE Defense 1 Report LaTeX**: [`docs/report_ieee_defense1.tex`](docs/report_ieee_defense1.tex)
* **Overleaf Project**: Maintained collaboratively for Defense 1 evaluation.
