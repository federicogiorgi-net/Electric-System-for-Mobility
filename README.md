# Electric Systems for Mobility — Frecciarossa 1000 Dynamic Simulation

Dynamic simulation of the **Frecciarossa 1000 (ETR1000)** high-speed train on the **Turin–Milan** line.  
The project models traction and braking forces (including regenerative braking), energy consumption, and electrical performance under different operating conditions.

> **Course**: Electric Systems for Mobility (Politecnico di Milano)  
> **Authors**: Federico Giorgi, Leonardo Vincenzo Rossi, Alfonso Di Nardi, Giorgio Calabrese  
> **Academic Year**: 2020/2021

---

## Key Features
- Full dynamic simulation of a 148 km high-speed route with intermediate stops.
- Traction and braking modeling with adhesion check and free-running control.
- Regenerative braking analysis based on braking effort percentage.
- Electrical border points (EBP) handling: zero power sections for pantograph lowering.
- Calculation of:
  - Speed and acceleration profiles
  - Traction/braking forces vs adhesion limits
  - Power, current, and energy consumption
  - Resistance components (aerodynamic, grade, curvature)
- Automatic generation of multiple plots for performance analysis.

## Requirements
- **MATLAB** R2020a or later (no proprietary toolboxes required).
- Works on Windows, macOS, or Linux.
- Optional: **Octave** (partial compatibility).

---

## How to Run
1. Clone the repository:
   ```bash
   git clone https://github.com/<username>/Electric-Systems-for-Mobility.git
   cd Electric-Systems-for-Mobility
2. Open MATLAB and set the repository folder as the Current Folder.
3. Run the script:
     ```bash
     run('src/Script_ESM.m')
   
The script will:

- Simulate the entire route with stops.
- Compute dynamic and electrical quantities.
- Generate plots for speed, acceleration, power, energy, and forces.

## References
- Hitachi Rail & Bombardier ETR1000 technical data.
- OpenRailwayMap for speed limits and route geometry.
- Trenitalia official timetables.
- Full reference list in docs/ESMProject_Report.pdf.

## License

This project is released under the MIT License. See the LICENSE file for details.
