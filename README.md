# BLDC-Actuated Stewart Platform for Aerial Additive Manufacturing

This repository contains all **CAD models**, **control code**, and **analysis scripts** developed for the Master's thesis:  
*"Feasibility Study of a Brushless DC Actuated Stewart Platform for Aerial Additive Manufacturing"*.

The project investigates the use of **BLDC motors** in a 6-DOF Stewart platform, aiming to achieve **high responsiveness, precision, and lightweight design** suitable for UAV integration.

```

## 📂 Repository Structure

/CAD  
   /Base  
      BasePlate.f3d  
      BasePlate.step  
      BasePlate.stl  
   /Arms  
      LeverArm.f3d  
      LeverArm.step  
      LeverArm.stl  
   /Couplers  
      Coupler.f3d  
      Coupler.step  
      Coupler.stl  
   /Assembly  
      StewartPlatform.f3d  
      StewartPlatform.step  
      StewartPlatform.stl  

/Code  
   /arduino  
      motor_control.ino  
   /matlab  
      workspace_analysis.m  
   /python  
      error_analysis.py  

/Docs  
   README.md   

```

## 🛠️ CAD Models

- **Format Provided:** `.f3d` (Fusion 360 native), `.step` (universal exchange), `.stl` (3D-printable).  
- **Components:**
  - Base Plate  
  - Lever Arms  
  - Couplers (motor–potentiometer integration)  
  - Moving Platform  
  - Full Assembly  

```

## 💻 Code

- **Arduino**  
  - Motor control sketches for BLDC actuation via AM32 ESCs.  
  - Implements PD control loop with logging functionality.  

- **MATLAB**  
  - Scripts for workspace analysis (grid search + alpha-shape volume).  
  - Inverse kinematics implementation for rotary Stewart platform.  

- **Python**  
  - Post-processing of experimental data.  
  - Angle error vs. pose error analysis and plotting tools.  

```

## ⚡ System Overview

- **Actuators:** Repeat Compact Brushless Planetary 22mm Gearmotor  
- **ESCs:** Repeat Dual AM32 Brushless Drive ESC  
- **Microcontroller (prototype):** Arduino Elegoo Uno R3  
- **Microcontroller (future):** Teensy 4.1  
- **Sensors:** 10kΩ potentiometers (prototype), AS5048A magnetic encoders (planned upgrade)  
- **Fabrication:** FDM 3D printing (PLA)  

```

## 📊 Key Features

- Feasibility study of BLDC actuation for Stewart platforms.  
- Evaluation of workspace, precision, and responsiveness.  
- Comparison of sequential vs coordinated motor operation.  
- Pose-level error analysis (translation & orientation).  

```

## 📑 Thesis Reference

This repository supports the Master's thesis:  
**[Title of Thesis]**  
Author: *Ibrahim [Your Surname]*  
Institution: *University College London (UCL)*  
Year: *2025*

```

## 🔗 How to Cite

If you use this work, please cite:

I. [Your Surname], "Feasibility Study of a Brushless DC Actuated Stewart Platform for Aerial Additive Manufacturing," Master's Thesis, University College London, 2025.

```
