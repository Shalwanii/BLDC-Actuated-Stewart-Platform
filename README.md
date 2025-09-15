# BLDC-Actuated Stewart Platform for Aerial Additive Manufacturing

This repository contains all **CAD models**, **control code**, and **analysis scripts** developed for the Master's thesis:  
*"Feasibility Study of a Brushless DC Actuated Stewart Platform for Aerial Additive Manufacturing"*.

The project investigates the use of **BLDC motors** in a 6-DOF Stewart platform, aiming to achieve **high responsiveness, precision, and lightweight design** suitable for UAV integration.


/CAD  
   /Ball_Joint  
      Ball_Joint_Baseless.f3d  
      Ball_Joint_Baseless.step  
      Ball_Joint_Baseless.stl  

   /Lever_Arm  
      Lever_Arm.f3d  
      Lever_Arm.step  
      Lever_Arm.stl  

   /Motor_Facemount  
      Motor_Facemount.f3d  
      Motor_Facemount.step  
      Motor_Facemount.stl  

   /Moving_Base  
      Moving_Base.f3d  
      Moving_Base.step  
      Moving_Base.stl  

   /Potentiometer_Coupler  
      Potentiometer_Coupler.f3d  
      Potentiometer_Coupler.step  
      Potentiometer_Coupler.stl  

   /Potentiometer_Mount  
      Potentiometer_Case.f3d  
      Potentiometer_Case.step  
      Potentiometer_Case.stl  
      Potentiometer_L_Bracket.f3d  
      Potentiometer_L_Bracket.step  
      Potentiometer_L_Bracket.stl  

   /Shaft  
      Shaft.f3d  
      Shaft.step  
      Shaft.stl  

   /Stationary_Base  
      Stationary_Base.f3d  
      Stationary_Base.step  
      Stationary_Base.stl  

   Final_Platform_Assembly.step  
   Potentiometer.SLDPRT  
   Repeat_Compact.STEP  

/Arduino  
   Single_Motor_Test.ino  
   Sequential_Motor_Test.ino  
   Coordinated_Multi_Motor_Test.ino  

/Matlab  
   stewartIK.m  
   workspace.m  

/Python  
   stewartFK.py  

/Docs  
   README.md
 
```
```
## 🛠️ CAD Models

- **Format Provided:** `.f3d` (Fusion 360 native), `.step` (universal exchange), `.stl` (3D-printable).  
- **Components:**
  - Ball Joint  
  - Lever Arm
  - Motor Facemount
  - Moving Base
  - Potentiometer Coupler
  - Potentiometer Mount
  - Shaft  
  - Stationary Base
  - Moving Platform  
  - Full Assembly  

```
```
## 💻 Code

- **Arduino**  
  -   Single-Motor Calibration: Each motor was driven individually under different tolerance thresholds to tune the PD controller and characterise convergence behaviour with potentiometer feedback.  
  - Sequential Motor Testing: Motors were activated one at a time with random target angles while the others were held neutral, measuring settling time, steady-state error, and reachability within defined angle windows.
  - Coordinated Multi-Motor Evaluation: All six motors were commanded simultaneously and achieved poses were compared against desired ones to assess system-level accuracy, convergence, and pose errors.

- **MATLAB**  
  - Scripts for workspace analysis.  
  - Inverse kinematics implementation for rotary Stewart platform.  

- **Python**  
  - Forward Kinematics to obtain Pose from Joint Angles  

```

```
## ⚡ System Overview

- **Actuators:** Repeat Compact Brushless Planetary 22mm Gearmotor  
- **ESCs:** Repeat Dual AM32 Brushless Drive ESC  
- **Microcontroller (prototype):** Arduino Elegoo Uno R3  
- **Microcontroller (future):** Teensy 4.1  
- **Sensors:** 10kΩ potentiometers (prototype), AS5048A magnetic encoders (planned upgrade)  
- **Fabrication:** FDM 3D printing (PLA)  

```
```
## 📊 Key Features

- Feasibility study of BLDC actuation for Stewart platforms.  
- Evaluation of workspace, precision, and responsiveness.  
- Comparison of sequential vs coordinated motor operation.  
- Pose-level error analysis (translation & orientation).  

```