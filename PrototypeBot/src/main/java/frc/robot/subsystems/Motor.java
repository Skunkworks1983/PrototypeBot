// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Motor extends SubsystemBase {
  TalonFX talon;
  CANSparkMax spark;
  TalonSRX sim;
  int motorType;
  int id;

  final DutyCycleOut motorDutyCycle = new DutyCycleOut(0.0);
  
  public Motor(int motorType, int id) {
    this.motorType = motorType;
    this.id = id;
    if (motorType == 1) {
      talon = new TalonFX(id);
    } else if (motorType == 2) {
      spark = new CANSparkMax(id, MotorType.kBrushless);
    } else if (motorType == 3) {
      sim = new TalonSRX(id);
    }
  }

  public void setPercentOutput (double speed) {
    if (motorType == 1) {
      SmartDashboard.putBoolean("setpercentoutput running", true);
      talon.setControl(motorDutyCycle.withOutput(speed));
    } else if (motorType == 2) {
      spark.set(speed);
    } else if (motorType == 3) {
      sim.set(ControlMode.PercentOutput, speed);
    }
  }
}
