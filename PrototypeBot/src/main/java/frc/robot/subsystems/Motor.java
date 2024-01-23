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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Motor extends SubsystemBase {
  TalonFX talon;
  CANSparkMax spark;
  TalonSRX sim;
  int motorType;
  int id;
  double rPS;
  double pos;
  double lastPos;
  double seconds;
  double lastSeconds;
  double speed;
  PIDController velocityController;

  private final Timer timer = new Timer();
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

    pos = 0;
    lastPos = 0;
    seconds = 0;
    lastSeconds = 0;
    speed = 0;

    velocityController = new PIDController(
      Constants.VELOCITY_KP, 
      Constants.VELOCITY_KI, 
      Constants.VELOCITY_KD);

    timer.restart();
  }

  public void setPercentOutput (double speed) {
    if (motorType == 1) {
      talon.setControl(motorDutyCycle.withOutput(speed));
    } else if (motorType == 2) {
      spark.set(speed);
    } else if (motorType == 3) {
      sim.set(ControlMode.PercentOutput, speed);
    }
  }

  public double getPosition() {
    if (motorType == 1) {
      return talon.getPosition().getValueAsDouble();
    } else if (motorType == 2) {
      return spark.getEncoder().getPosition();
    }
    return 0;
  }

  public void setVelocity (double revsPerSecond) {
    velocityController.setSetpoint(revsPerSecond);
    while (!velocityController.atSetpoint()) {
      double increase = velocityController.calculate(rPS);
      if (motorType == 1) {
        talon.setControl(motorDutyCycle.withOutput(speed + increase));
      } else if (motorType == 2) {
        spark.set(speed + increase);
      }
      double error = velocityController.getPositionError();
      SmartDashboard.putNumber("VELOCITY ERROR", error);
    }
  }

  @Override
  public void periodic() {
    pos = getPosition();
    seconds = timer.get();
    rPS = (pos - lastPos) / (seconds - lastSeconds); 
    lastPos = pos;
    lastSeconds = seconds;
  }
}
