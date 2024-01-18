// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivebase extends SubsystemBase {

  TalonFX Talon_1;
  TalonFX Talon_2;
  TalonFX Talon_3;
  TalonFX Talon_4;

  CANSparkMax Spark_1;
  CANSparkMax Spark_2;
  CANSparkMax Spark_3;
  CANSparkMax Spark_4;

  private static Drivebase drivebase;
  TalonFX [] talonMotors;
  CANSparkMax canMotors;
  final DutyCycleOut motorDutyCycle = new DutyCycleOut(0.0);
  
  public Drivebase(boolean motorType) {

    if (motorType)
    Talon_1 = new TalonFX(0, Constants.CANIVORE_NAME);
    Talon_2 = new TalonFX(0, Constants.CANIVORE_NAME);
    Talon_3 = new TalonFX(0, Constants.CANIVORE_NAME);
    Talon_4 = new TalonFX(0, Constants.CANIVORE_NAME);

    Spark_1 = new CANSparkMax(0, MotorType.kBrushless);

    TalonFX [] talonMotors = {Talon_1, Talon_2, Talon_3, Talon_4};
    CANSparkMax [] canMotors = {Spark_1, Spark_2, Spark_3, Spark_4};
  }

  public void setDutyCycleMode (int motorNum, double speed) {

    talonMotors[motorNum].setControl(motorDutyCycle.withOutput(speed));
  }

  public void setVelocityMode () {
  }

public static Drivebase getInstance(boolean motorType) {
    if (drivebase == null) {
      drivebase = new Drivebase(motorType);
    }
    return drivebase;
  }
}
