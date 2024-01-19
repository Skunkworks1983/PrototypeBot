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

  TalonSRX sim_1;
  TalonSRX sim_2;
  TalonSRX sim_3;
  TalonSRX sim_4;

  private static Drivebase drivebase;

  TalonFX [] talonMotors = {Talon_1, Talon_2, Talon_3, Talon_4};
  CANSparkMax [] canMotors = {Spark_1, Spark_2, Spark_3, Spark_4};
  TalonSRX [] sims = {sim_1, sim_2, sim_3, sim_4};

  int motorType;

  final DutyCycleOut motorDutyCycle = new DutyCycleOut(0.0);
  
  private Drivebase() {
      Talon_1 = new TalonFX(0, Constants.CANIVORE_NAME);
      Talon_2 = new TalonFX(0, Constants.CANIVORE_NAME);
      Talon_3 = new TalonFX(0, Constants.CANIVORE_NAME);
      Talon_4 = new TalonFX(0, Constants.CANIVORE_NAME);

      Spark_1 = new CANSparkMax(0, MotorType.kBrushless);
      Spark_1 = new CANSparkMax(0, MotorType.kBrushless);
      Spark_1 = new CANSparkMax(0, MotorType.kBrushless);
      Spark_1 = new CANSparkMax(0, MotorType.kBrushless);

      sim_1 = new TalonSRX(0);
      sim_2 = new TalonSRX(0);
      sim_3 = new TalonSRX(0);
      sim_4 = new TalonSRX(0);
    }

  public void setPercentOutput (int motorNum, double speed) {

    if (motorType == 0) {
      talonMotors[motorNum].setControl(motorDutyCycle.withOutput(speed));
    } else if (motorType == 1) {
      canMotors[motorNum].set(speed);
    } else if (motorType ==2) {
      sim_1.set(ControlMode.PercentOutput, speed);
    }
  }

  

  public static Drivebase getInstance() {
    if (drivebase == null) {
      drivebase = new Drivebase();
    }
    return drivebase;
  }
}
