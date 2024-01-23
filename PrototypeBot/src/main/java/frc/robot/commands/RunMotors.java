// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Motor;
import frc.robot.subsystems.OI;

public class RunMotors extends Command {

  Motor motor1;
  Motor motor2;
  Motor motor3;
  Motor motor4;

  SendableChooser<Integer> motor1Status = new SendableChooser<>();
  SendableChooser<Integer> motor2Status = new SendableChooser<>();
  SendableChooser<Integer> motor3Status = new SendableChooser<>();
  SendableChooser<Integer> motor4Status = new SendableChooser<>();

  SendableChooser<Boolean> motor1Joystick = new SendableChooser<>();
  SendableChooser<Boolean> motor2Joystick = new SendableChooser<>();
  SendableChooser<Boolean> motor3Joystick = new SendableChooser<>();
  SendableChooser<Boolean> motor4Joystick = new SendableChooser<>();

  double runSpeed;

  OI oi = OI.getInstance();

  public RunMotors(Motor motor1, Motor motor2, Motor motor3, Motor motor4) {
    this.motor1 = motor1;
    this.motor2 = motor2;
    this.motor3 = motor3;
    this.motor4 = motor4;

    motor1Status.setDefaultOption("1: OFF", 0);
    motor1Status.addOption("1: PCT OUTPUT", 1);
    motor1Status.addOption("2: VELOCITY", 2);

    motor2Status.setDefaultOption("2: OFF", 0);
    motor2Status.addOption("2: PCT OUTPUT", 1);
    motor2Status.addOption("3: VELOCITY", 2);

    motor3Status.setDefaultOption("3: OFF", 0);
    motor3Status.addOption("3: PCT OUTPUT", 1);
    motor3Status.addOption("3: VELOCITY", 2);


    motor4Status.setDefaultOption("4: OFF", 0);
    motor4Status.addOption("4: PCT OUTPUT", 1);
    motor4Status.addOption("3: VELOCITY", 2);


    motor1Joystick.setDefaultOption("Joystick", true);
    motor1Joystick.addOption("Input", false);

    motor2Joystick.setDefaultOption("Joystick", true);
    motor2Joystick.addOption("Input", false);

    motor3Joystick.setDefaultOption("Joystick", true);
    motor3Joystick.addOption("Input", false);

    motor4Joystick.setDefaultOption("Joystick", true);
    motor4Joystick.addOption("Input", false);

    SmartDashboard.putData("MOTOR 1 CONTROL", motor1Status);
    SmartDashboard.putData("MOTOR 2 CONTROL", motor2Status);
    SmartDashboard.putData("MOTOR 3 CONTROL", motor3Status);
    SmartDashboard.putData("MOTOR 4 CONTROL", motor4Status);

    SmartDashboard.putData("MOTOR 1 INPUT", motor1Joystick);
    SmartDashboard.putData("MOTOR 2 INPUT", motor2Joystick);
    SmartDashboard.putData("MOTOR 3 INPUT", motor3Joystick);
    SmartDashboard.putData("MOTOR 4 INPUT", motor4Joystick);

    SmartDashboard.putNumber("MOTOR 1 SPEED", 0);
    SmartDashboard.putNumber("MOTOR 2 SPEED", 0);
    SmartDashboard.putNumber("MOTOR 3 SPEED", 0);
    SmartDashboard.putNumber("MOTOR 4 SPEED", 0);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double joystickValue = MathUtil.applyDeadband(oi.getY(), .1);
    SmartDashboard.putNumber("JOYSTICK", joystickValue);

    double motor1Input = SmartDashboard.getNumber("MOTOR 1 SPEED", 0); 
    double motor2Input = SmartDashboard.getNumber("MOTOR 2 SPEED", 0); 
    double motor3Input = SmartDashboard.getNumber("MOTOR 3 SPEED", 0);
    double motor4Input = SmartDashboard.getNumber("MOTOR 4 SPEED", 0);
    
    if (motor1Status.getSelected() == 1) {
      if (motor1Joystick.getSelected()) {
        motor1.setPercentOutput(joystickValue);
      } else {
        motor1.setPercentOutput(motor1Input);
      }

    } else if (motor1Status.getSelected() == 2) {

      if (motor1Joystick.getSelected()) {
        motor1.setVelocity(joystickValue);
      } else {
        motor1.setVelocity(motor1Input);
      }

    } else {

      motor1.setPercentOutput(0);
    }
    

    if (motor2Status.getSelected() == 1) {
      if (motor2Joystick.getSelected()) {
        motor2.setPercentOutput(joystickValue);
      } else {
        motor2.setPercentOutput(motor2Input);
      }
      
    } else if (motor2Status.getSelected() == 2) {

      if (motor2Joystick.getSelected()) {
        motor2.setVelocity(joystickValue);
      } else {
        motor2.setVelocity(motor1Input);
      }

    } else {
        motor2.setPercentOutput(0);
    }


    if (motor3Status.getSelected() == 1) {
      if (motor3Joystick.getSelected()) {
        motor3.setPercentOutput(joystickValue);
      } else {
        motor3.setPercentOutput(motor3Input);
      }
    } else if (motor3Status.getSelected() == 2) {

      if (motor3Joystick.getSelected()) {
        motor3.setVelocity(joystickValue);
      } else {
        motor3.setVelocity(motor1Input);
      }

    } else {
        motor3.setPercentOutput(0);
    }


    if (motor4Status.getSelected() == 1) {
      if (motor4Joystick.getSelected()) {
        motor4.setPercentOutput(joystickValue);
      } else {
        motor4.setPercentOutput(motor4Input);
      }
    } else if (motor4Status.getSelected() == 2) {

      if (motor4Joystick.getSelected()) {
        motor4.setVelocity(joystickValue);
      } else {
        motor4.setVelocity(motor1Input);
      }

    } else {
        motor4.setPercentOutput(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
