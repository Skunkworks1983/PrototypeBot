// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.OI;

public class RunMotors extends Command {

  Drivebase drivebase;
  OI oi;

  SendableChooser<Integer> motorType = new SendableChooser<>();
  SendableChooser<Integer> controlMode = new SendableChooser<>();
  SendableChooser<Boolean> motor1 = new SendableChooser<>();
  SendableChooser<Boolean> motor2 = new SendableChooser<>();
  SendableChooser<Boolean> motor3 = new SendableChooser<>();
  SendableChooser<Boolean> motor4 = new SendableChooser<>();

  int motorTypeSelected;
  boolean motor1Selected;
  boolean motor2Selected;
  boolean motor3Selected;
  boolean motor4Selected;

  public RunMotors(Drivebase drivebase, OI oi) {

    this.drivebase = drivebase;
    this.oi = oi;

    addRequirements(drivebase, oi);

    motorType.setDefaultOption("Talons", 0);
    motorType.addOption("Sparks", 1);
    motorTypeSelected = motorType.getSelected();

    motor1.setDefaultOption("Run", true);
    motor1.addOption("Off", false);
    motor1Selected = motor1.getSelected();

    motor2.setDefaultOption("Run", true);
    motor2.addOption("Off", false);
    motor2Selected = motor1.getSelected();

    motor3.setDefaultOption("Run", true);
    motor3.addOption("Off", false);
    motor3Selected = motor1.getSelected();

    motor4.setDefaultOption("Run", true);
    motor4.addOption("Off", false);
    motor4Selected = motor1.getSelected();
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double joystickValue = oi.getY();
    SmartDashboard.putNumber("Joystick", joystickValue);
    double runSpeed = 0;
    SmartDashboard.putNumber("run speed", 0);

    if (MathUtil.applyDeadband(joystickValue, .1)==0) {
      double value = SmartDashboard.getNumber("run speed", 0);
      SmartDashboard.putNumber("run speed", value);
      runSpeed = value;
    }
    
    if (motor1Selected) {
      drivebase.setPercentOutput(1, runSpeed);
    }
    if (motor2Selected) {
      drivebase.setPercentOutput(2, runSpeed);
    }
    if (motor3Selected) {
      drivebase.setPercentOutput(3, runSpeed);
    }
    if (motor4Selected) {
      drivebase.setPercentOutput(4, runSpeed);
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
