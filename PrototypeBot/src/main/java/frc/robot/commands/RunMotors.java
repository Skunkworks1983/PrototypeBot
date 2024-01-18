// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;

public class RunMotors extends Command {

  private final SendableChooser<Boolean> motorType = new SendableChooser<>();
  private final SendableChooser<Integer> motor1 = new SendableChooser<>();
  private final SendableChooser<Integer> motor2 = new SendableChooser<>();
  private final SendableChooser<Integer> motor3 = new SendableChooser<>();
  private final SendableChooser<Integer> motor4 = new SendableChooser<>();

  boolean selected;
  Drivebase drivebase;

  public RunMotors(Drivebase drivebase, OI oi) {

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    motorType.addOption("Talons", true);
    motorType.addOption("Sparks", false);

    selected = motorType.getSelected();
    drivebase = Drivebase.getInstance(selected);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

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
