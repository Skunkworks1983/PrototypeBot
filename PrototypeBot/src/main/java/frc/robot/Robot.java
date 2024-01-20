// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.RunMotors;
import frc.robot.subsystems.Motor;
import frc.robot.subsystems.OI;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  OI oi = OI.getInstance();

  SendableChooser<Integer> motor1Type = new SendableChooser<>();
  SendableChooser<Integer> motor2Type = new SendableChooser<>();
  SendableChooser<Integer> motor3Type = new SendableChooser<>();
  SendableChooser<Integer> motor4Type = new SendableChooser<>();

  SendableChooser<Integer> motor1Status = new SendableChooser<>();
  SendableChooser<Integer> motor2Status = new SendableChooser<>();
  SendableChooser<Integer> motor3Status = new SendableChooser<>();
  SendableChooser<Integer> motor4Status = new SendableChooser<>();

  Motor motor1;
  Motor motor2;
  Motor motor3;
  Motor motor4;

  double runSpeed = 0;


  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    motor1Type.setDefaultOption("Motor 1: ---", 0);
    motor1Type.addOption("Motor 1: Talon", 1);
    motor1Type.addOption("Motor 1: Spark", 2);
    motor1Type.addOption("Motor 1: Sim", 3);

    motor2Type.setDefaultOption("Motor 2: ---", 0);
    motor2Type.addOption("Motor 2: Talon", 1);
    motor2Type.addOption("Motor 2: Spark", 2);
    motor2Type.addOption("Motor 2: Sim", 3);

    motor3Type.setDefaultOption("Motor 3: ---", 0);
    motor3Type.addOption("Motor 3: Talon", 1);
    motor3Type.addOption("Motor 3: Spark", 2);
    motor3Type.addOption("Motor 3: Sim", 3);

    motor4Type.setDefaultOption("Motor 4: ---", 0);
    motor4Type.addOption("Motor 4: Talon", 1);
    motor4Type.addOption("Motor 4: Spark", 2);
    motor4Type.addOption("Motor 4: Sim", 3);

    double motor1Id = SmartDashboard.getNumber("MOTOR 1 ID", 1);
    SmartDashboard.putNumber("MOTOR 1 ID", motor1Id);

    double motor2Id = SmartDashboard.getNumber("MOTOR 2 ID", 2);
    SmartDashboard.putNumber("MOTOR 2 ID", motor2Id);

    double motor3Id = SmartDashboard.getNumber("MOTOR 3 ID", 3);
    SmartDashboard.putNumber("MOTOR 3 ID", motor3Id);

    double motor4Id = SmartDashboard.getNumber("MOTOR 4 ID", 4);
    SmartDashboard.putNumber("MOTOR 4 ID", motor4Id);

    motor1Status.setDefaultOption("Run", 0);
    motor1Status.addOption("Off", 1);

    motor2Status.setDefaultOption("Run", 0);
    motor2Status.addOption("Off", 1);

    motor3Status.setDefaultOption("Run", 0);
    motor3Status.addOption("Off", 1);
    
    motor4Status.setDefaultOption("Run", 0);
    motor4Status.addOption("Off", 1);
    
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {

    if (motor1Type.getSelected() != 0) {
      motor1 = new Motor(motor1Type.getSelected(), (int)motor1Id);
    }
    if (motor2Type.getSelected() != 0) {
      motor2 = new Motor(motor2Type.getSelected(), (int)motor2Id);
    }
    if (motor3Type.getSelected() != 0) {
      motor3 = new Motor(motor3Type.getSelected(), (int)motor3Id);
    }
    if (motor4Type.getSelected() != 0) {
      motor4 = new Motor(motor4Type.getSelected(), (int)motor4Id);
    }
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    double joystickValue = oi.getY();
    SmartDashboard.putNumber("Joystick", joystickValue);

    if (MathUtil.applyDeadband(joystickValue, .1)==0) {
      runSpeed = SmartDashboard.getNumber("run speed", 0);
      SmartDashboard.putNumber("run speed", runSpeed);
    }
    
    if (motor1Status.getSelected() == 0) {
      motor1.setPercentOutput(runSpeed);
    }
    if (motor2Status.getSelected() == 0) {
      motor2.setPercentOutput(runSpeed);
    }
    if (motor3Status.getSelected() == 0) {
      motor3.setPercentOutput(runSpeed);
    }
    if (motor4Status.getSelected() == 0) {
      motor4.setPercentOutput(runSpeed);
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
