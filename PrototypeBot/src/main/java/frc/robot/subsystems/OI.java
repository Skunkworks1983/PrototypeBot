// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OI extends SubsystemBase {

  private static OI oi;
  Joystick joystick;
  
  private OI() {
    joystick = new Joystick(0);
  }

  public double getY() {
    return joystick.getY();
  }

  public static OI getInstance () {
    if (oi == null) {
      oi = new OI();
    }
    return oi;
  }
}
