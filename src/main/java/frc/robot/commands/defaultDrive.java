// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class defaultDrive extends CommandBase {

  private final Drivetrain sub_Drivetrain;
  Joystick j_leftAxis;
  Joystick j_rightAxis;

  public defaultDrive(Drivetrain Drivetrain, Joystick leftJoy, Joystick rightJoy) {
    sub_Drivetrain = Drivetrain;
    j_leftAxis = leftJoy;
    j_rightAxis = rightJoy;
    addRequirements(sub_Drivetrain);
  }

  @Override
  public void initialize(){
    sub_Drivetrain.setDriveCoast();
  }

  @Override
  public void execute() {
    sub_Drivetrain.setSpeeds(j_leftAxis.getRawAxis(1), j_rightAxis.getRawAxis(1));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
