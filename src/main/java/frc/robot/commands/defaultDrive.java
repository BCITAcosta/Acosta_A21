// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class defaultDrive extends CommandBase {

  private final Drivetrain sub_Drivetrain;
  DoubleSupplier j_leftAxis;
  DoubleSupplier j_rightAxis;

  public defaultDrive(Drivetrain Drivetrain, DoubleSupplier leftAxis, DoubleSupplier rightAxis) {
    sub_Drivetrain = Drivetrain;
    j_leftAxis = leftAxis;
    j_rightAxis = rightAxis;
    addRequirements(sub_Drivetrain);
  }

  @Override
  public void execute() {
    sub_Drivetrain.setSpeeds(j_leftAxis.getAsDouble(), j_rightAxis.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
