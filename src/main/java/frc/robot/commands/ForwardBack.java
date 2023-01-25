package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ForwardBack extends SequentialCommandGroup{

    public ForwardBack(Drivetrain sub_Drivetrain){
        addCommands(
            new DriveDistance(sub_Drivetrain, 0.25, 24.0),
            new DriveDistance(sub_Drivetrain, -0.25, -24.0));
    } 
}
