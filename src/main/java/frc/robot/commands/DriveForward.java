package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveForward extends CommandBase{

    private final Drivetrain sub_drivetrain;

    public DriveForward(Drivetrain drivetrain){
        sub_drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        sub_drivetrain.resetEncoders();
        sub_drivetrain.setDriveBrake();
    }

    @Override
    public void execute(){
        sub_drivetrain.driveForward(0.25,24.0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}
