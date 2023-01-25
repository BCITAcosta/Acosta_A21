package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveDistance extends CommandBase{

    private final Drivetrain sub_drivetrain;
    private double speed;
    private double distance;

    public DriveDistance(Drivetrain drivetrain, double speed, double distance){
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
        
    }

    @Override
    public void end(boolean interrupted){
        sub_drivetrain.stop();
    }

    @Override
    public boolean isFinished(){
        return sub_drivetrain.DriveDistance(speed, distance);
    }
    
}
