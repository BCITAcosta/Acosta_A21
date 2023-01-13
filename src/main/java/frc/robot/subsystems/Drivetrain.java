package frc.robot.subsystems;

import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase{
    private final DriveHalf drive_Left = new DriveHalf("Left", DriveConstants.FrontLeftSparkMaxID, DriveConstants.RearLeftSparkMaxID, DriveConstants.LeftEncoderA, DriveConstants.LeftEncoderB);
    private final DriveHalf drive_Right = new DriveHalf("Right", DriveConstants.FrontRightSparkMaxID, DriveConstants.RearRightSparkMaxID, DriveConstants.RightEncoderA, DriveConstants.RightEncoderB);

    public Drivetrain(){
        configureDrive();
    }
    
    public void configureDrive(){
        drive_Right.setInverted();
        //drive_Left.setInverted();
        drive_Left.configureEncoder();
        drive_Right.configureEncoder();
    }

    public void setSpeeds(double leftSpeed, double rightSpeed){
        drive_Left.setSpeed(leftSpeed);
        drive_Right.setSpeed(rightSpeed);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Left Encoder",drive_Left.getEncoderValue());
        SmartDashboard.putNumber("Right Encoder", drive_Right.getEncoderValue());
        SmartDashboard.putNumber("Left Distance", drive_Left.getEncoderDist());
        SmartDashboard.putNumber("Right Distance", drive_Right.getEncoderDist());
    }
}