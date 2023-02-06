package frc.robot.subsystems;

import frc.robot.Constants.DriveConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase{
    private final DriveHalf drive_Left = new DriveHalf("Left", DriveConstants.FrontLeftSparkMaxID, DriveConstants.RearLeftSparkMaxID, DriveConstants.LeftEncoderA, DriveConstants.LeftEncoderB);
    private final DriveHalf drive_Right = new DriveHalf("Right", DriveConstants.FrontRightSparkMaxID, DriveConstants.RearRightSparkMaxID, DriveConstants.RightEncoderA, DriveConstants.RightEncoderB);

    private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(23.5));
    private final DifferentialDriveOdometry m_odometry;    
    private final ADIS16470_IMU imu = new ADIS16470_IMU();

    public Drivetrain(){
        configureDrive();

        m_odometry = new DifferentialDriveOdometry(getRotation(), drive_Left.getMotorFrontEncoderDistMeters(), drive_Right.getMotorFrontEncoderDistMeters());
    }
    
    public void configureDrive(){
        drive_Left.configureMotors();
        drive_Right.configureMotors();
        drive_Right.setInverted(false);
        drive_Left.setInverted(false);
        drive_Left.configureEncoder(false);
        drive_Right.configureEncoder(true);
    }

    public void setSpeeds(double leftSpeed, double rightSpeed){
        drive_Left.setSpeed(leftSpeed);
        drive_Right.setSpeed(-rightSpeed);
    }

    public void setDriveCoast(){
        drive_Left.setIdleCoast();
        drive_Right.setIdleCoast();
    }

    public void setDriveBrake(){
        drive_Left.setIdleBrake();
        drive_Right.setIdleBrake();
    }

    public void stop(){
        setSpeeds(0, 0);
    }

    public void resetEncoders(){
        drive_Left.resetEncoderValue();
        drive_Right.resetEncoderValue();
    }

    public double getHeading(){
        return (Math.floorMod((long) imu.getAngle(), (long) 360));
    }

    public double getTurnRate(){
        return imu.getRate();
    }

    public Rotation2d getRotation(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public void resetOdometry(Pose2d pose){
        m_odometry.resetPosition(getRotation(), drive_Left.getMotorFrontEncoderDistMeters(), drive_Right.getMotorFrontEncoderDistMeters(), pose);
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative){
        
    }

    public boolean DriveDistance(double speed, double distance){
    
        if (distance < 0){
            setSpeeds(-speed, -speed);
        }else{
            setSpeeds(speed, speed);
        }
        
        // setSpeeds(speed, speed);

        if (Math.abs(drive_Left.getEncoderDist()) >= Math.abs(distance) || 
        Math.abs(drive_Right.getEncoderDist()) >= Math.abs(distance)){
            stop();
            return true;
        }
        return false;
    }

    @Override
    public void periodic(){
        
        // SmartDashboard.putNumber("Left Encoder",drive_Left.getEncoderValue());
        // SmartDashboard.putNumber("Right Encoder", drive_Right.getEncoderValue());
        // SmartDashboard.putNumber("Left Distance", drive_Left.getEncoderDist());
        // SmartDashboard.putNumber("Right Distance", drive_Right.getEncoderDist());
        // SmartDashboard.putBoolean("Left Direction", drive_Left.getEncoderDir());
        // SmartDashboard.putBoolean("Right Direction", drive_Right.getEncoderDir());

        // SmartDashboard.putNumber("Left Motor Encoder", drive_Left.getMotorEncoderVal());
        // SmartDashboard.putNumber("Right Motor Encoder", drive_Right.getMotorEncoderVal());

        // SmartDashboard.putNumber("LF Motor Encoder", drive_Left.getMotorFrontEncoderDistMeters());
        // SmartDashboard.putNumber("LR Motor Encoder", drive_Left.getMotorRearEncoderDistMeters());
        // SmartDashboard.putNumber("RF Motor Encoder", drive_Right.getMotorFrontEncoderDistMeters());
        // SmartDashboard.putNumber("RR Motor Encoder", drive_Right.getMotorRearEncoderDistMeters());

        // SmartDashboard.putNumber("Angle", getHeading());        

        m_odometry.update(getRotation(), drive_Left.getMotorFrontEncoderDistMeters(), drive_Right.getMotorFrontEncoderDistMeters());
    }
}