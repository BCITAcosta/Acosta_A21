package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.DriveConstants;

public class DriveHalf {
  private final CANSparkMax motor_FrontDriveMotor;
  private final CANSparkMax motor_RearDriveMotor;
  private final Encoder encoder_DriveEncoder;
  //private final String halfID;

  public DriveHalf(String halfID, int frontMotorSparkMaxID, int rearMotorSparkMaxID, int EncoderA, int EncoderB) {
    //this.halfID = halfID;
    motor_FrontDriveMotor = new CANSparkMax(frontMotorSparkMaxID, MotorType.kBrushless);
    motor_RearDriveMotor = new CANSparkMax(rearMotorSparkMaxID, MotorType.kBrushless);
    encoder_DriveEncoder = new Encoder(EncoderA, EncoderB, false, Encoder.EncodingType.k4X);
  }

  public void configureMotors(){
    motor_FrontDriveMotor.setSmartCurrentLimit(DriveConstants.DriveCurrentLimit);
    motor_FrontDriveMotor.setIdleMode(IdleMode.kCoast);

    motor_RearDriveMotor.setSmartCurrentLimit(DriveConstants.DriveCurrentLimit);
    motor_RearDriveMotor.setIdleMode(IdleMode.kCoast);
  }

  public void configureEncoder(){
    encoder_DriveEncoder.setDistancePerPulse(12.56637061/2048.);
  }

  public void setSpeed(double speed){
    motor_FrontDriveMotor.set(speed);
    motor_RearDriveMotor.set(speed);
  }

  public int getEncoderValue(){
    return encoder_DriveEncoder.getRaw();
  }

  public double getEncoderDist(){
    return encoder_DriveEncoder.getDistance();
  }

  public void setInverted(){
    if (motor_FrontDriveMotor.getInverted()){
      motor_FrontDriveMotor.setInverted(false);
    }else{
      motor_FrontDriveMotor.setInverted(true);
    }

    if (motor_RearDriveMotor.getInverted()){
      motor_RearDriveMotor.setInverted(false);
    }else{
      motor_RearDriveMotor.setInverted(true);
    }

    encoder_DriveEncoder.setReverseDirection(true);
  }
}
