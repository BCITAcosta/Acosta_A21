package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.DriveConstants;

public class DriveHalf {
  private final CANSparkMax motor_FrontDriveMotor;
  private final CANSparkMax motor_RearDriveMotor;
  private final Encoder encoder_DriveEncoder;
  private final RelativeEncoder  encoder_motor_FrontDrive;
  private final RelativeEncoder encoder_motor_RearDrive;
  //private final String halfID;

  public DriveHalf(String halfID, int frontMotorSparkMaxID, int rearMotorSparkMaxID, int EncoderA, int EncoderB) {
    //this.halfID = halfID;
    motor_FrontDriveMotor = new CANSparkMax(frontMotorSparkMaxID, MotorType.kBrushless);
    motor_RearDriveMotor = new CANSparkMax(rearMotorSparkMaxID, MotorType.kBrushless);
    encoder_DriveEncoder = new Encoder(EncoderA, EncoderB, false, Encoder.EncodingType.k4X);
    encoder_motor_FrontDrive = motor_FrontDriveMotor.getEncoder();
    encoder_motor_RearDrive = motor_RearDriveMotor.getEncoder();

  }

  public void configureMotors(){
    motor_FrontDriveMotor.setSmartCurrentLimit(DriveConstants.DriveCurrentLimit);
    motor_RearDriveMotor.setSmartCurrentLimit(DriveConstants.DriveCurrentLimit);
    setIdleCoast();
  }

  public double getMotorFrontEncoderVal(){
    return encoder_motor_FrontDrive.getPosition();
  }

  public double getMotorRearEncoderVal(){
    return encoder_motor_FrontDrive.getPosition();
  }

  public double getMotorFrontEncoderDist(){
    return (2*Math.PI*DriveConstants.WheelRadius*(encoder_motor_FrontDrive.getPosition()/DriveConstants.DriveGearboxRatio));
  }

  public double getMotorRearEncoderDist(){
    return (2*Math.PI*DriveConstants.WheelRadius*(encoder_motor_RearDrive.getPosition()/DriveConstants.DriveGearboxRatio));
  }

  public void setIdleCoast(){
    motor_FrontDriveMotor.setIdleMode(IdleMode.kCoast);
    motor_RearDriveMotor.setIdleMode(IdleMode.kCoast);
  }

  public void setIdleBrake(){
    motor_FrontDriveMotor.setIdleMode(IdleMode.kBrake);
    motor_RearDriveMotor.setIdleMode(IdleMode.kBrake);
  }

  public void configureEncoder(boolean reverse){
    encoder_DriveEncoder.setDistancePerPulse(12.56637061/2048.);
    encoder_DriveEncoder.setReverseDirection(reverse);
  }

  public void setSpeed(double speed){
    motor_FrontDriveMotor.set(speed);
    motor_RearDriveMotor.set(speed);
  }

  public void resetEncoderValue(){
    encoder_DriveEncoder.reset();
  }

  public int getEncoderValue(){
    return encoder_DriveEncoder.getRaw();
  }

  public double getEncoderDist(){
    return encoder_DriveEncoder.getDistance();
  }

  public boolean getEncoderDir(){
    return encoder_DriveEncoder.getDirection();
  }

  public void setInverted(boolean invert){
    if (motor_FrontDriveMotor.getInverted() && invert){
      motor_FrontDriveMotor.setInverted(true);
    }else{
      motor_FrontDriveMotor.setInverted(false);
    }

    if (motor_RearDriveMotor.getInverted() && invert){
      motor_RearDriveMotor.setInverted(true);
    }else{
      motor_RearDriveMotor.setInverted(false);
    }
  }
}
