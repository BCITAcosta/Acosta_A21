package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase{
    NetworkTable limelight_table;
    final DoubleSubscriber ty, tx;

    double valuex, valuey;

    public Limelight(){
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable limelight_table = inst.getTable("limelight");
        ty = limelight_table.getDoubleTopic("ty").subscribe(0.0);
        tx = limelight_table.getDoubleTopic("tx").subscribe(0.0);
    }

    @Override
    public void periodic(){
        
        double valuey = ty.get();
        double valuex = tx.get();
        SmartDashboard.putNumber("valuey", valuey);
        SmartDashboard.putNumber("valuex", valuex);

    }

}
