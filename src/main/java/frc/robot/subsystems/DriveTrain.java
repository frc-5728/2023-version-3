package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {    
    CANSparkMax m0CanSparkMax = new CANSparkMax(0, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax m1CanSparkMax = new CANSparkMax(1, MotorType.kBrushless);
    

    private final DifferentialDriveOdometry odometry;
    
    AHRS gyro = new AHRS();

    // parameters
    

    public DriveTrain() {
        odometry = new DifferentialDriveOdometry(gyro.getAngle(), leftEncoder, rightEncoder);
    }
    
    @Override
    public void periodic() {
        
    }
}

