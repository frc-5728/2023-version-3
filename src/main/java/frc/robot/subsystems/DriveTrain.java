package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;

public class DriveTrain {
    CANSparkMax leftSparkMax = new CANSparkMax(0, MotorType.kLeft);
    CANSparkMax rightSparkMax = new CANSparkMax(1, MotorType.kRight);
}
