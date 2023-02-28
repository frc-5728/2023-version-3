package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.commands.TankDrive;

public class DriveTrain extends SubsystemBase {
    // motors
    private CANSparkMax m0CanSparkMax = new CANSparkMax(RobotMap.MOTOR_LEFT0_ID, MotorType.kBrushless);
    private CANSparkMax m1CanSparkMax = new CANSparkMax(RobotMap.MOTOR_LEFT1_ID, MotorType.kBrushless);

    private CANSparkMax m2CanSparkMax = new CANSparkMax(RobotMap.MOTOR_RIGHT0_ID, MotorType.kBrushless);
    private CANSparkMax m3CanSparkMax = new CANSparkMax(RobotMap.MOTOR_RIGHT1_ID, MotorType.kBrushless);

    // drive trains
    // these shit didn't work; don't use them
    private final MotorControllerGroup leftMotors = new MotorControllerGroup(m0CanSparkMax, m1CanSparkMax);
    private final MotorControllerGroup rightMotors = new MotorControllerGroup(m2CanSparkMax, m3CanSparkMax);

    private final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

    // now for the encoders
    
    // https://www.youtube.com/watch?v=_mKlRbapkXo&ab_channel=EastRobotics
    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    public final PIDController turnController;

    // usually paired together to access the robot's locaition/physics info
    // private final DifferentialDriveOdometry odometry;
    private final DifferentialDriveKinematics kinematics;

    AHRS gyro = new AHRS();

    public DriveTrain() {
        
        kinematics = new DifferentialDriveKinematics(0.5);

        turnController = new PIDController(kP, kI, kD);
        turnController.enableContinuousInput(-180.0f, -180.0f);

        setDefaultCommand(new TankDrive(this));
    }

    public void pidWrite(double output) {
        setSpeed(output);
    }

    public void setSpeed(double speed) {
        // give the speed in meters per sec written in chassis
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(speed, speed, 0);

        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

        // drive.tankDrive(wheelSpeeds.leftMetersPerSecond,
        // wheelSpeeds.rightMetersPerSecond);
        setLeftSpeed(wheelSpeeds.leftMetersPerSecond);
        setRightSpeed(wheelSpeeds.rightMetersPerSecond);
    }

    public void setLeftSpeed(double speed) {
        m0CanSparkMax.set(speed);
        m1CanSparkMax.set(speed);
    }

    public void setRightSpeed(double speed) {
        m2CanSparkMax.set(-speed);
        m3CanSparkMax.set(-speed);
    }

    @Override
    public void periodic() {
        
        // we dont have encoders

        // odometry.update(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
    }
}
