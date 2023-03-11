package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    public final PIDController pidController;

    // this gives the cirumference of the wheel
    private final double wheelCircum = Units.inchesToMeters(6) * Math.PI; 

    // usually paired together to access the robot's locaition/physics info
    // private final DifferentialDriveOdometry odometry;
    private final DifferentialDriveKinematics kinematics;
    private final DifferentialDriveOdometry odometry;

    // the encoders 
    public final RelativeEncoder leftEncoder = m0CanSparkMax.getEncoder();
    public final RelativeEncoder rightEncoder = m2CanSparkMax.getEncoder();
    
    public final AHRS gyro = new AHRS();

    public DriveTrain() {
        // leftEncoder.setPositionConversionFactor((double) 1.0/wheelCircum);
        // rightEncoder.setPositionConversionFactor((double) 1.0/wheelCircum);
        // leftEncoder.setVelocityConversionFactor((double) 1.0/wheelCircum);
        // rightEncoder.setVelocityConversionFactor((double) 1.0/wheelCircum);
        
        kinematics = new DifferentialDriveKinematics(0.5);
        odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());

        pidController = new PIDController(kP, kI, kD);

        setDefaultCommand(new TankDrive(this));
    }

    public void pidWrite(double output) {
        setSpeed(output);
    }

    public void setSpeed(double speed) {
        setLeftSpeed(speed);
        setRightSpeed(speed);
    }

    public void setLeftSpeed(double speed) {
        m0CanSparkMax.set(speed);
        m1CanSparkMax.set(speed);
    }

    public void setRightSpeed(double speed) {
        m2CanSparkMax.set(-speed);
        m3CanSparkMax.set(-speed);
    }

    // for accessing values in the odemetry

    @Override
    public void periodic() {
        // displaying data
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drive");
        
        SmartDashboard.putNumber("Gyro Displacement X: ", gyro.getDisplacementX());
        SmartDashboard.putNumber("Gyro Displacement Y: ", gyro.getDisplacementY());

        SmartDashboard.putNumber("Pitch", gyro.getPitch());
        SmartDashboard.putNumber("Yaw", gyro.getYaw());
        SmartDashboard.putNumber("Angle", gyro.getAngle());

        SmartDashboard.putNumber("left Encoder Position", leftEncoder.getPosition());
        SmartDashboard.putNumber("right Encoder Position", rightEncoder.getPosition());

        odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    }
}
