// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ArmTeleOp;
import frc.robot.commands.Autos;
import frc.robot.commands.DrawerInOut;
import frc.robot.commands.DropCone;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ElevatorTeleOp;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.HatchMechanismCommand;
import frc.robot.commands.Move;
import frc.robot.commands.MoveEncoder;
import frc.robot.commands.TankDrive;
import frc.robot.commands.TurnLeft;
import frc.robot.commands.TurnRight;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmProfiledPID;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.HatchMechanism;
import frc.robot.subsystems.Vision.ReflectiveTapeSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ReflectiveTapeSubsystem m_rtSubsystem = new ReflectiveTapeSubsystem();

  public final DriveTrain driveTrain = new DriveTrain();
  private final Elevator elevator = new Elevator();
  private final Arm arm = new Arm();
  private final HatchMechanism hatchMechanism = new HatchMechanism();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController controller =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

      private final Joystick joystick = new Joystick(RobotMap.JOYSTICK_BUTTON_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    SmartDashboard.putData("Drop Cone", new DropCone(m_rtSubsystem, driveTrain));

  }

  private void configureBindingsDriveTrain() {
    controller.start().onTrue(new TankDrive(driveTrain));
    
    controller.leftBumper().onTrue(new TurnLeft(driveTrain, 45));
    controller.rightBumper().onTrue(new TurnRight(driveTrain, 45));

    controller.povUp().onTrue(new MoveEncoder(driveTrain, 1));
    // controller.povUp().onTrue(new Move(0.01, driveTrain));
    // controller.povDown().onTrue(new Move(-0.01, driveTrain));
  }

  private void configureBindingsFeatures() {
    // controller.povUp().onTrue();
    controller.povLeft().onTrue(new DrawerInOut(arm, -2));
    controller.povRight().onTrue(new DrawerInOut(arm, 2));

    controller.b().onTrue(new ArmCommand(arm, 180));
    // elevator.enable();

    controller.a().onTrue(new HatchMechanismCommand(hatchMechanism));

    JoystickButton triggerButton = new JoystickButton(joystick, RobotMap.JOYSTICK_TRIGGER_ID);
    triggerButton.onTrue(new HatchMechanismCommand(hatchMechanism));


    JoystickButton upJoystickButton = new JoystickButton(joystick, RobotMap.ELEVATOR_UP_JOYSTICK);
    JoystickButton downJoystickButton = new JoystickButton(joystick, RobotMap.ELEVATOR_DOWN_JOYSTICK);

    upJoystickButton.onTrue(new ElevatorCommand(elevator, 0));

    JoystickButton armUpJoystickButton = new JoystickButton(joystick, RobotMap.ARM_UP_JOYSTICK);
    JoystickButton armDownJoystickButton = new JoystickButton(joystick, RobotMap.ARM_DOWN_JOYSTICK);

    JoystickButton resetJoystickArmButtons = new JoystickButton(joystick, RobotMap.RESET_ARM_JOYSTICK);
    JoystickButton resetJoystickElevatorButtons = new JoystickButton(joystick, RobotMap.RESET_ELEVATOR_JOYSTICK);
    resetJoystickArmButtons.onTrue(new ArmTeleOp(arm));

    resetJoystickElevatorButtons.onTrue(new ElevatorTeleOp(elevator));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // bind the triggers to the commands here
    configureBindingsDriveTrain();
    configureBindingsFeatures();
    
    // xButtonTrigger.onTrue(); // some elevator command activation here probs

    
    // these below are boilder plate codes
    
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
        // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // controller.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // this function is hooked up and called in robot.java

    // An example command will be run in autonomous
    return Autos.sequence(m_exampleSubsystem);
  }
}
