// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveTrain;

public class TankDrive extends CommandBase {
  // used during teleop tank drive
  
  private final DriveTrain driveTrain;
  private final XboxController controller;
  
  /** Creates a new TankDrive. */
  public TankDrive(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    controller = new XboxController(OperatorConstants.kDriverControllerPort);
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // called in every periodic
    // double leftStickY = controller.getRawAxis(RobotMap.LEFT_STICK_Y);
    // double rightStickY = controller.getRawAxis(RobotMap.RIGHT_STICK_Y);

    double triggerVal = controller.getRightTriggerAxis() - controller.getLeftTriggerAxis();

    // then for the left and right turns during teleop tank drive
    // that we will use the left axis (i think its 0 as id)
    // http://www.team358.org/files/programming/ControlSystem2015-2019/images/Logitech-F310_ControlMapping.pdf
    double stick = controller.getLeftX() * RobotMap.TURNING_RATE;

    // for a slow controlling mode on the right side
    double slowTrigger = controller.getRightY() * RobotMap.SLOW_MODE_RATE;
    double slowStick = controller.getRightX() * RobotMap.SLOW_MODE_RATE;

    // this is using manning robotic's GTA drive where they control robot like the GTA game

    driveTrain.setLeftSpeed(triggerVal + stick + (slowTrigger + slowStick));
    driveTrain.setRightSpeed(triggerVal - stick + (slowTrigger - slowStick));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
