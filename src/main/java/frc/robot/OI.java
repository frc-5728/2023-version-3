package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Move;

public class OI extends XboxController {
    // declares buttons for binding commands (adding event listeners)
    Trigger xButton = new JoystickButton(this, XboxController.Button.kX.value);
    
    // a quick convention thing where all the controller codes should be put here
    public OI() {
        super(RobotMap.XBOXCONTROLLER_ID);

        xButton.onTrue(new Move(2));
    }
}
