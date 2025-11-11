package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class XBOXController {
    private XboxController controller = new XboxController(0);

    public boolean getControllerXButton() {
        return(controller.getLeftStickButton());
    }

    public double driveX() {
        if (Math.abs(controller.getLeftX()) >= 0.1) {
            return(controller.getLeftX() * 0.2);
        } else {
            return(0);
        }
    }

    public double driveY() {
        if (Math.abs(controller.getLeftY()) >= 0.1) {
            return(controller.getLeftY() * 0.2);
        } else {
            return(0);
        }
    }

    public double rotate() {
        if (Math.abs(controller.getRightX()) >= 0.1) {
            return(controller.getRightX() * 0.5);
        } else {
            return(0);
        }
    }
}
