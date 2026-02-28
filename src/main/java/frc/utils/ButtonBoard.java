package frc.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Custom GenericHID class for team 3128's button board setup
 */
public class ButtonBoard {

    private GenericHID device;

    private Trigger[] triggers;

    /**
     * Creates a new button board object.
     * @param port The port on DriverStation the button board is connected to.
     */
    public ButtonBoard(int port) {
        triggers = new Trigger[17];
        device = new GenericHID(port);

        // 2023 buttonBoard has 12 buttons & 4 axis direction
        for (int i = 1; i <= 12; i++) {
            int buttonId = i;
            triggers[buttonId] = new Trigger(() -> device.getRawButton(buttonId)); 
        }
        //For Joystick
        triggers[13] = new Trigger(() -> device.getRawAxis(0) == -1.0);
        triggers[14] = new Trigger(() -> device.getRawAxis(0) == 1.0);
        triggers[15] = new Trigger(() -> device.getRawAxis(1) == 1.0);
        triggers[16] = new Trigger(() -> device.getRawAxis(1) == -1.0);
            
    }


    /**
   * Constructs a Trigger instance around the trigger1 button's digital signal.
   *
   * @return a Trigger instance representing the trigger1 button's digital signal.
   */
    public Trigger getTrigger(int buttonId) {
        return triggers[buttonId];  
    }
    /**
     * Gets the boolean value of the button
     * @return a boolean value representing the state of the button.
     */
    public boolean getButton(int buttonId) {
        return device.getRawButton(buttonId);
    }

    public void setButton(int buttonId, boolean value) {
        device.setOutput(buttonId, value);
    }
    /**
   * Constructs a Trigger instance around the b1 button's digital signal.
   *
   * @return a Trigger instance representing the b1 button's digital signal.
   */
    public Trigger b1() {
        return triggers[1];
    }
    /**
   * Constructs a Trigger instance around the b2 button's digital signal.
   *
   * @return a Trigger instance representing the b2 button's digital signal.
   */
    public Trigger b2() {
        return triggers[2];  
    }
    /**
   * Constructs a Trigger instance around the b3 button's digital signal.
   *
   * @return a Trigger instance representing the b3 button's digital signal.
   */
    public Trigger b3() {
        return triggers[3];  
    }
    /**
   * Constructs a Trigger instance around the b4 button's digital signal.
   *
   * @return a Trigger instance representing the b4 button's digital signal.
   */
    public Trigger b4() {
        return triggers[4];  
    }
    /**
   * Constructs a Trigger instance around the b5 button's digital signal.
   *
   * @return a Trigger instance representing the b5 button's digital signal.
   */
    public Trigger b5() {
        return triggers[5];  
    }
    /**
   * Constructs a Trigger instance around the b6 button's digital signal.
   *
   * @return a Trigger instance representing the b6 button's digital signal.
   */
    public Trigger b6() {
        return triggers[6];  
    }
    /**
   * Constructs a Trigger instance around the b7 button's digital signal.
   *
   * @return a Trigger instance representing the b7 button's digital signal.
   */
    public Trigger b7() {
        return triggers[7];  
    }
    /**
   * Constructs a Trigger instance around the b8 button's digital signal.
   *
   * @return a Trigger instance representing the b8 button's digital signal.
   */
    public Trigger b8() {
        return triggers[8];  
    }
    /**
   * Constructs a Trigger instance around the b9 button's digital signal.
   *
   * @return a Trigger instance representing the b9 button's digital signal.
   */
    public Trigger b9() {
        return triggers[9];  
    }
    /**
   * Constructs a Trigger instance around the b10 button's digital signal.
   *
   * @return a Trigger instance representing the b10 button's digital signal.
   */
    public Trigger b10() {
        return triggers[10];  
    }
    /**
   * Constructs a Trigger instance around the b11 button's digital signal.
   *
   * @return a Trigger instance representing the b11 button's digital signal.
   */
    public Trigger b11() {
        return triggers[11];  
    }
    /**
   * Constructs a Trigger instance around the b12 button's digital signal.
   *
   * @return a Trigger instance representing the b12 button's digital signal.
   */
    public Trigger b12() {
        return triggers[12];  
    }

}
