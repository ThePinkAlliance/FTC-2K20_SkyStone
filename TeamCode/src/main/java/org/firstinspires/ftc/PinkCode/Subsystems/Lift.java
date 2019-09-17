package org.firstinspires.ftc.PinkCode.Subsystems;

import org.firstinspires.ftc.PinkCode.Calculations.PD;

// Abstract Class to Define the Methods of the Lift Subsystem
public abstract class Lift extends Subsystem {
    // Method for Raising/Lowering the Lift Using Commands
    public static void lift_by_command(double command) {
        // Define Commands
        lift_command = command;
    }

    // Method for Controlling the Lift Using Positions
    public static void lift_to_position(double position) {
        // Define Commands
        lift_target_position = position;
        lift_hold_position = robot.right_lift.getCurrentPosition();
        lift_error = lift_target_position - lift_hold_position;
        lift_speed = lift_hold_position - previous_lift_position;
        previous_lift_position = lift_hold_position;
        lift_command = PD.get_lift_command(lift_error, lift_speed);
    }

    // Method for Stopping the Lift
    public static void lift_stop() {
        // Define Commands
        lift_command = 0;
    }
}