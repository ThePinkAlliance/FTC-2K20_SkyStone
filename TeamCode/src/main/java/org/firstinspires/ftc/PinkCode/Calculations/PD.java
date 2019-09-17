package org.firstinspires.ftc.PinkCode.Calculations;

import com.qualcomm.robotcore.util.Range;

// Abstract Class to Find the Motor Commands for Each Subsystem Using a PD
public abstract class PD {
    // Use a PD to Determine the Lift Motor Command
    public static double get_lift_command (double error, double currentVel) {

        // Define Variables
        double lift_command;

        // Calculate Motor Command
        lift_command = (Presets.LIFT_Kp * error) - (Presets.LIFT_Kd * currentVel);
        lift_command = Range.clip(lift_command, Presets.LIFT_MIN_POWER, Presets.LIFT_MAX_POWER);

        // Return Value
        return lift_command;
    }
}