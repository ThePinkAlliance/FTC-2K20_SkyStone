package org.firstinspires.ftc.PinkCode.Subsystems;

// Abstract Class to Define the Methods of the Extender Subsystem
public abstract class Scorer extends Subsystem {
    // Method for Rotating the Scoring Bucket to a Position
    public static void score_rotate_to_position(double position) {
        // Define Commands
        score_target_position = position;
    }

    // Method for Rotating the Scoring Bucket Using Commands
    public static void score_rotate_by_command(double command) {
        // Define Commands
        score_target_position = command;
    }

}