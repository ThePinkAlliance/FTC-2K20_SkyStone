package org.firstinspires.ftc.PinkCode.Subsystems;

// Abstract Class to Define the Methods of the Extender Subsystem
public abstract class Scorer extends Subsystem {
    // Method for Rotating the Scoring Bucket to a Position
    public static void score_rotate_to_position(double position) {
        // Define Commands
        score_target_position = position;
    }

    public static void score_collect(double position)
    {
        score_collect_position = position;
    }

}