package org.firstinspires.ftc.PinkCode.Subsystems;

// Abstract Class to Define the Methods of the Extender Subsystem
public abstract class Scorer extends Subsystem {
    // Method for Rotating the Scoring Bucket to a Position
    public static void score_rotate_to_position(double position) {
        scoreL_rotate_command = position;
        scoreR_rotate_command = position;
    }

    public static void score_collect(double position)
    {
        score_collect_position = position;
    }

    public static void score_cap(double position) {
        score_cap_position = position;
    }
    public static void sideClaw(double position) {
        sideClaw_rotate_pos = position;
    }
    public static void sideFlap(double position) {
        sideFlap_rotate_pos = position;
    }
}