package org.firstinspires .ftc.PinkCode.Subsystems;

public class Hooks extends Subsystem {
    // Method for Rotating the Scoring Bucket to a Position
    public static void hook_rotate_to_position(double position) {
        // Define Commands
        hook_left_target_position = position;
        hook_right_target_position = -position;
    }

}