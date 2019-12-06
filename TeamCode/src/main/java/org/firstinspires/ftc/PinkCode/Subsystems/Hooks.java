package org.firstinspires .ftc.PinkCode.Subsystems;

public class Hooks extends Subsystem {
    // Method for Rotating the Scoring Bucket to a Position
    public static void hook_rotate_down_position() {
        // Define Commands
        hook_left_target_position = 1;
        hook_right_target_position = 0;
    }

    public static void hook_rotate_up_position() {
        hook_left_target_position = 0;
        hook_right_target_position = 1;
    }

}