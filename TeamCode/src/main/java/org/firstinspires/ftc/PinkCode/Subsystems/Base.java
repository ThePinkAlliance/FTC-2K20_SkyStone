package org.firstinspires.ftc.PinkCode.Subsystems;

// Abstract Class to Define the Methods of the Base Subsystem
public abstract class Base extends Subsystem {
    // Method for Driving in Tank Drive using Commands
    public static void drive_by_command(boolean strafe, double rightF,double rightB, double leftF,double leftB)
    {
        // Define Commands
        if(strafe)
        {
            front_right_wheel_command = -rightF;
            back_right_wheel_command = rightB;
            front_left_wheel_command = leftF;
            back_left_wheel_command = -leftB;
        }
        else {
            front_right_wheel_command = rightF;
            back_right_wheel_command = rightB;
            front_left_wheel_command = leftF;
            back_left_wheel_command = leftB;
        }
    }
    // Method for Stopping the Drive Train
    public static void drive_stop() {
        // Define Commands
        front_right_wheel_command = 0;
        back_right_wheel_command = 0;
        front_left_wheel_command = 0;
        back_left_wheel_command = 0;
    }
}