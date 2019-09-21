package org.firstinspires.ftc.PinkCode.Subsystems;


import org.firstinspires.ftc.PinkCode.Calculations.Presets;

// Abstract Class to Define the Methods of the Collector Subsystem
public abstract class Collector extends Subsystem{
    
    // Method for Collecting
    public static void collect() {
        // Define Commands
        collect_command_left = Presets.COLLECTOR_COLLECT_POWER;
        collect_command_right = -Presets.COLLECTOR_COLLECT_POWER;
    }

    // Method for Ejecting
    public static void eject() {
        // Define Commands
        collect_command_left = Presets.COLLECTOR_EJECT_POWER;
        collect_command_right = -Presets.COLLECTOR_EJECT_POWER;
    }

    // Method for Stopping the Collector
    public static void collect_stop() {
        // Define Commands
        collect_command_left = 0;
        collect_command_right = 0;
    }
}
