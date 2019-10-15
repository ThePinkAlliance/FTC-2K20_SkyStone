package org.firstinspires.ftc.PinkCode.Calculations;

// Abstract Class to Define Preset Values Used Throughout Subsystems
public abstract class Presets {
    //Collector Presets
    public static final double COLLECTOR_COLLECT_POWER = 1; // Power Sent to Motor While Collecting
    public static final double COLLECTOR_EJECT_POWER = -1; // Power Sent to Motor While Ejecting

    // Lift Presets TODO: Confirm Lift Presets
    public static final double LIFT_MAX_POSITION = 4400; //Fully Raised
    public static final double LIFT_CLEAR_POSITION = 3000;
    public static final double LIFT_STOW_POSITION = 0; // Lowered Position for Collection from Sort
    static final double LIFT_Kp = .01; // Kp for Lift PD
    static final double LIFT_Kd = 0.001; // Kd for Lift PD
    static final double LIFT_MIN_POWER = -1; // Most Power Sent While Lowering Lift
    static final double LIFT_MAX_POWER = 1; // Most Power Sent While Raising Lift

    //Scorer Presets
    public static final double SCORER_STOW = 0;
    public static final double SCORER_SCORE_POSITION = 1;
    public static final double SCORER_COLLECT = 1;
    public static final double SCORER_EJECT = 0;
}