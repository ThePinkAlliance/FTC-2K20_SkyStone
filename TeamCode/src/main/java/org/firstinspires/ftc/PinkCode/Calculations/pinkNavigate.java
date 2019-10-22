package org.firstinspires.ftc.PinkCode.Calculations;

import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.PinkCode.Robot.Hardware;

public class pinkNavigate {
    static final double COUNTS_PER_INCH = 49.8; // Counts
    static final double POSITION_THRESHOLD = 1.0;   // Base travel
    static final double ANGLE_THRESHOLD = 4.0;     // Degrees
    Hardware robot;
    static double leftMotorCmd, rightMotorCmd;

    // Tank drive two wheels to target positions in inches.
    // Returns true when both arrive at the target.
    public static boolean driveToPos (double targetPosInches, double targetAngleDeg, double currentBasePosCounts, double currentAngleDeg,
                                      double linearSpeedCounts, double maxPower)
    {
        double angleErrorDegrees = targetAngleDeg - currentAngleDeg;
        double currentPosInches = (currentBasePosCounts / COUNTS_PER_INCH);
        double linearSpeedInches = linearSpeedCounts / COUNTS_PER_INCH;
        double angleOffset;
        double linearError = targetPosInches - currentPosInches;
        double angularError = targetAngleDeg - currentAngleDeg;
        double motorCmd = PD.getMotorCmd(0.05, 0.1, linearError, linearSpeedInches);

        // Determine the baseline motor speed command, but limit it to leave room for the turn offset
        motorCmd = Range.clip(motorCmd, -0.6, 0.6);

        // Determine and add the angle offset
        angleOffset = PD.getMotorCmd(0.03, 0.001, angularError, 0);
        leftMotorCmd = motorCmd - angleOffset;
        rightMotorCmd = motorCmd + angleOffset;
        leftMotorCmd = Range.clip(leftMotorCmd, -1.0, 1.0);
        rightMotorCmd = Range.clip(rightMotorCmd, -1.0, 1.0);

        // Limit the max motor command for gentle motion
        leftMotorCmd = Range.clip(leftMotorCmd, -maxPower, maxPower);
        rightMotorCmd = Range.clip(rightMotorCmd, -maxPower, maxPower);

        // True if navigated to position
        return (Math.abs(linearError) < POSITION_THRESHOLD) && (Math.abs(angleErrorDegrees) < ANGLE_THRESHOLD);
    }

    public void stopBase ()
    {
        leftMotorCmd = 0;
        rightMotorCmd = 0;
    }

    public static double getRightMotorCmd ()
    {
        return rightMotorCmd;
    }

    public static double getLeftMotorCmd ()
    {
        return leftMotorCmd;
    }
}