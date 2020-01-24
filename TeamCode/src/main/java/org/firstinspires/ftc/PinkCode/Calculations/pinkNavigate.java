package org.firstinspires.ftc.PinkCode.Calculations;

import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.PinkCode.Robot.Hardware;

public class pinkNavigate {
    static final double COUNTS_PER_INCH = 24.9; // Counts Previous 49.8
    static final double POSITION_THRESHOLD = 1.0;   // Base travel
    static final double ANGLE_THRESHOLD = 4.0;     // Degrees
    Hardware robot;
    static double leftFMotorCmd, rightFMotorCmd, leftBMotorCmd, rightBMotorCmd, linearError;

    // Tank drive two wheels to target positions in inches.
    // Returns true when both arrive at the target.
    public static boolean driveToPos (double targetPosInches, double targetAngleDeg, double currentBasePosCounts, double currentAngleDeg,
                                      double linearSpeedCounts, double maxPower)
    {
        double angleErrorDegrees = targetAngleDeg - currentAngleDeg;
        double currentPosInches = (currentBasePosCounts / COUNTS_PER_INCH);
        double linearSpeedInches = linearSpeedCounts / COUNTS_PER_INCH;
        double angleOffset;
        linearError = targetPosInches - currentPosInches;
        double angularError = targetAngleDeg - currentAngleDeg;
        double motorCmd = PD.getMotorCmd(0.02, 0.07, linearError, linearSpeedInches);

        // Determine the baseline motor speed command, but limit it to leave room for the turn offset
        motorCmd = Range.clip(motorCmd, -0.6, 0.6);

        // Determine and add the angle offset
        angleOffset = PD.getMotorCmd(0.02, 0.001, angularError, 0);
        leftFMotorCmd = motorCmd - angleOffset;
        rightFMotorCmd = motorCmd + angleOffset;
        leftFMotorCmd = Range.clip(leftFMotorCmd, -1.0, 1.0);
        rightFMotorCmd = Range.clip(rightFMotorCmd, -1.0, 1.0);

        // Limit the max motor command for gentle motion
            leftFMotorCmd = Range.clip(leftFMotorCmd, -maxPower, maxPower);
            rightFMotorCmd = Range.clip(rightFMotorCmd, -maxPower, maxPower);
            leftBMotorCmd = leftFMotorCmd;
            rightBMotorCmd = rightFMotorCmd;

        // True if navigated to position
        return (Math.abs(linearError) < POSITION_THRESHOLD) && (Math.abs(angleErrorDegrees) < ANGLE_THRESHOLD);
    }

    public static boolean strafeToPos(double targetPosInches, double targetAngleDeg, double currentBasePosCounts, double currentAngleDeg,
                                      double linearSpeedCounts, double maxPower)
    {
        double angleErrorDegrees = targetAngleDeg - currentAngleDeg;
        double currentPosInches = (currentBasePosCounts / COUNTS_PER_INCH);
        double linearSpeedInches = linearSpeedCounts / COUNTS_PER_INCH;
        double angleOffset;
        linearError = targetPosInches - currentPosInches;
        double linearError2 = -linearError;
        double angularError = targetAngleDeg - currentAngleDeg;
        double motorCmd = PD.getMotorCmd(0.01, 0.05, linearError, linearSpeedInches); //kp .05: .02 KD: .1
        double motorCmd2 = PD.getMotorCmd(0.01, 0.05, linearError2, linearSpeedInches); //kp .05 .02 KD: .1

        // Determine the baseline motor speed command, but limit it to leave room for the turn offset
        motorCmd = Range.clip(motorCmd, -0.6, 0.6);

        // Determine and add the angle offset
        angleOffset = PD.getMotorCmd(0.02, 0.001, angularError, 0); //kp .03
        leftFMotorCmd = motorCmd - angleOffset;
        leftBMotorCmd = motorCmd2 - angleOffset;
        rightFMotorCmd = motorCmd2 + angleOffset;
        rightBMotorCmd = motorCmd + angleOffset;
        leftFMotorCmd = Range.clip(leftFMotorCmd, -1.0, 1.0);
        rightFMotorCmd = Range.clip(rightFMotorCmd, -1.0, 1.0);
        leftBMotorCmd = Range.clip(leftBMotorCmd, -1.0, 1.0);
        rightBMotorCmd = Range.clip(rightBMotorCmd, -1.0, 1.0);

        // Limit the max motor command for gentle motion
            leftFMotorCmd = Range.clip(leftFMotorCmd, -maxPower, maxPower);
            rightFMotorCmd = Range.clip(rightFMotorCmd, -maxPower, maxPower);
            leftBMotorCmd = Range.clip(leftBMotorCmd, -maxPower, maxPower);
            rightBMotorCmd = Range.clip(rightBMotorCmd, -maxPower, maxPower);

        // True if navigated to position
        return (Math.abs(linearError) < POSITION_THRESHOLD) && (Math.abs(angleErrorDegrees) < ANGLE_THRESHOLD);
    }

    public static double strafeByCommand(double maxPower) {

        leftFMotorCmd = -maxPower;
        leftBMotorCmd = maxPower;
        rightFMotorCmd = maxPower;
        rightBMotorCmd = -maxPower;
//        leftFMotorCmd = Range.clip(leftFMotorCmd, -maxPower, maxPower);
//        rightFMotorCmd = Range.clip(rightFMotorCmd, -maxPower, maxPower);
//        leftBMotorCmd = Range.clip(leftBMotorCmd, -maxPower, maxPower);
//        rightBMotorCmd = Range.clip(rightBMotorCmd, -maxPower, maxPower);
        return leftBMotorCmd;
    }

    public static void stopBase ()
    {
        leftFMotorCmd = 0;
        rightFMotorCmd = 0;
        leftBMotorCmd = 0;
        rightBMotorCmd = 0;
    }

    public static double getRightFMotorCmd ()
    {
        return rightFMotorCmd;
    }

    public static double getRightBMotorCmd ()
    {
        return rightBMotorCmd;
    }

    public static double getLeftFMotorCmd ()
    {
        return leftFMotorCmd;
    }

    public static double getLeftBMotorCmd ()
    {
        return leftBMotorCmd;
    }

    public static double getLinearError () {
        return linearError;
    }
}