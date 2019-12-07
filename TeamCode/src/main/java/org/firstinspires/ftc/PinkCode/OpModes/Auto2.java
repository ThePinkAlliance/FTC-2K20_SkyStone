package org.firstinspires.ftc.PinkCode.OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.PinkCode.Calculations.Presets;
import org.firstinspires.ftc.PinkCode.Robot.Hardware;
import org.firstinspires.ftc.PinkCode.Calculations.pinkNavigate;
import org.firstinspires.ftc.PinkCode.Subsystems.Scorer;
import org.firstinspires.ftc.PinkCode.Subsystems.Subsystem;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;


import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.Locale;

/*
 * This OpMode uses the Hardware class to define its hardware.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous (name = "PINK Auto")
public class Auto2 extends OpMode {


    private Hardware robot = new Hardware();
    private BNO055IMU imu;
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    private double currentBaseAngle;
    private double targetBasePos, targetBaseAngle;
    private double baseScorePos, baseScoreAngle;
    private boolean blueAlliance;        // Selected alliance color
    private boolean cornerStartingPos;   // Corner or middle starting position
    private double markedTime; // Represents a set point in time
    private double leftWheelPos = 0, rightWheelPos = 0;
    private double previousBasePos;
    private double linearBaseSpeed = 0;

    private double temp;

    private OpenCvCamera phoneCam;
    private SkystoneDetector skyStoneDetector;

    private boolean strafe = false;

    private RelicRecoveryVuMark image = null;
    int jewelColor = 0;

    private ElapsedTime runtime = new ElapsedTime();
    private VuforiaLocalizer vuforia;
    private VuforiaTrackable relicTemplate;

    // Represents a logical step for the robot to perform
    // Also conveniently shows steps in order done during auto
    private enum Stage {
        INITIALIZE,
        DRIVE_FORWARD,
        SCAN,
        RESET,
        COLLECT_CUBE,
        DRIVE_BACKWARD,
        SCORER_COLLECT,
        TURN_TO_FOUNDATION,
        MOVE_TO_FOUNDATION,
        TURN_AGAIN,
        FORWARD_TO_FOUNDATION,
        HOOK_AND_SCORE,
        MOVE_FOUNDATION,
        UNHOOK,
        SCORE,
        LIFT,
        ROTATE_SCORER,
        LIFT_DOWN,
        TURN_TO_PARK,
        TURN_FOUNDATION,
        PARK,
        STOP
    }

    private Stage stage;

    @Override
    public void init() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        robot.init(hardwareMap);
        Subsystem.robot.init(hardwareMap);

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        Subsystem.robot.rightF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Subsystem.robot.rightB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Subsystem.robot.leftF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Subsystem.robot.leftB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Subsystem.robot.leftF_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Subsystem.robot.leftB_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Subsystem.robot.rightF_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Subsystem.robot.rightB_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        blueAlliance = true;
        cornerStartingPos = true;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.openCameraDevice();

        skyStoneDetector = new SkystoneDetector();
        phoneCam.setPipeline(skyStoneDetector);

        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        // Wait for the game to start (driver presses PLAY).
        telemetry.addData("Status", "Waiting for start");    //
        telemetry.update();
        strafe = false;
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        if (gamepad1.x) {
            blueAlliance = true;
        } else if (gamepad1.b) {
            blueAlliance = false;
        }
        if (gamepad1.y) {
            cornerStartingPos = true;
        } else if (gamepad1.a) {
            cornerStartingPos = false;
        }
        if (blueAlliance) {
            telemetry.addData("Alliance Color", "BLUE");
        } else {
            telemetry.addData("Alliance Color", "RED");
        }
        if (cornerStartingPos) {
            telemetry.addData("Starting Pos  ", "CORNER");
        } else {
            telemetry.addData("Starting Pos  ", "MIDDLE");
        }
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        stage = Stage.INITIALIZE;
        phoneCam.pauseViewport();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
//        currentBaseAngle = getHeading();  // Degrees
        leftWheelPos = robot.leftF_drive.getCurrentPosition();
        rightWheelPos = robot.rightF_drive.getCurrentPosition();
        double currentBasePos;
            if (strafe)
                currentBasePos = (Math.abs(leftWheelPos) + Math.abs(rightWheelPos)) / 2.0;
            else
                currentBasePos = (leftWheelPos + rightWheelPos) / 2.0;

        linearBaseSpeed = currentBasePos - previousBasePos;
        previousBasePos = currentBasePos;

        switch (stage) {
            case INITIALIZE:
                pinkNavigate.stopBase();
                targetBasePos = 0;
                targetBaseAngle = 0;
                Subsystem.robot.right_hook.setPosition(1);
                Subsystem.robot.left_hook.setPosition(0);
                Subsystem.robot.scorer_collect.setPosition(Presets.SCORER_EJECT);
                pinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, 0.2);
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    markedTime = runtime.milliseconds();
                    stage = Stage.DRIVE_FORWARD;
                    break;

            case DRIVE_FORWARD:
                targetBasePos = baseScorePos + 15;
                targetBaseAngle = baseScoreAngle;
                currentBaseAngle = getHeading();  // Degrees
//                Subsystem.robot.collect_right.setPower(-1);
//                Subsystem.robot.collect_left.setPower(1);
                strafe = false;
                if (pinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, 0.2)) {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    markedTime = runtime.milliseconds();
                    strafe = true;
                    stage = Stage.SCAN;
                }
                break;

            case SCAN:
                Scan();
                if(blueAlliance) {
                    targetBasePos = baseScorePos + 40;
                } else {
                    targetBasePos = baseScorePos - 40;
                }
                targetBaseAngle = baseScoreAngle;
                currentBaseAngle = getHeading();
                if(blueAlliance) {
                    if (pinkNavigate.strafeToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, .1) || Scan() < 75 && Scan() > 30) {
                        baseScorePos = targetBasePos;
                        baseScoreAngle = targetBaseAngle;
                        markedTime = runtime.milliseconds();
                        temp = pinkNavigate.getLinearError();
                        strafe = false;
                        stage = stage.RESET;
                    }
                } else {
                    if (pinkNavigate.strafeToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, .1) || Scan() > 175 && Scan() < 220) {
                        baseScorePos = targetBasePos;
                        baseScoreAngle = targetBaseAngle;
                        markedTime = runtime.milliseconds();
                        temp = pinkNavigate.getLinearError();
                        strafe = false;
                        stage = stage.RESET;
                    }
                }
                break;

            case RESET:
                pinkNavigate.stopBase();
                if(runtime.milliseconds() - markedTime > 500) {
                    stage = stage.COLLECT_CUBE;
                }
                break;

            case COLLECT_CUBE:
                strafe = false;
                targetBasePos = baseScorePos + 17;
                targetBaseAngle = baseScoreAngle - 20;
                currentBaseAngle = getHeading();
                Subsystem.robot.collect_right.setPower(-1);
                Subsystem.robot.collect_left.setPower(1);
                if (pinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, .2) && runtime.milliseconds() - markedTime > 2000) {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    Subsystem.robot.scorer_collect.setPosition(Presets.SCORER_COLLECT);
                    markedTime = runtime.milliseconds();
                    stage = stage.SCORER_COLLECT;
                }
                break;

            case SCORER_COLLECT:
                targetBasePos = baseScorePos;
                targetBaseAngle = baseScoreAngle;
                currentBaseAngle = getHeading();
                Subsystem.robot.collect_right.setPower(-1);
                Subsystem.robot.collect_left.setPower(1);
                if (pinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, .2) && runtime.milliseconds() - markedTime > 3000) {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    Subsystem.robot.collect_right.setPower(0);
                    Subsystem.robot.collect_left.setPower(0);
                    markedTime = runtime.milliseconds();
                    stage = stage.DRIVE_BACKWARD;
                }
            case DRIVE_BACKWARD:
                targetBasePos = baseScorePos - 15;
                targetBaseAngle = baseScoreAngle;
                currentBaseAngle = getHeading();
                Subsystem.robot.collect_right.setPower(0);
                Subsystem.robot.collect_left.setPower(0);
                if (pinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, .2)) {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    stage = stage.TURN_TO_FOUNDATION;
                }
                break;

            case TURN_TO_FOUNDATION:
                if (blueAlliance) {
                    targetBasePos = baseScorePos;
                    targetBaseAngle = baseScoreAngle + 104;
                } else {
                    targetBaseAngle = baseScoreAngle - 70;
                    targetBasePos = baseScorePos;
                }
                currentBaseAngle = getHeading();
                if (pinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, .2)) {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    stage = stage.MOVE_TO_FOUNDATION;
                }
                break;

            case MOVE_TO_FOUNDATION:
                targetBasePos = (baseScorePos + 109 - temp);
                targetBaseAngle = baseScoreAngle;
                currentBaseAngle = getHeading();
                if (pinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, .2)) {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    markedTime = runtime.milliseconds();
                    stage = stage.TURN_AGAIN;
                }
                break;

            case TURN_AGAIN:
                if (blueAlliance) {
                    targetBasePos = baseScorePos;
                    targetBaseAngle = baseScoreAngle + 90;
                } else {
                    targetBaseAngle = baseScoreAngle - 90;
                    targetBasePos = baseScorePos;
                }
                currentBaseAngle = getHeading();
                if (pinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, .2)) {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    stage = stage.FORWARD_TO_FOUNDATION;
                }
                break;

            case FORWARD_TO_FOUNDATION:
                targetBasePos = baseScorePos - 10;
                targetBaseAngle = baseScoreAngle;
                currentBaseAngle = getHeading();
                if (pinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, .2)) {
                    Subsystem.robot.right_hook.setPosition(0);
                    Subsystem.robot.left_hook.setPosition(1);
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    markedTime = runtime.milliseconds();
                    stage = stage.HOOK_AND_SCORE;
                }
                break;

            case HOOK_AND_SCORE:
                Subsystem.robot.right_hook.setPosition(0);
                Subsystem.robot.left_hook.setPosition(1);
                targetBasePos = baseScorePos;
                targetBaseAngle = baseScoreAngle;
                currentBaseAngle = getHeading();
//                Subsystem.robot.left_lift.setPower(1);
//                Subsystem.robot.right_lift.setPower(1);
//                if (Subsystem.robot.left_lift.getCurrentPosition() > 1200) {
//                    Subsystem.robot.scorer_rotate.setPosition(1);
                    if (pinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, .1) & runtime.milliseconds() - markedTime > 2000) {
                        baseScorePos = targetBasePos;
                        baseScoreAngle = targetBaseAngle;
                        markedTime = runtime.milliseconds();
                        stage = stage.UNHOOK;
                    }
//                }
                break;

            case UNHOOK:
                targetBaseAngle = baseScoreAngle;
                targetBasePos = baseScorePos;
                currentBaseAngle = getHeading();
//                Subsystem.robot.left_lift.setPower(-1);
//                Subsystem.robot.right_lift.setPower(-1);
//                if(Subsystem.robot.left_lift.getCurrentPosition() < 700) {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    markedTime = runtime.milliseconds();
                    stage = stage.SCORE;
//                }
                break;

            case SCORE:
                targetBaseAngle = baseScoreAngle;
                targetBasePos = baseScorePos;
                currentBaseAngle = getHeading();
//                Subsystem.robot.scorer_collect.setPosition(Presets.SCORER_EJECT);
                if (pinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, .1) && runtime.milliseconds() - markedTime > 1500) {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    stage = stage.LIFT;
                }
                break;

            case LIFT:
                targetBaseAngle = baseScoreAngle;
                targetBasePos = baseScorePos;
                currentBaseAngle = getHeading();
//                Subsystem.robot.left_lift.setPower(1);
//                Subsystem.robot.right_lift.setPower(1);
//                if (pinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, .1) && Subsystem.robot.left_lift.getCurrentPosition() > 1500) {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    markedTime = runtime.milliseconds();
                    stage = stage.ROTATE_SCORER;
//                }
                break;
            case ROTATE_SCORER:
                targetBaseAngle = baseScoreAngle;
                targetBasePos = baseScorePos;
                currentBaseAngle = getHeading();
//                Subsystem.robot.scorer_rotate.setPosition(0);
                if (pinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, .1) && runtime.milliseconds() - markedTime > 2000) {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    stage = stage.LIFT_DOWN;
                }
                break;

            case LIFT_DOWN:
                targetBaseAngle = baseScoreAngle;
                targetBasePos = baseScorePos;
                currentBaseAngle = getHeading();
//                Subsystem.robot.left_lift.setPower(-1);
//                Subsystem.robot.right_lift.setPower(-1);
//                Scorer.score_collect(Presets.SCORER_COLLECT);
//                if (pinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, .1) && Subsystem.robot.left_lift.getCurrentPosition() < 100) {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    stage = stage.MOVE_FOUNDATION;
//                }
                break;
            //TODO: EasyOpenCV
            case TURN_TO_PARK:
                currentBaseAngle = getHeading();
                if (blueAlliance) {
                    targetBasePos = baseScorePos;
                    targetBaseAngle = baseScoreAngle + 180;
                } else {
                    targetBasePos = baseScorePos;
                    targetBaseAngle = baseScoreAngle - 180;
                }
                if (pinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, .1) && pinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, .1)) {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    stage = stage.PARK;
                }
                break;

            case PARK:
                currentBaseAngle = getHeading();
                targetBasePos = baseScorePos + 30;
                targetBaseAngle = baseScoreAngle;
                if (pinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, .1)) {
                    baseScoreAngle = targetBaseAngle;
                    baseScorePos = targetBasePos;
                    stage = stage.STOP;
                }
                break;

            case MOVE_FOUNDATION:
                currentBaseAngle = getHeading();
                targetBasePos = baseScorePos + 40;
                targetBaseAngle = baseScoreAngle;
                if (pinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, .5)) {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    stage = stage.TURN_FOUNDATION;
                }
                break;

            case TURN_FOUNDATION:
                currentBaseAngle = getHeading();
                if (blueAlliance) {
                    targetBasePos = baseScorePos;
                    targetBaseAngle = baseScoreAngle + 90;
                }else {
                    targetBasePos = baseScorePos;
                    targetBaseAngle = baseScoreAngle - 90;
                }
                if (pinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, .2)) {
                    Subsystem.robot.right_hook.setPosition(1);
                    Subsystem.robot.left_hook.setPosition(0);
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    stage = stage.PARK;
                }
                break;
            case STOP:
                Subsystem.robot.right_hook.setPosition(1);
                Subsystem.robot.left_hook.setPosition(0);
                targetBaseAngle = baseScoreAngle;
                targetBasePos = baseScorePos;
                currentBaseAngle = getHeading();
                pinkNavigate.stopBase();
        }

        Subsystem.robot.leftF_drive.setPower(pinkNavigate.getLeftFMotorCmd());
        Subsystem.robot.leftB_drive.setPower(pinkNavigate.getLeftBMotorCmd());
        Subsystem.robot.rightF_drive.setPower(pinkNavigate.getRightFMotorCmd());
        Subsystem.robot.rightB_drive.setPower(pinkNavigate.getRightBMotorCmd());

        telemetry.addData("Stage  ", stage);
        telemetry.addData("Correct Motor Power: ", pinkNavigate.getLeftFMotorCmd());
        telemetry.addData("LeftF MotorPower: ", Subsystem.robot.leftF_drive.getPower());
//        telemetry.addData("LB MP: ", Subsystem.robot.leftB_drive.getPower());
//        telemetry.addData("RF MP: ", Subsystem.robot.rightF_drive.getPower());
//        telemetry.addData("RB MP: ", Subsystem.robot.rightB_drive.getPower());

//        telemetry.addData("Target Base Angle", targetBaseAngle);
//        telemetry.addData("current Base Angle:", currentBaseAngle);
        telemetry.addData("Target Base Pos", targetBasePos);
        telemetry.addData("Current Base Pos", currentBasePos/24.9);
//        telemetry.addData("if < 1 stop moving", Math.abs(pinkNavigate.getLinearError()));


    }
    private double getHeading() {
        Orientation angles;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) > 45 && !blueAlliance)
            return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) - 360;
        else if(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) < -45 && blueAlliance)
            return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) + 360;
        else
            return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }

    private double Scan() {

        phoneCam.resumeViewport();
        return skyStoneDetector.getScreenPosition().y;
    }
    public void stop() {
        Subsystem.robot.leftF_drive.setPower(0);
        Subsystem.robot.leftB_drive.setPower(0);
        Subsystem.robot.rightF_drive.setPower(0);
        Subsystem.robot.rightB_drive.setPower(0);
    }
}