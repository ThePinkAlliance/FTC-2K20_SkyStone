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
import org.firstinspires.ftc.PinkCode.Subsystems.Lift;
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


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/*
 * This OpMode uses the Hardware class to define its hardware.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous (name = "Auto")
public class Auto extends OpMode {
    private int BackTemp = 0;
    private int temp = 0;
    private int tempAngle = 0;
    private int skyLocation = -1;
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 2f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {5f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {7f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera phoneCam;


    double speed;

    private Hardware robot = new Hardware();
    private BNO055IMU imu;
    private double currentBaseAngle;
    private double targetBasePos, targetBaseAngle;
    private double baseScorePos, baseScoreAngle;
    private boolean blueAlliance;        // Selected alliance color
    private boolean cornerStartingPos;   // Corner or middle starting position
    private double markedTime; // Represents a set point in time
    private double leftWheelPos = 0, rightWheelPos = 0;
    private double previousBasePos;
    private double linearBaseSpeed = 0;
    private double angularSpeed = 0;

    private ElapsedTime runtime = new ElapsedTime();

    // Represents a logical step for the robot to perform
    // Also conveniently shows steps in order done during auto
    private enum Stage {
        INITIALIZE,
        DRIVE_FORWARD,
        SKY1,
        SKY2,
        SKY3,
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
        TURN_FOUNDATION,
        TURN_TO_PARK,
        TURN_FOUNDATION2,
        MOVE_FOUNDATION2,
        PARK,
        BACKUP,
        STRAFE,
        STOP
    }

    private Stage stage = Stage.INITIALIZE;

    @Override
    public void init() {
        /*
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        robot.init(hardwareMap);
        Subsystem.robot.init(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        // Send telemetry message to signify robot waiting;
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

        // Wait for the game to start (driver presses PLAY).
        telemetry.addData("Status", "Waiting for start");    //
        telemetry.update();
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


        runtime.reset();
        telemetry.addData("Values", valLeft+"   "+valMid+"   "+valRight);
        telemetry.addData("Height", rows);
        telemetry.addData("Width", cols);


        telemetry.addData("skyLocation", skyLocation);
        tempAngle = 0;
        temp = 0;
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
//        currentBaseAngle = getHeading();  // Degrees
        leftWheelPos = robot.leftF_drive.getCurrentPosition();
        rightWheelPos = robot.rightF_drive.getCurrentPosition();
        double currentBasePos = (leftWheelPos + rightWheelPos) / 2.0;
        linearBaseSpeed = currentBasePos - previousBasePos;
        previousBasePos = currentBasePos;
        telemetry.addData("Values", valLeft+"   "+valMid+"   "+valRight);
        telemetry.addData("skyLocation", skyLocation);

        switch (stage) {

            case INITIALIZE:
                if(valLeft == 0) {
                    skyLocation = 1;
                } else if(valMid == 0) {
                    skyLocation = 2;
                } else if(valRight == 0) {
                    skyLocation = 3;
                }
                if(skyLocation != -1) {
                    phoneCam.pauseViewport();
                }
                targetBasePos = 0;
                targetBaseAngle = 0;
                Subsystem.robot.scorer_collect.setPosition(Presets.SCORER_EJECT);
                pinkNavigate.stopBase();
                baseScorePos = targetBasePos;
                baseScoreAngle = targetBaseAngle;
                markedTime = runtime.milliseconds();
                stage = Stage.DRIVE_FORWARD;
                break;

            case DRIVE_FORWARD:
                targetBasePos = baseScorePos + 15;
                targetBaseAngle = baseScoreAngle;
                currentBaseAngle = getHeading();  // Degrees
                Subsystem.robot.collect_right.setPower(-1);
                Subsystem.robot.collect_left.setPower(1);
                if (pinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, 0.2)) {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    markedTime = runtime.milliseconds();
                    if(skyLocation == 1) {
                        stage = stage.SKY1;
                    } else if(skyLocation == 2) {
                        stage = stage.SKY2;
                    } else {
                        stage = stage.SKY3;
                    }
                }
                break;

            case SKY1:
                if(blueAlliance) {
                    targetBasePos = baseScorePos + 30;
                    targetBaseAngle = baseScoreAngle + 30;
                } else {
                    targetBasePos = baseScorePos + 30;
                    targetBaseAngle = baseScoreAngle + 30;
                }
                Subsystem.robot.scorerL_rotate.setPosition(Presets.SCORER_STOW);
                Subsystem.robot.scorerR_rotate.setPosition(Presets.SCORER_STOW);
                currentBaseAngle = getHeading();
                Subsystem.robot.collect_right.setPower(-1);
                Subsystem.robot.collect_left.setPower(1);
                if(pinkNavigate.driveToPos(targetBasePos,targetBaseAngle,currentBasePos,currentBaseAngle,linearBaseSpeed,.2) && runtime.milliseconds() - markedTime > 2000)
                {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    Subsystem.robot.scorer_collect.setPosition(Presets.SCORER_COLLECT);
                    Subsystem.robot.collect_right.setPower(0);
                    Subsystem.robot.collect_left.setPower(0);
                    markedTime = runtime.milliseconds();
                    if(blueAlliance) {
                        temp = 78;
                    } else {
                        temp = 73;
                    }
                    BackTemp = 28;
                    if(blueAlliance)
                        tempAngle = 57;
                    else
                        tempAngle = 57;
                    markedTime = runtime.milliseconds();
                    stage = stage.SCORER_COLLECT;
                }
                break;

            case SKY2:
                if(blueAlliance) {
                    targetBasePos = baseScorePos + 30;
                    targetBaseAngle = baseScoreAngle + 20;
                } else {
                    targetBasePos = baseScorePos + 30;
                    targetBaseAngle = baseScoreAngle + 10;
                }
                currentBaseAngle = getHeading();
                Subsystem.robot.scorerL_rotate.setPosition(Presets.SCORER_STOW);
                Subsystem.robot.scorerR_rotate.setPosition(Presets.SCORER_STOW);
                Subsystem.robot.collect_right.setPower(-1);
                Subsystem.robot.collect_left.setPower(1);
                if(pinkNavigate.driveToPos(targetBasePos,targetBaseAngle,currentBasePos,currentBaseAngle,linearBaseSpeed,.2) && runtime.milliseconds() - markedTime > 2000)
                {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    Subsystem.robot.scorer_collect.setPosition(Presets.SCORER_COLLECT);
                    Subsystem.robot.collect_right.setPower(0);
                    Subsystem.robot.collect_left.setPower(0);
                    markedTime = runtime.milliseconds();
                    if(blueAlliance) {
                        temp = 75;
                    } else {
                        temp = 70;
                    }
                    BackTemp = 28;
                    if(blueAlliance)
                        tempAngle = 67;
                    else
                        tempAngle = 67;
                    stage = stage.SCORER_COLLECT;
                }
                break;

            case SKY3:
                if(blueAlliance) {
                    targetBasePos = baseScorePos + 25;
                    targetBaseAngle = baseScoreAngle - 10;
                } else {
                    targetBasePos = baseScorePos + 25;
                    targetBaseAngle = baseScoreAngle - 15;
                }
                currentBaseAngle = getHeading();
                Subsystem.robot.scorerL_rotate.setPosition(Presets.SCORER_STOW);
                Subsystem.robot.scorerR_rotate.setPosition(Presets.SCORER_STOW);
                Subsystem.robot.collect_right.setPower(-1);
                Subsystem.robot.collect_left.setPower(1);
                if(pinkNavigate.driveToPos(targetBasePos,targetBaseAngle,currentBasePos,currentBaseAngle,linearBaseSpeed,.2) && runtime.milliseconds() - markedTime > 2000)
                {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    Subsystem.robot.scorer_collect.setPosition(Presets.SCORER_COLLECT);
                    Subsystem.robot.collect_right.setPower(0);
                    Subsystem.robot.collect_left.setPower(0);
                    markedTime = runtime.milliseconds();
                    if(blueAlliance) {
                        temp = 77;
                    } else {
                        temp = 72;
                    }
                    BackTemp = 23;
                    if(blueAlliance)
                        tempAngle = 97;
                    else
                        tempAngle = 97;
                    stage = stage.SCORER_COLLECT;
                }
                break;

            case SCORER_COLLECT:
                pinkNavigate.stopBase();
                if(runtime.milliseconds() - markedTime > 1000)
                {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    markedTime = runtime.milliseconds();
                    stage = stage.DRIVE_BACKWARD;
                }
                break;

            case DRIVE_BACKWARD:
                targetBasePos = baseScorePos - BackTemp;
                targetBaseAngle = baseScoreAngle;
                currentBaseAngle = getHeading();
                if(pinkNavigate.driveToPos(targetBasePos,targetBaseAngle,currentBasePos,currentBaseAngle,linearBaseSpeed,.2))
                {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    markedTime = runtime.milliseconds();
                    stage = stage.TURN_TO_FOUNDATION;
                }
                break;

            case TURN_TO_FOUNDATION:
                if(blueAlliance) {
                    targetBasePos = baseScorePos;
                    targetBaseAngle = 92; //baseScoreAngle + tempAngle;
                } else {
                    targetBasePos = baseScorePos;
                    targetBaseAngle = -92; //baseScoreAngle - tempAngle;
                }
                currentBaseAngle = getHeading();
                telemetry.addData("angle", targetBaseAngle);
                if(pinkNavigate.driveToPos(targetBasePos, targetBaseAngle,currentBasePos,currentBaseAngle,linearBaseSpeed, .1) && runtime.milliseconds() - markedTime > 2500)
                {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    markedTime = runtime.milliseconds();
                    stage = stage.MOVE_TO_FOUNDATION;
                }
                break;

            case MOVE_TO_FOUNDATION:
                targetBasePos = baseScorePos + temp;
                targetBaseAngle = baseScoreAngle;
                currentBaseAngle = getHeading();
                if(pinkNavigate.driveToPos(targetBasePos,targetBaseAngle,currentBasePos,currentBaseAngle,linearBaseSpeed,.3))
                {
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
                Lift.lift_to_position(500);
                Subsystem.set_motor_powers();
                //Subsystem.set_servo_positions();
                Subsystem.robot.scorerL_rotate.setPosition(Presets.SCORER_SCORE_POSITION);
                Subsystem.robot.scorerR_rotate.setPosition(Presets.SCORER_SCORE_POSITION);
                Subsystem.robot.scorer_collect.setPosition(Presets.SCORER_COLLECT);
                pinkNavigate.stopBase();
                if (Subsystem.robot.left_lift.getCurrentPosition() > 450 && runtime.milliseconds() - markedTime > 2000) {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    markedTime = runtime.milliseconds();
                    stage = stage.LIFT_DOWN;
                }
                break;

            case LIFT_DOWN:
                targetBasePos = baseScorePos + 20;
                targetBaseAngle = baseScoreAngle;
                currentBaseAngle = getHeading();
                Lift.lift_to_position(50);
//                Subsystem.set_servo_positions();
                Subsystem.set_motor_powers();
                Subsystem.robot.scorerL_rotate.setPosition(Presets.SCORER_SCORE_POSITION);
                Subsystem.robot.scorerR_rotate.setPosition(Presets.SCORER_SCORE_POSITION);
                Subsystem.robot.scorer_collect.setPosition(Presets.SCORER_COLLECT);
                pinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, .2);
                if(Subsystem.robot.left_lift.getCurrentPosition() < 100) {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    markedTime = runtime.milliseconds();
                    stage = stage.SCORE;
                }
                break;

            case SCORE:
                targetBaseAngle = baseScoreAngle;
                targetBasePos = baseScorePos;
                currentBaseAngle = getHeading();
                Lift.lift_to_position(100);
//                Subsystem.set_servo_positions();
                Subsystem.set_motor_powers();
                Subsystem.robot.scorer_collect.setPosition(Presets.SCORER_EJECT);
                Subsystem.robot.scorerL_rotate.setPosition(Presets.SCORER_SCORE_POSITION);
                Subsystem.robot.scorerR_rotate.setPosition(Presets.SCORER_SCORE_POSITION);
                pinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, .2);
                if(runtime.milliseconds() - markedTime > 1500) {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    stage = stage.ROTATE_SCORER;
                }
                break;

            case ROTATE_SCORER:
                targetBaseAngle = baseScoreAngle;
                targetBasePos = baseScorePos;
                currentBaseAngle = getHeading();
                Lift.lift_to_position(100);
                Subsystem.set_servo_positions();
                Subsystem.set_motor_powers();
                Subsystem.robot.scorerL_rotate.setPosition(Presets.SCORER_STOW);
                Subsystem.robot.scorerR_rotate.setPosition(Presets.SCORER_STOW);
                pinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, .2);
                if (runtime.milliseconds() - markedTime > 1000) {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    stage = stage.TURN_FOUNDATION;
                }
                break;

//            TODO: EasyOpenCV
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
                targetBasePos = baseScorePos + 40;
                targetBaseAngle = baseScoreAngle;
                if (pinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, .1)) {
                    baseScoreAngle = targetBaseAngle;
                    baseScorePos = targetBasePos;
                    stage = stage.STOP;
                }
                break;

            case MOVE_FOUNDATION:
                Subsystem.robot.right_hook.setPosition(0);
                Subsystem.robot.left_hook.setPosition(1);
                currentBaseAngle = getHeading();
                targetBasePos = baseScorePos + 25;
                targetBaseAngle = baseScoreAngle;
                if (pinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, .5)) {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    stage = stage.TURN_FOUNDATION;
                }
                break;

            case TURN_FOUNDATION:
                Subsystem.robot.right_hook.setPosition(0);
                Subsystem.robot.left_hook.setPosition(1);
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
                    markedTime = runtime.milliseconds();
                    stage = stage.STRAFE;
                }
                break;

            case MOVE_FOUNDATION2:
                Subsystem.robot.right_hook.setPosition(0);
                Subsystem.robot.left_hook.setPosition(1);
                currentBaseAngle = getHeading();
                if (blueAlliance) {
                    targetBasePos = baseScorePos + 8;
                    targetBaseAngle = baseScoreAngle;
                }else {
                    targetBasePos = baseScorePos + 8;
                    targetBaseAngle = baseScoreAngle;
                }
                if (pinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, .2)) {
                    Subsystem.robot.right_hook.setPosition(1);
                    Subsystem.robot.left_hook.setPosition(0);
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    stage = stage.TURN_FOUNDATION2;
                }
                break;

            case TURN_FOUNDATION2:
                Subsystem.robot.right_hook.setPosition(0);
                Subsystem.robot.left_hook.setPosition(1);
                currentBaseAngle = getHeading();
                if (blueAlliance) {
                    targetBasePos = baseScorePos;
                    targetBaseAngle = baseScoreAngle + 55;
                }else {
                    targetBasePos = baseScorePos;
                    targetBaseAngle = baseScoreAngle - 45;
                }
                if (pinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, .2)) {
                    Subsystem.robot.right_hook.setPosition(1);
                    Subsystem.robot.left_hook.setPosition(0);
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    stage = stage.BACKUP;
                }
                break;

            case BACKUP:
                targetBasePos = baseScorePos - 10;
                targetBaseAngle = baseScoreAngle;
                currentBaseAngle = getHeading();
                if(pinkNavigate.driveToPos(targetBasePos,targetBaseAngle,currentBasePos,currentBaseAngle,linearBaseSpeed,.2))
                {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    markedTime = runtime.milliseconds();
                    stage = stage.STRAFE;
                }
                break;

            case STRAFE:
                if(blueAlliance) {
                    targetBasePos = baseScorePos - 10;
                    targetBaseAngle = baseScoreAngle;
                    temp = 1;
                    speed = -.3;
                } else {
                    targetBasePos = baseScorePos + 10;
                    targetBaseAngle = baseScoreAngle;
                    temp = -1;
                    speed = .3;
                }
                currentBaseAngle = getHeading();
                if(pinkNavigate.strafeToPos(targetBasePos,targetBaseAngle,currentBasePos,currentBaseAngle,linearBaseSpeed,.2) || runtime.milliseconds() - markedTime > 800) {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    markedTime = runtime.milliseconds();
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
        telemetry.addData("LeftB MotorPower: ", Subsystem.robot.leftB_drive.getPower());
        telemetry.addData("RightF MotorPower: ", Subsystem.robot.rightF_drive.getPower());
        telemetry.addData("RightB MotorPower: ", Subsystem.robot.rightB_drive.getPower());

        telemetry.addData("Target Base Angle", targetBaseAngle);
        telemetry.addData("current Base Angle:", currentBaseAngle);
//        telemetry.addData("Target Base Pos", targetBasePos);
//        telemetry.addData("Current Base Pos", currentBasePos/24.9);


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


    public void stop() {
        Subsystem.robot.leftF_drive.setPower(0);
        Subsystem.robot.leftB_drive.setPower(0);
        Subsystem.robot.rightF_drive.setPower(0);
        Subsystem.robot.rightB_drive.setPower(0);
    }

    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private opencvSkystoneDetector.StageSwitchingPipeline.Stage stageToRenderToViewport = opencvSkystoneDetector.StageSwitchingPipeline.Stage.detection;
        private opencvSkystoneDetector.StageSwitchingPipeline.Stage[] stages = opencvSkystoneDetector.StageSwitchingPipeline.Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

    }
}