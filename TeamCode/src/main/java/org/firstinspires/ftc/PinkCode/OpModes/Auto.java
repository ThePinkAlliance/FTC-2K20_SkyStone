package org.firstinspires.ftc.PinkCode.OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.PinkCode.Calculations.Presets;
import org.firstinspires.ftc.PinkCode.Robot.Hardware;
import org.firstinspires.ftc.PinkCode.Calculations.PD;
import org.firstinspires.ftc.PinkCode.Calculations.pinkNavigate;
import org.firstinspires.ftc.PinkCode.Subsystems.Subsystem;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/*
 * This OpMode uses the Hardware class to define its hardware.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous (name = "Auto")
public class Auto extends OpMode {
    private Hardware robot = new Hardware();
    private BNO055IMU imu;
    private double currentBaseAngle;
    private double armMotorCmd;
    private double collectorArmTargetPos;
    private double flickerArmTargetPos, flickerFingerTargetPos;
    private double collectorFinger1TargetPos, collectorFinger2TargetPos, collectorFinger3TargetPos, collectorFinger4TargetPos, collectorRotateTargetPos;
    private double targetBasePos, targetBaseAngle;
    private double baseScorePos, baseScoreAngle;
    private boolean jewelFound = false;
    private boolean blueAlliance;        // Selected alliance color
    private boolean cornerStartingPos;   // Corner or middle starting position
    private double markedTime; // Represents a set point in time
    private boolean ourJewelIsTheFrontOne;
    private double leftWheelPos = 0, rightWheelPos = 0;
    private double previousBasePos;
    private double linearBaseSpeed = 0;
    private double columnOffset = 0;
    private double light1Power = 0;
    private double light2Power = 0;
    //private double craneRotatePos = 0;
    //private double craneExtendPos = 0;
    private double collectorArmPos = 0, collectorArmPreviousPos = 0, armSpeed = 0;
    //private double angularSpeed = 0;

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
        TURN,
        DRIVE_TO_CUBE,
        TURN_TO_CUBE,
        COLLECT_CUBE,
        DRIVE_BACKWARD,
        SCORER_COLLECT,
        DRIVE_BACKWARD2,
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
        PARK,
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
        switch (stage) {
            case INITIALIZE:
                targetBasePos = 0;
                targetBaseAngle = 0;
                Subsystem.robot.scorer_collect.setPosition(Presets.SCORER_EJECT);
                // Set the claw here and leave it since we don't use it
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
                Subsystem.robot.collect_right.setPower(-1);
                Subsystem.robot.collect_left.setPower(1);
                if (pinkNavigate.driveToPos(targetBasePos, targetBaseAngle, currentBasePos, currentBaseAngle, linearBaseSpeed, 0.1) && runtime.milliseconds() - markedTime > 2000) {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    markedTime = runtime.milliseconds();
                    stage = Stage.COLLECT_CUBE;
                }
                break;

            case COLLECT_CUBE:
                targetBasePos = baseScorePos + 2;
                targetBaseAngle = baseScoreAngle;
                currentBaseAngle = getHeading();
                Subsystem.robot.collect_right.setPower(-1);
                Subsystem.robot.collect_left.setPower(1);
                if(pinkNavigate.driveToPos(targetBasePos,targetBaseAngle,currentBasePos,currentBaseAngle,linearBaseSpeed,.1) && runtime.milliseconds() - markedTime > 2000)
                {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    Subsystem.robot.scorer_collect.setPosition(Presets.SCORER_COLLECT);
                    Subsystem.robot.collect_right.setPower(0);
                    Subsystem.robot.collect_left.setPower(0);
                    markedTime = runtime.milliseconds();
                    stage = stage.SCORER_COLLECT;
                }
                break;

            case SCORER_COLLECT:
                targetBasePos = baseScorePos;
                targetBaseAngle = baseScoreAngle;
                currentBaseAngle = getHeading();
                if(pinkNavigate.driveToPos(targetBasePos,targetBaseAngle,currentBasePos,currentBaseAngle,linearBaseSpeed,.1) && runtime.milliseconds() - markedTime > 2000)
                {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    markedTime = runtime.milliseconds();
                    stage = stage.DRIVE_BACKWARD;
                }
            case DRIVE_BACKWARD:
                targetBasePos = baseScorePos - 10;
                targetBaseAngle = baseScoreAngle;
                currentBaseAngle = getHeading();
                if(pinkNavigate.driveToPos(targetBasePos,targetBaseAngle,currentBasePos,currentBaseAngle,linearBaseSpeed,.1))
                {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    stage = stage.TURN_TO_FOUNDATION;
                }
                break;

            case TURN_TO_FOUNDATION:
                if(blueAlliance) {
                    targetBasePos = baseScorePos;
                    targetBaseAngle = baseScoreAngle + 92;
                } else {
                    targetBaseAngle = baseScoreAngle - 92;
                    targetBasePos = baseScorePos;
                }
                currentBaseAngle = getHeading();
                if(pinkNavigate.driveToPos(targetBasePos, targetBaseAngle,currentBasePos,currentBaseAngle,linearBaseSpeed, .05))
                {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    stage = stage.MOVE_TO_FOUNDATION;
                }
                break;

            case MOVE_TO_FOUNDATION:
                targetBasePos = baseScorePos + 50;
                targetBaseAngle = baseScoreAngle;
                currentBaseAngle = getHeading();
                if(pinkNavigate.driveToPos(targetBasePos,targetBaseAngle,currentBasePos,currentBaseAngle,linearBaseSpeed,.1))
                {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    markedTime = runtime.milliseconds();
                    stage = stage.DRIVE_BACKWARD2;
                }
                break;

            case DRIVE_BACKWARD2:
                targetBasePos = baseScorePos - 10;
                targetBaseAngle = baseScoreAngle;
                currentBaseAngle = getHeading();
                if(pinkNavigate.driveToPos(targetBasePos,targetBaseAngle,currentBasePos,currentBaseAngle,linearBaseSpeed,.1))
                {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    stage = stage.STOP;
                }
                break;
            case TURN_AGAIN:
                if(blueAlliance) {
                    targetBasePos = baseScorePos;
                    targetBaseAngle = baseScoreAngle + 70;
                } else {
                    targetBaseAngle = baseScoreAngle - 70;
                    targetBasePos = baseScorePos;
                }
                currentBaseAngle = getHeading();
                if(pinkNavigate.driveToPos(targetBasePos, targetBaseAngle,currentBasePos,currentBaseAngle,linearBaseSpeed, .1))
                {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    stage = stage.FORWARD_TO_FOUNDATION;
                }
                break;

            case FORWARD_TO_FOUNDATION:
                targetBasePos = baseScorePos - 10;
                targetBaseAngle = baseScoreAngle;
                currentBaseAngle = getHeading();
                if(pinkNavigate.driveToPos(targetBasePos,targetBaseAngle,currentBasePos,currentBaseAngle,linearBaseSpeed,.1))
                {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    markedTime = runtime.milliseconds();
                    stage = stage.HOOK_AND_SCORE;
                }
                break;

            case HOOK_AND_SCORE:
                Subsystem.robot.right_hook.setPosition(Presets.HOOK_DOWN);
                Subsystem.robot.left_hook.setPosition(Presets.HOOK_DOWN);
                targetBasePos = baseScorePos;
                targetBaseAngle = baseScoreAngle;
                currentBaseAngle = getHeading();
                Subsystem.robot.left_lift.setPower(1);
                Subsystem.robot.right_lift.setPower(1);
//                Subsystem.robot.scorer_rotate.setPosition(Presets.SCORER_SCORE_POSITION);
                if(pinkNavigate.driveToPos(targetBasePos,targetBaseAngle,currentBasePos,currentBaseAngle,linearBaseSpeed,.1) && Subsystem.robot.left_lift.getCurrentPosition() > 1200 && runtime.milliseconds() - markedTime > 3000)
                {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    markedTime = runtime.milliseconds();
                    stage = stage.MOVE_FOUNDATION;
                }
                break;

            case MOVE_FOUNDATION:
                currentBaseAngle = getHeading();
                targetBasePos = baseScorePos + 60;
                targetBaseAngle = baseScoreAngle;
                if(pinkNavigate.driveToPos(targetBasePos,targetBaseAngle,currentBasePos,currentBaseAngle,linearBaseSpeed,.3))
                {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    stage = stage.UNHOOK;
                }
                break;
            case UNHOOK:
                Subsystem.robot.right_hook.setPosition(Presets.HOOK_UP);
                Subsystem.robot.left_hook.setPosition(Presets.HOOK_UP);
                targetBaseAngle = baseScoreAngle;
                targetBasePos = baseScorePos;
                currentBaseAngle = getHeading();
                Subsystem.robot.left_lift.setPower(-1);
                Subsystem.robot.right_lift.setPower(-1);
                if(pinkNavigate.driveToPos(targetBasePos,targetBaseAngle,currentBasePos,currentBaseAngle,linearBaseSpeed,.1) && Subsystem.robot.left_lift.getCurrentPosition() < 500)
                {
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
                Subsystem.robot.scorer_collect.setPosition(Presets.SCORER_EJECT);
                if(pinkNavigate.driveToPos(targetBasePos,targetBaseAngle,currentBasePos,currentBaseAngle,linearBaseSpeed,.1) && runtime.milliseconds() - markedTime > 2000)
                {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    stage = stage.LIFT;
                }
                break;

            case LIFT:

                targetBaseAngle = baseScoreAngle;
                targetBasePos = baseScorePos;
                currentBaseAngle = getHeading();
                Subsystem.robot.left_lift.setPower(1);
                Subsystem.robot.right_lift.setPower(1);
                if(pinkNavigate.driveToPos(targetBasePos,targetBaseAngle,currentBasePos,currentBaseAngle,linearBaseSpeed,.1) &&Subsystem.robot.left_lift.getCurrentPosition() > 1500)
                {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    markedTime = runtime.milliseconds();
                    stage = stage.ROTATE_SCORER;
                }
                break;
            case ROTATE_SCORER:
                targetBaseAngle = baseScoreAngle;
                targetBasePos = baseScorePos;
                currentBaseAngle = getHeading();
//                Subsystem.robot.scorer_rotate.setPosition(Presets.SCORER_STOW);
                if(pinkNavigate.driveToPos(targetBasePos,targetBaseAngle,currentBasePos,currentBaseAngle,linearBaseSpeed,.1) &&runtime.milliseconds() - markedTime > 3000)
                {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    stage = stage.LIFT_DOWN;
                }
                break;

            case LIFT_DOWN:
                targetBaseAngle = baseScoreAngle;
                targetBasePos = baseScorePos;
                currentBaseAngle = getHeading();
                Subsystem.robot.left_lift.setPower(-1);
                Subsystem.robot.right_lift.setPower(-1);
                if(pinkNavigate.driveToPos(targetBasePos,targetBaseAngle,currentBasePos,currentBaseAngle,linearBaseSpeed,.1) &&Subsystem.robot.left_lift.getCurrentPosition() < 100)
                {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    stage = stage.STOP;
                }
                break;
            //TODO: EasyOpenCV
            case TURN_TO_PARK:
                currentBaseAngle = getHeading();
                if(blueAlliance) {
                    targetBasePos = baseScorePos;
                    targetBaseAngle = baseScoreAngle + 180;
                } else {
                    targetBasePos = baseScorePos;
                    targetBaseAngle = baseScoreAngle - 180;
                }
                if(pinkNavigate.driveToPos(targetBasePos,targetBaseAngle,currentBasePos,currentBaseAngle,linearBaseSpeed,.1) &&pinkNavigate.driveToPos(targetBasePos,targetBaseAngle,currentBasePos,currentBaseAngle,linearBaseSpeed,.1))
                {
                    baseScorePos = targetBasePos;
                    baseScoreAngle = targetBaseAngle;
                    stage = stage.PARK;
                }
                break;

            case PARK:
                currentBaseAngle = getHeading();
                targetBasePos = baseScorePos + 30;
                targetBaseAngle = baseScoreAngle;
                if(pinkNavigate.driveToPos(targetBasePos,targetBaseAngle,currentBasePos,currentBaseAngle,linearBaseSpeed,.1))
                {
                    baseScoreAngle = targetBaseAngle;
                    baseScorePos = targetBasePos;
                    stage = stage.STOP;
                }
                break;

            case STOP:
                targetBaseAngle = baseScoreAngle;
                targetBasePos = baseScorePos;
                currentBaseAngle = getHeading();
                if(pinkNavigate.driveToPos(targetBasePos,targetBaseAngle,currentBasePos,currentBaseAngle,linearBaseSpeed,.1))
                    stop();

        }

        Subsystem.robot.leftF_drive.setPower(pinkNavigate.getLeftFMotorCmd());
        Subsystem.robot.leftB_drive.setPower(pinkNavigate.getLeftBMotorCmd());
        Subsystem.robot.rightF_drive.setPower(pinkNavigate.getRightFMotorCmd());
        Subsystem.robot.rightB_drive.setPower(pinkNavigate.getRightBMotorCmd());

        telemetry.addData("Stage  ", stage);
        telemetry.addData("Image  ", image);
        telemetry.addData("Color  ", jewelColor);
        telemetry.addData("Correct Motor Power: ", pinkNavigate.getLeftFMotorCmd());
        telemetry.addData("Actual Motor Power: ", Subsystem.robot.leftF_drive.getPower());
        telemetry.addData("Target Base Angle", targetBaseAngle);
        telemetry.addData("current Base Angle:", currentBaseAngle);
        telemetry.addData("Target Base Pos", targetBasePos);


    }
    private double getHeading() {
        Orientation angles;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }

    public void stop() {
        Subsystem.robot.leftF_drive.setPower(0);
        Subsystem.robot.leftB_drive.setPower(0);
        Subsystem.robot.rightF_drive.setPower(0);
        Subsystem.robot.rightB_drive.setPower(0);
    }
}