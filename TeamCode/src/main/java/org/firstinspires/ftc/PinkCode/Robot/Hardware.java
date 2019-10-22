package org.firstinspires.ftc.PinkCode.Robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// Class to Define the Hardware of the Robot
public class Hardware {
    // Motors
    public DcMotor rightF_drive; // Port 3 Expansion Hub 1
    public DcMotor rightB_drive; // Port 3 Expansion Hub 1
    public DcMotor leftF_drive; // Port 3 Expansion Hub 2
    public DcMotor leftB_drive; // Port 3 Expansion Hub 2
    public DcMotor collect_left;
    public DcMotor collect_right; // Port 1 Expansion Hub 1
    public DcMotor right_lift; // Port 0 Expansion Hub 1
    public DcMotor left_lift; // Port 0 Expansion Hub 2

//    // Servos
    public Servo scorer_rotate;
    public Servo scorer_collect;
    public Servo left_hook;
    public Servo right_hook;
//    public Servo capstone;

    // Local OpMode Members
    private HardwareMap hwMap = null;

    // Method Called When Referencing Robot Hardware in Subsystems
    public void init (HardwareMap ahwMap) {
        // Reference to Hardware Map
        hwMap = ahwMap;

        // Motors
        rightF_drive = hwMap.get(DcMotor.class, "rightF_drive");
        rightB_drive = hwMap.get(DcMotor.class, "rightB_drive");
        leftF_drive = hwMap.get(DcMotor.class, "leftF_drive");
        leftB_drive = hwMap.get(DcMotor.class, "leftB_drive");
        collect_left = hwMap.get(DcMotor.class, "collect_left");
        collect_right = hwMap.get(DcMotor.class, "collect_right");
        right_lift = hwMap.get(DcMotor.class, "right_lift");
        left_lift =  hwMap.get(DcMotor.class, "left_lift");

        // Motor Configuration
        rightF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collect_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collect_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collect_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collect_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightF_drive.setDirection(DcMotor.Direction.REVERSE);
        rightB_drive.setDirection(DcMotor.Direction.REVERSE);
        leftF_drive.setDirection(DcMotor.Direction.FORWARD);
        leftB_drive.setDirection(DcMotor.Direction.FORWARD);
        collect_left.setDirection(DcMotor.Direction.FORWARD);
        collect_right.setDirection(DcMotor.Direction.REVERSE);
        right_lift.setDirection(DcMotor.Direction.FORWARD);
        left_lift.setDirection(DcMotor.Direction.FORWARD);

        rightF_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightB_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftF_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftB_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collect_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collect_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightF_drive.setPower(0);
        rightB_drive.setPower(0);
        leftF_drive.setPower(0);
        leftB_drive.setPower(0);
        collect_left.setPower(0);
        collect_right.setPower(0);
        right_lift.setPower(0);
        left_lift.setPower(0);

        // Servos
        scorer_rotate = hwMap.get(Servo.class, "scorer_rotate");
        scorer_collect = hwMap.get(Servo.class, "scorer_collect");
        left_hook = hwMap.get(Servo.class, "left_hook");
        right_hook = hwMap.get(Servo.class, "right_hook");
    }
}