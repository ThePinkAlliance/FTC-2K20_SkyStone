package org.firstinspires.ftc.PinkCode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.PinkCode.Subsystems.Subsystem;
import org.firstinspires.ftc.PinkCode.Subsystems.Base;
import org.firstinspires.ftc.PinkCode.Subsystems.Collector;
import org.firstinspires.ftc.PinkCode.Subsystems.Lift;
import org.firstinspires.ftc.PinkCode.Subsystems.Scorer;
import org.firstinspires.ftc.PinkCode.Robot.Controls;
import org.firstinspires.ftc.PinkCode.Calculations.Presets;


// Class for Player-Controlled Period of the Game Which Binds Controls to Subsystems
@TeleOp(name = "Teleop", group = "Teleop")
public class Teleop extends Controls {
    // Code to Run Once When the Drivers Press Init
    public void init() {
        // Initialization of Each Subsystem's Hardware Map
        Subsystem.robot.init(hardwareMap);
        Subsystem.robot.rightF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Subsystem.robot.rightB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Subsystem.robot.leftF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Subsystem.robot.leftB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Subsystem.robot.rightF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Subsystem.robot.rightB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Subsystem.robot.leftF_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Subsystem.robot.leftB_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Telemetry Update to Inform Drivers That the Program is Initialized
        telemetry.addData("Status: ", "Waiting for Driver to Press Play");
        telemetry.update();
    }

    // Code to Run Constantly After the Drivers Press Play and Before They Press Stop
    public void loop() {
        // Drive Train Control TODO Controls for Joysticks
        Base.drive_by_command(-gamepad1.right_stick_y, -gamepad1.left_stick_y, -gamepad1.right_stick_y, -gamepad1.left_stick_y);

        // Collector Controls
        if (base_y(false)) {
            Collector.collect();
        } else if (base_b(false)) {
            Collector.eject();
        } else if (base_right_bumper(false)) {
            Collector.collect();
        } else if (base_left_bumper(false)) {
            Collector.eject();
        } else if (tower_b(false)) {
            Collector.collect_stop();
        } else {
            Collector.collect_stop();
        }

        // Lift Controls TODO: Controls for Joysticks
        if (gamepad2.left_stick_y < -.1 || gamepad2.left_stick_y > .1) {
            Lift.lift_by_command(-gamepad2.left_stick_y);
        } else if (tower_back(false)) {
            Lift.robot.right_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Lift.robot.left_lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Lift.robot.right_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Lift.robot.left_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else if (tower_y(false)) {
            Lift.lift_to_position(Presets.LIFT_MAX_POSITION);
        } else if (tower_a(false)) {
            Lift.lift_to_position(Presets.LIFT_CLEAR_POSITION);
        } else {
            Lift.lift_stop();
        }

        // Scorer Controls
//        Scorer.score_rotate_by_command(gamepad2.left_stick_y);
        if (tower_y(false)) {
            if (Subsystem.robot.right_lift.getCurrentPosition() > Presets.LIFT_CLEAR_POSITION) {
                Scorer.score_rotate_to_position(Presets.SCORER_SCORE_POSITION);
            } else {
                Scorer.score_rotate_to_position(Presets.SCORER_STOW);
            }
            // Set Motor Powers and Servos to Their Commands
            Subsystem.set_motor_powers();
            Subsystem.set_servo_positions();

            // Add Telemetry to Phone for Debugging and Testing if it is Activated
            if (tower_start(false)) {
                telemetry.addData("Status: ", "Running Teleop");
                telemetry.addData("Powers: ", "");
                telemetry.addData("Base Right Power: ", Subsystem.robot.rightF_drive.getPower());
                telemetry.addData("Base Right Power: ", Subsystem.robot.rightB_drive.getPower());
                telemetry.addData("Base Left Power: ", Subsystem.robot.leftF_drive.getPower());
                telemetry.addData("Base Left Power: ", Subsystem.robot.leftB_drive.getPower());
                telemetry.addData("Collector Power: ", Subsystem.robot.collect.getPower());
                telemetry.addData("Collector Rotate Power: ", Subsystem.robot.collector_rotate.getPower());
                telemetry.addData("Lift Power: ", Subsystem.robot.right_lift.getPower());
                telemetry.addData("Tower Right Rotate Power: ", Subsystem.robot.scorer_rotate.getPosition());
                telemetry.addData("Right Extend Power: ", Subsystem.robot.right_extend.getPower());
                telemetry.addData("Left Extend Power: ", Subsystem.robot.left_extend.getPower());
                telemetry.addData("Positions: ", "");
                telemetry.addData("Base Right Position: ", Subsystem.robot.rightF_drive.getCurrentPosition());
                telemetry.addData("Base Right Position: ", Subsystem.robot.rightB_drive.getCurrentPosition());
                telemetry.addData("Base Left Position: ", Subsystem.robot.leftF_drive.getCurrentPosition());
                telemetry.addData("Base Left Position: ", Subsystem.robot.leftB_drive.getCurrentPosition());
                telemetry.addData("Lift Position: ", Subsystem.robot.right_lift.getCurrentPosition());
                telemetry.addData("Collector Rotate Position: ", Subsystem.robot.collector_rotate.getCurrentPosition());
                telemetry.addData("Flap Target Position: ", Subsystem.robot.score_flap.getPosition());
                telemetry.addData("Right Extend Position: ", Subsystem.robot.right_extend.getCurrentPosition());
//            telemetry.addData("Left Extend Position: ", Subsystem.robot.left_extend.getCurrentPosition());
                telemetry.update();
            } else {
                // Telemetry Update to Inform Drivers That the Program is Running and how to Access Telemetry
                telemetry.addData("Status: ", "Running Teleop");
                telemetry.addData("Press Start on the Tower Gamepad for Telemetry", "");
                telemetry.update();
            }
        }
    }
    // Code to Run Once When the Drivers Press Stop
    public void stop() {
        // Stop Sending Commands to Each Subsystem
        Base.drive_stop();
        Collector.collect_stop();
        Lift.lift_stop();

        // Set Motor Powers and Servos to Their Commands
        Subsystem.set_motor_powers();
        Subsystem.set_servo_positions();

        // Telemetry Update to Inform Drivers That the Program is Stopped
        telemetry.addData("Status: ", "Stopped");
        telemetry.update();
    }
}