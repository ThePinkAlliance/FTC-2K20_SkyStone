package org.firstinspires.ftc.PinkCode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.PinkCode.Calculations.Presets;
import org.firstinspires.ftc.PinkCode.Subsystems.Collector;
import org.firstinspires.ftc.PinkCode.Subsystems.Hooks;
import org.firstinspires.ftc.PinkCode.Subsystems.Lift;
import org.firstinspires.ftc.PinkCode.Subsystems.Scorer;
import org.firstinspires.ftc.PinkCode.Subsystems.Subsystem;
import org.firstinspires.ftc.PinkCode.Subsystems.Base;
import org.firstinspires.ftc.PinkCode.Robot.Controls;


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

        Scorer.score_rotate_to_position(Presets.SCORER_STOW);
        Scorer.score_collect(Presets.SCORER_EJECT);
        Hooks.hook_rotate_to_position(Presets.HOOK_UP);
        // Telemetry Update to Inform Drivers That the Program is Initialized
        telemetry.addData("Status: ", "Waiting for Driver to Press Play");
        telemetry.update();
    }

    // Code to Run Constantly After the Drivers Press Play and Before They Press Stop
    public void loop() {


        // Drive Train Control
        if( gamepad1.left_stick_y > .1  ||
            gamepad1.left_stick_y < -.1 ||
            gamepad1.left_stick_x > .1  ||
            gamepad1.left_stick_x < -.1 ||
            gamepad1.right_stick_x > .1 ||
            gamepad1.right_stick_x < -.1)
        {
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;

            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) + rightX;
            final double v3 = r * Math.sin(robotAngle) - rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            Base.drive_by_command(false,-v1,-v2,-v3,-v4);
        }
        else {
            Base.drive_stop();
        }

        // Collector Controls
        if(base_right_bumper(false))
            Collector.collect();
        else if(base_left_bumper(false))
            Collector.eject();
        else
            Collector.collect_stop();

        // Lift Controls
        if (gamepad2.left_stick_y < -.1 || gamepad2.left_stick_y > .1)
            Lift.lift_by_command(-gamepad2.left_stick_y);
        else if (tower_y(false)) {
            Scorer.score_collect(Presets.SCORER_COLLECT);
            Lift.lift_to_position(Presets.LIFT_MAX_POSITION);
        }else if (tower_a(false)) {
            Lift.lift_to_position(Presets.LIFT_STOW_POSITION);
            Scorer.score_collect(Presets.SCORER_EJECT);
        } else
            Lift.lift_stop();


        // Scorer Controls
        if(tower_y(false) && (Subsystem.robot.left_lift.getCurrentPosition() > Presets.LIFT_CLEAR_POSITION))
            Scorer.score_rotate_to_position(Presets.SCORER_SCORE_POSITION);
        else if(tower_a(false) && (Subsystem.robot.left_lift.getCurrentPosition() > Presets.LIFT_CLEAR_POSITION))
            Scorer.score_rotate_to_position(Presets.SCORER_STOW);
        else if(tower_right_bumper(false))
            Scorer.score_collect(Presets.SCORER_COLLECT);
        else if(tower_left_bumper(false))
            Scorer.score_collect(Presets.SCORER_EJECT);
        else if(gamepad2.right_trigger > .1 || gamepad2.right_trigger < -.1)
            Scorer.score_cap(Presets.CAP_EJECT);
        else if(gamepad2.left_trigger > .1 || gamepad2.left_trigger < -.1)
            Scorer.score_cap(Presets.CAP_STOW);


        if(base_y(false))
            Hooks.hook_rotate_to_position(Presets.HOOK_UP);
        else if(base_a())
            Hooks.hook_rotate_to_position(Presets.HOOK_DOWN);

        // Set Motor Powers and Servos to Their Commands
        Subsystem.set_motor_powers();
        Subsystem.set_servo_positions();

        // Add Telemetry to Phone for Debugging and Testing if it is Activated
        if (tower_start(false)) {
            telemetry.addData("Status: ", "Running Teleop");
            telemetry.addData("Powers: ", "");
            telemetry.addData("right_lift_Pos", Subsystem.robot.right_lift.getCurrentPosition());
            telemetry.addData("left_lift_Pos", Subsystem.robot.left_lift.getCurrentPosition());
            telemetry.addData("Base RightF Power: ", Subsystem.robot.rightF_drive.getPower());
            telemetry.addData("Base RightB Power: ", Subsystem.robot.rightB_drive.getPower());
            telemetry.addData("Base LeftF Power: ", Subsystem.robot.leftF_drive.getPower());
            telemetry.addData("Base LeftB Power: ", Subsystem.robot.leftB_drive.getPower());
            telemetry.update();
        } else {
            // Telemetry Update to Inform Drivers That the Program is Running and how to Access Telemetry
            telemetry.addData("Status: ", "Running Teleop");
            telemetry.addData("Press Start on the Tower Gamepad for Telemetry", "");
            telemetry.update();
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