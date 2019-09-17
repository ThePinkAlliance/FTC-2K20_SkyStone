package org.firstinspires.ftc.PinkCode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.PinkCode.Subsystems.Subsystem;
import org.firstinspires.ftc.PinkCode.Subsystems.Base;
import org.firstinspires.ftc.PinkCode.Subsystems.Collector;
import org.firstinspires.ftc.PinkCode.Subsystems.Lift;
import org.firstinspires.ftc.PinkCode.Subsystems.Scorer;
import org.firstinspires.ftc.PinkCode.Robot.Hardware;
import org.firstinspires.ftc.PinkCode.Calculations.Presets;

import static org.firstinspires.ftc.PinkCode.OpModes.Auto.center_auto.center_initialize;
import static org.firstinspires.ftc.PinkCode.OpModes.Auto.center_auto.center_stop;
import static org.firstinspires.ftc.PinkCode.OpModes.Auto.center_auto.move_to_build_site;
import static org.firstinspires.ftc.PinkCode.OpModes.Auto.center_auto.grab_block;

// Class for the Autonomous Period of the Game Calling Methods from Subsystems in Sequence
@Autonomous(name="Auto", group="Autonomous")
public class Auto extends OpMode{
    // Set Up Center Auto Case Statement
    static int phase = 0;
    public center_auto center_auto;
    public enum center_auto {
        center_initialize,
        move_to_build_site,
        move_to_block,
        grab_block,
        move_to_build_site2,
        center_stop,



    }

    @Override
    public double getRuntime() {
        return super.getRuntime();
    }

    private boolean skyDetected = false;
    private ElapsedTime runtime = new ElapsedTime();
    private double markedTime;
    public Hardware robot = new Hardware();


    public void init() {
        // Initialize Robot Hardware
        Subsystem.robot.init(hardwareMap);
        robot.init(hardwareMap);
        Subsystem.robot.rightF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Subsystem.robot.rightB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Subsystem.robot.leftF_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Subsystem.robot.leftB_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        center_auto = center_initialize;
    }
    public void loop() {
        // Center Auto Switch Statement
        telemetry.addData("Auto Phase: ", phase);
        telemetry.update();

        switch (center_auto) {
            case center_initialize:// Initialize
                markedTime = runtime.milliseconds();
                center_auto = move_to_build_site;
                break;

            case move_to_build_site:
                Base.drive_by_command(1,1,1,1);
                if((runtime.milliseconds() - markedTime) > 3000)
                {
                    center_auto = center_stop;
                    break;
                }
                else
                {
                    center_auto = move_to_build_site;
                }
            case move_to_block:
                Base.drive_by_command(-1,-1,-1,-1);
                if(skyDetected)
                {
                    markedTime = runtime.milliseconds();
                    center_auto = grab_block;
                }
                else
                {
                    Base.drive_by_command(-1,-1,-1,-1);
                }

            case grab_block:
                Collector.collect();
                if((runtime.milliseconds() - markedTime) > 3000)
                {
                    center_auto = center_stop;
                }
                else
                {
                    center_auto = grab_block;
                }

            case center_stop:  // Lower the collector bucket to start teleop at zero = down
                Collector.collect_stop();
                Lift.lift_stop();
                Scorer.score_rotate_to_position(Presets.SCORER_STOW);
                break;
        }

        Subsystem.set_motor_powers();
        Subsystem.set_servo_positions();
    }
}