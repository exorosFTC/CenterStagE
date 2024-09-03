package org.firstinspires.ftc.teamcode.Robot;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;


public class Claw {
    private Servo wrist;
    private Servo claw;
    private LinearOpMode opMode;

    private double open_claw = 0.6;
    private double close_claw = 0.8;

    private double pick_position_wrist = 0.97;
    private double score_position_wrist = 0.25;
    private double intermediary_position_wrist = 0.6;

    public Claw(LinearOpMode opMode) {
        this.opMode = opMode;

        wrist = opMode.hardwareMap.get(Servo.class, HardwareNames.wristServo);
        claw = opMode.hardwareMap.get(Servo.class, HardwareNames.clawServo);
    }

    public void setWristState(Core.WristState state) {
        switch (state) {
            case PICK: {
                wrist.setPosition(pick_position_wrist);
                claw.setPosition(open_claw);
            } break;

            case SCORE: {
                wrist.setPosition(score_position_wrist);
                claw.setPosition(close_claw);
            } break;

            case INTERMEDIARY: {
                claw.setPosition(close_claw);

                opMode.sleep(1000);

                wrist.setPosition(intermediary_position_wrist);
            } break;
        }
    }

    public void setClawState(Core.ClawState state) {
        switch (state) {
            case OPEN: {
                claw.setPosition(open_claw);
            } break;

            case CLOSE: {
                claw.setPosition(close_claw);
            } break;
        }
    }



}
