package org.firstinspires.ftc.teamcode.Robot;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Arm {
    private DcMotorEx leftArm;
    private DcMotorEx rightArm;
    private PIDController controller;

    private double target = 0;
    private double pick_target = 0, score_target = 510;

    private double p = 0.02, i = 0, d = 0.0000010, f = -0.2;
    private double ticks_in_degree = 1200;

    private LinearOpMode opMode;

    public Arm (LinearOpMode opMode) {
        this.leftArm = opMode.hardwareMap.get(DcMotorEx.class, HardwareNames.leftArm);
        this.rightArm = opMode.hardwareMap.get(DcMotorEx.class, HardwareNames.rightArm);


        this.rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightArm.setDirection(DcMotorSimple.Direction.REVERSE);

        this.leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.controller = new PIDController(p, i, d);

        this.opMode = opMode;
    }

    public void setArmState(Core.WristState state) {
        switch (state) {
            case PICK: {
                target = pick_target;
            } break;

            case SCORE: {
                target = score_target;
            } break;

        }
    }

    public void update() {
        int motorPositionL = leftArm.getCurrentPosition();
        int motorPositionR = rightArm.getCurrentPosition();


        double pid = controller.calculate(motorPositionL, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        double power = pid + ff;

        controller.setPID(p, 0, d);
        double pidR = controller.calculate(motorPositionR, target);
        double powerR = pidR + ff;

        opMode.telemetry.addData("amps", leftArm.getCurrent(CurrentUnit.AMPS));

        leftArm.setPower(power);
        rightArm.setPower(powerR);
    }


}
