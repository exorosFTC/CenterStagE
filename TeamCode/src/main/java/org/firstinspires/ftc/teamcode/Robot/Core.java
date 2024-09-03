package org.firstinspires.ftc.teamcode.Robot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Pose;


public class Core {
    private Chassis chassis;
    private Arm arm;
    private Claw claw;
    public Drone drone;

    public GamepadEx g1;

    public enum WristState {
        PICK, INTERMEDIARY, SCORE
    }

    public enum ClawState {
        OPEN, CLOSE
    }

    private WristState arm_state = WristState.PICK;

    public Core(LinearOpMode opMode) {
        chassis = new Chassis(opMode, HardwareNames.track_width);
        arm = new Arm(opMode);
        claw = new Claw(opMode);
        drone = new Drone(opMode);

        g1 = new GamepadEx(opMode.gamepad1);
    }

    public void drive(Pose velocity) {
        chassis.drive(velocity);
    }

    public void setArmState(WristState state) {
        arm_state = state;

        arm.setArmState(state);
        claw.setWristState(state);
    }

    public WristState getArmState() { return arm_state; }

    public void setClawState(ClawState state) {
        claw.setClawState(state);
    }

    public void update() {
        g1.readButtons();
        arm.update();
    }

}
