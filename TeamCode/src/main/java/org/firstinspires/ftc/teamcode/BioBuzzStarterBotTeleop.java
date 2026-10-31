/*   MIT License
 *   Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

/*
 * GAMEPAD 1 (driver)
 *   Left stick Y   forward / back
 *   Right stick X  turn
 *   LB (hold)      slow mode
 *
 * GAMEPAD 2 (operator)
 *   RT / LT        intake in / out
 *   Triangle       flywheel on / off
 *   RB (hold)      auto-shoot: spin up, feed windmill once at speed
 *   Cross          windmill forward
 *   Circle         windmill reverse
 *   D-pad L / R    left / right flower servo in
 *   LB             both flower servos out
 *   D-pad Up/Down  flywheel speed +/-
 */
@TeleOp(name = "BioBuzz StarterBot Teleop", group = "StarterBot")
public class BioBuzzStarterBotTeleop extends OpMode {

    private DcMotor leftDrive, rightDrive, intake;
    private DcMotorEx launcher;
    private CRServo leftIntakeServo, rightIntakeServo, windmillServo;

    static final int LAUNCHER_DEFAULT_VELOCITY = 1250;
    static final int LAUNCHER_READY_MARGIN = 50;
    static final int LAUNCHER_STEP = 50;
    static final int LAUNCHER_MIN_TARGET = 600;
    static final int LAUNCHER_MAX_TARGET = 2400;
    static final double SLOW_MODE_SCALE = 0.4;

    int launcherTarget = LAUNCHER_DEFAULT_VELOCITY;
    boolean flywheelOn = false;

    double leftPower, rightPower, intakePower;

    enum LauncherState { OFF, SPINNING_UP, READY }
    LauncherState lastState = null;

    @Override
    public void init() {
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmill");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        // Auto leaves the drive motors in RUN_TO_POSITION, so switch back to plain power control.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        telemetry.addLine("GP1: sticks drive, LB slow");
        telemetry.addLine("GP2: RT/LT intake, Tri flywheel, RB auto-shoot");
        telemetry.addLine("GP2: Cross/Circle windmill, D-pad L/R flower in, LB flower out");
        telemetry.addLine("GP2: D-pad Up/Down flywheel speed");
    }

    @Override
    public void loop() {
        boolean slow = gamepad1.left_bumper;
        double scale = slow ? SLOW_MODE_SCALE : 1.0;
        arcadeDrive(-gamepad1.left_stick_y * scale, gamepad1.right_stick_x * scale);

        if (gamepad2.dpadUpWasPressed()) launcherTarget += LAUNCHER_STEP;
        if (gamepad2.dpadDownWasPressed()) launcherTarget -= LAUNCHER_STEP;
        launcherTarget = Range.clip(launcherTarget, LAUNCHER_MIN_TARGET, LAUNCHER_MAX_TARGET);

        if (gamepad2.triangleWasPressed()) flywheelOn = !flywheelOn;

        intakePower = gamepad2.right_trigger - gamepad2.left_trigger;
        LauncherState state = launch();
        intakePower = Range.clip(intakePower, -1, 1);
        intake.setPower(intakePower);

        if (gamepad2.left_bumper) {
            leftIntakeServo.setPower(-1);
            rightIntakeServo.setPower(-1);
        } else {
            leftIntakeServo.setPower(gamepad2.dpad_left ? 1 : 0);
            rightIntakeServo.setPower(gamepad2.dpad_right ? 1 : 0);
        }

        if (state != lastState) {
            if (state == LauncherState.READY) gamepad2.rumbleBlips(1);
            switch (state) {
                case OFF:         gamepad2.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS); break;
                case SPINNING_UP: gamepad2.setLedColor(1, 1, 0, Gamepad.LED_DURATION_CONTINUOUS); break;
                case READY:       gamepad2.setLedColor(0, 1, 0, Gamepad.LED_DURATION_CONTINUOUS); break;
            }
            lastState = state;
        }

        telemetry.addData("Flywheel", "%s  %.0f / %d", state, launcher.getVelocity(), launcherTarget);
        telemetry.addData("Intake", "%.2f", intakePower);
        telemetry.addData("Drive", "%s  L %.2f  R %.2f", slow ? "SLOW" : "FULL", leftPower, rightPower);
    }

    @Override
    public void stop() {
        launcher.setVelocity(0);
        gamepad2.stopRumble();
    }

    void arcadeDrive(double forward, double rotate) {
        leftPower = forward + rotate;
        rightPower = forward - rotate;

        // Scale both sides down together if either exceeds 1, so turning stays proportional.
        double max = Math.max(1.0, Math.max(Math.abs(leftPower), Math.abs(rightPower)));
        leftPower /= max;
        rightPower /= max;

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    LauncherState launch() {
        boolean autoShoot = gamepad2.right_bumper;
        boolean spinning = flywheelOn || autoShoot;
        launcher.setVelocity(spinning ? launcherTarget : 0);

        boolean ready = spinning && launcher.getVelocity() >= launcherTarget - LAUNCHER_READY_MARGIN;

        if (autoShoot && ready) {
            windmillServo.setPower(1);
            intakePower += 0.5;
        } else if (gamepad2.cross) {
            windmillServo.setPower(1);
        } else if (gamepad2.circle) {
            windmillServo.setPower(-1);
        } else {
            windmillServo.setPower(0);
        }

        if (!spinning) return LauncherState.OFF;
        return ready ? LauncherState.READY : LauncherState.SPINNING_UP;
    }
}
