package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import java.util.Collections;

//@Autonomous(name = "Blank")
@TeleOp(name = "Servo")
@Disabled

public class ArmServoCommandOpmode extends CommandOpMode {

    TelemetryManager telemetryM;

    private Servo armServo;

    GamepadEx testGP1;
    public static double leftUpPosition = 0.0;
    public static double leftNoNectarPosition = .3;
    public static double leftDownPosition = .5;

    public static double rightUpPosition = 0.8;
    public static double rightNoNectarPosition = .3;
    public static double rightDownPosition = .5;

    public double test1;
    public double test2;
    public double test3;
    private double leftArmTargetPosition;
    private double rightArmTargetPosition;


    @Override
    public void initialize() {


        testGP1 = new GamepadEx(gamepad1);
        armServo = hardwareMap.get(Servo.class, "armServo");
        armServo.setDirection(Servo.Direction.FORWARD);
        armServo.setPosition(leftUpPosition);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
telemetryM.addData("INIT", "INIT");
        telemetryM.update(telemetry);

        testGP1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(Commands.runOnce(() -> armServo.setPosition(leftUpPosition)));

        testGP1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(Commands.runOnce(() -> armServo.setPosition(leftDownPosition)));

        testGP1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(armUpCommand());

        testGP1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(Commands.runOnce(() -> {
                    leftArmTargetPosition = leftUpPosition;
                    rightArmTargetPosition = rightUpPosition;
                    armServo.setPosition(leftArmTargetPosition);
                }));

        testGP1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(Commands.defer(() -> (
                                stepArmPositionDownCommand()),
                        Collections.emptyList()));
    }

    public double stepArmPositionDown() {
        if (leftArmTargetPosition == leftUpPosition) {
            leftArmTargetPosition = leftNoNectarPosition;

        } else if ( leftArmTargetPosition == leftNoNectarPosition) {
            leftArmTargetPosition = leftDownPosition;
        } else if ( leftArmTargetPosition == leftDownPosition) {
            leftArmTargetPosition = leftUpPosition;
        }
        armServo.setPosition(leftArmTargetPosition);
        return leftArmTargetPosition;
    }

    public Command stepArmPositionDownCommand() {
        return Commands.runOnce(() -> {
            stepArmPositionDown();
        });
    }

    public Command armUpCommand() {
        return Commands.runOnce(() -> {
            leftArmTargetPosition = leftUpPosition;
            rightArmTargetPosition = rightUpPosition;
            armServo.setPosition(leftArmTargetPosition);
        });
    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            run();

            telemetryM.addData("LAT", leftArmTargetPosition);
            telemetryM.update(telemetry);
        }
        reset();
    }

}