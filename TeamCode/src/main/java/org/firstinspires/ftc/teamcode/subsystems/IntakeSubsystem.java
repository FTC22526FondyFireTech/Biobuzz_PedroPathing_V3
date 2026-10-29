package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.simulator.simulators.MotorSimulator;
import org.firstinspires.ftc.teamcode.utils.Configurables;

@Configurable
public class IntakeSubsystem extends SubsystemBase {

    public static double upPosition = 0;
    public static double downPosition = .5;
    public Motor intakeMotor;
    private MotorSimulator intakeMotorSim;
    private boolean direction;


    public int pollenCount;
    public int redNectarCount;
    public int blueNectarCount;
    public int allianceNectarCount;
    public int opponemtNectarCount;


    private Servo armServo;
    public static double intakeSpeed = 0.9; // 1150 RPM * 0.77 (or / 77%) = 885.5
    protected double simDt = 0.01; // 10ms fixed physics step (100Hz)
    private double armTargetPosition;
    public boolean intakeRunning;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        if (!Configurables.doSimulation) {
            intakeMotor = new Motor(hardwareMap, "intake", Motor.GoBILDA.RPM_1150);
            intakeMotor.setInverted(true); // change to true if initially out-taking
            direction = intakeMotor.getInverted();
            armServo = hardwareMap.get(Servo.class, "armServo");
            armServo.setDirection(Servo.Direction.FORWARD);
            armServo.setPosition(upPosition);
        } else {
            intakeMotorSim = new MotorSimulator(true, 1150);
            intakeMotorSim.setInverted(true);
            direction = intakeMotorSim.isInverted();
        }
        intakeRunning = false;
    }

    public void runIntake() {
        intakeRunning = true;
        if (!Configurables.doSimulation)
            intakeMotor.set(intakeSpeed);
        else intakeMotorSim.setPower(intakeSpeed);
    }

    public void stopIntake() {
        intakeRunning = false;
        if (!Configurables.doSimulation)
            intakeMotor.stopMotor();
        else intakeMotorSim.setPower(0);
    }

    public void invertIntake() {
        direction = !direction;
        intakeMotor.setInverted(direction);
    }

    public double getVelocityRPM() {
        if (!Configurables.doSimulation) {
            return intakeMotor.getCorrectedVelocity();
        } else {
            return intakeSpeed * 1100;
        }
    }

    public double getPower() {
        if (!Configurables.doSimulation) {
            return intakeMotor.getRawPower();
        } else {
            return intakeMotorSim.getPower();
        }
    }

    @Override
    public void periodic() {
        if (Configurables.doSimulation)
            intakeMotorSim.update(intakeMotorSim.getPower(), simDt);

    }

    public Command runIntakeCommand() {
        return Commands.runOnce(this::runIntake, this);
    }

    public Command stopIntakeCommand() {
        return Commands.runOnce(this::stopIntake);

    }

    public Command invertIntakeCommand() {
        return Commands.runOnce(this::invertIntake);
    }


    public void positionArm(double position) {
        armTargetPosition = position;
        if (Configurables.doSimulation)
            armServo.setPosition(position);
    }

    public Command armDownCommand() {
        if (!Configurables.doSimulation)
            return Commands.runOnce(() -> positionArm(downPosition));
        else return Commands.runOnce(() -> armTargetPosition = downPosition);
    }

    public Command armUpCommand() {
        if (!Configurables.doSimulation)
            return Commands.runOnce(() -> positionArm(upPosition));
        else return Commands.runOnce(() -> armTargetPosition = upPosition);
    }

    public Command toggleArmPositionCommand() {
        return Commands.either(
                armUpCommand(),
                armDownCommand(),
                () -> armTargetPosition == downPosition);
    }

    public double getArmPosition() {
        return armTargetPosition;
    }


    public void showTelemetry(TelemetryManager telemetryM) {
            telemetryM.addData("IntakePower", getPower());
            telemetryM.addData("IntakeRPM", getVelocityRPM());
            telemetryM.addData("IntakeRunning", intakeRunning);
            telemetryM.addData("ArmPosition", getArmPosition());

    }


}