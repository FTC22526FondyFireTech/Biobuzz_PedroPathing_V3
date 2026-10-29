package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.gamepad.SlewRateLimiter;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.simulator.simulators.MotorSimulator;
import org.firstinspires.ftc.teamcode.utils.Configurables;

import java.util.Locale;

@Configurable
public class NectarShooterSubsystem extends SubsystemBase {

    public static double nectarShooterKp = 0.001;
    public static double nectarShooterKi = 0;
    public static double nectarShooterKd = 0;
    public static boolean changeNectarVelocityCoefficents = false;

    public static double holdPosition = 0;
    public static double shootPosition = .5;

    public double cpr = 28;
    public MotorEx nectarShooterMotor;
    private MotorSimulator shooterMotorSim;

    private boolean direction;

    public PIDController pidController;

    private double targetRPM = 5000;
    public double maxMotorRPM = 6000;
    private double maxShootRPM;

    private double minShootRPM;
    private double ffRatio = .95 / maxMotorRPM;
    public double pidout;

    public double ff;
    public int tst;


    public InterpLUT nectarSpeeds;

    private Servo nectarShootServo;

    public boolean isRunNectarShooterVelocity() {
        return runNectarShooterVelocity;
    }

    public void setRunNectarShooterVelocity(boolean runNectarShooterAtVelocity) {
        this.runNectarShooterVelocity = runNectarShooterAtVelocity;
    }

    private boolean runNectarShooterVelocity;


    static String f3(double v) {
        return String.format(Locale.US, "%.3f", v);
    }

    static String f0(double v) {
        return String.format(Locale.US, "%.0f", v);
    }

    public NectarShooterSubsystem(HardwareMap hardwareMap) {
        // specifying motor allows top rpm tp be read from motor
        if (!Configurables.doSimulation) {
            nectarShooterMotor = new MotorEx(hardwareMap, "nectar");

            maxShootRPM = maxMotorRPM;
            minShootRPM = maxShootRPM / 2;
            nectarShootServo = hardwareMap.get(Servo.class, "nectarShoot");
            nectarShootServo.setDirection(Servo.Direction.FORWARD);
            nectarShootServo.setPosition(holdPosition);
        } else {
            shooterMotorSim = new MotorSimulator(true, 1150);
            shooterMotorSim.setInverted(true);
            direction = shooterMotorSim.isInverted();
        }


        pidController = new PIDController(nectarShooterKp, nectarShooterKi, nectarShooterKd);

//distance in inches, speeds in RPM
        nectarSpeeds = new InterpLUT() {{
            {
                {
                    add(10, 900);
                    add(25.0, 1600);
                    add(30.0, 1660);
                    add(35.0, 1690);
                    add(40.0, 1700);
                    add(45.0, 1790);
                    add(50.0, 1800.);
                    add(55.0, 1900.);
                    add(100, 2000);

                }
            }

        }};
        nectarSpeeds.createLUT();
    }

    public void setVelocityCoefficients() {
        pidController.setPID(nectarShooterKp, nectarShooterKi, nectarShooterKd);
    }

    public void runNectarShooter(double pct) {
        if (!Configurables.doSimulation)
            nectarShooterMotor.set(pct);
        else shooterMotorSim.setPower(pct);
    }

    public Command jogNectarShooterCommand(double pct) {
        return Commands.runOnce(() -> runNectarShooter(pct), this);
    }

    public void stopNectarShooter() {
        if (!Configurables.doSimulation) {
            nectarShooterMotor.stopMotor();
            nectarShooterMotor.set(0);
            setRunNectarShooterVelocity(false);
        } else {
            shooterMotorSim.setPower(0);
        }
    }

    public Command stopNectarShooterCommand() {
        return Commands.runOnce(this::stopNectarShooter, this);
    }

    /**
     * Calculate the PIDF output from the controller and apply it to the motor
     * The F value is kf * target rpm
     * The controller setpoint is set by setTargetRPM()
     * The motor defaults to the raw power mode
     */
    public void runNectarShooterAtVelocity() {
        tst++;
        pidout = pidController.calculate(getNectarShooterRPM(), targetRPM);
        ff -= ffRatio * targetRPM;
        if (!Configurables.doSimulation)
            nectarShooterMotor.set(ff + pidout);
        else shooterMotorSim.setPower(ff + pidout);
    }


    public double getVelocityError() {
        return pidController.getVelocityError();
    }

    public Command runNectarShooterAtVelocityCommand() {
        return Commands.runOnce(() -> setRunNectarShooterVelocity(true));
    }

    public double getNectarShooterRPM() {
        if (!Configurables.doSimulation) {
            return nectarShooterMotor.getVelocity() * 60. / cpr;
        } else return shooterMotorSim.getVelocityRPM();
    }


    public boolean nectarShooterAtSpeed() {
        if (!Configurables.doSimulation)
            return Math.abs(getTargetRPM() - getNectarShooterRPM()) < getTargetRPM() / 10;
        else return true;
    }


    public double getTargetRPM() {
        return targetRPM;
    }

    /**
     * Allows driver to adjust shoot speed
     * Needs 2 buttons +val increases speed -val decreases speed
     *
     * @param RPM
     */
    public void setTargetRPM(double RPM) {
        this.targetRPM = RPM;
        pidController.setSetPoint(targetRPM);
    }

    public void changeTargetRPM(double val) {
        double tempRPM = getTargetRPM() + val;
        if (tempRPM > maxShootRPM) tempRPM = maxShootRPM;
        if (tempRPM < minShootRPM) tempRPM = minShootRPM;

        setTargetRPM(tempRPM);
    }

    public Command changeNectarTargetRPMCommand(double val) {
        return Commands.runOnce(() -> changeTargetRPM(val));
    }


    public Command setTargetRPMCommand(double RPM) {
        return Commands.runOnce(() -> setTargetRPM(RPM));
    }

    public Command getTargetRPMFromDistanceCommand(double inches) {
        return Commands.run(() -> getTargetRPMFromDistance(inches));
    }

    public double getTargetRPMFromDistance(double inches) {
        return nectarSpeeds.get(inches);
    }

    public double getTargetSpeed(double inches) {
        return nectarSpeeds.get(inches);
    }

    public void positionNectarShooter(double position) {
        if (Configurables.doSimulation)
            nectarShootServo.setPosition(position);
    }

    public Command shootNectarCommand() {
        return Commands.runOnce(() -> positionNectarShooter(shootPosition));
    }

    public Command holdNectarCommand() {
        return Commands.runOnce(() -> positionNectarShooter(holdPosition));
    }

    @Override
    public void periodic() {
        if (runNectarShooterVelocity) {
            runNectarShooterAtVelocity();
        }

        if (changeNectarVelocityCoefficents) {
            pidController.setPID(nectarShooterKp, nectarShooterKi, nectarShooterKd);
        }
    }


    public void showTelemetry(TelemetryManager telemetryM) {
        telemetryM.addData("NectarTargetRPM", f0(getTargetRPM()));
        telemetryM.addData("NectarActualRPM", f0(getNectarShooterRPM()));
        if (Configurables.showNectarDebug) {
            telemetryM.addData("PollenFF", f3(ff));
            telemetryM.addData("PollenPID", f3(pidout));
        }
    }
}
