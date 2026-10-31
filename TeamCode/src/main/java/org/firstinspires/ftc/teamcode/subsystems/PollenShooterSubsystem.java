package org.firstinspires.ftc.teamcode.subsystems;

import android.telecom.TelecomManager;

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
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.simulator.simulators.MotorSimulator;
import org.firstinspires.ftc.teamcode.utils.Configurables;

import java.util.Locale;

@Configurable
public class PollenShooterSubsystem extends SubsystemBase {

    public static double pollenShooterKp = 0.001;
    public static double pollenShooterKi = 0;
    public static double pollenShooterKd = 0;
    public static boolean changePollenVelocityCoefficents = false;
    private double targetRPM = 4000;
    public static double autoShootRPM = 4000;
    public static double autoSecondShootRPM = 3500;

    public static double holdPosition = 0;
    public static double shootPosition = .5;
    public double cpr;
    public MotorEx pollenShooterMotor;
    private MotorSimulator shooterMotorSim;

    private boolean direction;


    public PIDController pidController;

    public final double maxMotorRPM = 6000;
    private double maxShootRPM;
    private double minShootRPM;
    public double pidout;
    private double ffRatio = .95 / maxMotorRPM;
    public double ff;
    public int tst;

    InterpLUT pollenSpeeds;
    private Servo pollenShootServo;
    public double shooterServoTargetPosition = .05;

    public boolean isRunPollenShooterAtVelocity() {
        return runPollenShooterAtVelocity;
    }

    public void setRunPollenShooterAtVelocity(boolean runPollenShooterAtVelocity) {
        this.runPollenShooterAtVelocity = runPollenShooterAtVelocity;
    }

    private boolean runPollenShooterAtVelocity;


    static String f3(double v) {
        return String.format(Locale.US, "%.3f", v);
    }

    static String f0(double v) {
        return String.format(Locale.US, "%.0f", v);
    }

    static String f3M2I(double v) {
        return String.format(Locale.US, "%.3f", v * 39.37);
    }


    public PollenShooterSubsystem(HardwareMap hardwareMap) {
        // specifying motor allows top rpm tp be read from motor
        if (!Configurables.doSimulation) {
            pollenShooterMotor = new MotorEx(hardwareMap, "pollen");
            pollenShooterMotor.setInverted(true);
            maxShootRPM = maxMotorRPM * .9;
            minShootRPM = maxShootRPM / 2;
            cpr = 28;

            pollenShootServo = hardwareMap.get(Servo.class, "pollenShoot");
            pollenShootServo.setDirection(Servo.Direction.FORWARD);
            pollenShootServo.setPosition(holdPosition);
        } else {
            shooterMotorSim = new MotorSimulator(true, 1150);
            shooterMotorSim.setInverted(true);
            direction = shooterMotorSim.isInverted();
        }

        pidController = new PIDController(pollenShooterKp, pollenShooterKi, pollenShooterKd);


        //distance in inches, speeds in RPM
        pollenSpeeds = new InterpLUT() {
            {

                add(10, 100);
                add(25.0, 1500);
                add(30.0, 1600);
                add(35.0, 1750);
                add(40.0, 1800);
                add(45.0, 1900);
                add(50.0, 2000.);
                add(55.0, 2200.);
                add(100, 2500);
            }

        };
        pollenSpeeds.createLUT();

    }

    public void setVelocityCoefficients() {
        pidController.setPID(pollenShooterKp, pollenShooterKi, pollenShooterKd);
    }

    public void runPollenShooter(double pct) {
        if (!Configurables.doSimulation)
            pollenShooterMotor.set(pct);
        else shooterMotorSim.setPower(pct);
    }

    public Command jogPollenShooterCommand(double pct) {
        return Commands.runOnce(() -> runPollenShooter(pct), this);
    }

    public void stopPollenShooter() {
        if (!Configurables.doSimulation) {
            pollenShooterMotor.stopMotor();
            pollenShooterMotor.set(0);
            setRunPollenShooterAtVelocity(false);

        } else {
            shooterMotorSim.setPower(0);
        }
        setRunPollenShooterAtVelocity(false);
    }

    public Command stopPollenShooterCommand() {
        return Commands.runOnce(this::stopPollenShooter, this);
    }

    /**
     * Calculate the PIDF output from the controller and apply it to the motor
     * The F value is kf * target rpm
     * The controller setpoint is set by setTargetRPM()
     * The motor defaults to the raw power mode
     */
    public void runPollenShooterAtVelocity() {
        if (!Configurables.doSimulation) {
            tst++;
            pidout = pidController.calculate(getPollenShooterRPM(), targetRPM);
            ff = ffRatio * targetRPM;
            pollenShooterMotor.set(ff + pidout);
        } else shooterMotorSim.setPower(ff + pidout);
    }

    public boolean pollenShooterAtSpeed() {
        if (!Configurables.doSimulation)
            return Math.abs(getTargetRPM() - getPollenShooterRPM()) < getTargetRPM() / 10;
        else return true;
    }

    public double getVelocityError() {
        return pidController.getVelocityError();
    }

    public Command runPollenShooterAtVelocityCommand() {
        return Commands.runOnce(() -> setRunPollenShooterAtVelocity(true));
    }

    public double getPollenShooterRPM() {
        if (!Configurables.doSimulation) {
            return pollenShooterMotor.getVelocity() * 60. / cpr;
        } else if (runPollenShooterAtVelocity) return targetRPM;
        else return 0;
    }

    public  double getPollenCurrent(){
        return pollenShooterMotor.getCurrent(CurrentUnit.AMPS);
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
        targetRPM = RPM;
        pidController.setSetPoint(targetRPM);
    }

    public Command setTargetRPMCommand(double RPM) {
        return Commands.runOnce(() -> setTargetRPM(RPM));
    }

    public void changeTargetRPM(double val) {
        double tempRPM = getTargetRPM() + val;
        if (tempRPM > maxShootRPM) tempRPM = maxShootRPM;
        if (tempRPM < minShootRPM) tempRPM = minShootRPM;
        setTargetRPM(tempRPM);
    }

    public Command changePollenTargetRPMCommand(double val) {
        return Commands.runOnce(() -> changeTargetRPM(val));
    }

    public double getTargetRPMFromDistance(double inches) {
        if (inches > 100 || inches < 20) return 0;
        else
            return pollenSpeeds.get(inches);
    }


    public void positionPollenShooter(double position) {
        if (!Configurables.doSimulation)
            pollenShootServo.setPosition(position);
        else shooterServoTargetPosition = position;
    }

    public Command shootPollenCommand() {
        return Commands.runOnce(() -> positionPollenShooter(shootPosition));
    }

    public Command holdPollenCommand() {
        return Commands.runOnce(() -> positionPollenShooter(holdPosition));

    }

    public double getPollenShooterServoPosition() {
        return shooterServoTargetPosition;
    }

    @Override
    public void periodic() {
        if (runPollenShooterAtVelocity) {
            runPollenShooterAtVelocity();
        }

        if (changePollenVelocityCoefficents) {
            pidController.setPID(pollenShooterKp, pollenShooterKi, pollenShooterKd);
        }
    }

    public void showTelemetry(TelemetryManager telemetryM) {
        telemetryM.addData("PollenTargetRPM", f0(getTargetRPM()));
        telemetryM.addData("PollenActualRPM", f0(getPollenShooterRPM()));
        telemetryM.addData("PollenActualRPM", f0(getPollenShooterRPM()));
        telemetryM.addData("PollenAmps", f3(getPollenCurrent()));

        if (Configurables.showPollenDebug) {
            telemetryM.addData("PollenFF", ff);
            telemetryM.addData("PollenPID", pidout);
        }
    }
}

