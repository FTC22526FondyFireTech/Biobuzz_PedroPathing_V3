package org.firstinspires.ftc.teamcode.simulator.drivetrains;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.simulator.simulators.GyroSimulator;
import org.firstinspires.ftc.teamcode.simulator.simulators.MotorSimulator;
import org.firstinspires.ftc.teamcode.simulator.simulators.OdometrySimulator;
import org.firstinspires.ftc.teamcode.simulator.simulators.SimulatedLocalizer;
import org.firstinspires.ftc.teamcode.utils.Configurables;
import org.firstinspires.ftc.teamcode.utils.Drawing;
import org.firstinspires.ftc.teamcode.utils.FieldConstants;
import org.firstinspires.ftc.teamcode.utils.GlobalData;

import java.lang.reflect.Field;

@Configurable
public class MecanumDriveSubsystemSimulation extends SubsystemBase {

    private final TelemetryManager telemetryM;


    protected MotorSimulator simFL, simFR, simBL, simBR;
    protected GyroSimulator gyro;
    protected OdometrySimulator odometry;

    protected ElapsedTime simTimer;
    protected double simDt = 0.01; // 10ms fixed physics step (100Hz)
    protected double lastUpdateTime = 0;
    private double accumulatedTime = 0; // real time not yet caught up to by fixed-step updates
    private static final double MAX_CATCHUP_SECONDS = 0.5; // cap backlog after a genuine pause
    private boolean teleopDriveActive = false;
    public boolean isRobotCentric = false;
    private int currentFieldZoneSim;

   public Path pc = null;

    public boolean isRobotCentric() {
        return isRobotCentric;
    }

    public void setRobotCentric(boolean robotCentric) {
        isRobotCentric = robotCentric;
    }

    private int tstcnt;
    OpMode opMode;

    ElapsedTime et;
//
//    private static final Style robotLook = new Style(
//            "", "#3F51B5", 2.0
//    );

    public MecanumDriveSubsystemSimulation(OpMode opmode) {
        this.opMode = opmode;

        simFL = new MotorSimulator(true, 435);
        simFR = new MotorSimulator(true, 435);
        simBL = new MotorSimulator(false, 435);
        simBR = new MotorSimulator(false, 435);
        gyro = new GyroSimulator();

        // Create odometry calculator
        odometry = new OdometrySimulator(simFL, simFR, simBL, simBR, gyro);
        odometry.resetPose();

        // Reset timing
        simTimer = new ElapsedTime();
        simTimer.reset();
        lastUpdateTime = 0;
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        et = new ElapsedTime();
       Drawing.init();

    }


    /**
     * Runs once per scheduler loop (SubsystemBase auto-registers this). Advances the
     * Follower's localization + path-following state machine and pushes telemetry to
     * both the Driver Station and the FTC Panels dashboard.
     */
    @Override
    public void periodic() {
        double currentTime = simTimer.seconds();
        double realDt = currentTime - lastUpdateTime;
        lastUpdateTime = currentTime;

        // Advance in fixed-size physics steps so simulated speed is independent of how
        // often periodic() actually gets called. A single dt-capped step would silently
        // throttle simulated motion whenever the real loop runs slower than 1/simDt (e.g.
        // due to per-frame telemetry/network I/O) - it discarded the leftover real time
        // instead of catching up, making the robot appear to move far slower than commanded.
        // Cap the backlog (not the per-step dt) so a genuine multi-second pause doesn't
        // trigger a huge burst of catch-up steps in one frame.
        accumulatedTime = Math.min(accumulatedTime + realDt, MAX_CATCHUP_SECONDS);

        while (accumulatedTime >= simDt) {
            // Derive the rotate component driving the gyro from whatever wheel powers are
            // currently set - same mecanum forward kinematics as OdometrySimulator's inverse
            // of drive()'s FL=f+r+w, FR=f-r-w, BL=f-r+w, BR=f+r-w mixing (negated: with left
            // wheels positive/right wheels negative, the robot physically spins clockwise, i.e.
            // negative heading under the standard CCW-positive convention OdometrySimulator's
            // pose math uses - so the recovered rotate power must flip sign to drive the gyro
            // in that same CCW-positive convention). This keeps the gyro correct regardless of
            // whether powers came from drive() (teleop) or were written directly via
            // setDrivePowers() (e.g. a Pedro Pathing Follower driving the sim).
            double rotatePower = (simFR.getPower() - simFL.getPower() + simBR.getPower() - simBL.getPower()) / 4.0;

            simFL.update(simFL.getPower(), simDt);
            simFR.update(simFR.getPower(), simDt);
            simBL.update(simBL.getPower(), simDt);
            simBR.update(simBR.getPower(), simDt);

            // Update gyro from the actual rotate power (independent of wheel ticks)
            gyro.update(rotatePower, simDt);

            // Update odometry from encoder + gyro values
            odometry.update();

            accumulatedTime -= simDt;
        }
        if (et.seconds() > .5) {
            setActiveHiveTargetPose();
            et.reset();
        }

//
        Drawing.drawRobot(odometry.getRobotPose(), GlobalData.robotLook);
        Drawing.drwaLine(odometry.getRobotPose().x(), odometry.getRobotPose().y(),
                GlobalData.getActiveHiveTargetPose().x(), GlobalData.getActiveHiveTargetPose().y());
        Drawing.sendPacket();

    }

    public void setActiveHiveTargetPose() {

        double x = odometry.getRobotPose().x();
        double y = odometry.getRobotPose().y();


        if (x < 72 && y >= 72) {
            GlobalData.setCurrentOdometryZone(0);
            GlobalData.setActiveHiveTargetPose(FieldConstants.redScoringHiveTargetPose);
            GlobalData.setActiveStartPose(FieldConstants.redScoringLoHiveStartPose);
        } else if (x < 72 && y < 72) {
            GlobalData.setCurrentOdometryZone(1);
            GlobalData.setActiveHiveTargetPose(FieldConstants.redAudienceHiveTargetPose);
            GlobalData.setActiveStartPose(FieldConstants.redAudienceHiHiveStartPose);
        } else if (x >= 72 && y < 72) {
            GlobalData.setCurrentOdometryZone(2);
            GlobalData.setActiveHiveTargetPose(FieldConstants.toBlue(FieldConstants.redScoringHiveTargetPose));
            GlobalData.setActiveStartPose(FieldConstants.toBlue(FieldConstants.redScoringLoHiveStartPose));
        } else if (x >= 72 && y > 72) {
            GlobalData.setCurrentOdometryZone(3);
            GlobalData.setActiveHiveTargetPose(FieldConstants.toBlue(FieldConstants.redAudienceHiveTargetPose));
            GlobalData.setActiveStartPose(FieldConstants.toBlue(FieldConstants.redAudienceHiHiveStartPose));
        }

    }


    public void showTelemetry(TelemetryManager telemetryM) {

        if (Configurables.showPoseTelemetry) {
            // Pose telemetry
            telemetryM.addData("SimX (in)", odometry.getRobotPose().x());
            telemetryM.addData("SimY (in)", odometry.getRobotPose().y());
            telemetryM.addData("SimHeading (deg)", Math.toDegrees(odometry.getRobotPose().heading()));
            telemetryM.addData("Current Zone", currentFieldZoneSim);
            telemetryM.update();
        }

        // Motor telemetry

        if (Configurables.showMotorTelemetry) {
            telemetryM.addData("sim_motor_fl_power", simFL.getPower());
            telemetryM.addData("sim_motor_fl_rpm", simFL.getVelocityRPM());
            telemetryM.addData("sim_motor_fl_ipm", simFL.getVelocityIPM());

            telemetryM.addData("sim_motor_fl_ticks", simFL.getEncoderTicks());
            telemetryM.addData("sim_motor_fl_inches", simFL.getDistanceInches());

            telemetryM.addData("sim_motor_fr_power", simFR.getPower());
            telemetryM.addData("sim_motor_fr_rpm", simFR.getVelocityRPM());
            telemetryM.addData("sim_motor_fr_ticks", simFR.getEncoderTicks());
            telemetryM.addData("sim_motor_fr_inches", simFR.getDistanceInches());

            telemetryM.addData("sim_motor_br_power", simBL.getPower());
            telemetryM.addData("sim_motor_br_rpm", simBL.getVelocityRPM());
            telemetryM.addData("sim_motor_br_ticks", simBL.getEncoderTicks());
            telemetryM.addData("sim_motor_br_inches", simBL.getDistanceInches());

            telemetryM.addData("sim_motor_br_power", simBR.getPower());
            telemetryM.addData("sim_motor_br_rpm", simBR.getVelocityRPM());
            telemetryM.addData("sim_motor_br_ticks", simBR.getEncoderTicks());
            telemetryM.addData("sim_motor_br_inches", simBR.getDistanceInches());


        }

        if (Configurables.showEncoderTelemetry) {
            // Encoder telemetry (mecanum wheel config)
            telemetryM.addData("sim/encoder/leftFront", odometry.getLeftFrontTicks());
            telemetryM.addData("sim/encoder/rightFront", odometry.getRightFrontTicks());
            telemetryM.addData("sim/encoder/leftRear", odometry.getLeftRearTicks());
            telemetryM.addData("sim/encoder/rightRear", odometry.getRightRearTicks());

            // Gyro telemetry
            telemetryM.addData("sim/gyro/heading (deg)", gyro.getHeadingDegrees());
            telemetryM.addData("sim/gyro/angularVelocity (deg/s)", gyro.getAngularVelocityDegPerSec());

            // Debug telemetry
            telemetryM.addData("sim/delta/forward (in)", odometry.getLastDeltaForward());
            telemetryM.addData("sim/delta/strafe (in)", odometry.getLastDeltaStrafe());
            telemetryM.addData("sim/delta/heading (rad)", odometry.getLastDeltaHeading());
        }

        if (Configurables.showDebugTelemetry) {

            // telemetryM.addData("TicksPerinch", MotorSimulator.getTicksPerInch());
        }

    }

    /**
     * Switches the Follower into open, driver-controlled mode. Call this once before
     * feeding it stick inputs (a default teleop DriveCommand should do this in its
     * initialize()). Automatically re-armed by {@link #driveRobotCentric} /
     * {@link #driveFieldCentric} if a path-following command just finished.
     */
    public void startTeleopDrive() {
        teleopDriveActive = true;
    }

    /**
     * Drives with stick input relative to the robot's own heading.
     */
    public void driveRobotCentric(double forward, double strafe, double turn) {

    }

    /**
     * Drives with stick input relative to the field (forward is always "away from driver").
     */
    public void driveFieldCentric(double forward, double strafe, double turn) {

    }

    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        if (isRobotCentric()) {
            double frontLeftPower = forward + right + rotate;
            double frontRightPower = forward - right - rotate;
            double backRightPower = forward + right - rotate;
            double backLeftPower = forward - right + rotate;


            double maxPower = 1.0;
            double maxSpeed = 1.0;  // make this slower for outreaches

            // This is needed to make sure we don't pass > 1.0 to any wheel
            // It allows us to keep all the motors in proportion to what they should
            // be and not get clipped
            maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
            maxPower = Math.max(maxPower, Math.abs(frontRightPower));
            maxPower = Math.max(maxPower, Math.abs(backRightPower));
            maxPower = Math.max(maxPower, Math.abs(backLeftPower));


            // We multiply by maxSpeed so that it can be set lower for outreaches
            // When a young child is driving the robot, we may not want to allow full
            // speed.
            simFL.setPower(maxSpeed * (frontLeftPower / maxPower));
            simFR.setPower(maxSpeed * (frontRightPower / maxPower));
            simBL.setPower(maxSpeed * (backLeftPower / maxPower));
            simBR.setPower(maxSpeed * (backRightPower / maxPower));

        } else {
            double botHeading = getOdometry().getHeading();

            // Rotate the movement direction counter to the bot's rotation
            double rotX = right * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
            double rotY = right * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotate), 1);
            double frontLeftPower = (rotY + rotX + rotate) / denominator;
            double backLeftPower = (rotY - rotX + rotate) / denominator;
            double frontRightPower = (rotY - rotX - rotate) / denominator;
            double backRightPower = (rotY + rotX - rotate) / denominator;

            simFL.setPower(frontLeftPower);
            simBL.setPower(backLeftPower);
            simFR.setPower(frontRightPower);
            simBR.setPower(backRightPower);
        }


//        telemetryM.addData("YVEL",simFL.getPower());
//        telemetryM.update();
    }

    public void setDrivePowers(double[] powers) {
        simFL.setPower(powers[0]);
        simFR.setPower(powers[1]);
        simBL.setPower(powers[2]);
        simBR.setPower(powers[3]);
    }

    /**
     * Exposes the simulated odometry so a {@link SimulatedLocalizer}
     * can be built for a Pedro Pathing {@code Follower} to drive this subsystem.
     */
    public OdometrySimulator getOdometry() {
        return odometry;
    }

    public void stop() {
        driveRobotCentric(0, 0, 0);
    }

    /**
     * Follows an autonomous path and holds its end heading/position once finished.
     */


}
