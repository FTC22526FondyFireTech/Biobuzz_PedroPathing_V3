package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NectarShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PollenShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsysytem;
import org.firstinspires.ftc.teamcode.utils.AutoCommands;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.GPTrig;
import org.firstinspires.ftc.teamcode.utils.GlobalData;

import java.util.List;
import java.util.Locale;

@TeleOp(name = "Device Test", group = "Teleop")
//@Disabled

public class DeviceTestOpmode extends CommandOpMode {

    TelemetryManager telemetryM;
    GamepadEx testGP1;
    GamepadEx testGP2;

    GPTrig leftTrigger1;
    GPTrig rightTrigger1;


    GPTrig leftTrigger2;
    GPTrig rightTrigger2;
    MecanumDriveSubsystem drive;
    VisionSubsysytem vss;
    IntakeSubsystem intake;

    PollenShooterSubsystem pss;

    NectarShooterSubsystem nss;

    AutoCommands ac;

    private Follower follower;

    static String f3(double v) {
        return String.format(Locale.US, "%.3f", v);
    }

    static String f3M2I(double v) {
        return String.format(Locale.US, "%.3f", v * 39.37);
    }


    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetryM.update(telemetry);


        GlobalData.setRedAlliance();

        testGP1 = new GamepadEx(gamepad1);
        testGP2 = new GamepadEx(gamepad2);

        drive = new MecanumDriveSubsystem(this.hardwareMap, Pose.zero());

        drive.setDefaultCommand(new DriveCommand(drive,
                () -> testGP1.getLeftY(),
                () -> -testGP1.getLeftX(),
                () -> testGP1.getRightX()));


        vss = new VisionSubsysytem(this);

        pss = new PollenShooterSubsystem(this.hardwareMap);

        nss = new NectarShooterSubsystem(this.hardwareMap);

        intake = new IntakeSubsystem(this.hardwareMap);

        ac = new AutoCommands(drive, pss, nss);


        follower = Constants.createFollower(this.hardwareMap);


        //GAMEPAD!


        testGP1.getGamepadButton(GamepadKeys.Button.BACK)
                .whileActiveOnce(GlobalData.toggleAllianceCommand());

        testGP1.getGamepadButton(GamepadKeys.Button.START)
                .whileActiveOnce(drive.setPoseCommand(Pose.zero()));


        //Triggers

        leftTrigger1 = new GPTrig(testGP1, GPTrig.Side.LEFT);
        rightTrigger1 = new GPTrig(testGP1, GPTrig.Side.RIGHT);

        //DPAD

        testGP1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(intake.runIntakeCommand());
        testGP1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(intake.stopIntakeCommand());

        testGP1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(intake.armUpCommand());
        testGP1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(intake.armDownCommand());

        testGP1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(pss.jogPollenShooterCommand(.75));
        testGP1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(nss.jogNectarShooterCommand(.25));

        testGP1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(pss.shootPollenCommand());
        testGP1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(nss.shootNectarCommand());

        testGP1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(pss.holdPollenCommand());
        testGP1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(nss.holdNectarCommand());


//GAMEPAD2

        leftTrigger2 = new GPTrig(testGP2, GPTrig.Side.LEFT);
        rightTrigger2 = new GPTrig(testGP2, GPTrig.Side.RIGHT);

        testGP2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(pss.runPollenShooterAtVelocityCommand());

        testGP2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(nss.runNectarShooterAtVelocityCommand());

        leftTrigger2.whenActive(
                Commands.parallel(
                        pss.stopPollenShooterCommand(),
                        nss.stopNectarShooterCommand()));

        testGP2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(pss.changePollenTargetRPMCommand(100));

        testGP2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(pss.changePollenTargetRPMCommand(-100));

        testGP2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(nss.changeNectarTargetRPMCommand(100));

        testGP2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(nss.changeNectarTargetRPMCommand(-100));


    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            run();


            drive.setRobotCentric(testGP1.getGamepadButton(
                    GamepadKeys.Button.RIGHT_BUMPER).get());
            drive.showTelemetry(telemetryM);


            telemetryM.addData("IsBusy", follower.isBusy());

            double dist = 0;
            double angle = 0;
            dist = getDistanceToTarget(GlobalData.getActiveHiveTargetPose(), drive.getPose());
            angle = getAngleToTarget(GlobalData.getActiveHiveTargetPose(), drive.getPose());

            telemetryM.addData("PSOn", pss.isRunPollenShooterAtVelocity());
            telemetryM.addData("PSSPID", pss.pidout);
            telemetryM.addData("PSSSFF", pss.ff);
            telemetryM.addData("PSTGTRPM", pss.getTargetRPM());

            telemetryM.addData("POLLENRPM", pss.getPollenShooterRPM());
            telemetryM.addData("IntakeRPM", intake.getVelocityRPM());


//            telemetryM.addData("DistFromOdometry", f3(dist));
//            telemetryM.addData("AngleFromOdometry", f3(angle));
//
//            telemetryM.addData("TX", f3(GlobalData.getActiveHiveTargetPose().x()));
//            telemetryM.addData("TY", f3(GlobalData.getActiveHiveTargetPose().y()));

            List<LLResultTypes.FiducialResult> fr = vss.getFiducialResults();

            List<Integer> tagsSeen = vss.getTagsSeen(fr);
//            telemetryM.addData("TagsSeen", tagsSeen);
//
//
//            telemetryM.addData("ZoneFromTags", vss.findZoneFromTags(tagsSeen));
//            telemetryM.addData("Current Zone", vss.getCurrentTagZone());
//            telemetryM.addData("Is In Current Zone", vss.isInCurrentZone(fr));

            if (!fr.isEmpty()) {
                double[] temp = vss.getGroupTargetPose(fr);
                double x = temp[0];
                double y = temp[1];
                double z = temp[2];
                double yaw = temp[3];
                double pitch = temp[4];

//                telemetryM.addData("X", f3M2I(x));
//                telemetryM.addData("Y", f3M2I(y));
//                telemetryM.addData("Distance", f3M2I(z));
//                telemetryM.addData("Yaw", f3(yaw));
//                telemetryM.addData("Z", f3(pitch));
//

            }

            LLResult result = vss.getLLResults();
            if (result.isValid()) {

//                telemetryM.addData("tx", f3(result.getTx()));
//                telemetryM.addData("ty", f3(result.getTy()));
//                telemetryM.addData("ta", f3(result.getTa()));
//                telemetryM.addData("Tag Count", vss.getNumberTagsSeen(fr));

            }


            telemetryM.update(telemetry);
        }
        reset();
    }


    public double getAngleToTarget(Pose targetPose, Pose robotPose) {
        double XDiff = targetPose.x() - robotPose.x();
        double YDiff = targetPose.y() - robotPose.y();
        return Math.toDegrees(Math.atan2(YDiff, XDiff));
    }

    public double getDistanceToTarget(Pose targetPose, Pose robotPose) {
        double XDiff = targetPose.x() - robotPose.x();
        double YDiff = targetPose.y() - robotPose.y();
        telemetryM.addData("XDIFF", f3(XDiff));
        telemetryM.addData("YDIFF", f3(YDiff));

        // return Math.sqrt(Math.pow(XDiff, 2) + Math.pow(YDiff, 2));
        return Math.hypot(XDiff, YDiff);
    }


}