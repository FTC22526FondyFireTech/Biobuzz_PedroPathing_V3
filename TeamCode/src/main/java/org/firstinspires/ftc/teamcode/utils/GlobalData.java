package org.firstinspires.ftc.teamcode.utils;


import com.bylazar.field.Style;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class GlobalData {

    private static boolean redAlliance = false;

    private static boolean blueAlliance = false;

    public static boolean isBlueAlliance() {
        return blueAlliance;
    }

    public static void setBlueAlliance() {
        blueAlliance = true;
        redAlliance = false;
        robotLook = blueLook;
    }

    public static void resetAlliances() {
        blueAlliance = false;
        redAlliance = false;
        robotLook = redLook;
    }

    public static Command setRedAllianceCommand() {
        return new InstantCommand(GlobalData::setRedAlliance);
    }

    public static boolean isRedAlliance() {
        return redAlliance;
    }

    public static void setRedAlliance() {
        redAlliance = true;
        blueAlliance = false;
        robotLook = redLook;
    }

    public static Command setBlueAllianceCommand() {
        return new InstantCommand(GlobalData::setBlueAlliance);
    }

    public static Command toggleAllianceCommand() {
        return new ConditionalCommand(setBlueAllianceCommand(), setRedAllianceCommand(), GlobalData::isRedAlliance);
    }

    public static boolean autoHasRun = false;

    public static Pose robotPose = Pose.zero();

    public static Style robotLook = new Style("", "#3F51B5", 1.0);
    private static final Style redLook = new Style("", "#FF0000", 1.0);
    //  private static final Style blueLook = new Style("", "#0000FF", 1.0);
    private static final Style blueLook = new Style("", "#ADD8E6", 1.0);

    public static Pose getActiveHiveTargetPose() {
        return activeHiveTargetPose;
    }

    public static void setActiveHiveTargetPose(Pose activeTargetPose) {
        activeHiveTargetPose = activeTargetPose;
    }

    public static Pose getActiveStartPose() {
        return activeStartPose;
    }

    public static void setActiveStartPose(Pose activeStartPose) {
        GlobalData.activeStartPose = activeStartPose;
    }

    private static Pose activeStartPose = Pose.zero();

    public static int getCurrentOdometryZone() {
        return currentOdometryZone;
    }

    public static void setCurrentOdometryZone(int currentZone) {
        currentOdometryZone = currentZone;
    }

    private static Pose activeHiveTargetPose = Pose.zero();
    private static int currentOdometryZone = 0;


    public static boolean allianceSelected;
    public static boolean allianceIsConfirmed;
    public static boolean choicesComplete;


    //Method to select starting position and number of artifact group using gamepad
    public static void selectAlliance(OpMode opMode) {
        opMode.telemetry.setAutoClear(true);
        opMode.telemetry.clearAll();
        allianceSelected = false;
        resetAlliances();
        allianceIsConfirmed = false;
        //   opMode.gamepad1.rumble(250);

        while (!allianceIsConfirmed) {
            opMode.telemetry.addData("Selecting Alliance for Team:",
                    "Fondy Fire Tech", " ", "22526");
            opMode.telemetry.addLine("---------------------------------------");
            opMode.telemetry.addData("Select/Change Alliance using Bumpers on Gamepad 1:", "");
            opMode.telemetry.addData("    Blue   ", "Left");
            opMode.telemetry.addData("    Red    ", "Right");
            if (!allianceSelected)
                opMode.telemetry.addLine("NO Alliance Selected");
            if (isRedAlliance())
                opMode.telemetry.addLine("RED Alliance Selected");
            if (isBlueAlliance())
                opMode.telemetry.addLine("BLUE Alliance Selected");


            opMode.telemetry.addLine();

            opMode.telemetry.addLine("Press Left Trigger ToConfirm Selection and Exit");


            opMode.telemetry.addLine();


            if (opMode.gamepad1.left_bumper) {
                if (!isBlueAlliance()) {
                    GlobalData.setBlueAlliance();
                    opMode.telemetry.clear();
                }
            }
            if (opMode.gamepad1.right_bumper) {
                if (!isRedAlliance()) {
                    GlobalData.setRedAlliance();
                    opMode.telemetry.clear();
                }
            }

            allianceSelected = GlobalData.isRedAlliance() || GlobalData.isBlueAlliance();

            opMode.telemetry.addData("Selected", allianceSelected);

            allianceIsConfirmed = allianceSelected && opMode.gamepad1.left_trigger_pressed;
            opMode.telemetry.addData("Confirmed", allianceIsConfirmed);

            opMode.telemetry.update();
        }

        opMode.telemetry.clearAll();
        if (isRedAlliance())
            opMode.telemetry.addLine("RED Alliance Selected");
        if (isBlueAlliance())
            opMode.telemetry.addLine("BLUE Alliance Selected");

        opMode.telemetry.addLine("Alliance Selection Complete");
        opMode.telemetry.addLine("Stop and Init Opmode to Change");

        opMode.telemetry.update();
    }
}
