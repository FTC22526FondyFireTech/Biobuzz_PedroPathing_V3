package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class FieldConstants {

    public final static double fieldLength = 144;
    public final static double fieldWidth = 144;
    public final static double robotLength = 18;
    public final static double robotArmLengthAdder = 6;
    public final static double robotWidth = 18;

    public final static double flowerApproachClearance = 12;
    public static double loadZoneYLength = 24;
    public static double loadZoneXWidth = 12;
    public static double redLoadZoneYStart = 96;

    public final static double redHiveXCenter = 60;
    public final static double redScoringHiveYCenter = 86;
    public final static double redAudienceHiveYCenter = 54;
    public static PoseFactory factory = PoseFactory.radians();

    //Start Poses
    public static Pose redScoringLoHiveStartPose =
            factory.of(redHiveXCenter,
                    fieldLength - robotLength / 2, Math.PI / 2);
    public static Pose redAudienceHiHiveStartPose =
            factory.of(redHiveXCenter,
                    robotLength / 2,
                    1.5 * Math.PI);


    //Park poses
    public static Pose redScoringParkPose =
            factory.of(loadZoneXWidth,
                    redLoadZoneYStart + loadZoneYLength,
                    Math.PI / 2);
    public static Pose redAudienceParkPose =
            factory.of(loadZoneXWidth,
                    redLoadZoneYStart,
                    Math.PI);


    public static Pose redAudienceParkPoseFromStart =
            factory.of(loadZoneXWidth,
                    redLoadZoneYStart,
                    1.5 * Math.PI);


    // Flower Poses
    public static Pose redAllianceWallFlowerPose = factory.of(0, 48, 0);
    public static Pose scoringWallFlowerPose = factory.of(48, fieldLength, 0);


    //Hive targets

    public static Pose redAudienceHiveTargetPose = factory.of(redHiveXCenter, redAudienceHiveYCenter, -Math.PI / 2);
    public static Pose redScoringHiveTargetPose = factory.of(redHiveXCenter, redScoringHiveYCenter, Math.PI / 2);


    //Flower approach poses

    public static Pose scoringWallFlowerApproachPose =
            factory.of(scoringWallFlowerPose.x(),
                    fieldLength - robotLength / 2 - robotArmLengthAdder - flowerApproachClearance,
                    Math.PI / 2);


    //Shoot poses

    public static Pose redScoringSecondShootPose =
            factory.of(scoringWallFlowerPose.x(),//48
                    fieldLength - robotLength / 2 - robotArmLengthAdder - flowerApproachClearance,//117
                    Math.toRadians(105));
    public static Pose scoringWallFlowerApproachControlPointPose = factory.of(55, 114, 0);


    //Control Points for path from alliance wall to scoring wall
    private final Pose point2Control1 = factory.of(28, 65, 0);
    private final Pose point2Control2 = factory.of(29.6127, 107.7505, 0);
    private final Pose point2Control3 = factory.of(60, 112, 0);

    public static final Pose redAllianceParkfromStartControlPointPose = factory.of(20, 70, 0);
    public static final Pose redSecondMoveToShootControlPointPose = factory.of(22, 115, 0);
    public static final Pose lowHiveStartControlPointPose = factory.of(53, 108, 0);

    public static Pose redAllianceWallFlowerApproachPose =
            factory.of(redAllianceWallFlowerPose.x() + robotArmLengthAdder + flowerApproachClearance,
                    redAllianceWallFlowerPose.y(),
                    Math.PI);
    public static Pose redAllianceWallFlowerPickupPose =
            factory.of(redAllianceWallFlowerPose.x() + robotLength / 2 + robotArmLengthAdder,
                    redAllianceWallFlowerPose.y(),
                    Math.PI);


    // Flower pickup poses
    public static Pose scoringWallFlowerPickupPose =
            factory.of(scoringWallFlowerPose.x(),
                    fieldLength - robotLength / 2 - robotArmLengthAdder,
                    Math.PI / 2);

    /**
     * BIOBUZZ: blue = red rotated 180° about field center (72,72).
     */
    static double FIELD = 144;

    public static Pose toBlue(Pose red) {
        return factory.of(
                FIELD - red.x(),
                FIELD - red.y(),
                AngleUnit.normalizeRadians(red.heading() + Math.PI));
    }

    public static Pose forAlliance(Pose redPose, boolean isBlue) {
        return isBlue ? toBlue(redPose) : redPose;
    }

    public static Pose toBlueXY(Pose red) {
        return factory.of(
                FIELD - red.x(),
                FIELD - red.y(),
                0);

    }

}
