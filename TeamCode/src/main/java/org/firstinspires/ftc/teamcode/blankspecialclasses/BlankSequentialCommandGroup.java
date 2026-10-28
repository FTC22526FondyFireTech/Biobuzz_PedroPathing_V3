package org.firstinspires.ftc.teamcode.blankspecialclasses;


import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

/**
 * A complex auto command that drives forward,
 * releases a stone, and then drives backward.
 */
public class BlankSequentialCommandGroup extends SequentialCommandGroup {

    private static final double INCHES = 3.0;
    private static final double SPEED = 0.5;

    /**
     * Creates a new sequential command group.
     */
    public BlankSequentialCommandGroup( CommandOpMode opmode) {
        addCommands(

                new SequentialCommandGroup(
                        new ParallelCommandGroup()


                )


        );

    }

}


