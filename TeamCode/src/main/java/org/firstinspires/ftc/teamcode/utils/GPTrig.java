package org.firstinspires.ftc.teamcode.utils;


import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

public  class GPTrig extends Trigger {
    GamepadEx gp;
    Side side;
    public GPTrig(GamepadEx gp, Side side) {
        this.gp = gp;
        this.side=side;
    }
    @Override
    public boolean get() {
        if (side == side.LEFT)
            return gp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > .75;
        if (side == side.RIGHT)
            return gp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > .75;
        else return false;
    }
    public static enum Side {
        LEFT,
        RIGHT
    }
}
