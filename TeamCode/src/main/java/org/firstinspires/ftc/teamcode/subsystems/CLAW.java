package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

public class CLAW extends Subsystem {
    // BOILERPLATE
    public static final CLAW INSTANCE = new CLAW();
    private CLAW() { }

    // USER CODE
    public Servo servo;

    public String name = "Claw";

    final double clawScale0 = Subsystem_Constants.clawScale0;
    final double clawScale1 = Subsystem_Constants.clawScale1;
    final double closeClaw = Subsystem_Constants.closeClaw;
    final double openClaw = Subsystem_Constants.openClaw;
    final double openMore = Subsystem_Constants.openMore;

    public Command openClaw() {
        return new ServoToPosition(servo, // SERVO TO MOVE
                openClaw, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command closeClaw() {
        return new ServoToPosition(servo, // SERVO TO MOVE
                closeClaw, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command openClawMore() {
        return new ServoToPosition(servo, // SERVO TO MOVE
                openMore, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }

    @Override
    public void initialize() {
        servo = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, name);
        servo.scaleRange(clawScale0,clawScale1);
    }
}
