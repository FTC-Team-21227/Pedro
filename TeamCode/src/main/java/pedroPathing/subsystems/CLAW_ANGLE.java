package pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

public class CLAW_ANGLE extends Subsystem {
    // BOILERPLATE
    public static final CLAW_ANGLE INSTANCE = new CLAW_ANGLE();
    private CLAW_ANGLE() { }
    // USER CODE
    public Servo servo;

    public String name = "Claw_Angle";
    final double claw_AngleScale0 = Subsystem_Constants.claw_AngleScale0;
    final double claw_AngleScale1 = Subsystem_Constants.claw_AngleScale1;
    final double claw_AngleForward = Subsystem_Constants.claw_AngleForward;
    final double claw_AngleBackward = Subsystem_Constants.claw_AngleBackward;
    final double claw_AngleLeft = Subsystem_Constants.claw_AngleLeft;
    public Command forward() {
        return new ServoToPosition(servo,claw_AngleForward,this);
    }
    public Command backward() {
        return new ServoToPosition(servo,claw_AngleBackward,this);
    }
    public Command sub(){return new ServoToPosition(servo,claw_AngleLeft,this);}
    @Override
    public void initialize() {
        servo = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, name);
        servo.scaleRange(claw_AngleScale0,claw_AngleScale1);
    }
}
