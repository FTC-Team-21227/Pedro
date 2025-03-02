package pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

public class INTAKE_ANGLE extends Subsystem {
    // BOILERPLATE
    public static final INTAKE_ANGLE INSTANCE = new INTAKE_ANGLE();
    private INTAKE_ANGLE() { }

    // USER CODE
    public Servo servo;

    public String name = "Intake_Angle";
    final double intake_AngleScale0 = Subsystem_Constants.intake_AngleScale0;
    final double intake_AngleScale1 = Subsystem_Constants.intake_AngleScale1;
    final double intake_AngleFloor = Subsystem_Constants.intake_AngleFloor;
    final double intake_AngleBasket = Subsystem_Constants.intake_AngleBasket;
    final double intake_AngleRung = Subsystem_Constants.intake_AngleRung;
    final double intake_AngleStart = Subsystem_Constants.intake_AngleStart;
    final double intake_AngleWall = Subsystem_Constants.intake_AngleWall;
    final double intake_AngleVertical = Subsystem_Constants.intake_AngleVertical;

    public Command RotatePosition0_left() {
        return new ServoToPosition(servo,intake_AngleFloor,this);
    }
    public Command RotatePosition0_basket() {
        return new ServoToPosition(servo,intake_AngleBasket,this);
    }
    public Command RotatePosition0() {
        return new ServoToPosition(servo,intake_AngleRung,this);
    }
    public Command RotatePosition1() {
        return new ServoToPosition(servo,intake_AngleStart,this);
    }
    public Command RotatePositionNegative1() {
        return new ServoToPosition(servo,intake_AngleWall,this);
    }
    public Command RotatePosition2() {
        return new ServoToPosition(servo,intake_AngleVertical,this);
    }
    @Override
    public void initialize() {
        servo = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, name);
        servo.scaleRange(intake_AngleScale0,intake_AngleScale1);
    }
}
