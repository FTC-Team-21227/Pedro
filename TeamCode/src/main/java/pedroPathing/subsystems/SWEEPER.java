package pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

public class SWEEPER extends Subsystem {
    // BOILERPLATE
    public static final SWEEPER INSTANCE = new SWEEPER();
    private SWEEPER() { }
    // USER CODE
    public Servo servo;

    public String name = "Sweeper";
    final double sweeperScale0 = Subsystem_Constants.sweeperScale0;
    final double sweeperScale1 = Subsystem_Constants.sweeperScale1;
    final double closeSweeper = Subsystem_Constants.closeSweeper;
    final double openSweeper = Subsystem_Constants.openSweeper;


    public Command RotatePosition0() {
        return new ServoToPosition(servo,closeSweeper,this);
    }
    public Command RotatePosition1() {
        return new ServoToPosition(servo,openSweeper,this);
    }
    @Override
    public void initialize() {
        servo = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, name);
        servo.scaleRange(sweeperScale0,sweeperScale1);
    }
}
