package org.firstinspires.ftc.teamcode.ftc16072;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "OpModeKit", group = "FTCKitBot")
public class OpModeKit extends OpMode {
    private DcMotor left = null;
    private DcMotor right = null;
    private DcMotor armY = null;
    private Servo hand = null;
    private RevColorSensorV3 colorSensor = null;
    private TouchSensor touchSensor = null;

    boolean buttonState = false;

    @Override
    public void init() {
        left = hardwareMap.dcMotor.get("left_motor");
        right = hardwareMap.dcMotor.get("right_motor");
        left.setDirection(DcMotor.Direction.REVERSE);
        armY = hardwareMap.dcMotor.get("arm_motor");
        hand = hardwareMap.servo.get("hand_servo");
        colorSensor = (RevColorSensorV3) hardwareMap.get("color_sensor");
        touchSensor = hardwareMap.touchSensor.get("touch_sensor");
    }

    @Override
    public void loop() {
        gamepad1.setJoystickDeadzone(0.1f);

        float ljx = gamepad1.left_stick_x;
        float ljy = gamepad1.left_stick_y;
        telemetry.addData("Left joystick", ljx+", "+ljy);

        float rjx = gamepad1.right_stick_x;
        float rjy = gamepad1.right_stick_y;
        telemetry.addData("Right joystick", rjx+", "+rjy);
        if (Math.abs(ljy)>0.1) {
            if (rjx > 0) {
                right.setPower(ljy * (1 - rjx));
                left.setPower(ljy);
            } else {
                left.setPower(ljy * (1 + rjx));
                right.setPower(ljy);
            }
        }
        else {
            if (rjx > 0) {
                left.setPower(ljy-(0.9f*rjx));
                right.setPower(ljy);
            } else {
                right.setPower(ljy+(0.9f*rjx));
                left.setPower(ljy);
            }
        }
        float speed = 0.8f;
        float lt = gamepad1.left_trigger;
        telemetry.addData("Left trigger", lt);
        float rt = gamepad1.right_trigger;
        telemetry.addData("Right trigger", rt);
        armY.setPower((lt-rt)*speed);

        boolean rb = gamepad1.right_bumper;
        telemetry.addData("Right bumper", rb);
        boolean lb = gamepad1.left_bumper;
        telemetry.addData("Left bumper", lb);
        if (lb^rb){
            hand.setPosition(hand.getPosition()+(0.002*(lb ? -1 : 1)));
        }
        telemetry.addData("Left motor", left.getPower());
        telemetry.addData("Right motor", right.getPower());
        telemetry.addData("Arm motor", armY.getPower());
        telemetry.addData("Hand Servo", hand.getPosition());
        String rgba = "("+colorSensor.red()+", "+colorSensor.green()+", "+colorSensor.blue()+", "+colorSensor.alpha()+")";
        telemetry.addData("Color Sensor RGBA", rgba);
        if (touchSensor.isPressed()) {
            buttonState = !buttonState;
        }
        telemetry.addData("Button State", buttonState);
        telemetry.addData("Touch Sensor", touchSensor.isPressed() ? " Is Pressed" : "Not Pressed");

        telemetry.update();
    }
}