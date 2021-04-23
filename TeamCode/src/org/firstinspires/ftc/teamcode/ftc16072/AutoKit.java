package org.firstinspires.ftc.teamcode.ftc16072;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "AutoKit", group = "FTCKitBot")
public class AutoKit extends OpMode {
    private DcMotor left = null;
    private DcMotor right = null;
    private DcMotor armY = null;
    private Servo hand = null;
    private RevColorSensorV3 colorSensor = null;
    private TouchSensor touchSensor = null;

    boolean buttonState = false;
    boolean done = false;

    int numPresses = 0;
    int colorInd = 0; // 0 = green, 1 = yellow, 2 = red

    @Override
    public void init() {
        while (!super.isStarted()) {
            left = hardwareMap.dcMotor.get("left_motor");
            right = hardwareMap.dcMotor.get("right_motor");
            left.setDirection(DcMotor.Direction.REVERSE);
            armY = hardwareMap.dcMotor.get("arm_motor");
            hand = hardwareMap.servo.get("hand_servo");
            colorSensor = (RevColorSensorV3) hardwareMap.get("color_sensor");
            touchSensor = hardwareMap.touchSensor.get("touch_sensor");
            if (touchSensor.isPressed() ^ buttonState) {
                buttonState = !buttonState;
                if (buttonState) {
                    numPresses++;
                }
            }
            telemetry.addData("rgba: ","("+colorSensor.red()+", "+colorSensor.green()+", "+colorSensor.blue()+", "+colorSensor.alpha()+")");
            telemetry.addData("g-a: ",Math.abs(colorSensor.green()-colorSensor.red()));
            colorInd = (colorSensor.green()>colorSensor.red()) ? 0:2;
            colorInd = (Math.abs(colorSensor.green()-colorSensor.red()) <= 0.05) ? 1:colorInd;
            telemetry.addData("colorInd: ",colorInd);
            telemetry.update();
        }
        armY.setPower(-0.1);
    }

    @Override
    public void loop() {
        if (done) return;
        float power;
        if (colorInd == 0) {
            power = 1;
        }
        else if (colorInd == 1) {
            power = 0.5f;
        }
        else {
            power = 0;
        }
        int ticksPerRevHex = (int) armY.getMotorType().getTicksPerRev();
        if (power != 0) {
            for (int i = 0; i < numPresses; i++) {
                int lastPos = armY.getCurrentPosition();
                while (armY.getCurrentPosition() - lastPos < ticksPerRevHex / 8) {
                    armY.setPower(0.7);
                }
                lastPos = armY.getCurrentPosition();
                while (armY.getCurrentPosition() - lastPos < ticksPerRevHex / 8) {
                    armY.setPower(-0.1);
                }
            }
        }

        float startTime = (float) (System.nanoTime()/Math.pow(10,9));
        float lastTime = startTime;
        while (lastTime-startTime < numPresses) {
            lastTime = (float) (System.nanoTime()/Math.pow(10,9));
        }

        while (power == 0) {
            telemetry.addData("rgba: ","("+colorSensor.red()+", "+colorSensor.green()+", "+colorSensor.blue()+", "+colorSensor.alpha()+")");
            telemetry.addData("g-a: ",Math.abs(colorSensor.green()-colorSensor.red()));
            colorInd = (colorSensor.green()>colorSensor.red()) ? 0:2;
            colorInd = (Math.abs(colorSensor.green()-colorSensor.red()) <= 0.05) ? 1:colorInd;
            telemetry.addData("colorInd: ",colorInd);
            power = (new float[] {1,0.5f,0})[colorInd];
            telemetry.update();
        }
        float circumference = 0.295276f*((float) Math.PI);
        float numRevsFor3Ft = 3f/circumference;
        while (true) {
            int lastPos = armY.getCurrentPosition();
            while (armY.getCurrentPosition() - lastPos < ticksPerRevHex*numRevsFor3Ft) {
                left.setPower(power);
                right.setPower(power);
            }
            hand.setPosition(0.5);
            float lastTime2 = startTime;
            while (lastTime2-startTime < 0.5f) {
                lastTime2 = (float) (System.nanoTime()/Math.pow(10,9));
            }
            while (armY.getCurrentPosition() - lastPos < ticksPerRevHex*numRevsFor3Ft) {
                left.setPower(-power);
                right.setPower(-power);
            }
        }
        done = true;
    }
}
