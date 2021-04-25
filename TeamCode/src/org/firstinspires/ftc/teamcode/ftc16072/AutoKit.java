package org.firstinspires.ftc.teamcode.ftc16072;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutoKit", group = "FTCKitBot")
public class AutoKit extends LinearOpMode {
    private DcMotor left = null;
    private DcMotor right = null;
    private DcMotor armY = null;
    private Servo hand = null;
    private RevColorSensorV3 colorSensor = null;
    private TouchSensor touchSensor = null;

    private boolean buttonState = false;
    private boolean done = false;

    private boolean lockedIn = false;
    private int lastColorInd = -1;
    private float lastColorIndTime = -1;
    private float colorIndCounter = 0;
    private float lockInTime = 15;

    private int numPresses = 0;
    private int colorInd = 0; // 0 = green, 1 = yellow, 2 = red

    public void initialize() {
        left = hardwareMap.dcMotor.get("left_motor");
        right = hardwareMap.dcMotor.get("right_motor");
        left.setDirection(DcMotor.Direction.REVERSE);
        armY = hardwareMap.dcMotor.get("arm_motor");
        hand = hardwareMap.servo.get("hand_servo");
        colorSensor = (RevColorSensorV3) hardwareMap.get("color_sensor");
        touchSensor = hardwareMap.touchSensor.get("touch_sensor");
    }

    public void initialize_loop() {
        hand.setPosition(0);
        if (touchSensor.isPressed() ^ buttonState) {
            buttonState = !buttonState;
            if (buttonState) {
                numPresses++;
                lockedIn = false;
            }
        }
        if (lockedIn) {
            telemetry.addData("colorInd Locked:", true);
            telemetry.addData("colorInd:", colorInd);
            telemetry.update();
            return;
        }
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        int alpha = Range.clip((int)(colors.alpha * 256), 0, 255);
        int red = Range.clip((int)(colors.red * 256), 0, 255);
        int green = Range.clip((int)(colors.green * 256), 0, 255);
        int blue = Range.clip((int)(colors.blue * 256), 0, 255);
        telemetry.addData("rgba: ","("+red+", "+green+", "+blue+", "+alpha+")");
        telemetry.addData("g-a: ",Math.abs(green-red));
        colorInd = (green>red) ? 0:2;
        colorInd = (Math.abs(green-red) <= 15) ? 1:colorInd;
        telemetry.addData("colorInd: ",colorInd);
        if (colorInd == lastColorInd) {
            float deltaTime = (System.nanoTime()/(float) Math.pow(10,9))-lastColorIndTime;
            colorIndCounter += deltaTime;
            if (colorIndCounter >= lockInTime) {
                lockedIn = true;
            }
        }
        else if (lastColorInd == -1) {
            lastColorIndTime = System.nanoTime()/(float) Math.pow(10,9);
            lastColorInd = colorInd;
        }
        else {
            colorIndCounter = 0;
            lastColorInd = colorInd;
        }
        telemetry.addData("colorInd timer:", colorIndCounter);
        telemetry.addData("colorInd Locked:", lockedIn);
        telemetry.update();
    }

    public void finalLoop() {
        if (done) {
            return;
        }
        armY.setPower(0);
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
                while (Math.abs(armY.getCurrentPosition() - lastPos) < ticksPerRevHex / 6f) {
                    armY.setPower(-0.5);
                    telemetry.addData("armPos Up: ",Math.abs(armY.getCurrentPosition() - lastPos));
                    telemetry.addData("Must be greater than: ",ticksPerRevHex / 6f);
                    telemetry.update();
                }
                lastPos = armY.getCurrentPosition();
                while (Math.abs(armY.getCurrentPosition() - lastPos) < ticksPerRevHex / 10f) {//because of how the motor has some give in that it wont turn if the arm only turns a little
                    telemetry.addData("armPos Down: ",Math.abs(armY.getCurrentPosition() - lastPos));
                    telemetry.addData("Must be greater than: ",ticksPerRevHex / 10f);
                    telemetry.update();
                    armY.setPower(0);
                }
            }
            float startTime = (float) (System.nanoTime()/Math.pow(10,9));
            float lastTime = startTime;
            while (Math.abs(startTime-lastTime) < numPresses) {
                lastTime = (float) (System.nanoTime()/Math.pow(10,9));
                telemetry.addData("Time: ", Math.abs(startTime-lastTime));
                telemetry.addData("Time to wait: ", numPresses);
                telemetry.update();
            }
        }
        else {
            while (power < 1) {
                telemetry.addData("rgba: ","("+colorSensor.red()+", "+colorSensor.green()+", "+colorSensor.blue()+", "+colorSensor.alpha()+")");
                telemetry.addData("g-a: ",Math.abs(colorSensor.green()-colorSensor.red()));
                colorInd = (colorSensor.green()>colorSensor.red()) ? 0:2;
                colorInd = (Math.abs(colorSensor.green()-colorSensor.red()) <= 0.05) ? 1:colorInd;
                telemetry.addData("colorInd: ",colorInd);
                power = (new float[] {1,0.5f,0})[colorInd];
                telemetry.update();
            }
            return;
        }
        float circumference = 0.295276f*((float) Math.PI);
        float numRevsFor3Ft = 3f/circumference;
        int lastPos = left.getCurrentPosition();
        float ticksPerRev = (float) left.getMotorType().getTicksPerRev();
        while (Math.abs(left.getCurrentPosition()) - lastPos < ticksPerRev*numRevsFor3Ft) {
            left.setPower(-power);
            right.setPower(-power);
        }
        left.setPower(0);
        right.setPower(0);
        hand.setPosition(0.5);
        //wait
        float startTime2 = (float) (System.nanoTime()/Math.pow(10,9));
        float lastTime2 = startTime2;
        while (Math.abs(startTime2-lastTime2) < 1) {
            lastTime2 = (float) (System.nanoTime()/Math.pow(10,9));
        }
        //go back
        lastPos = left.getCurrentPosition();
        while (Math.abs(left.getCurrentPosition() - lastPos) < ticksPerRev*numRevsFor3Ft) {
            left.setPower(power);
            right.setPower(power);
        }
        left.setPower(0);
        right.setPower(0);
        done = true;
    }

    public void runOpMode() {
        initialize();
        while (!super.isStarted()) {
            initialize_loop();
        }
        while (!done) {
            finalLoop();
        }
        telemetry.addData("Auto status: ", "Done");
        telemetry.update();
    }
}