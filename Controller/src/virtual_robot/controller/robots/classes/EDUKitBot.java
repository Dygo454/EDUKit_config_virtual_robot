package virtual_robot.controller.robots.classes;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.MotorType;
import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.transform.Rotate;
import virtual_robot.controller.BotConfig;
import virtual_robot.controller.VirtualBot;
import virtual_robot.controller.VirtualRobotController;

@BotConfig(name = "EDU Kit Bot", filename = "edu_kit_bot")
public class EDUKitBot extends VirtualBot {

    private MotorType motorType;
    private DcMotorExImpl leftMotor = null;
    private DcMotorExImpl rightMotor = null;
    private DcMotorExImpl armMotor = null;
    private ServoImpl servo = null;
    private VirtualRobotController.ColorSensorImpl colorSensor = null;

    private double armRot;

    @FXML Group arm;
    @FXML Group hand;

    private double wheelCircumference;
    private double interWheelDistance;



    public EDUKitBot(){
        super();
    }

    public void initialize(){
        super.initialize();
        hardwareMap.setActive(true);
        leftMotor = (DcMotorExImpl)hardwareMap.get(DcMotorEx.class, "left_motor");
        rightMotor = (DcMotorExImpl)hardwareMap.get(DcMotorEx.class, "right_motor");
        armMotor = (DcMotorExImpl)hardwareMap.get(DcMotorEx.class, "arm_motor");
        servo = (ServoImpl)hardwareMap.servo.get("hand_servo");
        colorSensor = (VirtualRobotController.ColorSensorImpl)hardwareMap.colorSensor.get("color_sensor");
        wheelCircumference = Math.PI * botWidth / 4.5;
        interWheelDistance = botWidth * 8.0 / 9.0;
        hardwareMap.setActive(false);
        armRot = 120;
        arm.getTransforms().add(new Rotate(0, 37.5, 67.5));
        hand.getTransforms().add(new Rotate(0, 0, 0));
    }

    protected void createHardwareMap(){
        motorType = MotorType.Neverest40;
        hardwareMap = new HardwareMap();
        hardwareMap.put("left_motor", new DcMotorExImpl(motorType));
        hardwareMap.put("right_motor", new DcMotorExImpl(motorType));
        hardwareMap.put("arm_motor", new DcMotorExImpl(motorType));
        hardwareMap.put("hand_servo", new ServoImpl());
        hardwareMap.put("color_sensor", controller.new ColorSensorImpl());
    }

    public synchronized void updateStateAndSensors(double millis){
        double deltaLeftPos = leftMotor.update(millis);
        double deltaRightPos = rightMotor.update(millis);
        double deltaArmRot = armMotor.update(millis);
        double leftWheelDist = -deltaLeftPos * wheelCircumference / motorType.TICKS_PER_ROTATION;
        double rightWheelDist = deltaRightPos * wheelCircumference / motorType.TICKS_PER_ROTATION;
        double armDegrees = deltaArmRot * 360 / (motorType.TICKS_PER_ROTATION);
        double distTraveled = (leftWheelDist + rightWheelDist) / 2.0;
        double headingChange = (rightWheelDist - leftWheelDist) / interWheelDistance;
        double deltaRobotX = -distTraveled * Math.sin(headingRadians + headingChange / 2.0);
        double deltaRobotY = distTraveled * Math.cos(headingRadians + headingChange / 2.0);
        colorSensor.updateColor(x, y);

        x += deltaRobotX;
        y += deltaRobotY;
        headingRadians += headingChange;
        armRot += armDegrees;

        if (headingRadians > Math.PI) headingRadians -= 2.0 * Math.PI;
        else if (headingRadians < -Math.PI) headingRadians += 2.0 * Math.PI;

        constrainToBoundaries();
    }

    public synchronized void updateDisplay(){
        super.updateDisplay();
        ((Rotate) arm.getTransforms().get(0)).setAngle(armRot);
        ((Rotate) hand.getTransforms().get(0)).setAngle(servo.getInternalPosition());
    }

    public void powerDownAndReset(){
        leftMotor.stopAndReset();
        rightMotor.stopAndReset();
        armMotor.stopAndReset();
    }


}