import android.graphics.Point;
import android.graphics.Rect;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;

@Autonomous
public class LinearRedFoundationSide extends LinearOpMode {
    BoschIMU imu;
    DcMotor motorName;
    Camera phone;
    Servo servoName;

    private boolean found    = false; // Is the skystone found
    private Point screenPosition = new Point(); // Screen position of the Skystone
    private Rect foundRect = new Rect(); // Found rect

    int step = 0;

    public void stopMotors() {
        motorName.setPower(0);
    }

    public void methodName() {
        motorName.setPower(-1);
    }

    public void method2Name() {
        servoName.setPosition(0.85);
    }

    public void method3Name() {
        servoName.setPosition(0.85);
    }

    /**
     * Returns the skystone's last position in screen pixels
     * @return position in screen pixels
     */
    public Point getScreenPosition(){
        return screenPosition;
    }

    /**
     * Returns the skysonte's found rectangle
     * @return skystone rect
     */
    public Rect getFoundRect() {
        return foundRect;
    }

    /**
     * Returns if a skystone is being tracked/detected
     * @return if a skystone is being tracked/detected
     */
    public boolean isFound() {
        return skystoneDetector.isFound();
    }

    @Override
    public void runOpMode() {
        imu = new BoschIMU(hardwareMap.get(BNO055IMU.class, "imu"));
        imu.init(); //Initialize IMU
        motorName = hardwareMap.dcMotor.get("Motor Name"); //Initialize motor
        servoName = hardwareMap.servo.get("Servo Name"); //Initialize servo

        telemetry.addData("Status", "DogeCV 2018.0 - Skystone Example");
        telemetry.addData("Status", "DogeCV 2018.0 - Skystone Detector");


        // Set up detector
        // Optional tuning

        motorName.setDirection(DcMotorSimple.Direction.REVERSE);
        motorName.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2Name.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        telemetry.addData("Is found result", skystoneDetector.isFound());
        skystoneDetector.enable();
        methodName();
        sleep(1000);
        stopMotors();
        sleep(200);
        isFound();
        sleep(2000);
        if (skystoneDetector.isFound()) {
            step++;
        }
        if (step == 0) {
            method2Name();
            sleep(1000);
            stopMotors();
            isFound();
            sleep(2000);
            if (skystoneDetector.isFound()) {
                step++;
                step++;
            }
        }
        if (step == 0) {
            method2Name();
            sleep(1000);
            stopMotors();
            isFound();
            sleep(2000);
            if (skystoneDetector.isFound()) {
                step++;
                step++;
                step++;
            }
        }
        if (step == 1) {
            method3Name();
            sleep(1000);
            stopMotors();
            sleep(1000);
            method2Name();
            sleep(1000);
            stopmotors();
            sleep(1000);
        }
        if (step == 2) {
            methodName();
            sleep(1000);
            stopMotors();
            sleep(1000);
            method2Name();
            sleep(1000);
            stopmotors();
            sleep(1000);
        }
        if (step == 3) {
            method2Name();
            sleep(1000);
            stopMotors();
            sleep(1000);
            methodName();
            sleep(1000);
            stopmotors();
            sleep(1000);
        }
        if (step == 0) {
            methodName();
            sleep(1000);
            stopMotors();
            sleep(1000);
            method3Name();
            sleep(1000);
            stopMotors();
            sleep(1000);
        }
    }
}

