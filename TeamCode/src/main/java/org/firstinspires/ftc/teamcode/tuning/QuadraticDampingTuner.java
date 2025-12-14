package org.firstinspires.ftc.teamcode.tuning;

import android.annotation.SuppressLint;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.blackice.FollowerConstants;
import org.firstinspires.ftc.teamcode.blackice.core.Follower;
import org.firstinspires.ftc.teamcode.blackice.geometry.Vector;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class QuadraticDampingTuner extends LinearOpMode {
    public static double[] TEST_POWERS = {1, 1, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2};
    public static int ms = 1000;

    @SuppressLint("DefaultLocale")
    public void runOpMode() {
        Follower follower = FollowerConstants.createFollower(hardwareMap);
        
        waitForStart();
        
        List<double[]> data = new ArrayList<>();
        
        for (int i = 0; i < TEST_POWERS.length; i++) {
            double power = TEST_POWERS[i];

            if (i % 2 != 0) {
                power = -power;
            }
            
            follower.update();

            follower.drivetrain.followVector(new Vector(1, 0).times(power), 0);
            sleep(ms);

            follower.update();
            double velocity = follower.localizer.getVelocity().computeMagnitude();
            Vector startingPos = follower.localizer.getPose().getPosition();
            
            follower.drivetrain.zeroPowerBrakeMode();
            follower.drivetrain.zeroPower();
            
            sleep(2000);
            
            follower.update();
            Vector newPosition = follower.localizer.getPose().getPosition();
            
            double brakingDistance = newPosition.minus(startingPos).computeMagnitude();
            
            data.add(new double[]{velocity, brakingDistance});
            telemetry.addData(
                Integer.toString(i), stringify(new double[]{velocity, brakingDistance}));
        }
        
        double[] coefficients = QuadraticRegression.quadraticFit(data);
        Log.i("data", data.toString());
        Log.i("equ", String.format("y = %.5fx^2 + %.5fx", coefficients[1],
                                  coefficients[0]));
        
        telemetry.addData(
            "Final Equation",
            String.format("y = %.5fx^2 + %.5fx", coefficients[1], coefficients[0]));
        telemetry.addData(
            "Plug in these constants to the TuningConstants Class: ",
            String.format(coefficients[1] + ", " + coefficients[0]));
        
        telemetry.update();
        
        while (opModeIsActive()) {
            idle();
        }
    }
    
    public String stringify(double[] array) {
        return String.format("%.3f", array[0]) + " " + String.format("%.3f", array[1]);
    }
}
