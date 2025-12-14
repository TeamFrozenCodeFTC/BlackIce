package org.firstinspires.ftc.teamcode.blackice;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.blackice.core.Follower;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;


public class ExampleAuto extends OpMode {
    Follower follower;
    
    Pose targetPose = new Pose(48, 0, 0);
    Pose startingPose = new Pose(0, 0, 0);
    
    int state = 0;
    
    @Override
    public void init() {
        follower = FollowerConstants.createFollower(hardwareMap);
    }
    
    @Override
    public void loop() {
        switch (state) {
            case 0:
                follower.holdPose(targetPose);
                if (follower.localizer.getPose().getPosition().getX() >= 48.5) {
                    state++;
                }
                break;
            case 1:
                follower.holdPose(startingPose);
                if (follower.localizer.getPose().getPosition().getX() <= 0.5) {
                    state++;
                }
                break;
        }
        
        follower.update();
    }
}
