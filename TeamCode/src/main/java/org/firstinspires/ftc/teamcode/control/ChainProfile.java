package org.firstinspires.ftc.teamcode.control;
import java.util.ArrayList;
import java.util.Arrays;
public class ChainProfile extends MotionProfile {
    private ArrayList<MotionProfile> profiles;
    public ChainProfile(MotionProfile... profiles) {
        ti = profiles[0].ti;
        i = profiles[0].i;
        tf = profiles[profiles.length - 1].tf;
        this.profiles = new ArrayList<>(Arrays.asList(profiles));
    }
    public MotionState state(double t) {
        int i = profiles.size() - 1;
        while (i >= 1) {
            if (profiles.get(i).ti < t) {
                break;
            }
            i--;
        }
        return profiles.get(i).state(t);
    }
}
