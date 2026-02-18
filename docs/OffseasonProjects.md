Projects
* Swerve
    * Autopilot: add rotation radius and try to remove custom code to handle rotation start: https://www.chiefdelphi.com/t/introducing-autopilot-a-novel-solution-to-holonomic-motion-control/504244/80
    * AP: improve custom theta profiled pid
    * ~~profile current swerve odometry (encoders only) to determine its accuracy and determine why it isn't accurate~~
        * ~~we might be able to use april tags to serve as a source of true position (after verifying accuracy manually)~~
    * CTReSwerve
        * ~~integrate vision into drive odometry~~
    * Research Phoenix Pro Time sync, what does it give us? How hard is it to integrate into our current solution.
        * Test out odometry using Phoenix Pro Time sync, see if accuracy improves. [Team 2056 seems to think it helps.](https://2056.ca/wp-content/uploads/2024/05/OPR24-2056-Technical-Binder.pdf)
    * Research Phoenix Pro FOC, does it live up to [the claim of 15% increased peak power](https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/talonfx-control-intro.html#field-oriented-control)? how do we implement it and use it on all of our motors as many teams have.
        * [world champs use it on every motor](https://www.chiefdelphi.com/t/2910-cad-and-tech-binder-release-2025/500310/211?u=scoomoo28)
        * [many teams use it already](https://www.chiefdelphi.com/t/what-motor-s-are-you-planning-on-using-on-swerve-in-2024/446809)
    * [Perform system identification](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html) on the drive base
        * if we do this well it should improve drive odometry because it will reduce wheel slipping
        * bowan thinks this isn't that useful, i forget why
    * ~~Steal/implement a point based (only straight lines) autonomous that doesn't use path planner. determine if it provides the benefits for us that other teams are seeing~~
        * https://github.com/Team2056/PublicCodeBank/blob/main/src/main/cpp/Subsystems/OPROdometry.cpp
        * https://www.chiefdelphi.com/t/2910-cad-and-tech-binder-release-2025/500310/139
    * Test wheel slippage current threshold using [this guide](https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html). add stator current limit to our TunerConstants for drive motors.
    * Improve joystick deadband when in "precision move mode", current deadband causes jerky and imprecise motion
* Vision
    * determine how to calibrate cameras for different lighting situations
    * Use mrcal for camera calibration instead of built in photon vision calibration. More accurate and can use thousands of frames instead of tens: https://www.chiefdelphi.com/t/frc-1155-the-sciborgs-2024-build-thread-open-alliance/441531/42
    * Improve camera offset calculation process: https://www.chiefdelphi.com/t/how-does-everyone-tune-camera-offsets-for-3d-apriltags/508051/11
    * Look into Constrained PNP to speed up vision cycle times: https://www.chiefdelphi.com/t/1189-gearheads-2025-open-alliance-thread/477173/25
* Generic Robot Submodules
    * swerve features
        * tune a new PID for the drivebase that will automatically engage while the elevator is extended that will prevent tipping and allow smooth precise movement.
* Logging/Debugging
    * Improve advantage scope match logging/recording
        * Are we auto downloading logs? https://www.chiefdelphi.com/t/advantage-scope-auto-export-logs/509363
        * data logs! link: https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog.html
        * possibly need a new solution or develop a hack to get this to work
    * Better log swerve behavior for deeper debugging
        * [encoder odometry update frequency](https://v6.docs.ctr-electronics.com/en/stable/docs/application-notes/update-frequency-impact.html)
            * CTRe gives this to us for free
        * vision odometry update frequency
            * effected by number of cameras on pi? does using one pi per camera improve this?
    * Look into AdvantageKit
        * Should allow us to record inputs like sensors and joysticks as well as outputs such as motor outputs. Then replay them later in simulation for deep debugging. Could be very useful when trying to find root cause of complex or hard to replicate bugs.
* New sensors
    * other sensors? lidar, quest, etc, things that can be used for high precision readings that we could integrate with our vision data.
* Document code retrieval and deployment procedure for drive team to use with no software assistance. So they can always know where to get the "latest" code and deploy it onto the robot for drive practice.
* Move away from using pathPlanner to make autonomous's instead write command sequences in code and that will allow custom logic to make decisions.
* try choreo, you can use choreo paths with path planner
* Better understand motion magic constraints, velocity and acceleration, and how they can improve motion of a subsystem
* ~~Develop a fudge layer on top of the field positions that you can slightly adjust positions for each point and for each field~~
    * Implemented a basic solution for Reef positions only
* Develop a system that the driver can trigger with controller that will mark a time in a match. could be used for better timestamp debugging in competition.
* Look into maplesim to improve our robot simulation so that the robot will have real collisions with walls/gameobjects etc
    * cheesypoofs use it in their 2025 robot code
* ~~Build a tool that will detect relative positions of April tags on a comp field to compare to ideal field~~
    * already exists: https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/wpical/index.html
* Figure out why vision and odometry are both way off when measuring a simple distance
* calculate and graph robot acceleration
    * use swerve chassis velocity over time to calculate a moving avg accel
    * use the IMU (pigeon) to get accel
    * combine the two values to get a hybrid accel estimate, cheesy poofs do this, try to steal
    * graph acceleration to determine if you are hitting acceleration constraints in auto
* ~~Compare local field to ideal field using WPICal in order to practice for mapping a competition field~~
    * ~~Take a video of a  ChArUco board in finney and a video of the reef with the april tags~~
    * ~~calibrate your phone camera with the board video~~
    * ~~map the field with the april tags video~~
    * ~~use the april tag field layout .json file in your robot code as an optional override~~
    * (THIS IS LIMELIGHT ONLY) ~~apply the .fmap to your camera's coprocessor.~~