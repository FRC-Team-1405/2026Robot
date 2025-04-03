# 2026Robot

Offseason Projects
  - profile current swerve odometry (encoders only) to determine its accuracy and determine why it isn't perfectly accurate (slippage? other?)
    - implement any identified improvements in software/hardware
  - determine how to calibrate cameras for different lighting situations
  - better understand the vision to pose estimation cycle
    - our cameras are running at around 30fps, are we updating estimated poses that many times? if not is it due to multiple cameras on the same pi?
    - does increase in FPS greatly improve vision estimates? not at all?
    - can we run 3 or 4 cameras on the same pi without much issue.
  - other sensors? lidar, lasers, etc, things that can be used for high precision readings that we could integrate with our vision data.

Software Improvements:
  - logging/publishing camera target poses (we currently log estimated poses for each camera)
  - figure out how to export data from advantagescope for better match analysis
    - likely we will need to reduce the amount of data we are publishing to smartdashboard.
    - i don't think advantage scope will let us import log data

Event Prep:
  - drive team should know how to setup the advantagescope for recording
  - bring april tags to the competition
