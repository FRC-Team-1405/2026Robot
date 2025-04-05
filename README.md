# 2026Robot

## Offseason Projects
  - buy a pigeon (IMU) start using CTRe Swerve Drive (via the generator) hopefully this will improve our encoder-only odometry
  - profile current swerve odometry (encoders only) to determine its accuracy and determine why it isn't perfectly accurate (slippage? other?)
    - implement any identified improvements in software/hardware
  - determine how to calibrate cameras for different lighting situations
  - better understand the vision to pose estimation cycle
    - our cameras are running at around 30fps, are we updating estimated poses that many times? if not is it due to multiple cameras on the same pi?
    - does increase in FPS greatly improve vision estimates? not at all?
    - can we run 3 or 4 cameras on the same pi without much issue.
  - other sensors? lidar, lasers, etc, things that can be used for high precision readings that we could integrate with our vision data.
  - tune a new PID for the drivebase that will automatically engage while the elevator is extended that will prevent tipping and allow smooth precise movement.

## Software Improvements:
  - logging/publishing camera target poses (we currently log estimated poses for each camera)
  - figure out how to export data from advantagescope for better match analysis
    - likely we will need to reduce the amount of data we are publishing to smartdashboard.
    - i don't think advantage scope will let us import log data

## Considerations for 2026 Game
  - if there is a large obstacle that will restrict vision (like a giant reef) we need a camera for the driver to use when they are behind that obstacle so they aren't bumping into coral/algae when trying to place and not realizing it and droping pieces and taking 10+ sec to place a coral.

## Event Prep:
  - drive team should know how to setup the advantagescope for recording
  - bring april tags to the competition

## Thoughts for drive team
  - when facing defense like in this [match](https://www.thebluealliance.com/match/2025ohcl_qm50) i love the spin move we do to nudge the bot out of our way. if we approach the feeder with the side of our bot instead of the front (intake) we can spin after contact to then face the intake toward the feeder. we should practice this bc it seems to work very well against defensive bots.
