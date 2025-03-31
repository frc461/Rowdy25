TODO:

Alpha bot:
- [x] Test swerve drive
- [x] Test turret function
- [x] Tune slippage current
- [x] Test cameras & filters
- [x] Test intake command
- [x] Test wrist CANCoder offset
- [x] Tune elevator
- [x] Test pivot gravity gains
- [x] Remove voltage constraints if everything works
- [x] Test manual controls
- [x] Test presets
- [x] Swerve automation pt. 1 - rotational alignment

Friday, 2/14/2025 to test:
- [x] Retune wrist feedforward + feedback
- [x] Wrist presets + tuning
- [x] Pivot cancoder issues
- [x] Decide on L1 position
- [x] Tune Canandcolor for intake (coral)
- [x] Elevator limit switch

Friday, 2/14/2025 software only:
- [x] Driver only - work on OnFalse triggers instead of OnTrue because of debounce hold to pathfind/direct move to position then OnFalse will subsequently deploy the state properly and effectively
- [x] Double click debouncer
- [x] Move to algae vs coral (drive to object overhaul)
- [x] Swerve automation pt. 2 - pathfinding/translation alignment

Saturday, 2/15/2025 to test:
- [x] Test robot state chooser

Saturday, 2/15/2025 software only:
- [x] Auto - java structure (chooser), paths (pathmanager), commands

Tuesday, Wednesday, 2/18-19/2025 to test:
- [x] Fix pivot cancoder issues
- [x] Reconfigure wrist cancoder
- [x] Redo wrist presets as needed
- [x] Fast wrist
- [x] Intake
- [x] Test localization trust filter & cameras
- [x] change tolerances and pids and retest pathfinding to reef (try no pid but constant slow velocity)
- [x] Test multiple chooser
- [x] Preliminary auto things

Tuesday, 2/18/2025 software only:
- [x] finish paths
- [x] check for correct event markers & tune locations of each marker
- [x] perfect/tune the control points (consistent in transitioning between paths)
- [x] coral station optimization

Saturday, 2/22/2025 to test:
- [x] test updated keybinds for mishiwaka
- [x] discuss two options for reef translation auto-align - driver does it while true (pov) or operator does it (more complicated)

Tuesday, 2/25/2025 to test:
- [x] tune max tag clear dist
- [x] test new single tag pose (need to zero properly first)
- [x] tune max tag clear dist for above single tag pose calculation
- [x] (try for comp bot)
- [x] auto
  - [x] try not corner of the coral station
  - [x] try cameras looking forward

Thursday, 2/27/2025 to test:
- [x] try updating pivot G gains
- [x] auto
  - [x] new camera mounts on
  - [x] try slower rotation and faster linear acceleration
- [x] climb subsystem
  - [x] climb presets

INMIS:
- [x] auto
  - [x] try variety of paths
- [x] test climb subsystem more

Monday, 3/04/2025 software only:
- [ ] new spherical pathing for pathfinding to branch

Comp bot/later:
- [x] Re-add automatic intake logic
- [ ] Tune Canandcolor for algae intake
- [x] Implement climbing
- [x] SysID for elevator, pivot, wrist comp bot
- [x] Fine tune state transitions for efficiency
- [x] Test ground intake/object detection (after mishiwaka)


Saturday, 3/29/2025 software:
comp bot:
- [x] distance sensor + software implementation
- [x] more efficient dependent pathfinding
- [x] camera sensing vs. distance sensor via boolean trustCameras for coral obstruction

ground intake:
- [x] ground intake motor setup + logic revamp
- [x] pathing to coral

Saturday, 3/29/2025 to test:
comp bot:
- [x] test scoring
- [x] test bugfix for mid-stow bug going into teleop

Sunday, 3/30/2025 software:
ground intake:
- [x] clarify algae intake sensor
- [x] fix command to direct drive to note and consider making an automation class for compiling path finding to certain locations and searching for coral
- [x] revamp pathfinding location
- [x] double-sided reef outtake
- [x] revamp/fix auto
- [x] (if possible) algae intake
- [x] implement independent system in the case of camera failure that utilizes the canandcolor proximity sensor to adapt to coral obstruction (boolean trustCameras in Localizer.java)

Monday, 3/31/2025 to test:
comp bot:
- [ ] test proximity threshold network table entry, intake stuck
- [ ] try high I without the tension
- [ ] test dynamic autos (intake stuck/incorrect intake)
- [ ] test/configure distance sensor

ground intake:
- [ ] (once assembled) ground intake motor setup + logic revamp
- [ ] test pathfinding locations and auto scoring
- [ ] test auto (with dynamic if possible)
- [ ] (if possible) test algae net and processor scoring
