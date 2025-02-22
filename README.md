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
- [ ] test updated keybinds for mishiwaka
- [ ] discuss two options for reef translation auto-align - driver does it while true (pov) or operator does it (more complicated)
- [ ] auto
   - [ ] try '2,x' paths
   - [ ] try retuning swerve offsets/sysid
   - [ ] try with 3rd camera in corner as well as in center looking forward
   - [ ] try not corner of the coral station

Comp bot/later:
- [x] Re-add automatic intake logic
- [ ] Tune Canandcolor for algae intake
- [ ] Implement climbing
- [ ] SysID for elevator, pivot, wrist comp bot
- [x] Fine tune state transitions for efficiency
- [ ] Test ground intake/object detection (after mishiwaka)