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
- [ ] Wrist presets + tuning (change upper limit back to double if motor protection isn't necessary anymore)
- [ ] Pivot cancoder issues
- [ ] Decide on L1 position
- [ ] Test robot state selector
- [ ] Tune Canandcolor for intake (coral + algae)
- [ ] Test localization trust filter
- [ ] Ask tech about elevator springs for limit switch????

Friday, 2/14/2025 software only:
- [ ] Driver only - work on OnFalse triggers instead of OnTrue because of debounce hold to pathfind/direct move to position then OnFalse will subsequently deploy the state properly and effectively
- [ ] Double click debouncer
- [ ] Move to algae vs coral (drive to object overhaul)
- [ ] Swerve automation pt. 2 - pathfinding/translation alignment
- [ ] Auto - java structure first, then paths, then test with other mechanisms

Comp bot/later:
- [ ] Implement climbing
- [ ] SysID for elevator, pivot, wrist
- [ ] Fine tune state transitions for efficiency