# 2026-11-17 wrist/drill test verdict

- Home: completed with original guarded effort-JTC home script; final idle, errors none, load0, open gripper.
- Manual wrist path: 1446 guarded controller packets, zero forwarder refusals. Actual contact-stage TCP ended z0.130m (raw wrist goal z0.150m plus field). Operator reported excessive descent; grasp height/contact alignment NOT validated.
- Manual close gesture: action succeeded, but settled fingers1.7mm means EMPTY. Lift correctly refused; no payload changes or lift/place cycle performed.
- Assisted-manual field: native offset cap2cm/10deg, intent escape and retained expiry checks pass. Hardware logs show cap active. Further guard prevents attraction at engagement until10mm inward hand intent: unit tested, not yet hardware tested.
- Autonomous request: first activation lost driver UDP, passive reconnect restored feedback. Retried actual /teleop/grasp; canonical broker refused live object cloud absent/stale. No autonomous grasp trajectory or lift executed.
- Final: Home/passive; JTCinactive, broadcastersactive, robotclaimreleased. Own wrist MPC and GraspGen stopped. Production cameras/masked map preserved.

Do not claim successful pickup, payload lifting, autonomous execution, or verified real-camera contact calibration.
