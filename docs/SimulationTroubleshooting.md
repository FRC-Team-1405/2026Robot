# Simulation Troubleshooting

| Problem                           | Solution                                      |
|-----------------------------------|-----------------------------------------------|
| Robot drives backwards (wrong driver station perspective) in simulation | When switched between alliances in simulation, ensure you enter disabled mode before you enter teleop. CommandSwerveDrivetrain.periodic only updates the DS perspective when disabled mode is triggered as a protection mechanism. |
