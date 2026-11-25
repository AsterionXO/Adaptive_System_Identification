# Bill of Materials

| Component | Specification | Reference | Qty |
|---|---|---|---|
| Arduino Uno | ATmega328P, 10-bit ADC, PWM on D9 (≈490 Hz), USB powered | — | 1 |
| Op-amp | LM358 dual op-amp, single 5 V supply | U1A (amplifier), U1B (buffer) | 1 |
| Resistor | 47 kΩ | R3 (RC divider), Rf (feedback), offset supply to P2 | 3 |
| Resistor | 10 kΩ | R1, R2 (summing), R4 (RC divider), Rg (gain) | 4 |
| Potentiometer | 10 kΩ multiturn trimmer | P2 (offset Vp) | 1 |
| Capacitor | 100 µF electrolytic | C (RC time constant) | 1 |
| Breadboard | Solderless, with jumper wires | — | 1 |
| Digital multimeter | MAS830L | DC calibration readings | 1 |
| USB cable | USB A–B | Power and serial link to the PC | 1 |

## Connections

| LM358 pin | Function | Connected to |
|---|---|---|
| 1 | U1A output | Rf, Arduino A1 (output d) |
| 2 | U1A inverting input | Rf, Rg to GND |
| 3 | U1A non-inverting input | R1 (from x), R2 (from P2 wiper Vp) |
| 4 | GND | Arduino GND |
| 5 | U1B non-inverting input | Capacitor node Vn (R3, R4, C) |
| 6 | U1B inverting input | Pin 7 (unity-gain feedback) |
| 7 | U1B output | Node x: R1, Arduino A0 |
| 8 | V+ | Arduino 5 V |

Arduino D9 connects to the capacitor node through R3. The capacitor's negative lead goes to GND.

See `circuit_diagram.png` for the schematic and `hardware_setup.jpeg` for the assembled board.
