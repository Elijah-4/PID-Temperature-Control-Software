=====================================================================
PID Annealing Controller (PID_Annealing_Controller.m) - User Guide
=====================================================================

This program provides a graphical, non-blocking PID temperature control system for annealing experiments. It interfaces with a programmable power supply (PSU) and a National Instruments DAQ for temperature measurement and control.

---------------------------------------------------------------------
1. SETUP & REQUIREMENTS
---------------------------------------------------------------------
- MATLAB (R2019b or newer recommended)
- Instrument Control Toolbox (for serial communication)
- Data Acquisition Toolbox (for NI DAQ)
- Connect your PSU to the specified COM port (default: COM5)
- Connect your temperature sensor to the DAQ (default: Dev4, ai0)

---------------------------------------------------------------------
2. RUNNING THE PROGRAM
---------------------------------------------------------------------
1. Open MATLAB and navigate to the project folder.
2. Open and run `PID_Annealing_Controller.m`.
3. The main window will appear with a live temperature plot and control panel.
4. Enter your desired target temperature and click 'Set Target'.
5. Wait for the system to reach and stabilize at the target temperature.
6. Once stable, you can configure and start an annealing sequence.
7. To exit, click 'Exit'. You will be prompted to save your data and plot.

---------------------------------------------------------------------
3. UI CONTROLS
---------------------------------------------------------------------
- **Target Temp (°C):** Enter the desired setpoint temperature.
- **Set Target:** Starts PID control to reach the setpoint.
- **Start Annealing:** Begins a stepped annealing process (configurable).
- **Exit:** Stops the experiment and prompts to save data.
- **Annealing Settings:**
    - End Mode: Choose to end annealing by temperature or voltage.
    - End Value: The final value for the chosen mode.
    - Step Size (V): Voltage increment per step.
    - Step Time (min): Duration of each step.

---------------------------------------------------------------------
4. DATA & OUTPUTS
---------------------------------------------------------------------
- On exit, you can save:
    - A CSV file with time, temperature, voltage, and target history.
    - A PNG image of the temperature plot.
    - A MATLAB .mat file with the same data.

---------------------------------------------------------------------
5. PID CONTROLLER TUNING (RECOMMENDED: PD->I METHOD)
---------------------------------------------------------------------
The controller uses a PID algorithm with the following gains:
- Proportional (Kp)
- Integral (Ki)
- Derivative (Kd)

**Recommended Tuning Procedure (PD->I):**
1. **Set Ki = 0** (disable integral action).
2. **Tune Kp and Kd:**
    - Increase Kp until the system responds quickly but does not oscillate excessively.
    - Increase Kd to dampen any oscillations and reduce overshoot.
    - The goal is a fast, stable response with no steady-state error correction yet.
3. **Add Ki:**
    - Gradually increase Ki until any steady-state error is eliminated.
    - Too much Ki can cause slow oscillations or instability; use the smallest value that corrects steady-state error in a reasonable time.

**Where to Edit Gains:**
- Open `PID_Annealing_Controller.m` and look for these lines near the top:
    CONTROL_KP = ...
    CONTROL_KI = ...
    CONTROL_KD = ...
- Adjust the values and re-run the program to test the effect.

**Tips:**
- If the system overshoots or oscillates, reduce Kp or increase Kd.
- If the system is too slow, increase Kp.
- If the system never reaches the setpoint, increase Ki (after tuning Kp and Kd).
- Always make small changes and test after each adjustment.

---------------------------------------------------------------------
6. TROUBLESHOOTING
---------------------------------------------------------------------
- If the program cannot connect to the PSU or DAQ, check the COM port and device IDs.
- If the temperature reading is NaN or incorrect, check sensor wiring and DAQ configuration.
- If the system is unstable, re-tune the PID gains as described above.

---------------------------------------------------------------------
For further questions or issues, consult the code comments or contact the developer.
===================================================================== 