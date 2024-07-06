# The notes below track my early code in [robot-old](robot-old) that led up to the most recent version.
* This code replicates the functionality of the CircuitPython code up to:
    * page 306 (in book hard copy) / page 329 (in pdf)
    * To Do Next: *Using Sensors to track relative pose*

* Next step: use Numpy which is contained in ulab module
    * To get ulab, need to download `RPI_PICO_W.uf2` file from [GitHub Micropython Builder](https://github.com/v923z/micropython-builder/releases/tag/latest)
        * Replace existing firmware with this new firmware.
        * Running the command `help("modules")` now lists 'ulab' as an available module.

* Successfully ran MicroPython code displaying 20 random dots on arena (Fig 13.7)
    * page 309 (in book hard copy) / page 332 (in pdf)
    * In the current configuration, the Pico (presumed to be mounted on the robot) has a file `main.py` which operates as a BLE peripheral device, sending the arena boundaries and some (fictitious) poses to a BLE central device (laptop).
        * `main.py` imports the file `robot.py`.
        * So far, the only thing being provided by `robot.py` is the `uart` object used to channel BLE communications. That is about to change. 
        
* Next step: Add *Start* button to laptop display
    * page 313 (in book hard copy) / page 336 (in pdf)
    * I didn't bother to implement the collision avoidance code in MicroPython. It seems like a lot of extra work without much benefit. The displayed poses are still just the random fictitious ones.
    * Starting in the next step, I am going to have to use a **real robot** to implement some *real robot behavior* so that it can send some *real pose data* (not fictitious) to the laptop.

* Now, I'm *almost* ready to follow along with the code in the *Pose movement probabilities* section (pp 319-323)
    * Unfortunately, when I try to re-acquaint myself with the way the robot connects to the laptop via BLE, it doesnt work. See [robot2laptop_connex_problem.md](robot2laptop_connex_problem.md).
    * After getting that working again, when I run my new `main.py` file on the PicoBot, it doesn't have a command handler, so although the robot works in teleop mode and sends data out on uart1, the connection with the laptop is broken. Here's what I get:

    ![Broken connection between robot & laptop](imgs/Screenshot.png)
    * So before proceeding , I need to integrate my picobot code within an architecture that contains a command handler that talks to the laptop code.
    * That most recent code is in the [`robot`](robot) folder, with no unneeded extraneous files. (The libraries are placed in a `/lib` folder on the Pico.)

