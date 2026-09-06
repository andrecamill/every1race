<div align="center">
  <h3 align="center">every1race</h3>

  <p align="center">
    open-source toolset for open sim racing
    <br />
    <br />
  </p>
</div>

<b>every1race</b> is a set of tools to make sim racing accessible to everyone

tools focus on making any type of peripheral comfortable and fun for sim racing

#### tools include:

a <i>Wiimote</i> controller implementation that takes advantage of <i>MotionPlus</i> (accelerometer) included in most of wiimotes

a basic pedal system based on a microcontroller, for convenience, I used <i>Arduino</i> C library for microcontroller software, as it is one of the most supported and easiest libraries for interface with microcontrollers. I also avoided using native USB controls as most microcontrollers lack native USB (<i>Arduino UNO, ESP-32 DevKit</i> ecc...)

#### analog input emulation

as you may notice, most of the controls rely on emulated analog inputs

this is because I mainly designed this toolset for usage on <i>Assetto Corsa</i>, which, on original launcher doesn't allow digital inputs for throttle, brake, clutch, handbrake ecc.

#### disclaimer

this is still a PoC (proof-of-concept)

code, especially analog input emulation is made with the help of LLMs, everything is and will be reviewed by developer 

this toolset is designed for use on Linux based systems: made on NixOS environment
 