# Humanoid Line Follower Robot Simulation in Webots    
## Project Overview
This repository contains a comprehensive simulation of an autonomous bipedal humanoid robot, named "VoBiRo," developed within the Webots simulation environment. The project's core objective is to achieve stable, vision-based line-following behavior. This is accomplished through a sophisticated control architecture that integrates dynamic gait generation, real-time balance correction using Inertial Measurement Unit (IMU) feedback, and a robust fall-recovery system.

## Getting the Repository

You can obtain the project by either cloning the Git repository or downloading it as a ZIP:

**Clone via Git:**
```ruby
git clone https://github.com/ricoagustiawan/Humanoid_Webots_Line_Follower.git
```

**Download as ZIP:**   
Click the Code button, and select Download ZIP. After downloading, extract the ZIP archive to your desired folder.
<details>
  <summary>Details</summary>

### Extracting the 3D Model Assets

After cloning or downloading, there are additional 3D model asset files that need to be extracted. The repository includes a multi-part RAR archive (`TeenSizeAcrylic.part01.rar, TeenSizeAcrylic.part02.rar, ... up to part05.rar`) which contains the humanoid robot’s 3D meshes and textures. To set up these assets:

1. **Locate the RAR files**: They should be in the root of the project folder (`TeenSizeAcrylic.part01.rar through TeenSizeAcrylic.part05.rar`). Make sure all five parts are present in the same directory.

2. **Extract the archive**: Using a tool like **WinRAR** (or any tool that supports multi-part RAR), right-click on `TeenSizeAcrylic.part01.rar` and choose **“Extract Here”**. The extractor will automatically combine all parts (part02, part03, etc.) and output a folder named `TeenSizeAcrylic` containing the robot’s 3D model files.

</details>

**Make sure the project files are all in a known location on your computer, as you will need to open the simulation world file in Webots.**

## Project Directory Structure

Once the assets are extracted, your project folder should have the following structure (important for Webots to find everything):  
```
Humanoid_Webots_Line_Follower/    ← (Project root folder)  
 ├── humanoid_R1/  
 │   ├── controllers/    # C++ controller code for the humanoid robot  
 │   └── worlds/         # Webots world files (environment setup, robot instance)  
 └── TeenSizeAcrylic/    # 3D model assets (meshes, textures) for the humanoid:contentReference
```
## Running the Simulation
Launch the Webots application.

1. **Open the world in Webots**: Launch the Webots application. Go to `File > Open World...` and navigate to `humanoid_R1/worlds/TeenSize.wbt`.
2. **Initialize the controller**: Once the world is loaded, make sure the humanoid robot is using the correct controller, it should already be set to the provided controller (**“enum”**). If it’s not set, choose the appropriate controller from the dropdown (the controller binaries will be in the `controllers` folder).
3. **Compile the controller code**: Before running, compile the controller by clicking the **Build** icon. Webots will build the C++ controller.
4. **Start the simulation**: Now click the **Play** button to run the simulation.


## Usage: Control Modes
The robot can be operated in two modes. The inclusion of a manual mode is a powerful feature for development, as it allows the locomotion and perception systems to be tested independently. This decoupling is a critical methodology for debugging complex autonomous systems.

1. `LINE_FOLLOWER` (Default): The robot operates autonomously, using its camera to follow the line on the floor.
2. `MANUAL_KEYBOARD`: The user has direct control over the robot's movements via the keyboard.

> Switching Modes: Press the 'M' key to toggle between LINE_FOLLOWER and MANUAL_KEYBOARD modes.   

## Keyboard Command Reference
The following table summarizes all keyboard commands for interacting with the simulation.   

|      Key     |     Action     | Description   |
| :---         |     :---:      | :---          |
| `SPACE`      | Toggle Walking | Starts or stops the stepping motion   |
| `M`          | Switch Mode	  | Toggles between `LINE_FOLLOWER` and `MANUAL_KEYBOARD`   |
| `UP ARROW	`  | Move Forward	  | Sets forward step amplitude (`XAmplitude`) to +1.0        |
| `DOWN ARROW` | Move Backward  | Sets forward step amplitude (`XAmplitude`) to -1.0        |
| `LEFT ARROW` | Turn Left  	  | Sets turning amplitude (`AAmplitude`) to +0.5             |
| `RIGHT ARROW`| Turn Right 	  | Sets turning amplitude (`AAmplitude`) to -0.5             |
| `,` (comma)  | Strafe Left	  | Sets lateral step amplitude (`YAmplitude`) to +0.3        |
| `.` (period) | Strafe Right   | Sets lateral step amplitude (`YAmplitude`) to -0.3        |



