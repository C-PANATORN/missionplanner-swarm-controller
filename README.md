# missionplanner-swarm-controller
`missionplanner-swarm-controller` is an extension of `missionplanner` that enables coordinated control of multiple UAVs via `mavlink` protocol.  

>This project was developed through a collaboration between King Mongkut's University of Technology Thonburi (KMUTT) and R V Connex Co., Ltd.

## Prerequisites 
To compile Mission Planner, we recommend using Visual Studio. You can download Visual Studio Community from the [Visual Studio Download page](https://visualstudio.microsoft.com/downloads/ "Visual Studio Download page").

Visual Studio is a comprehensive suite with built-in Git support, but it can be overwhelming due to its complexity. To streamline the installation process, you can customize your installation by selecting the relevant "Workloads" and "Individual components" based on your software development needs.

To simplify this selection process, we have provided a configuration file that specifies the components required for MissionPlanner development. Here's how you can use it:

1. Go to "More" in the Visual Studio installer.
2. Select "Import configuration."
3. Use the following file: [vs2022.vsconfig](https://raw.githubusercontent.com/ArduPilot/MissionPlanner/master/vs2022.vsconfig "vs2022.vsconfig").

By following these steps, you'll have the necessary components installed and ready for Mission Planner development.

*Currently VSCode with C# plugin is able to parse the code but cannot build.*

## Installation
Download the main [Mission Planner](https://github.com/ArduPilot/MissionPlanner) source code with submodules:
```
git clone --recurse-submodules https://github.com/ArduPilot/MissionPlanner.git
cd MissionPlanner
git submodule update --init
```

Replace the default `Swarm` folder with this project’s version:
```
# Go back to the parent directory
cd ..

# Clone this repository
git clone https://github.com/C-PANATORN/missionplanner-swarm-controller.git

# Replace the Swarm folder
rm -rf MissionPlanner/Swarm
cp -r missionplanner-swarm-controller/Swarm MissionPlanner/Swarm
```

Follow the instructions in the [Ardupilot Dev Wiki](https://ardupilot.org/dev/docs/building-mission-planner.html#building-mission-planner) to compile the code.

## Build
To build the code:

- Open MissionPlanner.sln with Visual Studio
- From the Build menu, select "Build MissionPlanner"

*Note: if you run into build issues, please make sure Visual Studio and Git Submodules are properly configured*

### Debugging compiler 
When rebuilding missionplanner, clean cache from previous installations using the following steps to ensure a clean build.

```
Visual Studio: Clean Solution  
Close Visual Studio  
Shell:  
cd  
git clean -nfdx -e .vs/  
git clean -fdx -e .vs/  
Open Visual Studio:  
Build Mission Planner
```

## Usage
If compiled successfully, launch the `MissionPlanner.exe` file located in `MissionPlanner\bin\Debug\net461`, this will open the program in Debug mode. 

To make changes to the code, edit the code within your IDE and rebuild missionplanner following the debugging compiler steps.

## Tested Environment
This system was developed and tested with the following setup:

- **Operating System**: Windows 11  
- **IDE / Compiler**: Visual Studio 2022 Community Edition  
- **Ground Control Software**: Mission Planner 1.3.82  
- **Firmware**: ArduPilot 4.7.0 (Development branch)  

Building Mission Planner on other systems isn't supported currently.

## License
This project is licensed under the GNU General Public License v3.0, in accordance with the Mission Planner license.
See the [LICENSE](LICENSE) file for details.

## Acknowledgments
- [Mission Planner](https://github.com/ArduPilot/MissionPlanner) (base project, GPLv3)  
- [ArduPilot](https://github.com/ArduPilot/ardupilot) (firmware)  