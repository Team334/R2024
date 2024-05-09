<h1 align="center" style="font-size: 3.5rem"><b>Ascent</b></h1>

<p align="center">This code is for 2024 FRC Competition, CRESCENDO.</p>

<img src="./robot.png" width=1000 />

> The code consists of Java using GradleRIO, WPIlib, Limelight, etc. Our robot uses swerve drive. You can find more below.:

## Tools we use
| Name | URL |
| ---- | --- |
| GradleRIO | https://github.com/wpilibsuite/GradleRIO |
| WPILIB | https://github.com/wpilibsuite/allwpilib |
| Limelight | https://limelightvision.io/ |
| TalonFX/Phoenix | https://v6.docs.ctr-electronics.com/en/stable/ |
| REV Robotics | https://docs.revrobotics.com/docs/ |
| FRC Driver Station (NI FRC Game tools) | https://www.ni.com/en/support/downloads/drivers/download.frc-game-tools.html |
| Pathplanner | https://pathplanner.dev/home.html |
| Elastic Dashboard | https://github.com/Gold872/elastic-dashboard |

## Contributing
First of all, thank you for taking your time to look over the code and trying making this code better. There are guidelines when contributing be sure to read [CONTRIBUTING.md](https://github.com/Team334/R2024/blob/main/.github/CONTRIBUTING.md)

## Getting Started

### Cloning the repository
> Exceptionally you download as zip as unzip.

To do this:
```bash
git clone https://github.com/Team334/R2024
```

### Building

If you have wpilib vscode you can press the button in the top right corner (WPILIB Icon) and type build and you should see a option: `WPILIB: Build Robot Code`

> If you running from a different ide:

### Windows:

**Command Prompt**
```bash
gradlew build
```

Powershell
```
./gradlew build
```

### Linux
```bash
chmod +x gradlew
./gradlew build
```
> Alternatively you can run `chmod +x gradlew && gradlew build`

Note: `chmod +x gradlew` is giving it permission. Read more [here](https://en.wikipedia.org/wiki/Chmod).

## Additonal Gradle Commands
> All these commands are in the WPILIB (Icon) button (in the top right corner) options if you have WPILIB vscode.

### Deploying
Deploying will build your code, and deploy it to the robot.

```bash
./gradlew deploy
```
> (You have to be connected to the robot for this to work.)

### Clean
This removes all cache (gradle default uses cache to rebuild) when gradle builds. This is to ensure your dependences/vendordeps work.

```bash
./gradlew clean
```

### Debug
This outputs what is happening while its building and a good way to see what its doing.

```bash
./gradlew --debug
```

### Scan
This is preferbly not used, it will scan the project and send all the data into a website where it will give a link to email to you.

```bash
./gradlew --scan [command]
```
> You have to accept the terms to get the link.

> Command can be like build, simulate, etc


### Simulation
This is used to simulate the robot code and check if something is working right instead of deploying to check which might the robot go crazy.

```bash
./gradlew simulateJava
```

A debugging version of simulation (This builds slower):
```bash
./gradlew simulateJavaDebug
```

> You have to enable desktop support. See more [here](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/introduction.html)
