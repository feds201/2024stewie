# FED201 FRC Robot - "Stewie"

This repository contains the code for the FED201 FRC robot, named "Stewie". The robot is designed to participate in the FIRST Robotics Competition (FRC). The code is written in Java and uses the Gradle build system.

## Features

- **Phoenix Swerewe Generator**: The robot uses the Phoenix Swerewe Generator for advanced motion control. This allows the robot to perform complex maneuvers with high precision.

- **Dual Limelight Cameras**: The robot is equipped with two Limelight cameras. One camera is used for game piece detection, while the other is used for AprilTag and distance estimation. This dual-camera setup allows the robot to effectively interact with its environment and perform tasks accurately.

## Getting Started

To get started with the project, clone the repository to your local machine:

```bash
git clone https://github.com/feds201/2024stewie.git
```

Then, navigate to the project directory:

```bash
cd 2024stewie
```

You can then open the project in your preferred IDE. If you're using IntelliJ IDEA, you can simply open the project directory.

## Building and Running

To build the project, use the Gradle wrapper included in the repository:

```bash
./gradlew build
```

To deploy the code to the robot, connect your machine to the robot's network and run:

```bash
./gradlew deploy
```

## Contributing

We welcome contributions to the project. If you'd like to contribute, please fork the repository and make your changes in a separate branch. Once you're ready, open a pull request against our repository.

## License

This project is licensed under the MIT License. See the `LICENSE` file for more details.

## Contact

If you have any questions or issues, please open an issue on the GitHub repository. We'll do our best to respond as quickly as possible.
