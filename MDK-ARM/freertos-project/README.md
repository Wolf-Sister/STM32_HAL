# FreeRTOS Project

This project is a FreeRTOS-based application designed for STM32 microcontrollers. It demonstrates the use of FreeRTOS for task management, inter-task communication, and PID control.

## Project Structure

```
freertos-project
├── Core
│   ├── Inc
│   │   ├── main.h          # Main program header file, defining global variables and function prototypes.
│   │   ├── freertos.h      # FreeRTOS related header file, defining task, queue, and semaphore interfaces.
│   │   ├── PID.h           # Header file for PID controller interface, including structures and function prototypes.
│   │   └── queue.h         # Header file for queue interface, including creation, sending, and receiving function prototypes.
│   └── Src
│       ├── main.c          # Entry point of the program, initializing hardware and FreeRTOS, creating tasks, and starting the scheduler.
│       ├── freertos.c      # Implementation of FreeRTOS initialization, including task, queue, and timer creation.
│       ├── PID.c           # Implementation of PID controller functionalities, including PID computation and updates.
│       └── queue.c         # Implementation of queue functionalities, including creation and operations.
├── Drivers
│   └── STM32F4xx_HAL_Driver
│       ├── Inc             # Directory containing STM32F4xx HAL driver header files.
│       └── Src             # Directory containing STM32F4xx HAL driver source files.
├── FreeRTOS
│   ├── Source              # Directory containing FreeRTOS source code.
│   └── Config
│       └── FreeRTOSConfig.h # FreeRTOS configuration file, defining task stack sizes, priorities, and other configuration options.
├── .vscode
│   └── launch.json         # VS Code debug configuration file, defining debugger settings.
├── Makefile                # Makefile for building the project, defining compilation and linking rules.
├── README.md               # This file contains project documentation, describing functionality and usage.
└── LICENSE                 # File containing project license information.
```

## Features

- **Task Management**: The project utilizes FreeRTOS to manage multiple tasks efficiently.
- **Inter-task Communication**: Queues are used for communication between tasks, ensuring data integrity and synchronization.
- **PID Control**: Implements a PID controller for precise control of motors or other actuators.

## Usage

1. **Setup**: Ensure you have the necessary toolchain for STM32 development.
2. **Build**: Use the provided Makefile to compile the project.
3. **Upload**: Flash the compiled binary to your STM32 microcontroller.
4. **Run**: Start the application and monitor the output via a serial terminal.

## License

This project is licensed under the terms found in the LICENSE file.