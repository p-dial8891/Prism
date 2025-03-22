# Prism

## Introduction
This project attempts to address the need for a portable method of entering passwords at a computer without typing them manually.

## System Concept

### Principle
To do this, we need to take advantage of the prevalent use of USB keyboards as an interface to type in passwords. By using the Human Interface Device protocol defined in the USB standards, any string of typable characters can be sent to a terminal in an automated fashion by using a bespoke device. This device emulates a 101-key USB keyboard and types characters into the terminal without human intervention.

### User Interface
One of the goals of the project is to provide a means to send passwords automatically from a user's personal Bluetooth "smart" device like a smartphone. This Bluetooth device would be controlling the USB keyboard device using Bluetooth Low Energy communication. Therefore, the USB keyboard device will have BLE capabilities as well.

Once the communication and USB interface are defined, a frontend to the system would maintain a database of passwords, usernames, and email addresses which the user can select and send to the target terminal.

### Security Considerations
Passwords being sensitive data have to be stored and transferred over comm channels securely. To achieve this, the data over Bluetooth has to be encrypted and the passwords stored in the user device have to be encrypted as well. In addition to this, since the software is open source, the system should only work if the personal Bluetooth device has been authenticated. 

So, before enabling the transfer of characters, a token will be sent which will serve as a challenge from the USB-Bluetooth keyboard device to the user's personal smart device. The response to this device will be a 16-byte encrypted block of bytes which will be derived by encrypting the token using a key and initialization vector that can be defined once while configuring the software. This key and IV will be an arbitrary choice by the user and will help in making the system unique to the user thereby preventing unauthorized control of the USB-Bluetooth keyboard. Furthermore, the token will only be sent upon pressing a button which will also initialize the device simultaneously.

### Usage
![Concept](https://github.com/user-attachments/assets/1b8c81fe-bc2c-43ab-a688-51d66a193520)

### Checkout...
[Hackster.io article](https://www.hackster.io/hardcoder/project-prism-a-password-manager-like-no-other-65b33d)

[Video](https://www.youtube.com/watch?v=PWb6pFhg7UU)
