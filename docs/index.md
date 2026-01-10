---
layout: default
title: io_decoder - USB Operator Panel Interface for LinuxCNC
---

<style>
  .nav-container {
    position: sticky;
    top: 0;
    z-index: 9999;
    background-color: rgba(255, 255, 255, 0.95);
    backdrop-filter: blur(5px);
    border-bottom: 2px solid #1e6bb8;
    padding: 12px 0;
    margin-bottom: 25px;
    display: flex;
    justify-content: center;
    flex-wrap: wrap; /* Per vederlo bene anche su cellulare */
    gap: 12px;
  }
  .nav-btn {
    text-decoration: none !important;
    padding: 8px 18px;
    border-radius: 25px;
    font-size: 0.95em;
    font-weight: 600;
    transition: all 0.3s ease;
    border: 1.5px solid #1e6bb8;
    color: #1e6bb8 !important;
    display: flex;
    align-items: center;
    gap: 8px;
  }
  .nav-btn:hover {
    background-color: #1e6bb8;
    color: white !important;
    transform: translateY(-2px); /* Effetto al passaggio del mouse */
  }
  .lang-btn {
    background-color: #f8f9fa;
  }
</style>

<div class="nav-container">
  <a href="./quickstart.en" class="nav-btn">⚡ Quick Start</a>
  <a href="./README.en" class="nav-btn">📖 Manuale</a>
  <a href="https://github.com/bobwolfrst/io_decoder-linuxCNC" class="nav-btn">💻 GitHub</a>
  <a href="./index.it" class="nav-btn lang-btn">
    <img src="https://flagcdn.com/w20/it.png" width="20" alt="Italy Flag"> Italiano
  </a>
</div>
# io_decoder

### The I/O solution for LinuxCNC: Stable, Fast, Plug & Play.



**io_decoder** is not just another generic driver. It is a hardware and software ecosystem specifically designed to overcome the common limitations of USB interfaces on LinuxCNC, delivering high-level performance for your machine. This project was born to provide a flexible, large-scale, and structured control panel for LinuxCNC without wasting precious real-time pins required for machine motion.

---

## 🎯 Why io_decoder?
<img src="mini_assieme_usb_IODECODER_5.png" align="right" width="280" style="margin-left: 20px; border-radius: 8px; border: 1px solid #ddd;">
Here is how **io_decoder** changes the game:

### 1. Zero Port Conflicts (Custom USB ID)
Stop struggling with `/dev/ttyACM0` ports that change names upon reboot. Our board is uniquely recognized by the system. Configure it once, and it works forever.

### 2. "Bare-Metal" Firmware (No Bootloader)
We have removed the Arduino bootloader for two critical reasons:
* **Safety:** No random pin state changes during startup (typical of bootloaders).
* **Speed:** The board is operational within a few milliseconds.

### 3. Native HAL Driver
The driver is written in C to integrate perfectly into the LinuxCNC Real-Time environment, ensuring minimal latency and a level of stability that generic drivers simply cannot offer.

### 4. Simple yet Robust System
Developed to be as easy to install and use as possible. Stable and tested both on the USB software management side and the electronic hardware side.

---

## 🛠 Technical Specifications

| Feature | Description |
| :--- | :--- |
| **Compatibility** | LinuxCNC 2.8+ (HAL component) |
| **Connection** | High-Speed USB with custom ID |
| **Digital Inputs** | From 8 to 128 with additional functionalities |
| **Encoder Inputs** | 4 with additional functionalities |
| **ADC Inputs** | 3 with additional functionalities |
| **Digital Outputs** | From 8 to 128 with additional functionalities |
| **DAC Outputs** | 2 with additional functionalities |

* **Technical Documentation & Installation:** Available in the [README.en](./README.en.md).

---

## 📦 Get the Hardware

While the HAL software is open to the community, it requires our dedicated **io_decoder base** board and **io_decoder expansion_8** boards to function.

* **Project Status:** In development.
* **Technical Specs:** Available in the Installation-Requirements section of the [README.en](./README.en.md).
* **Contact:** For purchases, prototypes, or integration support, write to: `bobwolf.rst@gmail.com`

---

> **Note for Developers:** > If you wish to contribute to the HAL driver, clone the repository and consult the developer section.

[📖 English Manual](./README.en.md)