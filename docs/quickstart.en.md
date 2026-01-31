---
layout: default
title: Quick Start - io_decoder
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
    flex-wrap: wrap;
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
    transform: translateY(-2px);
  }
  /* Specific style for contact button */
  .contact-btn {
    border-color: #28a745;
    color: #28a745 !important;
  }
  .contact-btn:hover {
    background-color: #28a745;
    color: white !important;
  }
  .lang-btn { background-color: #f8f9fa; }
</style>

<div class="nav-container">
  <a href="https://bobwolfrst.github.io/io_decoder-linuxCNC/index" class="nav-btn">🏠 Home</a>
  <a href="./README.en.md" class="nav-btn">📖 Manual</a>
  <a href="https://github.com/bobwolfrst/io_decoder-linuxCNC" class="nav-btn">💻 GitHub</a>
  <a href="mailto:io.decoder.rst%40gmail.com" class="nav-btn contact-btn">✉️ Contact</a>
  <a href="https://bobwolfrst.github.io/io_decoder-linuxCNC/quickstart.it" class="nav-btn lang-btn">
    <img src="https://flagcdn.com/w20/it.png" width="20" alt="Italy Flag"> Italiano
  </a>
</div>

# ⚡ Quick Start Guide

Welcome to the **io_decoder** quick start guide. Follow these steps to get your board up and running in less than 5 minutes.

---

### 1. System Requirements
* **OS:** Linux with Real-Time kernel.
* **LinuxCNC:** Version 2.8 or higher.
* **Hardware:** One free USB port.

---

### 2. Driver Installation
Open the terminal in the project folder and compile the HAL module.

Create a udev rule to set a unique persistent symlink for the device.

---

### 3. HAL Configuration
Add the following lines to your .hal configuration file to integrate the board:

```bash
   loadrt io_decoder output=24 input=24
   addf io_decoder.update	servo-thread
```

---

### 4. Testing and Diagnostics
Check that everything is working by launching the LinuxCNC 'halshow' tool:

In the Pins section, look for `io_decoder.in.00-0` to see real-time signals (e.g., if you have a button connected to terminal 0 of board 00).

---

The [full manual](./README.en.md) explains in depth all the configurations and possibilities of this hardware/software system for LinuxCNC.

## 🔗 Useful Resources
* 🏠 [Back to Home Page](https://bobwolfrst.github.io/io_decoder-linuxCNC/index)
* 📖 [Read the Full Technical Manual](./README.en.md)

<hr style="margin-top: 50px; border: 0; border-top: 1px solid #eee;">
<footer style="padding: 20px 0; text-align: center; color: #666; font-size: 0.9em;">
  <p><strong>io_decoder</strong> - Driver Open Source per LinuxCNC</p>
  <p>
    <a href="mailto:io.decoder.rst%40gmail.com" style="color: #1e6bb8; text-decoration: none;">✉️ Contact</a> | 
    <a href="https://github.com/bobwolfrst/io_decoder-linuxCNC" style="color: #1e6bb8; text-decoration: none;">💻 GitHub Repository</a>
  </p>
  <p style="font-size: 0.8em;">© 2026 - Creato da bobwolfrst. Rilasciato sotto licenza GPL.</p>
</footer>

