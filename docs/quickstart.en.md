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
  .lang-btn { background-color: #f8f9fa; }
</style>

<div class="nav-container">
  <a href="./" class="nav-btn">🏠 Home</a>
  <a href="./README" class="nav-btn">📖 Manual</a>
  <a href="https://github.com/bobwolfrst/io_decoder-linuxCNC" class="nav-btn">💻 GitHub</a>
  <a href="./quickstart.it" class="nav-btn lang-btn">🇮🇹 Versione Italiana</a>
</div>

# ⚡ Quick Start Guide

Welcome to the **io_decoder** quick start guide. Follow these steps to get your board up and running in less than 5 minutes.

---

### 1. System Requirements
* **OS:** Linux with Real-Time kernel (RT-Preempt).
* **LinuxCNC:** Version 2.8 or higher.
* **Hardware:** One free USB port (bus-powered).

---

### 2. Driver Installation
Open your terminal in the project folder and compile the HAL module.

**[INSERT BASH BLOCK HERE: sudo make install]**

*This command compiles the C driver and sets up udev rules for USB access.*

---

### 3. HAL Configuration
Add these lines to your .hal configuration file to integrate the board:

**[INSERT HAL BLOCK HERE FOR LOADING DRIVER AND PINS]**

---

### 4. Testing and Diagnostics
Verify everything is working by opening the terminal and launching the monitoring tool:

**[INSERT COMMAND HERE: halshow]**

In the Pins section, look for `io_decoder.0` to see real-time signals as you press physical buttons.

---

## 🔗 Useful Links
* 🏠 [Back to Home Page](./)
* 📖 [Check the Full Technical Manual](./README)
* 🛠️ [MPG and Joystick Configuration Examples](./README#examples)