---
layout: default
permalink: /index.it/
title: io_decoder - Interfaccia pannello operatore USB per LinuxCNC 
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
    .youtube-btn {
    border-color: #ff0000;
    color: #ff0000 !important;
  }
  .youtube-btn:hover {
    background-color: #ff0000;
    color: white !important;
  }
  /* Stile specifico per il tasto contatto */
  .contact-btn {
    border-color: #28a745;
    color: #28a745 !important;
  }
  .contact-btn:hover {
    background-color: #28a745;
    color: white !important;
  }
  .lang-btn {
    background-color: #f8f9fa;
  }
  .demo-btn {
    border-color: #f39c12;
    color: #f39c12 !important;
  }
  .demo-btn:hover {
    background-color: #f39c12;
    color: white !important;
  }
</style>

<div class="nav-container">
  <a href="https://www.youtube.com/playlist?list=PL9D_TSVxg-gDFtA9_k6njBeVTL0IYY7Ct" target="_blank" class="nav-btn youtube-btn">▶ Video Test YouTube</a>
  <a href="https://bobwolfrst.github.io/io_decoder-linuxCNC/quickstart.it" class="nav-btn">⚡ Quick Start</a>
  <a href="https://bobwolfrst.github.io/io_decoder-linuxCNC/demo_mode.it" class="nav-btn demo-btn">🚀 Demo Mode</a>
  <a href="https://github.com/bobwolfrst/io_decoder-linuxCNC/blob/main/docs/README.it.md" class="nav-btn">📖 Manuale</a>
  <a href="https://github.com/bobwolfrst/io_decoder-linuxCNC" class="nav-btn">💻 GitHub</a>
  <a href="mailto:io.decoder.rst%40gmail.com" class="nav-btn contact-btn">✉️ Contatto</a>
  <a href="https://bobwolfrst.github.io/io_decoder-linuxCNC/index" class="nav-btn lang-btn">
    <img src="https://flagcdn.com/w20/gb.png" width="20" alt="UK Flag"> English
  </a>
</div>
# io_decoder

### La soluzione per l'I/O su LinuxCNC: Stabile, Veloce, Plug&Play.



**io_decoder** non è il solito driver generico. È un ecosistema hardware e software nato per risolvere i problemi comuni delle interfacce USB usate su linuxCNC, offrendo prestazioni di livello per la tua macchina CNC. Il progetto è nato per dare la possibilità di poter realizzare un ampio, flessibile e strutturato pannello di controllo per linuxCNC senza perdere preziosi pin che servono per i movimenti in realtime della macchina. 

---

## 🎯 Perché io_decoder?
<img src="mini_usb_IODEC_5.png" align="right" width="280" style="margin-left: 20px; border-radius: 8px; border: 1px solid #ddd;">
Ecco come **io_decoder** cambia le regole del gioco:

### 1. Zero conflitti di porta (Custom USB ID)
Basta impazzire con le porte `/dev/ttyACM0` che cambiano nome al riavvio. La nostra scheda viene riconosciuta univocamente dal sistema. La configuri una volta, funziona per sempre.

### 2. Firmware "Bare-Metal" (Senza Bootloader)
Abbiamo eliminato il bootloader di Arduino per due motivi critici:
* **Sicurezza:** Nessun movimento casuale dei pin all'accensione (tipico del bootloader).
* **Velocità:** La scheda è operativa in pochi millisecondi.

### 3. Driver HAL Nativo
Il driver è scritto in C per integrarsi perfettamente nel Real-Time di LinuxCNC, garantendo una latenza minima e una stabilità che i driver generici non possono offrire.

### 4. Sistema semplice ma robusto
Sviluppato per essere il più semplice da installare ed usare. Stabile e testato sia dal lato della gestione software dell'USB sia nel lato elettronico.

---

## 🛠 Caratteristiche Tecniche

| Funzionalità | Descrizione |
| :--- | :--- |
| **Compatibilità** | LinuxCNC 2.8+ (HAL component) |
| **Connessione** | USB High-Speed con ID personalizzato |
| **Ingressi digitali** | da 8 a 128 con funzionalità di keyboard emulata e toggle. |
| **Ingressi encoder** | 4 con funzionalità up/down e invert. |
| **Ingressi ADC** | 3 con funzionalità joystick, invert e scale. |
| **Uscite digitali** | da 8 a 128 con funzionalità blink.   |
| **Uscite DAC** | 2 con funzionalità invert e scale. |

* **Documentazione tecnica ed installazione:** Disponibile nel [README.it](./README.it.md).

---

## 📦 Ottieni l'Hardware

Il software HAL è aperto alla comunità, e per usarlo è necessaria la nostra scheda dedicata **io_decoder base** e le schede **io_decoder expansion_8**.

* **Stato del progetto:** In sviluppo.
* **Documentazione tecnica:** Disponibile nel paragrafo Installazione-Requisiti del [README.it](./README.it.md) .
* **Contatti:** Per prototipi o supporto all'integrazione, utilizza il pulsante [✉️ Contatto]

---


> **Nota per gli sviluppatori:** > Se vuoi contribuire al driver HAL, clona la repository e consulta la sezione dedicata agli sviluppatori.
  
[📖 Manuale Italiano](./README.it.md)
[🚀 Demo/Eval mode in Italiano](https://bobwolfrst.github.io/io_decoder-linuxCNC/demo_mode.it)  

<hr style="margin-top: 50px; border: 0; border-top: 1px solid #eee;">
<footer style="padding: 20px 0; text-align: center; color: #666; font-size: 0.9em;">
  <p><strong>io_decoder</strong> - Driver Open Source per LinuxCNC</p>
  <p>
    <a href="mailto:io.decoder.rst%40gmail.com" style="color: #1e6bb8; text-decoration: none;">✉️ contatto</a> | 
    <a href="https://github.com/bobwolfrst/io_decoder-linuxCNC" style="color: #1e6bb8; text-decoration: none;">💻 GitHub Repository</a>
  </p>
  <p style="font-size: 0.8em;">© 2026 - Creato da bobwolfrst. Rilasciato sotto licenza GPL.</p>
</footer>
