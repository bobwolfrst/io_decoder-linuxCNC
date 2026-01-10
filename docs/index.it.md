---
layout: default
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
  <a href="./quickstart.it" class="nav-btn">⚡ Quick Start</a>
  <a href="./README.it" class="nav-btn">📖 Manuale</a>
  <a href="https://github.com/bobwolfrst/io_decoder-linuxCNC" class="nav-btn">💻 GitHub</a>
  <a href="./index" class="nav-btn lang-btn">🇬🇧 Switch to English</a>
</div>
# io_decoder

### La soluzione per l'I/O su LinuxCNC: Stabile, Veloce, Plug&Play.



**io_decoder** non è il solito driver generico. È un ecosistema hardware e software nato per risolvere i problemi comuni delle interfacce USB usate su linuxCNC, offrendo prestazioni di livello per la tua macchina CNC. Il progetto è nato per dare la possibilità di poter realizzare un ampio, flessibile e strutturato pannello di controllo per linuxCNC senza perdere preziosi pin che servono per i movimenti in realtime della macchina. 

---

## 🎯 Perché io_decoder?
<img src="mini_assieme_usb_IODECODER_5.png" align="right" width="280" style="margin-left: 20px; border-radius: 8px; border: 1px solid #ddd;">
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
| **Ingressi digitali** | da 8 a 128 con funzionalità aggiuntive. |
| **Ingressi encoder** | 4 con funzionalità aggiuntive. |
| **Ingressi ADC** | 3 con funzionalità aggiuntive. |
| **Uscite digitali** | da 8 a 128 con funzionalità aggiuntive.   |
| **Uscite DAC** | 2 con funzionalità aggiuntive. |

* **Documentazione tecnica ed installazione:** Disponibile nel [README.it](./README.it.md)..

---

## 📦 Ottieni l'Hardware

Il software HAL è aperto alla comunità, e per usarlo è necessaria la nostra scheda dedicata **io_decoder base** e le schede **io_decoder expansion_8**.

* **Stato del progetto:** In sviluppo.
* **Documentazione tecnica:** Disponibile nel paragrafo Installazione-Requisiti del [README.it](./README.it.md) .
* **Contatti:** Per acquisti, prototipi o supporto all'integrazione, scrivi a: `bobwolf.rst@gmail.com

---


> **Nota per gli sviluppatori:** > Se vuoi contribuire al driver HAL, clona la repository e consulta la sezione dedicata agli sviluppatori.
>
> [📖 Manuale Italiano](./README.it.md)
