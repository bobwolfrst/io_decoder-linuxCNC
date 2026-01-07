---
layout: default
title: io_decoder - Interfaccia pannello operatore USB per LinuxCNC
---

# io_decoder
### La soluzione definitiva per l'I/O su LinuxCNC: Stabile, Veloce, Plug&Play.



**io_decoder** non è il solito driver generico. È un ecosistema hardware e software nato per risolvere i problemi comuni delle interfacce USB economiche, offrendo prestazioni di livello industriale per la tua macchina CNC. 

---

## 🎯 Perché io_decoder?

Se usi LinuxCNC, conosci i problemi delle schede standard. Ecco come **io_decoder** cambia le regole del gioco:

### 1. Zero conflitti di porta (Custom USB ID)
Basta impazzire con le porte `/dev/ttyACM0` che cambiano nome al riavvio. La nostra scheda viene riconosciuta univocamente dal sistema. La configuri una volta, funziona per sempre.

### 2. Firmware "Bare-Metal" (Senza Bootloader)
Abbiamo eliminato il bootloader di Arduino per due motivi critici:
* **Sicurezza:** Nessun movimento casuale dei pin all'accensione (tipico del bootloader).
* **Velocità:** La scheda è operativa in pochi millisecondi.

### 3. Driver HAL Nativo
Il driver è scritto in C per integrarsi perfettamente nel Real-Time di LinuxCNC, garantendo una latenza minima e una stabilità che i driver generici non possono offrire.

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

* **Documentazione tecnica:** Disponibile nel [README](./README.md).

---

## 📦 Ottieni l'Hardware

Il software HAL è aperto alla comunità, ma per le massime prestazioni consigliamo l'utilizzo della nostra scheda dedicata **io_decoder-v4**.

* **Stato del progetto:** In pre-produzione.
* **Documentazione tecnica:** Disponibile nel [README-Installazione-Requisiti](./README.md) .
* **Contatti:** Per acquisti, prototipi o supporto all'integrazione, scrivi a: `bobwolf.rst@gmail.com

---


> **Nota per gli sviluppatori:** > Se vuoi contribuire al driver HAL, clona la repository e consulta la sezione dedicata agli sviluppatori.
