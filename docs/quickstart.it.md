---
layout: default
title: Guida Rapida - io_decoder
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
</style>

<div class="nav-container">
  <a href="https://bobwolfrst.github.io/io_decoder-linuxCNC/index.it" class="nav-btn">🏠 Home</a>
  <a href="./README.it.md" class="nav-btn">📖 Manuale</a>
  <a href="https://github.com/bobwolfrst/io_decoder-linuxCNC" class="nav-btn">💻 GitHub</a>
  <a href="mailto:io.decoder.rst%40gmail.com" class="nav-btn contact-btn">✉️ Contatto</a>
  <a href="https://bobwolfrst.github.io/io_decoder-linuxCNC/quickstart.en" class="nav-btn lang-btn">
    <img src="https://flagcdn.com/w20/gb.png" width="20" alt="UK Flag"> English
  </a>
</div>

# ⚡ Guida Rapida (Quick Start)

Benvenuto nella guida rapida di **io_decoder**. Segui questi passaggi per configurare la tua scheda in meno di 5 minuti.

---

### 1. Requisiti di Sistema
* **Sistema Operativo:** Linux con kernel Real-Time.
* **LinuxCNC:** Versione 2.8 o superiore.
* **Hardware:** Una porta USB libera.

---

### 2. Installazione del Driver

Apri il terminale nella cartella del progetto e compila il modulo HAL.

Crea una regola udev per fissare il collegamento con un nome univoco 

---

### 3. Configurazione HAL
Aggiungi queste righe al tuo file di configurazione .hal per integrare la scheda:

```bash
   loadrt io_decoder output=24 input=24
   addf io_decoder.update	servo-thread
```

---

### 4. Test e Diagnostica
Verifica il funzionamento lanciando lo strumento di monitoraggio 'halshow' di linuxCNC:

Nella sezione Pins, cerca io_decoder.in.00-0 per vedere i segnali in tempo reale se per esempio hai messo un pulsante collegato al morsetto 0 della scheda 00.

---

Nel [manuale](./README.it.md) è spiegato a fondo tutte le configurazioni e possibilità di questo sistema hardware/software per linuxCNC.

## 🔗 Risorse Utili
* 🏠 [Torna alla Home Page](https://bobwolfrst.github.io/io_decoder-linuxCNC/index.it)
* 📖 [Consulta il Manuale Tecnico Completo](./README.it.md)

<hr style="margin-top: 50px; border: 0; border-top: 1px solid #eee;">
<footer style="padding: 20px 0; text-align: center; color: #666; font-size: 0.9em;">
  <p><strong>io_decoder</strong> - Driver Open Source per LinuxCNC</p>
  <p>
    <a href="mailto:io.decoder.rst%40gmail.com" style="color: #1e6bb8; text-decoration: none;">✉️ Contatto</a> | 
    <a href="https://github.com/bobwolfrst/io_decoder-linuxCNC" style="color: #1e6bb8; text-decoration: none;">💻 GitHub Repository</a>
  </p>
  <p style="font-size: 0.8em;">© 2026 - Creato da bobwolfrst. Rilasciato sotto licenza GPL.</p>
</footer>