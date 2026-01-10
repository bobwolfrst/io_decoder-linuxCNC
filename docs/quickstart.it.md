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
  .lang-btn { background-color: #f8f9fa; }
</style>

<div class="nav-container">
  <a href="./" class="nav-btn">🏠 Home</a>
  <a href="./README.it" class="nav-btn">📖 Manuale</a>
  <a href="https://github.com/bobwolfrst/io_decoder-linuxCNC" class="nav-btn">💻 GitHub</a>
  <a href="./quickstart.en" class="nav-btn lang-btn">🇬🇧 English</a>
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

**[INSERISCI QUI IL BLOCCO BASH: sudo make install]**

*Questo comando compila il driver C e configura le regole udev per l'accesso USB.*

---

### 3. Configurazione HAL
Aggiungi queste righe al tuo file di configurazione .hal per integrare la scheda:

**[INSERISCI QUI IL BLOCCO HAL PER CARICARE IL DRIVER E I PIN]**

---

### 4. Test e Diagnostica
Verifica il funzionamento aprendo il terminale e lanciando lo strumento di monitoraggio:

**[INSERISCI QUI IL COMANDO: halshow]**

Nella sezione Pins, cerca io_decoder.0 per vedere i segnali in tempo reale.

---

## 🔗 Risorse Utili
* 🏠 [Torna alla Home Page](./)
* 📖 [Consulta il Manuale Tecnico Completo](./README.it)