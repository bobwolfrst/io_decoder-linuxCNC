---
layout: default
---
# io_decoder
<a id="indice"></a>
## Indice

- [io\_decoder](#io_decoder)
  - [Indice](#indice)
  - [Descrizione](#descrizione)
  - [Caratteristiche principali](#caratteristiche-principali)
    - [Componente HAL](#componente-hal)
    - [Hardware USB](#hardware-usb)
      - [Caratteristiche dipendenti da hardware e firmware della scheda USB](#caratteristiche-dipendenti-da-hardware-e-firmware-della-scheda-usb)
  - [Installazione](#installazione)
    - [Sinossi](#sinossi)
    - [Funzioni](#funzioni)
    - [Pin](#pin)
    - [Requisiti](#requisiti)
    - [Comandi di installazione](#comandi-di-installazione)
  - [Utilizzo](#utilizzo)
    - [Esempio di configurazione](#esempio-di-configurazione)
    - [Funzionalità disponibili](#funzionalità-disponibili)
    - [Keyboard](#keyboard)
  - [Dipendenze](#dipendenze)
  - [Autori](#autori)
  - [Licenza](#licenza)

[📖 README](../README.md) | [🏠 Project Home]({{ '/index.it' | relative_url }})  
[🚀 Demo/Eval mode in Italiano]({{ '/demo_mode.it' | relative_url }})  
  
  
<a id="descrizione"></a>
## Descrizione
io_decoder è un componente di HAL per Linuxcnc. Permette di controllare, attraverso un hardware esterno con connessione USB, gli input e gli output che servono per gestire un pannello operatore di macchina CNC.
È un componente in tempo reale ma la sua parte di comunicazione USB è gestita da un thread non realtime.
Gestisce in maniera ottimale input ed output non critici come pulsanti, interruttori e spie di segnalazione per l'interazione manuale del pannello di controllo e quadretti di gestione dislocati sulla macchina.  

<a id="caratteristiche-principali"></a>
[torna all'indice](#indice)  
## Caratteristiche principali
### Componente HAL
- **Input digitali**: 8-128 liberamente configurabili da file .hal; questo valore deve essere coerente con l'hardware installato.
  - antirimbalzo software di 20ms.
  - pin aggiuntivo con toggle del valore sul fronte di salita dell'input.  
  - funzionalità di keyboard simulata.
  - in caso di disconnessione usb, il valore del pin di hal viene portato a 0.  

- **Output digitali**: 8-128 liberamente configurabili da file .hal; questo valore deve essere coerente con l'hardware installato. 
  - Funzione per far lampeggiare le uscite. Impostabile singolarmente nella frequenza e periodo per ogni uscita.
  - in caso di disconnessione usb, l'uscita fisica viene spenta dal watchdog dell'Arduino.  

- **Encoder in quadratura (jog-weel)**: il numero può variare in base al firmware della scheda usb.
  - uscita encoder per il valore del conteggio
  - ingresso invert per l'inversione del conteggio.
  - uscita up e uscita down per poter pilotare direttamente componenti HAL tipo UPDOWN o MULTISWITCH.

- **DAC**: il numero può variare in base al firmware della scheda usb.
  - ingresso dac per il valore di uscita.
  - ingresso scale per scalare il valore di uscita.
  - ingresso invert per l'inversione del valore di uscita.

- **ADC**: il numero può variare in base al firmware della scheda usb.
  - uscita adc con il valore acquisito, filtrato e convertito.
  - uscita raw con il valore acquisito senza conversioni
  - ingresso scale per scalare il valore acquisito.
  - ingresso invert per l'inversione del valore acquisito.
  - uscita up e uscita down per poter pilotare direttamente componenti HAL tipo UPDOWN o MULTISWITCH.  
  - uscita con valore derivato dal conteggio up/down  (joystick)

- **Pin diagnostica**:
  - uscita per segnalare lo stato del componente
  - uscita per la percentuale totale degli errori nel flusso USB
  - uscita per la percentuale degli errori di parsing ogni minuto
  - uscita per segnalare lo stato della connessione USB  
  - uscita del valore del jitter del base-thread  
  - uscita del valore del jitter del servo-thread
  - uscita del valore del jitter del usb-thread interno
  - uscita del valore del jitter della trasmissione USB   

<a id="hardware-usb"></a>
### Hardware USB  

#### expansion_8  
- **Input digitali**: 8-128 liberamente configurabili con schede di espansione da 8 pin ognuna. Viene accettato come ingresso solamente contatti puliti tra il pin comune e l'ingresso digitale.  
- **Output digitali**: 8-128 liberamente configurabili con schede di espansione da 8 pin ognuna. Ogni uscita può pilotare un carico da 50mA@30Vdc con un massimo di 300mA per scheda di espansione.

#### Caratteristiche dipendenti da hardware e firmware della scheda USB
- **Firmware 101**
  - Encoder in quadratura: 4 @5Vdc
  - DAC (PWM): 2 @8bit 5Vdc
  - ADC: 3 @10bit 5Vdc
  - La cadenza di comunicazione HAL<=>USB è di 20ms (50Hz)  

- **Firmware 255**
  - versione speciale di valutazione per arduino UNO R3
  - 4 input digitali
  - 4 output digitali
  - Encoder in quadratura: 1 @5Vdc
  - DAC (PWM): 1 @8bit 5Vdc
  - ADC: 1 @10bit 5Vdc
  
#### Dimensioni e collegamenti
- [**Pinout scheda io_decoder USB**]({{ '/IODECODER.schemi.it' | relative_url }})

[torna all'indice](#indice)  
<a id="installazione"></a>
  
## Installazione
### Sinossi
- **loadrt io_decoder** [input=*num*] [output=*num*] [usb_port_name=*"string"*] [firmware=*num*] [verbose=*num*] [keymap_file=*"string"*] [uinput_chmod_cmd=*"string"*]  
   - **input**: questo valore deve essere coerente con l'hardware installato. Il numero deve essere un multiplo di 8 (min 8 max 128) altrimenti da errore all'avvio. Valore di default= 8
   - **output**: questo valore deve essere coerente con l'hardware installato. Il numero deve essere un multiplo di 8 (min 8 max 128) altrimenti da errore all'avvio. Valore di default= 8
   - **usb_port_name**: per poter nominare la porta a piacere e renderla fissa; vedere sezione [**definire porta USB**](#comandi-di-installazione) . Valore di default "/dev/io_decoder"
   - **firmware**: parametro per configurare il componente con le varie funzionalità della scheda USB. E' indicato sul pcb della scheda USB. Valore di default= 101
   - **verbose**: per abilitare il livello dei messaggi di errore sulla GUI. il numero attiva il tipo di messaggio indicato e quelli di valore inferiore. default 1.  
     - 0=nessuno.
     - 1=componente. Invia messaggio in caso di disconnessione o di riavvio della comunicazione USB e segnala i messaggi della funzionalità keyboard se non è attivata per qualsiasi motivo.
     - 2=minimi. Messaggi di percentuale di errore parsing.
     - 3=tutti.
   - **keymap_file**: file di testo per impostare le corrispondenze [**input => simulazione tastiera**](#keyboard). Valore di default "io_decoder-keymap.cfg"
   - **uinput_chmod_cmd**: parametro stringa per dare i permessi di scrittura su UINPUT per la [**funzionalità di tastiera simulata**](#keyboard). Se si vuole essere sicuri di non dare i permessi il parametro deve essere "" (uinput_chmod_cmd="" senza niente all'interno delle virgolette) . Valore di default "chmod 0666 /dev/uinput" .  

### Funzioni
```bash
addf io_decoder.update	servo-thread
```  
<a id="pin"></a>
### Pin
- **IO digitali**
-  Input
	- **io_decoder.in.*MM*-*N*** (bit out): pin per leggere lo stato degli ingressi digitali. *MM*= numero a due cifre per indicare la posizione della scheda di espansione. *N*= numero ad una cifra per indicare l'ingresso sulla scheda di espansione. Su ogni pin è presente un antirimbalzo software di 20ms. Il pin non viene creato nel caso in cui l'input abbia funzionalità keyboard. default 0.    
    - **io_decoder.in.*MM*-*N*.toggle** (bit out): questo pin varia il proprio valore da 0 ad 1 e da 1 a 0, sul fronte di salita del proprio ingresso digitale. Il pin non viene creato nel caso in cui l'input abbia funzionalità keyboard. default 0.  
	- **io_decoder.in.*MM*-*N-keyboard*** (bit out): pin che viene usato per inviare i segnali alla tastiera simulata. Viene creato nel caso in cui l'input abbia funzionalità tastiera, può essere usato ancora come pin di hal ed ha le medesime caratteristiche del pin ordinario. [**per configurare i pin**](#keyboard). default 0.   
 - Output
	- **io_decoder.out.*MM*-*N*** (bit in): pin per settare lo stato delle uscite digitali. *MM*= numero a due cifre per indicare la posizione della scheda di espansione. *N*= numero ad una cifra per indicare l'uscita sulla scheda di espansione. default 0.     
   - **io_decoder.out.*MM*-*N*.blink-en** (bit in): enable che abilita il lampeggio dell'uscita. se il corrispondente pin hal di output è a 0 ed enable è in qualsiasi stato, l'uscita è spenta. se è ad 1 ed enable è a 0, l'uscita è accesa fissa. se è ad 1 ed enable è ad 1, l'uscita lampeggia con la frequenza impostata. default 0.  
   - **io_decoder.out.*MM*-*N*.blink-freq** (float in): frequenza (Hz) del lampeggio dell'uscita. minimo 0,25Hz massimo 16Hz. sopra e sotto questi valori la frequenza viene settata al minimo o al massimo. la possibilità di impostare la frequenza per ogni lampeggio in modo indipendente per ogni uscita è per evitare di avere lampeggi simultanei di tutte le uscite. è sufficiente dare 0,01/0,02Hz di differenza fra un'uscita e l'altra per creare un effetto caotico, e non visivamente sincronizzato, delle segnalazioni.  default 1Hz.  
   - **io_decoder.out.*MM*-*N*.blink-width** (float in): varia la quantità del periodo che l'impulso sta alto. valori 0/1. dafault 0.5 (50%).  

- Encoder
	- **io_decoder.enc.*N*** (S32 out): per il valore del conteggio. default 0.  
	- **io_decoder.enc.*N*.invert** (bit in): per l'inversione del conteggio. default 0.  
	- **io_decoder.enc.*N*.up** (bit out): per poter pilotare direttamente componenti HAL tipo UPDOWN o MULTISWITCH.
	- **io_decoder.enc.*N*.down** (bit out): per poter pilotare direttamente componenti HAL tipo UPDOWN o MULTISWITCH.

- ADC
	- **io_decoder.adc.*N*** (float out): valore acquisito, filtrato e convertito. default 0.  
	- **io_decoder.adc.*N*.raw** (S32 out): valore acquisito senza conversioni. default 0.  
	- **io_decoder.adc.*N*.invert** (bit in): per l'inversione del valore acquisito. default 0.  
	- **io_decoder.adc.*N*.scale** (float in):per scalare il valore acquisito. default 0.  
  - sezione joystick
    - **io_decoder.adc.*N*.joy.center** (float in):per centrare il joystick. valore -1/+1. default 0.
    - **io_decoder.adc.*N*.joy.deadband** (float in):per il punto centrale con uscita disattivata. valore 0/+1. default 0.
    - **io_decoder.adc.*N*.joy.factor** (float in):fattore di sensibilità. valore 0/+1. default 0.
    - **io_decoder.adc.*N*.joy.pulse.up** (bit out):per poter pilotare direttamente componenti HAL tipo UPDOWN o MULTISWITCH.
    - **io_decoder.adc.*N*.joy.pulse.down** (bit out):per poter pilotare direttamente componenti HAL tipo UPDOWN o MULTISWITCH.
    - **io_decoder.adc.*N*.joy.count** (S32 out):valore diretto ottenuto dal conteggio up/down.  

 - DAC
	- **io_decoder.dac.*N*** (float in): valore di uscita per il DAC. default 0.  
	- **io_decoder.dac.*N*.scale** (float in): per scalare il valore di uscita. default 0.  
	- **io_decoder.dac.*N*.invert** (bit in): per l'inversione del valore di uscita. default 0.  

  - Diagnostica
    - **io_decoder.diag.comm-state** (S32 out): uscita che segnala lo stato della comunicazione del componente.  
      - HANDSHAKE = 0  
      - HANDSHAKE_WAIT_RESPONSE = 1  
      - COMMUNICATING = 2  
      - COMMUNICATING_WAIT_RESPONSE = 3  
      - ERROR = 4
    - **io_decoder.diag.error-count** (float out): percentuale totale degli errori durante la comunicazione USB, da quando il componente è attivo. Aggiornato ogni minuto.
    - **io_decoder.diag.parse-error** (float out): Percentuale di errore dell'ultimo minuto. Se supera il 5% è possibile che il sistema PC/LinuxCNC sia lento e produca valori di jitter alti. Se il cavo USB non è di buona qualità oppure è installato in posizioni che captano molti disturbi elettrici/elettromagnetici il valore sale frequentemente sopra l'1%. Questo valore dovrebbe rimanere sempre sotto l'1%. 
    - **io_decoder.diag.usb-connected** (bit out): Segnala che la comunicazione USB è attiva = 1 o scollegata/inattiva = 0. 
    - **io_decoder.diag.servo-thread-time** (S32 in): Da collegare al pin servo-thread.time per conteggiare il jitter. 
    - **io_decoder.diag.base-thread-time** (S32 in): Da collegare al pin base-thread.time per conteggiare il jitter. 
    - **io_decoder.diag.servo-thread-jitter** (S32 out): Valore del jitter del servo-thread nell'ultimo minuto. 
    - **io_decoder.diag.base-thread-jitter** (S32 out): Valore del jitter del base-thread nell'ultimo minuto. 
    - **io_decoder.diag.usb-thread-jitter** (S32 out): Valore del jitter dell'usb-thread, creato internamente al componente, nell'ultimo minuto. Questo valore dovrebbe essere il più inferiore possibile, tenendo conto che il thread è richiamato con cadenza di 1ms. Valori molto alti indicano che il PC/sistema non è performante ed induce ritardi eccessivi nel realtime.
    - **io_decoder.diag.usb-communication-jitter** (S32 out): Valore del jitter dei tempi di esecuzione della sola ricetrasmissione USB nell'ultimo minuto. Questo valore dovrebbe essere il più basso possibile, tenendo conto che tutto il processo di ricezione e trasmissione sulla USB impiega circa 8ms al massimo.  

<a id="requisiti"></a>

### Requisiti  
- **Hardware**
  - [Pinout scheda io_decoder USB](IODECODER.schemi.it.md)

- **Software**
  - LinuxCNC 2.8 e successivi
  - il componente è stato realizzato su un sistema creato su un'immagine rilasciata di LinuxCNC 2.8, ma potrebbe funzionare anche con versioni precedenti.

<a id="comandi-di-installazione"></a>

### Comandi di installazione
- compilare il componente:  
  Con il terminale aprire la cartella dove è salvato il file .c del componente e digitare:   
  ```bash
  sudo halcompile --install io_decoder.c
  ```

- definire porta USB:
1. Trova le informazioni sull’apparecchio USB  
    Collega il dispositivo e dai questo comando per trovare le sue caratteristiche:
    ```bash
    lsusb
    ```  
    Nella risposta al comando, con la scheda collegata all'usb, dovrebbe apparire una riga come questa:  
    **Bus 001 Device 008: ID 2341:8036 Arduino SA Leonardo (CDC ACM, HID)**    

2. Crea una regola udev  
    Aprire un terminale ed aprire la cartella delle regole con:  
    ```bash
    cd /etc/udev/rules.d/
    ```  
    Creare una regola per un device seriale USB:  
    ```bash
    sudo nano /etc/udev/rules.d/99-io_decoder.rules
    ```  
    Una volta eseguito, il terminale aprirà l'editor di testo nano. il numero 99 serve per dire al sistema di leggere     la regola per ultima, fra tutte quelle che ha. Il nome del file 99-io_decoder.rules è quello che ho scelto io; ma     questo può essere qualsiasi.  
    Incolla la seguente regola all'interno del file:  
    ```bash
    SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="8036", ATTRS{product}=="io_decoder", SYMLINK+="io_decoder", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"  
    ```

    Premi Ctrl + O per salvare (ti chiederà di confermare il nome del file e se non esiste questo verrà creato, premi     Invio).  
    Premi Ctrl + X per uscire dall'editor.  
    Così il tuo device sarà accessibile come /dev/io_decoder indipendentemente dalla porta fisica.  
    Se vuoi cambiare il nome del link simbolico dell'usb, modifica il valore scritto in SYMLINK+  
    >**ATTENZIONE**
    Con MODE="0666" vengono aperti i permessi di lettura/scrittura della USB a chiunque usi il sistema. Se vuoi tenere il sistema con delle sicurezze maggiori         devi creare, con i comandi di prompt, un gruppo che possa contenere la comunicazione con i permessi adeguati ed impostarlo nella stringa con                       GROUP="nome_tuo_gruppo".  

4. Ricarica le regole udev
    ```bash
    sudo udevadm control --reload-rules    
    sudo udevadm trigger
    ``` 

5. Verifica  
    Scollega e ricollega il dispositivo, poi verifica con:  
    ```bash
    ls -l /dev/io_decoder
    ```  
    Con queste impostazioni il valore di default dichiarato in fase di inizializzazione del componente è soddisfatto.
    Comunque il tutto è liberamente configurabile.  

[torna all'indice](#indice)
<a id="utilizzo"></a>
  
## Utilizzo
Ideato per realizzare/gestire un pannello operatore di macchine CNC.
Per l'utilizzo di questo sistema nel controllo dei motori e sensori vari del macchinario, e per la natura del sistema Linuxcnc, la comunicazione USB non è affidabile ed è altamente sconsigliata per questo scopo. Anche se il sistema di questo componente è stato testato in vari modi, per sicurezza, il pulsante di emergenza dovrebbe essere collegato direttamente ai pin fisici previsti dall'hardware di Linuxcnc.  
In caso di disconnessione software dell'USB o di rallentamenti da parte del sistema operativo, per colpa di PC datati o non performanti che producono valori alti di jitter dei thread, il sistema è stabile e reagisce alle varie situazioni, rendendo il componente sempre attivo.  

<a id="esempio-di-configurazione"></a>

### Esempio di configurazione
```bash
   loadrt io_decoder output=24 input=24 usb_port_name="/dev/io_decoder" verbose=3 firmware=101 keymap_file="io_decoder-keymap.cfg" uinput_chmod_cmd="chmod 0666 /dev/uinput"
```  

<a id="funzionalità-disponibili"></a>

### Funzionalità disponibili
- I pin UP e DOWN dell'encoder e dell'ADC permettono attraverso componenti HAL tipo UPDOWN o MULTISWITCH di selezionare dei valori, tipo selezione asse o            avanzamento, con un encoder rotativo incrementale o un joystick analogico.  
- Oltre i pin di HAL il componente genera dei messaggi di errore visibili sulla GUI di Linuxcnc, se abilitati in fase di inizializzazione del componente nel file    .hal della macchina, in caso di:  
  - timeout della comunicazione  
  - disconnessione dell'USB  
  - troppi errori nella comunicazione in caso di cavo danneggiato o disturbi elettrici/elettromagnetici  
  - ripristino della comunicazione USB (non è un errore ma funziona, per praticità, su configurazioni base come la mia)
- Se viene persa la connessione USB i valori che hal riceve dalla scheda USB sono congelati e pin di hal degli input sono forzati a 0; allo stesso tempo le uscite   fisiche sulla scheda USB sono disattivate.  

<a id="keyboard"></a>

### Keyboard
Funzionalità che permette di usare gli input fisici per inviare dei comandi di tastiera simulata che possono essere usati per stampare a video od inviare comandi come se fossero digitati sulla tastiera di sistema.  
Se il file [**"io_decoder-keymap.cfg"**](IOdecoder-keymap.cfg.md) (o quello che è specificato da **keymap_file**) è presente nella cartella della configurazione della macchina, la funzionalità è abilitata. Se non è presente oppure è vuoto o contiene solo commenti, la funzionalità non viene abilitata.  
  
**Configurare le corrispondenze input=>keyboard:**  
La fuzionalità utilizza UINPUT per inviare i comandi, per cui ogni codice numerico che invia indica la posizione sulla tastiera; non è un numero assoluto per ogni tipo di tastiera (US, UK, IT, FR, etc) ma è un codice che indica la posizione del tasto sulla tastiera installata nel sistema. Se viene inviato un codice 27 ed è installata una tastiera US il risultato a video sarà ] (parentesi quadra destra); su una tastiera IT si avrà un + (più).  
Può essere inviato anche un codice singolo o composto per creare combinazioni come:  
 - shift+a = A 
 - ctrl+x = taglia
 - alt_gr+à = @ (nella tastiera italiana)
 - alt+f = apertura menu a tendina dove è possibile
 - shift come tasto singolo
 - e tutte le altre combinazioni possibili
  
Da verificare per ogni installazione il risultato effettivo del codice inviato.  
    
**Formato**: in.*MM*-*N* <Keysym_string> #commento  
Consiglio di mettere le righe di configurazione con i numeri progressivi, per avere un quadro generale delle funzioni ordinato, ma non è obbligatorio.
  - in   = prefisso
  - *MM* = numero a due cifre per indicare la posizione della scheda di espansione.
  - *N*  = numero ad una cifra per indicare l'ingresso sulla scheda di espansione.
  - <Keysym_string> = codice da inviare alla tastiera virtuale.
  - *#* Tutto quello che è scritto dopo è commento e non viene considerato.
  
    Il codice da inviare può essere sia un numero che un prefisso + numero. Il prefisso serve per inviare il modificatore insieme al valore del tasto:  
    - s = shift  
    - c = control  
    - a = alt  
    - g = alt grafico      
  - codice su mappa tastiera:    
    - 30  -> a
    - s30 -> A
    - c44 -> control+z (taglia)
    - 42  -> left shift
    - s40 -> @ su tastiera UK
    - g39 -> @ su tastiera IT
  - esempi di righe di configurazione:
    - in.00-3 s20 #T	   -> all'input 00-3 è associato il codice della T (t maiuscola)
    - in.01-5 c46 #ctrl+c  -> all'input 01-5 è associata l'azione copia
    - in.00-1 18 #e	       -> all'input 00-1 è associato il codice della e (e minuscola)

  **Keycodes**  
  La [mappa della tastiera]({{ '/ISO_kb_(105)_QWERTY_UK_IT.png' | relative_url }}) ed i [keycodes]({{ '/keycodes_list' | relative_url }}) li ho ricavati in vari modi attraverso ricerche in rete e prove varie; sicuramente non è precisa al 100%. La mappa serve per capire che valore invia al sistema il tasto sulla tua tastiera che poi in base a come è configurata restituirà a video il simbolo associato. La funzionalità keyboard invia il valore della posizione del tasto sulla tastiera non il simbolo associato.  

  >## Attenzione!!!  
  >**questa funzionalità, di default il parametro è uinput_chmod_cmd="chmod 0666 /dev/uinput", prevede che vengano dati dei permessi di lettura e scrittura su UINPUT a tutti gli utenti, con il comando chmod richiamato automaticamente dal componente stesso. Se vuoi settare il sistema con i tuoi permessi personalizzati devi lavorare con i comandi di prompt per fare la tua configurazione dei permessi. Con il parametro uinput_chmod_cmd passato nel file .hal di configurazione della macchina con valore uinput_chmod_cmd="" il comando chmod del componente non viene eseguito ed il sistema deve già essere configurato con i permessi corretti per usare la funzionalità tastiera. Con uinput_chmod_cmd può essere inserito il comando personalizzato per dare i permessi adeguati alle proprie esigenze.**
  **Se il file che viene richiamato dal parametro keymap_file non esiste, è vuoto oppure non ha pin validi, e quindi non viene usata la funzionalità di tastiera simulata, non viene inizializzato UINPUT e quindi non vengono impostati i permessi dell'utente per questa funzionalità.**
  **I permessi di default sono temporanei per il periodo in cui il componente è funzionante.**  

<a id="dipendenze"></a>
[torna all'indice](#indice)  
  
## Dipendenze
Realizzato su LinuxCNC 2.8 e non ci dovrebbe essere dipendenze aggiuntive.

<a id="autori"></a>
[torna all'indice](#indice)  
  
## Autori
Roberto "bobwolf" Sassoli ed il suo gemello virtuale.

<a id="licenza"></a>
[torna all'indice](#indice)  
  
## Licenza

Questo software è distribuito sotto licenza GNU General Public License, versione 2 (GPLv2).  
This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.  

**Disclaimer:**  
Questo software viene fornito “così com’è”, senza alcuna garanzia.  
L’autore non è responsabile per danni derivanti dall’uso del programma.  

Copyright (c) 2026 [bobwolf]

[🔝 torna all'indice](#indice) | [📖 README](../README.md) | [🏠 Project Home]({{ '/index.it' | relative_url }})  
[🚀 Demo/Eval mode in Italiano]({{ '/demo_mode.it' | relative_url }})  
  

>
> <hr style="margin-top: 50px; border: 0; border-top: 1px solid #eee;">
<footer style="padding: 20px 0; text-align: center; color: #666; font-size: 0.9em;">
  <p><strong>io_decoder</strong> - Driver Open Source per LinuxCNC</p>
  <p>
    <a href="mailto:io.decoder.rst%40gmail.com" style="color: #1e6bb8; text-decoration: none;">✉️ contatto</a> | 
    <a href="https://github.com/bobwolfrst/io_decoder-linuxCNC" style="color: #1e6bb8; text-decoration: none;">💻 GitHub Repository</a>
  </p>
  <p style="font-size: 0.8em;">© 2026 - Creato da bobwolfrst. Rilasciato sotto licenza GPL.</p>
</footer>

