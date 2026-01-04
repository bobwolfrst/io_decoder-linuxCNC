# IO_DECODER

## Indice


- [**Caratteristiche**](#Caratteristiche)  
- [**Pinout**](#Pinout)  
- [**Utilizzo**](#Utilizzo)  
- [**Dove acquistarlo**](#Dove-acquistarlo)  
- [**Autori**](#Autori)  

[BACK TO README.it](README.it.md)

---

## Caratteristiche

Il sistema si compone di una scheda principale che ha a bordo il microprocessore con il collegamento USB e le periferiche previste dal firmware inserito. Inoltre ha i bus di collegamento alle espansioni di input ed output. Queste schede di espansione sono le medesime sia per l'input che l'output e sono liberamente scambiabili fra i due bus. Sul pcb della scheda principale è indicato il connettore del bus lato INPUT e lato OUTPUT. Nelle sezioni Pinout ed Utilizzo ci sono i diagrammi che illustrano l'installazione e la modalità di collegamento elettrico. 

[torna all'indice](#Indice) 
## Pinout

### Scheda io_decoder
#### Caratteristiche dipendenti da hadware e firmware della scheda USB
  - **Firmware 101**
    - Encoder in quadratura: 4 @5Vdc
    - DAC: 2 @8bit 5Vdc
    - ADC: 3 @10bit 5Vdc  
![schemi_io_decoder_1_master](git_USB_pannello_IO_4.png)  

### Scheda io_decoder usata in input
  - **Input digitali**: 8-128 (16 espansioni) liberamente configurabili con schede di espansione da 8 pin ognuna. Viene accettato come ingresso solamente contatti puliti tra il pin comune e l'ingresso digitale.  
![schemi_io_decoder_input_side](git_input_pannello_espansione_4.png)  

### Scheda io_decoder usata in output
  - **Output digitali**: 8-128 (16 espansioni) liberamente configurabili con schede di espansione da 8 pin ognuna. Ogni uscita può pilotare un carico da 50mA@30Vdc con un massimo di 300mA per scheda di espansione.  
![schemi_io_decoder_output_side](git_output_pannello_espansione_4.png)  

[torna all'indice](#Indice) 
## Utilizzo

![schemi_io_decoder_panoramica](git_assieme_usb_4.png)

[torna all'indice](#Indice) 
## Dove acquistarlo
Il sistema è ancora in fase di prototipo

[torna all'indice](#Indice) 
## Autori

Roberto "bobwolf" Sassoli

[BACK TO README.it](README.it.md)  
[BACK TO README](../README.md)


Copyright (c) 2025 [bobwolf]

