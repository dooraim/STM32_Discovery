# STM32_Discovery
Programs for the STM32, in C

Testo del progetto
IL programma esegue le seguenti operazioni:
  ❖ Campiona l'accelerometro a 100Hz;
  ❖ Implementa un filtro FIR passa basso per filtrare ciascun asse dell'accelerometro a 5Hz;
  ❖ Implementa il calcolo dell'orientamento planare dai dati accelerometrici filtrati;
  ❖ Implementa il calcolo della media sui dati di orientamento su una finestra di 5 campioni;
  ❖ Utilizzando la media sui dati di orientamento, accende solo il LED orientato a terra. Altri devono essere spenti;
Implementare la seguente interfaccia utente:
  ❖ All'avvio inizializzare tutto, accendere il LED verde (per USB è già acceso quando è collegato un cavo) e attendere l'input dell'utente;
  ❖ Dovrebbero essere supportati i seguenti comandi:
    o "s": avvia / interrompe lo streaming;
    o "d": attiva o disattiva lo streaming dei dati o lo streaming dei risultati;
  ❖ Quando non è in streaming, i LED devono essere spenti. Streaming dati:
  ❖ Inviare i dati dell'accelerometro (in g o mg) a 5Hz.
  ❖ Formato di esempio: X: 0000 Y: 0000 Z: 0000 (puoi anche trasmettere in streaming solo numeri in CSV) Streaming dei risultati:
  ❖ Invia media calcolata sui dati di orientamento (in gradi) a 20Hz.
  ❖ Formato di esempio: roll: passo 0000: 0000 (puoi anche trasmettere in streaming solo numeri in CSV)
