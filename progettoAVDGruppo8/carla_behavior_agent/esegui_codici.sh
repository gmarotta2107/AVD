#!/bin/bash

# Esegui il primo comando in background
nohup ./run_test.sh > output.log 2>&1 &

# Esegui il secondo comando in background
nohup python3 server_http.py > output2.log 2>&1 &

# Notifica che i comandi sono stati eseguiti
echo "Comandi eseguiti in background. Output salvato in output.log e output2.log"