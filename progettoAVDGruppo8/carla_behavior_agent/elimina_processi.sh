#!/bin/bash

# Trova i processi che contengono "run_test.sh" o "python3" nel comando e li termina
ps aux | grep -E 'run_test.sh|python3' | grep -v 'grep' | awk '{print $2}' | xargs kill -9

# Notifica che i processi sono stati terminati
echo "Tutti i processi con 'run_test.sh' o 'python3' sono stati terminati."