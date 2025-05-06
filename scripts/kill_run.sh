#!/bin/bash

# Find the process running run.sh (excluding the grep itself)
PROCESS=$(ps -eo pid,pgid,cmd | grep './[r]un.sh')

if [ -n "$PROCESS" ]; then
  echo "Found run.sh process:"
  echo "$PROCESS"
  
  # Extract PID (2nd column)
  PID=$(echo "$PROCESS" | awk '{print $2}')
  echo "Killing process with PID: $PID"
  kill -SIGINT -"$PID"

  if [ $? -eq 0 ]; then
    echo "SIGINT sent successfully."
  else
    echo "Failed to send SIGINT."
  fi
else
  echo "No run.sh process found."
fi
