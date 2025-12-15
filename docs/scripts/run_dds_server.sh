i#!/bin/bash
set -e

IP="192.168.186.3"
PORT="11811"

echo "Starting FastDDS server..."

if ! command -v fastdds &> /dev/null; then
    echo "❌ fastdds not installed"
    exit 1
fi

fastdds discovery -i 0 -l $IP -p $PORT
RET=$?

if [ $RET -ne 0 ]; then
    echo "❌ FastDDS failed (exit: $RET)"
else
    echo "✔ FastDDS server running"
fi

