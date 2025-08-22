#!/bin/bash

# CARLA XML-RPC Server Startup Script (Fixed)
set -e

DEFAULT_HOST="localhost"
DEFAULT_PORT="8090"
DEFAULT_CARLA_HOST="localhost"
DEFAULT_CARLA_PORT="2000"
DEFAULT_LOG_LEVEL="INFO"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_info()    { echo -e "${BLUE}[INFO]${NC} $1"; }
print_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
print_error()   { echo -e "${RED}[ERROR]${NC} $1"; }

HOST="$DEFAULT_HOST"
PORT="$DEFAULT_PORT"
CARLA_HOST="$DEFAULT_CARLA_HOST"
CARLA_PORT="$DEFAULT_CARLA_PORT"
LOG_LEVEL="$DEFAULT_LOG_LEVEL"

while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--host)        HOST="$2"; shift 2 ;;
        -p|--port)        PORT="$2"; shift 2 ;;
        -c|--carla-host)  CARLA_HOST="$2"; shift 2 ;;
        -P|--carla-port)  CARLA_PORT="$2"; shift 2 ;;
        -l|--log-level)   LOG_LEVEL="$2"; shift 2 ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo "  -h, --host HOST        XML-RPC server host (default: $DEFAULT_HOST)"
            echo "  -p, --port PORT        XML-RPC server port (default: $DEFAULT_PORT)"
            echo "  -c, --carla-host HOST  CARLA server host (default: $DEFAULT_CARLA_HOST)"
            echo "  -P, --carla-port PORT  CARLA server port (default: $DEFAULT_CARLA_PORT)"
            echo "  -l, --log-level LEVEL  Log level (DEBUG, INFO, WARNING, ERROR)"
            exit 0
            ;;
        *) print_error "Unknown option: $1"; exit 1 ;;
    esac
done

# Validate log level
case "$LOG_LEVEL" in DEBUG|INFO|WARNING|ERROR) ;; 
    *) print_error "Invalid log level"; exit 1 ;; 
esac

SERVER_SCRIPT="carla_xmlrpc_server.py"
if [[ ! -f "$SERVER_SCRIPT" ]]; then
    print_error "Cannot find $SERVER_SCRIPT (run from bridge directory)"
    exit 1
fi

print_info "Starting CARLA XML-RPC Server:"
echo "  XML-RPC Server: $HOST:$PORT"
echo "  CARLA Server:  $CARLA_HOST:$CARLA_PORT"
echo "  Log Level:     $LOG_LEVEL"
echo ""

export CARLA_XMLRPC_LOG_LEVEL="$LOG_LEVEL"

# Map log level to --debug flag
DEBUG_FLAG=""
if [[ "$LOG_LEVEL" == "DEBUG" ]]; then
    DEBUG_FLAG="--debug"
fi

python3 "$SERVER_SCRIPT" \
    --host "$HOST" \
    --port "$PORT" \
    --carla-host "$CARLA_HOST" \
    --carla-port "$CARLA_PORT" \
    $DEBUG_FLAG

if [[ $? -eq 0 ]]; then
    print_success "Server stopped successfully"
else
    print_error "Server exited with error code $?"
    exit 1
fi
