#!/bin/bash

# CARLA XML-RPC Server Startup Script
# This script starts the CARLA XML-RPC server with proper configuration

set -e

# Default configuration
DEFAULT_HOST="localhost"
DEFAULT_PORT="8000"
DEFAULT_CARLA_HOST="localhost"
DEFAULT_CARLA_PORT="2000"
DEFAULT_TIMEOUT="10.0"
DEFAULT_LOG_LEVEL="INFO"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to show usage
show_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -h, --host HOST          XML-RPC server host (default: $DEFAULT_HOST)"
    echo "  -p, --port PORT          XML-RPC server port (default: $DEFAULT_PORT)"
    echo "  -c, --carla-host HOST    CARLA server host (default: $DEFAULT_CARLA_HOST)"
    echo "  -P, --carla-port PORT    CARLA server port (default: $DEFAULT_CARLA_PORT)"
    echo "  -t, --timeout TIMEOUT    Connection timeout in seconds (default: $DEFAULT_TIMEOUT)"
    echo "  -l, --log-level LEVEL    Log level (DEBUG, INFO, WARNING, ERROR) (default: $DEFAULT_LOG_LEVEL)"
    echo "  --help                   Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                                    # Use default settings"
    echo "  $0 -h 0.0.0.0 -p 8000                # Bind to all interfaces on port 8000"
    echo "  $0 -c 192.168.1.100 -P 2000          # Connect to remote CARLA server"
    echo "  $0 -l DEBUG                           # Enable debug logging"
}

# Parse command line arguments
HOST="$DEFAULT_HOST"
PORT="$DEFAULT_PORT"
CARLA_HOST="$DEFAULT_CARLA_HOST"
CARLA_PORT="$DEFAULT_CARLA_PORT"
TIMEOUT="$DEFAULT_TIMEOUT"
LOG_LEVEL="$DEFAULT_LOG_LEVEL"

while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--host)
            HOST="$2"
            shift 2
            ;;
        -p|--port)
            PORT="$2"
            shift 2
            ;;
        -c|--carla-host)
            CARLA_HOST="$2"
            shift 2
            ;;
        -P|--carla-port)
            CARLA_PORT="$2"
            shift 2
            ;;
        -t|--timeout)
            TIMEOUT="$2"
            shift 2
            ;;
        -l|--log-level)
            LOG_LEVEL="$2"
            shift 2
            ;;
        --help)
            show_usage
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            show_usage
            exit 1
            ;;
    esac
done

# Validate log level
case "$LOG_LEVEL" in
    DEBUG|INFO|WARNING|ERROR)
        ;;
    *)
        print_error "Invalid log level: $LOG_LEVEL"
        print_error "Valid levels: DEBUG, INFO, WARNING, ERROR"
        exit 1
        ;;
esac

# Check if Python is available
if ! command -v python3 &> /dev/null; then
    print_error "Python 3 is not installed or not in PATH"
    exit 1
fi

# Check if the server script exists
SERVER_SCRIPT="carla_xmlrpc_server.py"
if [[ ! -f "$SERVER_SCRIPT" ]]; then
    print_error "Server script not found: $SERVER_SCRIPT"
    print_error "Please run this script from the bridge directory"
    exit 1
fi

# Check if CARLA Python API is available
PYTHON_PATH="PythonAPI/carla"
if [[ ! -d "$PYTHON_PATH" ]]; then
    print_warning "CARLA Python API not found in $PYTHON_PATH"
    print_warning "Make sure CARLA is properly installed and the Python API is available"
fi

# Display configuration
print_info "Starting CARLA XML-RPC Server with configuration:"
echo "  XML-RPC Server: $HOST:$PORT"
echo "  CARLA Server: $CARLA_HOST:$CARLA_PORT"
echo "  Timeout: ${TIMEOUT}s"
echo "  Log Level: $LOG_LEVEL"
echo ""

# Check if port is already in use
if command -v netstat &> /dev/null; then
    if netstat -tuln 2>/dev/null | grep -q ":$PORT "; then
        print_warning "Port $PORT is already in use"
        read -p "Do you want to continue anyway? (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            print_info "Aborted by user"
            exit 0
        fi
    fi
fi

# Set environment variables
export PYTHONPATH="$PYTHON_PATH:$PYTHONPATH"
export CARLA_XMLRPC_LOG_LEVEL="$LOG_LEVEL"

# Start the server
print_info "Starting server..."
print_info "Press Ctrl+C to stop the server"

python3 "$SERVER_SCRIPT" \
    --host "$HOST" \
    --port "$PORT" \
    --carla-host "$CARLA_HOST" \
    --carla-port "$CARLA_PORT" \
    --timeout "$TIMEOUT" \
    --log-level "$LOG_LEVEL"

# Check exit status
if [[ $? -eq 0 ]]; then
    print_success "Server stopped successfully"
else
    print_error "Server exited with error code $?"
    exit 1
fi
