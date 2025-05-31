#!/bin/bash
# Setup script for the lbx_robotics Conda environment

# --- Configuration ---
ENV_NAME="lbx_robotics_env"
ENV_FILE="environment.yaml"
REQ_FILE="requirements.txt"
# Get the directory of this script to ensure relative paths work
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"

# --- Colors for Output ---
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# --- Helper Functions ---
echo_error() {
    echo -e "${RED}${BOLD}ERROR:${NC}${RED} $1${NC}" >&2
}

echo_success() {
    echo -e "${GREEN}${BOLD}✅ SUCCESS:${NC}${GREEN} $1${NC}"
}

echo_info() {
    echo -e "${BLUE}INFO:${NC} $1"
}

echo_warn() {
    echo -e "${YELLOW}${BOLD}WARNING:${NC}${YELLOW} $1${NC}"
}

# --- Pre-flight Checks ---
echo_info "Starting environment setup for '$ENV_NAME'..."

if ! command -v conda &> /dev/null; then
    echo_error "Conda is not installed or not in your PATH. Please install Miniconda or Anaconda."
    exit 1
fi
echo_success "Conda found."

if [ ! -f "$SCRIPT_DIR/$ENV_FILE" ]; then
    echo_error "Environment file '$ENV_FILE' not found in $SCRIPT_DIR/"
    exit 1
fi

if [ ! -f "$SCRIPT_DIR/$REQ_FILE" ]; then
    echo_warn "Requirements file '$REQ_FILE' not found in $SCRIPT_DIR/. Pip installation step will be skipped if environment is created from scratch."
fi

# --- Create or Update Conda Environment ---
if conda env list | grep -q "^$ENV_NAME\s"; then
    echo_info "Conda environment '$ENV_NAME' already exists."
    # Optionally, add logic here to ask if the user wants to update or remove & recreate.
    # For now, we'll proceed to ensure pip packages are installed/updated.
else
    echo_info "Conda environment '$ENV_NAME' not found. Creating it from $ENV_FILE..."
    if conda env create -f "$SCRIPT_DIR/$ENV_FILE" -n "$ENV_NAME"; then
        echo_success "Conda environment '$ENV_NAME' created successfully."
    else
        echo_error "Failed to create Conda environment '$ENV_NAME' from $ENV_FILE."
        exit 1
    fi
fi

# --- Install Pip Dependencies --- 
echo_info "Installing/updating pip packages from $REQ_FILE into '$ENV_NAME' environment..."
if [ -f "$SCRIPT_DIR/$REQ_FILE" ]; then
    # Use conda run to execute pip install within the target environment
    # --no-capture-output and --live-stream allow seeing pip's output directly
    if conda run -n "$ENV_NAME" --no-capture-output --live-stream pip install -r "$SCRIPT_DIR/$REQ_FILE"; then
        echo_success "Pip packages installed/updated successfully from $REQ_FILE."
    else
        echo_error "Failed to install/update pip packages from $REQ_FILE."
        echo_info "You might need to activate the environment (\`conda activate $ENV_NAME\`) and run \`pip install -r $REQ_FILE\` manually to troubleshoot."
        exit 1 # Exit if pip install fails, as it might be critical
    fi
else
    echo_info "Pip requirements file '$REQ_FILE' not found. Skipping pip installation."
fi

# --- Final Instructions / Activation ---
echo ""
echo_success "-------------------------------------------------------------"
echo_success " Environment setup complete for '$ENV_NAME'! "
echo_success "-------------------------------------------------------------"
echo ""
echo_info "To use the environment in future sessions, run:"
echo_info "  ${CYAN}conda activate $ENV_NAME${NC}"
echo ""
echo_info "Attempting to activate the environment and start a new interactive shell session..."

# Try to ensure conda is initialized for the current shell and then activate & exec.
# This makes the script more robust when called in various ways.
_CONDA_ACTIVATE_SCRIPT="$(conda info --base)/etc/profile.d/conda.sh"
if [ -f "$_CONDA_ACTIVATE_SCRIPT" ]; then
    # Source conda.sh to make `conda activate` available in the current script's subshell
    # This is often more reliable than `eval $(conda shell.bash hook)` for script execution context
    source "$_CONDA_ACTIVATE_SCRIPT"
    conda activate "$ENV_NAME"
    if [ "$CONDA_DEFAULT_ENV" == "$ENV_NAME" ]; then
        echo_success "Environment '$ENV_NAME' is now active."
        echo_info "Starting a new shell session within the environment..."
        exec "$SHELL" -l # Start a login shell to ensure .bashrc/.zshrc etc. are sourced if they modify prompt
    else
        echo_error "Failed to automatically activate Conda environment '$ENV_NAME'."
        echo_info "Please activate it manually: ${CYAN}conda activate $ENV_NAME${NC}"
        exit 1
    fi
else
    echo_error "Conda activation script not found at $_CONDA_ACTIVATE_SCRIPT."
    echo_info "Please ensure Conda is properly installed and initialized for your shell."
    echo_info "Then, activate the environment manually: ${CYAN}conda activate $ENV_NAME${NC}"
    exit 1
fi

# The script should ideally not reach here if exec was successful
echo_error "If you see this message, automatic shell replacement might not have fully worked."
echo_info "The environment '$ENV_NAME' should be active. If not, please activate manually: ${CYAN}conda activate $ENV_NAME${NC}" 