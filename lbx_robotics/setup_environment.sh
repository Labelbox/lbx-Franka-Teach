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
NC='\033[0m' # No Color

# --- Helper Functions ---
echo_error() {
    echo -e "${RED}ERROR: $1${NC}" >&2
}

echo_success() {
    echo -e "${GREEN}SUCCESS: $1${NC}"
}

echo_info() {
    echo -e "${BLUE}INFO: $1${NC}"
}

echo_warn() {
    echo -e "${YELLOW}WARNING: $1${NC}"
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
        echo_success "Pip packages installed/updated successfully."
    else
        echo_error "Failed to install/update pip packages from $REQ_FILE."
        echo_info "You might need to activate the environment (\`conda activate $ENV_NAME\`) and run \`pip install -r $REQ_FILE\` manually to troubleshoot."
        exit 1 # Exit if pip install fails, as it might be critical
    fi
else
    echo_info "Pip requirements file '$REQ_FILE' not found. Skipping pip installation."
fi

# --- Final Instructions / Activation ---
echo_success "Environment setup complete for '$ENV_NAME'."
echo_info "To activate the environment, run: conda activate $ENV_NAME"
echo_info "Attempting to activate the environment and start a new shell session now..."

# Try to activate and start a new shell. This is the most direct way to get the user "into" the env.
# This works best if the script is run directly like: ./setup_environment.sh
if [ -z "$_CONDA_SET_UP" ]; then # Check if conda init has been run in this shell
    # Attempt to initialize conda for the current shell if `conda activate` might fail
    # This is a more robust way to ensure `conda activate` works in various shell startup states
    CURRENT_SHELL=$(basename "$SHELL")
    echo_info "Attempting to initialize Conda for current shell ($CURRENT_SHELL)..."
    eval "$(conda shell.$CURRENT_SHELL hook 2> /dev/null)" || echo_warn "Failed to initialize conda for $CURRENT_SHELL. Manual activation might be needed."
    export _CONDA_SET_UP=true # Mark that we've tried to set up conda hooks
fi

# Now try activating and exec-ing into a new shell
conda activate "$ENV_NAME"
if [ $? -eq 0 ]; then
    echo_info "Successfully activated '$ENV_NAME'. Starting new shell session..."
    exec "$SHELL"
else
    echo_error "Failed to activate Conda environment '$ENV_NAME' automatically."
    echo_info "Please activate the environment manually: conda activate $ENV_NAME"
    exit 1 # Exit with error if activation fails, as subsequent steps in other scripts might depend on it
fi

# The script should not reach here if exec "$SHELL" was successful
echo_error "If you see this message, automatic shell replacement failed after activation."
echo_info "The environment '$ENV_NAME' should be active. If not, please activate it manually: conda activate $ENV_NAME" 