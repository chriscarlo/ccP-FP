#!/bin/bash

set -x

# Configuration
DEVICE_IP="192.168.1.89"
DEVICE_USER="comma"
BRANCH_NAME="Chubbs"

# Confirmation function - Returns true for 'y' or 'Y', false otherwise
confirm() {
    read -p "$1 (y/n): " -n 1 -r
    echo
    if [[ "$REPLY" =~ ^[Yy]$ ]]; then
        return 0
    else
        return 1
    fi
}

# Function to check if prebuilt exists
check_prebuilt() {
    # Remove any existing host key to prevent verification issues
    ssh-keygen -R "$DEVICE_IP" > /dev/null 2>&1

    # Check for the 'prebuilt' file
    PREBUILT_FILE=$(ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null "$DEVICE_USER@$DEVICE_IP" 'test -f /data/openpilot/prebuilt && echo "yes" || echo "no"')

    # Additionally, check if any .so files exist as an indicator
    ESSENTIAL_SO=$(ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null "$DEVICE_USER@$DEVICE_IP" 'find /data/openpilot -type f -name "*.so" | grep -q . && echo "yes" || echo "no"')

    if [[ "$PREBUILT_FILE" == "yes" || "$ESSENTIAL_SO" == "yes" ]]; then
        echo "yes"
    else
        echo "no"
    fi
}

# Function to perform compilation
compile_device() {
    echo "Performing device compilation..."

    echo "Removing old SSH keys..."
    ssh-keygen -R "$DEVICE_IP" > /dev/null 2>&1

    echo "Connecting to device at $DEVICE_IP..."
    OUTPUT=$(ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null "$DEVICE_USER@$DEVICE_IP" << 'ENDSSH'
        set +x  # Disable debug mode for cleaner output
        set -e
        cd /data/openpilot

        # Setup environment
        export PATH="/data/data/com.termux/files/usr/bin:$PATH"
        export LD_LIBRARY_PATH="/usr/lib/aarch64-linux-gnu:/usr/local/lib:$LD_LIBRARY_PATH"

        # Clean and compile
        echo "Cleaning previous compilation..."
        if ! scons -c; then
            echo "ERROR: Clean failed"
            exit 1
        fi

        echo "Starting compilation..."
        if ! scons -j$(nproc); then
            echo "ERROR: Compilation failed"
            exit 1
        fi

        # Wait for compilation to complete
        while pgrep -f "scons" > /dev/null; do
            echo "Waiting for compilation to finish..."
            sleep 5
        done

        # Final check if compilation succeeded
        if ! scons --check; then
            echo "ERROR: Compilation failed during final check"
            exit 1
        fi

        echo "Compilation completed successfully"

        # Add compiled files
        git add -f .
        git commit -m "compile" --allow-empty

        # Delete all .cc files except specified ones
        find . -type f -name '*.cc' ! \( -name 'main.cc' -o -name 'map_renderer.cc' -o -name 'transform.cc' \) -delete

        # Delete all .h files except specified ones
        find . -type f -name '*.h' ! \( -name 'sensor.h' -o -name 'version.h' -o -name 'map_renderer.h' -o -name 'ox03c10_registers.h' -o -name 'ar0231_cl.h' -o -name 'os04c10_cl.h' -o -name 'ox03c10_c.h' \) -delete

        # Remove other unwanted file types
        find . -name '*.o' -delete
        find . -name '*.os' -delete
        find . -name '*.pyc' -delete
        find . -name 'moc_*' -delete
        find . -name '__pycache__' -delete
        find . -name '*.a' -delete

        # Remove build system artifacts
        rm -rf .sconsign.dblite Jenkinsfile release/

        # Remove generated directories
        rm -rf selfdrive/ubloxd/generated cereal/gen/cpp

        # Remove remaining MPC-generated files
        rm -rf selfdrive/controls/lib/longitudinal_mpc_lib/c_generated_code/main_*
        rm -rf selfdrive/controls/lib/lateral_mpc_lib/c_generated_code/main_*

        # Additional Deletions
        # Remove specific directories except .github/workflows and delete body/
        find .github -mindepth 1 ! -path './.github/workflows*' -delete
        rm -rf .vscode .devcontainer debug \
               tools/cabana tools/camerastream tools/car_porting tools/joystick \
               tools/latencylogger tools/lib/tests tools/plotjuggler tools/profiling \
               tools/replay tools/scripts tools/serial tools/sim tools/ssh \
               tools/ubuntu_setup tools/webcam body/

        # Remove Docker-related files
        rm -f Dockerfile* .dockerignore

        # Remove test files and directories
        find . -type f -iname '*test*' -delete
        find . -type d -iname '*test*' -exec rm -rf {} +

        # Remove all files named LICENSE (case-insensitive)
        find . -type f -iname 'license*' -delete

        # Remove teleoprtc files
        find . -type f -iname '*teleoprtc*' -delete

        # Remove configuration files
        rm -f .clang-tidy .editorconfig

        # Remove debug-related files and directories
        find . -type f -iname '*debug*' -delete
        find . -type d -iname '*debug*' -exec rm -rf {} +

        touch prebuilt

        # Amend with cleaned state
        git add -f .
        git commit --amend -m "Prebuilt" --allow-empty

        # Output hash
        echo "HASH=$(git rev-parse HEAD)"
ENDSSH
    )
    # Check SSH exit status
    SSH_STATUS=$?
    if [ $SSH_STATUS -ne 0 ]; then
        echo "ERROR: Compilation SSH session failed with status $SSH_STATUS" >&2
        echo "Output was:" >&2
        echo "$OUTPUT" >&2
        exit 1
    fi

    # Extract hash
    COMMIT_HASH=$(echo "$OUTPUT" | grep "^HASH=" | cut -d= -f2)

    if ! [[ $COMMIT_HASH =~ ^[0-9a-f]{40}$ ]]; then
        echo "ERROR: Invalid commit hash: $COMMIT_HASH" >&2
        echo "Full output was:" >&2
        echo "$OUTPUT" >&2
        exit 1
    fi

    echo "$COMMIT_HASH"
}

# Function to skip compilation and clean
skip_compile_device() {
    echo "Proceeding without recompilation..." >&2  # Redirect to stderr
    # Disable set -x for this function to prevent debug logs from polluting OUTPUT
    set +x
    OUTPUT=$(ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null "$DEVICE_USER@$DEVICE_IP" << 'ENDSSH'
        set +x  # Disable debug mode
        set -e
        cd /data/openpilot

        # Add compiled files
        git add -f .
        git commit -m "compile" --allow-empty

        # Delete all .cc files except specified ones
        find . -type f -name '*.cc' ! \( -name 'main.cc' -o -name 'map_renderer.cc' -o -name 'transform.cc' \) -delete

        # Delete all .h files except specified ones
        find . -type f -name '*.h' ! \( -name 'sensor.h' -o -name 'version.h' -o -name 'map_renderer.h' -o -name 'ox03c10_registers.h' -o -name 'ar0231_cl.h' -o -name 'os04c10_cl.h' -o -name 'ox03c10_c.h' \) -delete

        # Remove other unwanted file types
        find . -name '*.o' -delete
        find . -name '*.os' -delete
        find . -name '*.pyc' -delete
        find . -name 'moc_*' -delete
        find . -name '__pycache__' -delete
        find . -name '*.a' -delete

        # Remove build system artifacts
        rm -rf .sconsign.dblite Jenkinsfile release/

        # Remove generated directories
        rm -rf selfdrive/ubloxd/generated cereal/gen/cpp

        # Remove remaining MPC-generated files
        rm -rf selfdrive/controls/lib/longitudinal_mpc_lib/c_generated_code/main_*
        rm -rf selfdrive/controls/lib/lateral_mpc_lib/c_generated_code/main_*

        # Additional Deletions
        # Remove specific directories except .github/workflows and delete body/
        find .github -mindepth 1 ! -path './.github/workflows*' -delete
        rm -rf .vscode .devcontainer debug \
               tools/cabana tools/camerastream tools/car_porting tools/joystick \
               tools/latencylogger tools/lib/tests tools/plotjuggler tools/profiling \
               tools/replay tools/scripts tools/serial tools/sim tools/ssh \
               tools/ubuntu_setup tools/webcam body/

        # Remove Docker-related files
        rm -f Dockerfile* .dockerignore

        # Remove test files and directories
        find . -type f -iname '*test*' -delete
        find . -type d -iname '*test*' -exec rm -rf {} +

        # Remove all files named LICENSE (case-insensitive)
        find . -type f -iname 'license*' -delete

        # Remove teleoprtc files
        find . -type f -iname '*teleoprtc*' -delete

        # Remove configuration files
        rm -f .clang-tidy .editorconfig

        # Remove debug-related files and directories
        find . -type f -iname '*debug*' -delete
        find . -type d -iname '*debug*' -exec rm -rf {} +

        touch prebuilt

        # Amend with cleaned state
        git add -f .
        git commit --amend -m "Prebuilt" --allow-empty

        # Output hash
        echo "HASH=$(git rev-parse HEAD)"
ENDSSH
    )
    set -x  # Re-enable debug mode

    SSH_STATUS=$?
    if [ $SSH_STATUS -ne 0 ]; then
        echo "ERROR: Commit and clean SSH session failed with status $SSH_STATUS" >&2
        echo "Output was:" >&2
        echo "$OUTPUT" >&2
        exit 1
    fi

    # Extract hash
    COMMIT_HASH=$(echo "$OUTPUT" | grep "^HASH=" | cut -d= -f2)

    if ! [[ $COMMIT_HASH =~ ^[0-9a-f]{40}$ ]]; then
        echo "ERROR: Invalid commit hash: $COMMIT_HASH" >&2
        echo "Full output was:" >&2
        echo "$OUTPUT" >&2
        exit 1
    fi

    echo "$COMMIT_HASH"
}

# Function to handle workspace operations
workspace_operations() {
    local device_commit=$1
    echo "=== Starting workspace operations ==="
    export GIT_SSH_COMMAND="ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null"

    if ! git remote | grep -q "device"; then
        git remote add device ssh://"${DEVICE_USER}"@"${DEVICE_IP}":/data/openpilot
    fi

    git fetch device "$BRANCH_NAME" --force

    git checkout "$BRANCH_NAME" || { echo "ERROR: Failed to checkout branch '$BRANCH_NAME'"; exit 1; }
    git cherry-pick --no-commit --strategy=recursive -X theirs "$device_commit" || { echo "ERROR: Cherry-pick failed"; exit 1; }
    git add -f .
    git commit -C "$device_commit" || { echo "ERROR: Commit failed"; exit 1; }

    # Force push to origin
    git push --force origin "$BRANCH_NAME" || { echo "ERROR: Force push to origin failed"; exit 1; }
}

# Main execution
echo "Starting automated sync process..."
confirm "Begin sync process?"
if [[ $? -ne 0 ]]; then
    echo "Operation cancelled by user"
    exit 1
fi

PREBUILT=$(check_prebuilt)

if [[ "$PREBUILT" == "yes" ]]; then
    # Automatically skip recompilation without user confirmation
    COMMIT_HASH=$(skip_compile_device) || exit 1
else
    # Perform compilation if not prebuilt
    COMMIT_HASH=$(compile_device) || exit 1
fi

workspace_operations "$COMMIT_HASH"
echo "Sync process completed."
