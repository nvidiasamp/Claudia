#!/bin/bash
# Claudia Project CycloneDDS Environment Configuration Script
# Resolves "undefined symbol: ddsi_sertype_v0" compatibility issue
# Auto-generated: 2025-06-27
# Latest version!

echo "Configuring Unitree CycloneDDS environment..."

# Check if CycloneDDS installation exists
CYCLONEDDS_INSTALL_PATH="$HOME/cyclonedds/install"
if [ ! -d "$CYCLONEDDS_INSTALL_PATH" ]; then
    echo "Error: CycloneDDS not found at $CYCLONEDDS_INSTALL_PATH"
    echo "Please compile and install CycloneDDS 0.10.x first"
    echo "Reference: https://github.com/eclipse-cyclonedds/cyclonedds"
    exit 1
fi

# Set CycloneDDS environment variables
export CYCLONEDDS_HOME="$CYCLONEDDS_INSTALL_PATH"
export LD_LIBRARY_PATH="$CYCLONEDDS_INSTALL_PATH/lib:$LD_LIBRARY_PATH"

# Set ROS2 DDS implementation to CycloneDDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Verify core library file exists
LIBDDSC_PATH="$CYCLONEDDS_INSTALL_PATH/lib/libddsc.so"
if [ ! -f "$LIBDDSC_PATH" ]; then
    echo "Warning: Core library file does not exist: $LIBDDSC_PATH"
    echo "CycloneDDS may need to be recompiled"
fi

echo "CycloneDDS environment configuration complete"
echo "   CYCLONEDDS_HOME: $CYCLONEDDS_HOME"
echo "   LD_LIBRARY_PATH: $LD_LIBRARY_PATH"
echo "   RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"

# Optional: Test unitree_sdk2py import
if [ "$1" = "--test" ]; then
    echo ""
    echo "Testing unitree_sdk2py import..."
    python3 -c "
try:
    import unitree_sdk2py
    from unitree_sdk2py.core.channel import ChannelSubscriber
    print('   unitree_sdk2py import successful')
    print('   Core communication module available')
except Exception as e:
    print(f'   Import failed: {e}')
    print('   Please check CycloneDDS configuration or reinstall unitree_sdk2py')
"
fi

echo ""
echo "Usage:"
echo "   source scripts/setup/setup_cyclonedds.sh        # Configure environment"
echo "   source scripts/setup/setup_cyclonedds.sh --test # Configure and test"
