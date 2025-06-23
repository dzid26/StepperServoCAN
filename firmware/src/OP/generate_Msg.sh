#!/bin/bash
# Exit script on first error.
set -e

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
PROJECT_ROOT="$SCRIPT_DIR/../../.."

echo "Changing to project root: $PROJECT_ROOT"
cd "$PROJECT_ROOT"

echo "Initializing and updating submodules..."
git submodule update --init --recursive

cd cantools
echo "Installing cantools and its dependencies..."
pip install -e .

echo "Generating C source from DBC..."
python -m cantools generate_c_source "$PROJECT_ROOT/opendbc/opendbc/dbc/ocelot_controls.dbc" --node SSC --database-name Msg --use-float

echo "Moving generated files..."
mv Msg.h ../firmware/src/OP/Msg.h
mv Msg.c ../firmware/src/OP/Msg.c

echo "Done."
