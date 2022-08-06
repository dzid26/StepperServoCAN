#!/bin/bash

cd ../../..

git submodule init cantools
git submodule init opendbc

cd cantools
python -m cantools generate_c_source "../opendbc/ocelot_controls.dbc" --node EPAS --database-name Msg --use-float

mv Msg.h ../firmware/src/OP/Msg.h
mv Msg.c ../firmware/src/OP/Msg.c


