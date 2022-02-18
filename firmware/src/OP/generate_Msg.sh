#!/bin/bash

python -m cantools generate_c_source --no-floating-point-numbers OpenActuator.dbc --node Actuator --database-name Msg

