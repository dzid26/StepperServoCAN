
import subprocess
if "compiledb" not in COMMAND_LINE_TARGETS: #avoids infinite recursion
    subprocess.run(['pio', 'run','-t','compiledb'])
