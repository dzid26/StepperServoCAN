Import("env")
env.Append(LINKFLAGS=["--specs=nosys.specs", "--specs=nano.specs"])