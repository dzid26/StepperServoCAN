Import("env")


if(env.GetBuildType()=="debug"):
    env.Append(
        LINKFLAGS=[
            "--specs=rdimon.specs", #enables semihosting (printf via debugger)
            "--specs=nano.specs",
            # "-u_ printf_float",  #adds linker printing floating point numbers flag. This makes too big code
        ],
    )
else:
    env.Append(
        LINKFLAGS=[
            "--specs=nosys.specs",
            "--specs=nano.specs",
        ],
    )