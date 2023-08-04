Import("env")


if(env.GetBuildType()=="debug"):
    env.Append(
        LINKFLAGS=[
            "--specs=rdimon.specs", #enables semihosting (printf via debugger)
            "--specs=nano.specs",
        ],
    )
if(env.GetBuildType()=="release"):
    env.Append(
        LINKFLAGS=[
            "--specs=nosys.specs",
            "--specs=nano.specs",
        ],
    )