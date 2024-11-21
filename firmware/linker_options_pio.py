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


if(env.GetBuildType()=="debug, test"):
    env.Append(

    # Use the semihosted version of the syscalls
    LINKFLAGS=[
        "--specs=rdimon.specs",
    ],
    LIBS=[
        "rdimon",
    ],
)