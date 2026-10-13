Import("env")

env.Append(
    LINKFLAGS=[
        "-mcpu=cortex-m4",
        "-mthumb",
        "-mfpu=fpv4-sp-d16",
        "-mfloat-abi=hard"
    ]
)