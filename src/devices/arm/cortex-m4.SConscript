Import('env')

env.AppendUnique(CCFLAGS=['-mcpu=cortex-m4', '-mthumb'])
env.AppendUnique(ASFLAGS=['-mcpu=cortex-m4', '-mthumb'])
env.AppendUnique(LINKFLAGS=['-Wl,-G,0', '-mcpu=cortex-m4', '-mthumb'])
env.AppendUnique(CPPPATH=[Dir('.')])
objs = [env.Object(x) for x in ['arm.cc', 'primitives.s']]
Return('objs')
