Import('env')

env.AppendUnique(CCFLAGS=['-mcpu=cortex-m0plus'
                          ,'-mthumb'])
env.AppendUnique(ASFLAGS=['-mcpu=cortex-m0plus'
                          ,'-mthumb'])
env.AppendUnique(LINKFLAGS=['-Wl,-G,0', '-Wl,-N', '-mcpu=cortex-m0plus', '-mthumb'])
env.AppendUnique(CPPPATH=[Dir('.')])
objs = [env.Object(x) for x in ['arm.cc', 'primitives.s', 'div.s', 'thumb_case.s']]
Return('objs')