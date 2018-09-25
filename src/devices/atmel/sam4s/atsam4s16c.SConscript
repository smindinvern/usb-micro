Import('env')
env.Append(CPPDEFINES=[('CHIP_NAME', '"ATSAM4S16C"')])
env.Append(CCFLAGS=['-include', File("atsam4s16.hh")])

objs = env.SConscript('SConscript')
Return('objs')
