Import('env')
env.Append(CPPDEFINES=[('CHIP_NAME', '"ATSAMD21E18A"')])
env.Append(CCFLAGS=['-include', File("atsamd21/atsamd21x18.hh")])
env.Depends(File("atsamd21/atsamd21.hh"), File("atsamd21/atsamd21x18.hh"))

objs = env.SConscript('SConscript')
Return('objs')
