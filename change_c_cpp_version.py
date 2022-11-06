Import("env")

c = env["CFLAGS"]
c = [x for x in c if not "-std" in x]
c += ["-std=gnu11"]
env.Replace(CFLAGS=c)

cxx = env["CXXFLAGS"]
cxx = [x for x in cxx if not "-std" in x]
cxx += ["-std=c++14"]
env.Replace(CXXFLAGS=cxx)