import ctypes

LC_lib = ctypes.cdll.LoadLibrary("libtestLib_shared.dylib")
tg = LC_lib.calcTimeST_delta(ctypes.c_double(20.0), ctypes.c_double(3.2))

print(tg)