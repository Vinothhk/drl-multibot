import psutil

cpu_usage = psutil.virtual_memory()
used = psutil.virtual_memory()[3]/1000000
print(cpu_usage)
print(f'{used} mb')