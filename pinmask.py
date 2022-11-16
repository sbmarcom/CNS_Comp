
#   pin is rpi pin name
#   i/o is input/output
#   i/e is include/exclude (use/dont use)

#   pin    i/o  i/e
pins = {
    "27": ["o", "i"],  #13 - ydir+
    "26": ["o", "i"],  #37 - ystep+
    "25": ["i", "e"],  #22 - 
    "24": ["i", "e"],  #18 - 
    "23": ["o", "i"],  #16 - xdir+
    "22": ["o", "i"],  #15 - xdir-
    "21": ["o", "i"],  #40 - ystep-
    "20": ["o", "i"],  #38 - xstep- 
    "19": ["o", "i"],  #35 - xstep+ 
    "18": ["i", "i"],  #12 - z-home 
    "17": ["i", "i"],  #11 - phi-home
    "16": ["i", "i"],  #36 - theta-home
    "15": ["i", "e"],  #10 - UART RESERVED 
    "14": ["i", "e"],  #8  - UART RESERVED
    "13": ["o", "e"],  #33 - 
    "12": ["i", "e"],  #32 - 
    "11": ["i", "e"],  #23 - 
    "10": ["i", "e"],  #19 - 
    "9":  ["i", "e"],  #21 - 
    "8":  ["i", "e"],  #24 -  
    "7":  ["i", "i"],  #26 - ESTOP IN
    "6":  ["o", "i"],  #31 - zdir 
    "5":  ["o", "i"],  #29 - zstep
    "4":  ["o", "i"],  #7  - ydir-
    "3":  ["i", "e"],  #5  - 
    "2":  ["i", "e"]}  #3  - 

binaryio = ""
binaryie = ""

for num, pin in pins.items():
     binaryio = binaryio + ("1" if pin[0] is "o" else "0") 
     binaryie = binaryie + ("1" if pin[1] is "e" else "0") 

print("Pin Mask (dir):")
print("\tBin: ", binaryio)
print("\tDec: ", int(binaryio,2))
print("Exclude Mask (exclude):")
print("\tBin: ", binaryie)
print("\tDec: ", int(binaryie,2))


