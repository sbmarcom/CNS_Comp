
#   pin is rpi pin name
#   i/o is input/output
#   i/e is include/exclude (use/dont use)

#   pin    i/o  i/e
pins = {
    "27": ["o", "i"],  #13 - 
    "26": ["o", "i"],  #37 - 
    "25": ["i", "i"],  #22 - 
    "24": ["i", "i"],  #18 - 
    "23": ["i", "i"],  #16 - 
    "22": ["o", "i"],  #15 - 
    "21": ["o", "i"],  #40 - 
    "20": ["o", "e"],  #38 - SPI_6_MOSI_RESERVED 
    "19": ["o", "i"],  #35 -  
    "18": ["i", "i"],  #12 -  
    "17": ["i", "i"],  #11 - 
    "16": ["i", "i"],  #36 - 
    "15": ["i", "e"],  #10 - UART RESERVED 
    "14": ["i", "e"],  #8  - UART RESERVED
    "13": ["i", "i"],  #33 - IO_RST_TO_MOD
    "12": ["i", "i"],  #32 - IO_DETECT_TO__MOD
    "11": ["i", "i"],  #23 - 
    "10": ["i", "i"],  #19 - 
    "9":  ["i", "i"],  #21 - 
    "8":  ["i", "i"],  #24 -  
    "7":  ["i", "i"],  #26 - 
    "6":  ["i", "i"],  #31 -  
    "5":  ["i", "i"],  #29 - IO_ENABLE_TO_PP5V0_MOD
    "4":  ["o", "e"],  #7  - IO_ENABLE_TO_PP24V_MOD
    "3":  ["i", "i"],  #5  - 
    "2":  ["i", "i"]}  #3  - 

binaryio = ""
binaryie = ""

for num, pin in pins.items():
     binaryio = binaryio + ("1" if pin[0] == "o" else "0") 
     binaryie = binaryie + ("1" if pin[1] == "e" else "0") 

print("Pin Mask (dir):")
print("\tBin: ", binaryio)
print("\tDec: ", int(binaryio,2))
print("Exclude Mask (exclude):")
print("\tBin: ", binaryie)
print("\tDec: ", int(binaryie,2))


