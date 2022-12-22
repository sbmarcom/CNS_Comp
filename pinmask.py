
#   pin is rpi pin name
#   i/o is input/output
#   i/e is include/exclude (use/dont use)

#   pin    i/o  i/e
pins = {
    "27": ["o", "i"],  #13 - 
    "26": ["o", "i"],  #37 - 
    "25": ["i", "e"],  #22 - 
    "24": ["i", "e"],  #18 - 
    "23": ["i", "e"],  #16 - 
    "22": ["o", "e"],  #15 - 
    "21": ["o", "e"],  #40 - 
    "20": ["o", "i"],  #38 - SPI_6_MOSI_RESERVED 
    "19": ["o", "i"],  #35 -  
    "18": ["i", "e"],  #12 -  
    "17": ["i", "e"],  #11 - 
    "16": ["i", "e"],  #36 - 
    "15": ["i", "e"],  #10 - UART RESERVED 
    "14": ["i", "e"],  #8  - UART RESERVED
    "13": ["o", "e"],  #33 - IO_RST_TO_MOD
    "12": ["i", "i"],  #32 - IO_DETECT_TO__MOD
    "11": ["i", "e"],  #23 - 
    "10": ["i", "e"],  #19 - 
    "9":  ["i", "e"],  #21 - 
    "8":  ["i", "e"],  #24 -  
    "7":  ["i", "e"],  #26 - 
    "6":  ["o", "e"],  #31 -  
    "5":  ["o", "e"],  #29 - IO_ENABLE_TO_PP5V0_MOD
    "4":  ["o", "i"],  #7  - IO_ENABLE_TO_PP24V_MOD
    "3":  ["i", "e"],  #5  - 
    "2":  ["i", "e"]}  #3  - 

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


