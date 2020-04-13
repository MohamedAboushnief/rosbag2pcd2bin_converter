import glob
import os

path = './bin_files/'

b1=0
b2=0
b3=0
b4=0
b5=0
b6=0

for i, filename in enumerate(glob.glob(path + '*.bin')):
    os.rename(filename, os.path.join(path,  str(b6)+str(b5)+str(b4)+str(b3)+str(b2)+str(b1)+'.bin'))
    print(str(b6)+str(b5)+str(b4)+str(b3)+str(b2)+str(b1))

  
    b1=b1+1   
    if(b1==10):
        b1=0
        b2=b2+1
    if(b2==10):
        b2=0
        b3=b3+1    
    if(b3==10):
        b3=0
        b4=b4+1    
    if(b4==10):
        b4=0
        b5=b5+1    
    if(b5==10):
        b5=0
        b6=b6+1    