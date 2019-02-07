'''
Created on Sep 10, 2015

@author: ash
'''
import difflib

def mydiffing():
    a = 'abcdefg'
    b = 'abc'
    seq = difflib.SequenceMatcher(None, a, b)
    r = seq.ratio()
    m = r * (len(a) + len(b))  / 2
    r2 = m / len(b)
    print r, r2
    pass

if __name__ == '__main__':
    mydiffing()
    pass