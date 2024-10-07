#!/usr/bin/env python3

def addComplex(x,y):
    r0, i0 = x
    r1, i1 = y

    c = (r0 + r1, i0 + i1)
    return c

def multiplyComplex(x,y):
    r0, i0 = x
    r1, i1 = y

    c = (r0*r1 - i0*i1, r0*i1 +i0*r1)
    return c

def printComplex(x):
    real, imag = x
    print(f'{real} + {imag}i')

#===============================================================================
def main():

    # Example: (3 + 2i)(1 + 7i) = (3×1 − 2×7) + (3×7 + 2×1)i = −11 + 23i

    a = (3, 2)
    b = (1, 7)

    c = multiplyComplex(a, b)
    printComplex(c)



if __name__ == '__main__':
    main()
