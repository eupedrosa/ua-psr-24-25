#!/usr/bin/env python3
# --------------------------------------------------
# A simple python script to check perfect numbers
# Eurico Pedrosa.
# PSR, September 2024.
# --------------------------------------------------

maximum_number = 100  # maximum number to test.

def getDividers(value):
    """
    Return a list of dividers for the number value
    :param value: the number to test
    :return: a list of dividers.
    """

    dividers = []
    for d in range(1, value):
        if value % d == 0:
            dividers.append(d)

    return dividers

def isPerfect(value):
    """
    Checks whether the number value is perfect.
    :param value: the number to test.
    :return: True or False
    """

    dividers = getDividers(value)
    result = sum(dividers) # sum of a list

    if result == value:
        return True
    # your code here
    return False

def main():
    print("Starting to compute perfect numbers up to " + str(maximum_number))

    for i in range(1, maximum_number):
        if isPerfect(i):

            dividers = getDividers(i)
            msg = " + ".join( [str(x) for x in dividers] )

            # print("Number {} is perfect: {} = {}".format(i, msg, i))

            print(f"Number {i} is perfect {msg} = {i}")

            # print('Number ' + str(i) + ' is perfect.')

if __name__ == "__main__":
    main()
