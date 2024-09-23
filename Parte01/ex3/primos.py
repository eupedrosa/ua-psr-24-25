#!/usr/bin/env python3

import colorama

from colorama import Fore, Back, Style

maximum_number = 20

def isPrime(value):

    if value < 2:
        return False

    for i in range(2, value):
        if value % i == 0:
            return False

    return True

def main():
    print("Starting to compute prime numbers up to " + str(maximum_number))

    for i in range(0, maximum_number):
        if isPrime(i):
            print(Fore.GREEN + 'Number '
                + str(i) + ' is prime.' + Style.RESET_ALL)
        else:
            print('Number ' + str(i) + ' is not prime.')

if __name__ == "__main__":
    # __name__ is a special variable
    # Python automatically sets its value to “__main__” if the script is being run directly
    main()
