#!/usr/bin/env python3

import readchar

def printAllCharsUpTo(stop_char):

    char_start = ord('a')
    char_end   = ord(stop_char)

    for ch in range(char_start, char_end + 1):
        print(chr(ch), end = ", ")


def readAllUpTo(stop_char):

    key = readchar.readkey()
    while key != stop_char:
        print(f'Key: "{key}"')
        key = readchar.readkey()

def countNumbersUpTo(stop_char):
    total_numbers = 0
    total_others = 0

    key = readchar.readkey()
    while key != stop_char:
        print(f'Key: "{key}"')

        if key.isnumeric():
            total_numbers = total_numbers + 1
        else:
            total_others = total_others + 1

        key = readchar.readkey()

    print('You entered ' + str(total_numbers) + ' numbers.')
    print('You entered ' + str(total_others) + ' others.')

def main():

    print("Press a key...")

    # key = readchar.readkey()
    # printAllCharsUpTo(key)

    # readAllUpTo('x')
    countNumbersUpTo('x')

if __name__ == '__main__':
    main()
