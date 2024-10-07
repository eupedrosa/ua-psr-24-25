#!/usr/bin/env python3

import readchar

def countNumbersUpTo(stop_char):
    # lista vazia de keys
    inputs_number = []
    inputs_others = []

    input_dict = {} # Isto é um dicionário

    count = 1
    key = readchar.readkey()
    while key != stop_char:

        if key.isnumeric():
            inputs_number.append(key)
        else:
            inputs_others.append(key)

            input_dict[count] = key

        count = count + 1
        print(f'add "{key}"')

        key = readchar.readkey()

    print(inputs_number)
    print(sorted(inputs_number))

    # print(f'You entered {len(inputs_number)} numbers.')
    # print(f'You entered {len(inputs_others)} others.')

    # print(input_dict)

    # for rank, value in input_dict.items():
    #     print (f'key {value} with order {rank}')

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

# def countNumbersUpTo(stop_char):
#     total_numbers = 0
#     total_others = 0
#
#     key = readchar.readkey()
#     while key != stop_char:
#         print(f'Key: "{key}"')
#
#         if key.isnumeric():
#             total_numbers = total_numbers + 1
#         else:
#             total_others = total_others + 1
#
#         key = readchar.readkey()
#
#     print('You entered ' + str(total_numbers) + ' numbers.')
#     print('You entered ' + str(total_others) + ' others.')

def main():

    print("Press a key...")

    # key = readchar.readkey()
    # printAllCharsUpTo(key)

    # readAllUpTo('x')
    # countNumbersUpTo('x')
    countNumbersUpTo('x')

if __name__ == '__main__':
    main()
