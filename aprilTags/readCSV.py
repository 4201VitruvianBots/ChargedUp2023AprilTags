import csv

import numpy as np


def load_data(filename):
    my_list = []

    # loads csv data in python script
    with open(filename) as numbers:
        number_data = csv.reader(numbers, delimiter=',')
        next(number_data)
        for row in number_data:
            my_list.append(row)
        return my_list


def filter_data(row):
    # csv index 11
    row_one = row[0]

    # csv index 12
    row_two = row[1]

    # resultant row
    fix_row = []

    # removes every value except for numbers
    for row in row_one:
        if row == row_one[0]:
            remove_space = row.replace('data: [', '')
            fix_row.append(remove_space.replace(' ', ''))
            continue
        if row == row_one[1] or row == row_one[3]:
            remove_space = row.replace('.', '')
            fix_row.append(remove_space.replace(' ', ''))
            continue
        remove_space = row.replace(' ', '')
        fix_row.append(remove_space)
        continue

    for row in row_two:
        if row == row_two[2] or row == row_two[3]:
            remove_space = row.replace('.', '')
            fix_row.append(remove_space.replace(' ', ''))
            continue
        if row == row_two[4]:
            remove_space = row.replace(' ', '')
            fix_row.append(remove_space.replace('.]', ''))
            continue
        remove_space = row.replace(' ', '')
        fix_row.append(remove_space)
        continue

    data_row = []
    for i in fix_row:
        if len(i) == 0:
            continue
        data_row.append(float(i))
    return data_row


def readCsv(filename):
    # loads csv file
    new_list = load_data("{}.csv".format(filename))

    # creates a new array with only index 11 and 12: camera_matrix's data
    new_row_list = [row for idx, row in enumerate(new_list) if idx in (11, 12)]

    # runs filter to remove extra strings or spaces in data
    new_row = filter_data(new_row_list)

    # turns matrix data array into a numpy matrix
    arr = np.array(new_row)
    iMatrix = np.reshape(arr, (3, 3))

    # prints resultant
    print(iMatrix)

    return iMatrix
