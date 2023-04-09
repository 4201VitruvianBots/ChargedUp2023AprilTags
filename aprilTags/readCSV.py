import csv


def load_data(filename):
    my_list = []

    # loads csv data in python script
    with open(filename) as numbers:
        number_data = csv.reader(numbers, delimiter=',')
        next(number_data)
        for row in number_data:
            my_list.append(row)
        return my_list


def reformat_row(row):
    # csv index 11
    row_one = row[0]

    # csv index 12
    row_two = row[1]

    # resultant row
    fix_row = []

    # removes every value except for numbers
    for row in row_one:
        row.replace(' ', '')
        if row == row_one[0]:
            row.replace('data:', '')

        fix_row.append(row)

    for row in row_two:
        row.replace(' ', '')

        fix_row.append(row)

    # new data array
    return fix_row


new_list = load_data('OV2311_1.csv')

# creates a new array with only index 11 and 12: camera_matrix's data
new_row_list = [row for idx, row in enumerate(new_list) if idx in (11, 12)]

# reformat camera_matrix data to only have numerical indexes
new_row = reformat_row(new_row_list)
print(new_row)
