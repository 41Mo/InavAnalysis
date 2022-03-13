import csv
def get_data_from_csv(*col_names, file_name):
    return_dict = {}
    for data_col_name in col_names:
        with open(file_name, 'r') as csv_file:
            reader = csv.reader(csv_file,delimiter=';')
            header = next(reader)
            data = []
            time = []
            dataIndex = header.index(data_col_name)
            t = 0
            for row in reader:
                data.append(float(row[dataIndex]))
                time.append(float(t))
                t = t + 1
            return_dict.update({data_col_name:data})
            return_dict.update({data_col_name + "_time":time})
    return return_dict