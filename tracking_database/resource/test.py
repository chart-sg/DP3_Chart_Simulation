import json
from pprint import pprint

with open('get_casualty_summary.json','r') as string:
    my_dict=json.load(string)
string.close()

def iterate_multidimensional(my_dict):
    for k,v in my_dict.items():
        if k == "data":
            for item in v:
                # print(item)
                for obj in item:
                    if obj == ''
                    print (item[obj])
                
iterate_multidimensional(my_dict)
# pprint(my_dict['data'])